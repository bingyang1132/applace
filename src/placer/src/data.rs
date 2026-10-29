//! Placement data structures
//!
//! Extracts and prepares the data used by the optimizer from a CircuitDb; can also build the
//! same data from bbox/pin/sym JSON files (for testing the placer on its own).

use crate::error::{PlaceError, PlaceResult};
use crate::params::PlacementConfig;
use applace_core::CircuitDb;
use ndarray::Array1;
use serde::Deserialize;
use std::collections::HashMap;
use std::path::Path;

// === Structs used for JSON parsing (bbox / pin / sym) ===

#[derive(Debug, Deserialize)]
struct BboxRect {
    xl: f64,
    xh: f64,
    yl: f64,
    yh: f64,
}

#[derive(Debug, Deserialize)]
struct BboxCell {
    cell_name: String,
    bbox: BboxRect,
    /// Abstract form: whether this device is a PMOS, which is what decides
    /// the guard-ring reservation (`pmos_hard_expand_for_gr`) in the overlap term.
    /// Absent in older fixtures, where every device is treated as non-PMOS exactly
    /// as before.
    #[serde(default, rename = "isPmos")]
    is_pmos: bool,
}

#[derive(Debug, Deserialize)]
struct BboxFile {
    cells: Vec<BboxCell>,
    /// Abstract form: the placement region. The netlist path gets a boundary for
    /// free because its boxes carry absolute source-layout coordinates; the abstract
    /// form deliberately does not ship positions, so the region has to be stated.
    /// Absent = fall back to the union of the boxes, which is what this loader did
    /// before the field existed.
    #[serde(default)]
    boundary: Option<BboxRect>,
}

#[derive(Debug, Deserialize)]
struct PinEntry {
    cell_name: String,
    offset_x: f64,
    offset_y: f64,
    #[serde(rename = "isIO")]
    is_io: bool,
    nets: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct PinFile {
    pins: Vec<PinEntry>,
}

/// Placement data (prepared for optimization)
#[derive(Debug, Clone)]
pub struct PlaceData {
    // === Basic data ===
    pub num_cells: usize,
    pub num_pins: usize,
    pub num_nets: usize,

    // === Cell attributes ===
    pub cell_widths: Array1<f64>,  // [N]
    pub cell_heights: Array1<f64>, // [N]
    pub cell_areas: Array1<f64>,   // [N]
    /// Overlap-only geometry widths (PMOS GR expanded).
    pub cell_widths_keepout: Array1<f64>, // [N]
    /// Overlap-only geometry heights (PMOS GR expanded).
    pub cell_heights_keepout: Array1<f64>, // [N]
    /// Overlap-only geometry areas.
    pub cell_areas_keepout: Array1<f64>, // [N]
    /// PMOS mask used to build overlap-only geometry.
    pub is_pmos_mask: Array1<bool>, // [N]

    // === Pin attributes ===
    /// Pin offsets, laid out as [x0...x_{M-1}, y0...y_{M-1}]
    pub pin_offset: Array1<f64>, // [2*M]

    /// Pin-to-cell mapping
    pub pin_owner_cell: Array1<i32>, // [M]

    /// Whether the pin is an IO pin (excluded from optimization)
    pub io_pin_mask: Array1<bool>, // [M]

    // === Net attributes ===
    pub net_weights: Array1<f64>, // [K]

    // === Net-pin mapping (CSR format) ===
    pub net_pin_offsets: Array1<usize>, // [K+1]
    pub net_pin_index: Array1<i32>,     // [total_pins]

    // === Cell-pin mapping (CSR format) ===
    pub cell_pin_offsets: Array1<usize>, // [N+1]
    pub cell_pin_index: Array1<i32>,     // [total_pins]

    // === Overlap constraints ===
    pub overlap_pairs: Array1<i32>, // [2 * num_overlap_pairs]
    pub num_overlap_pairs: usize,

    // === Symmetry constraints ===
    pub sym_pairs: Array1<i32>,           // [2 * num_sym_pairs]
    pub sym_pair_group: Array1<i32>,      // [num_sym_pairs]
    pub sym_group_vertical: Array1<bool>, // [num_sym_groups] (true=vert, false=horiz)
    /// Declared symmetry-group names, parallel to `sym_group_vertical`.
    ///
    /// Only the JSON path (`from_json_files`) has names to carry: `sym.json`'s keys.
    /// The `CircuitDb` path leaves it empty, and every consumer must cope with that --
    /// it is reporting metadata (`placement_summary.json`), never an optimizer input.
    pub sym_group_names: Vec<String>,
    pub num_sym_pairs: usize,
    pub num_sym_groups: usize,
    pub sym_self_weight: f64,
    /// Symmetry anchor offsets from lower-left corner.
    /// Defaults to the cell center (w/2,h/2).
    pub sym_anchor_x: Array1<f64>, // [N]
    pub sym_anchor_y: Array1<f64>, // [N]

    // === Well cluster constraints ===
    pub well_group_offsets: Array1<usize>, // [num_clusters+1]
    pub well_group_index: Array1<i32>,     // [total_cluster_cells]
    pub cell_well_group: Array1<i32>,      // [N] (-1 = not in any cluster)
    pub num_clusters: usize,

    // === Array/collinear constraints ===
    pub collinear_member_index: Array1<i32>, // [total_array_members]
    pub collinear_member_offsets: Array1<usize>, // [num_collinear_groups+1]
    pub collinear_vertical: Array1<bool>,    // [num_collinear_groups]
    pub num_collinear_groups: usize,

    // === Boundary information ===
    pub boundary_xl: f64,
    pub boundary_yl: f64,
    pub boundary_xh: f64,
    pub boundary_yh: f64,
}

/// The keys one `sym.json` block may carry in the abstract form.
const SYM_BLOCK_KEYS: [&str; 2] = ["axis", "pairs"];

/// Reads a block's symmetry axis, and refuses anything the loader does not understand.
///
/// The axis is spelled out rather than carried as a boolean, and unknown keys are an
/// error rather than a shrug: in the old flat form a mistyped key looked exactly like a
/// device name, and a mistyped flag looked exactly like a pair.
fn sym_axis_is_vertical(
    block: &str,
    body: &serde_json::Map<String, serde_json::Value>,
) -> PlaceResult<bool> {
    for key in body.keys() {
        if !SYM_BLOCK_KEYS.contains(&key.as_str()) {
            return Err(PlaceError::DataInitError(format!(
                "sym.json block `{block}`: unknown key `{key}`. A block is \
                 {{\"axis\": \"vertical\"|\"horizontal\", \"pairs\": {{\"A\": \"B\"}}}}"
            )));
        }
    }
    match body.get("axis").map(|v| v.as_str()) {
        None => Ok(true),
        Some(Some("vertical")) => Ok(true),
        Some(Some("horizontal")) => Ok(false),
        Some(other) => Err(PlaceError::DataInitError(format!(
            "sym.json block `{block}`: axis is `{}`, want `vertical` or `horizontal`",
            other.unwrap_or("not a string")
        ))),
    }
}

impl PlaceData {
    /// Extracts the data from a CircuitDb
    pub fn from_circuit_db(db: &CircuitDb, params: &PlacementConfig) -> PlaceResult<Self> {
        let num_cells = db.num_cells();
        let num_pins = db.num_pins();
        let num_nets = db.num_nets();

        log::info!("Extracting placement data from CircuitDb:");
        log::info!(
            "  Cells: {}, Pins: {}, Nets: {}",
            num_cells,
            num_pins,
            num_nets
        );

        // === 1. Extract cell attributes ===
        let (
            cell_widths,
            cell_heights,
            cell_areas,
            cell_widths_keepout,
            cell_heights_keepout,
            cell_areas_keepout,
            is_pmos_mask,
        ) = extract_cell_properties(db, num_cells, params.pmos_hard_expand_for_gr)?;

        // === 2. Extract pin attributes ===
        let (pin_offset, pin_owner_cell, io_pin_mask) = extract_pin_properties(db, num_pins)?;

        // === 3. Extract net attributes and mappings ===
        let (net_weights, net_pin_offsets, net_pin_index) = extract_net_properties(db, num_nets)?;

        // === 4. Build the cell-pin mapping ===
        let (cell_pin_offsets, cell_pin_index) = build_cell_to_pin_map(db, num_cells)?;

        // === 5. Build the overlap constraints (all cell pairs by default) ===
        let (overlap_pairs, num_overlap_pairs) = build_overlap_pairs(num_cells);

        // === 6. Extract symmetry constraints ===
        let (sym_pairs, sym_pair_group, sym_group_vertical, num_sym_pairs, num_sym_groups) =
            extract_symmetry_constraints(db)?;

        // === 7. Extract well cluster constraints ===
        let (well_group_offsets, well_group_index, cell_well_group, num_clusters) =
            extract_well_groups(db, num_cells)?;

        // === 8. Extract array constraints ===
        let (
            collinear_member_index,
            collinear_member_offsets,
            collinear_vertical,
            num_collinear_groups,
        ) = extract_array_constraints(db)?;

        // === 9. Compute the boundary (derived from the total area at the target utilization) ===
        let total_area = cell_areas.sum().max(1.0);
        let (boundary_xl, boundary_yl, boundary_xh, boundary_yh) = compute_boundary(total_area);

        log::info!("Data extraction complete:");
        log::info!(
            "  Symmetry: {} pairs in {} groups",
            num_sym_pairs,
            num_sym_groups
        );
        log::info!("  Well Clusters: {}", num_clusters);
        log::info!("  Array Groups: {}", num_collinear_groups);
        let sym_anchor_x = cell_widths.mapv(|w| w * 0.5);
        let sym_anchor_y = cell_heights.mapv(|h| h * 0.5);

        Ok(Self {
            num_cells,
            num_pins,
            num_nets,
            cell_widths,
            cell_heights,
            cell_areas,
            cell_widths_keepout,
            cell_heights_keepout,
            cell_areas_keepout,
            is_pmos_mask,
            pin_offset,
            pin_owner_cell,
            io_pin_mask,
            net_weights,
            net_pin_offsets,
            net_pin_index,
            cell_pin_offsets,
            cell_pin_index,
            overlap_pairs,
            num_overlap_pairs,
            sym_pairs,
            sym_pair_group,
            sym_group_vertical,
            // The CircuitDb path carries no declared group names.
            sym_group_names: Vec::new(),
            num_sym_pairs,
            num_sym_groups,
            sym_self_weight: params.sym_self_weight,
            sym_anchor_x,
            sym_anchor_y,
            well_group_offsets,
            well_group_index,
            cell_well_group,
            num_clusters,
            collinear_member_index,
            collinear_member_offsets,
            collinear_vertical,
            num_collinear_groups,
            boundary_xl,
            boundary_yl,
            boundary_xh,
            boundary_yh,
        })
    }

    /// Builds a PlaceData from the three JSON files bbox / pin / sym (for testing the placer on its own, without a CircuitDb)
    pub fn from_json_files<P: AsRef<Path>>(
        bbox_path: P,
        pin_path: P,
        sym_path: P,
    ) -> PlaceResult<Self> {
        // 0.0 = no guard-ring reservation, which is what this entry point always did.
        Self::from_json_files_with_gr(bbox_path, pin_path, sym_path, 0.0)
    }

    /// Abstract form: same as [`Self::from_json_files`] but reserves guard-ring
    /// room around every device flagged `isPmos`, matching what the netlist path does
    /// with `pmos_hard_expand_for_gr`.
    pub fn from_json_files_with_gr<P: AsRef<Path>>(
        bbox_path: P,
        pin_path: P,
        sym_path: P,
        pmos_hard_expand_for_gr: f64,
    ) -> PlaceResult<Self> {
        let bbox_content = std::fs::read_to_string(bbox_path.as_ref())
            .map_err(|e| PlaceError::DataInitError(format!("read bbox: {}", e)))?;
        let pin_content = std::fs::read_to_string(pin_path.as_ref())
            .map_err(|e| PlaceError::DataInitError(format!("read pin: {}", e)))?;
        let sym_content = std::fs::read_to_string(sym_path.as_ref())
            .map_err(|e| PlaceError::DataInitError(format!("read sym: {}", e)))?;

        let bbox_file: BboxFile = serde_json::from_str(&bbox_content)
            .map_err(|e| PlaceError::DataInitError(format!("parse bbox: {}", e)))?;
        let pin_file: PinFile = serde_json::from_str(&pin_content)
            .map_err(|e| PlaceError::DataInitError(format!("parse pin: {}", e)))?;

        let num_cells = bbox_file.cells.len();
        if num_cells == 0 {
            return Err(PlaceError::DataInitError("bbox has no cells".into()));
        }

        // cell_name -> index (the bbox order is the cell index)
        let mut name_to_idx: HashMap<String, usize> = HashMap::new();
        for (i, c) in bbox_file.cells.iter().enumerate() {
            name_to_idx.insert(c.cell_name.clone(), i);
        }
        // short name (e.g. "M21") -> cell index, used for sym matching
        let short_to_idx =
            build_short_to_idx(bbox_file.cells.iter().map(|c| c.cell_name.as_str()))?;

        // === 1. Cell attributes ===
        let mut cell_widths = Vec::with_capacity(num_cells);
        let mut cell_heights = Vec::with_capacity(num_cells);
        let mut cell_areas = Vec::with_capacity(num_cells);
        let mut cell_widths_keepout = Vec::with_capacity(num_cells);
        let mut cell_heights_keepout = Vec::with_capacity(num_cells);
        let mut cell_areas_keepout = Vec::with_capacity(num_cells);
        let mut is_pmos_mask = Vec::with_capacity(num_cells);
        let mut boundary_xl = f64::INFINITY;
        let mut boundary_yl = f64::INFINITY;
        let mut boundary_xh = f64::NEG_INFINITY;
        let mut boundary_yh = f64::NEG_INFINITY;

        for c in &bbox_file.cells {
            let w = (c.bbox.xh - c.bbox.xl).abs();
            let h = (c.bbox.yh - c.bbox.yl).abs();
            cell_widths.push(w);
            cell_heights.push(h);
            cell_areas.push(w * h);
            // `pmos_hard_expand_for_gr` is applied by `overlap_dims_for_device` in the
            // netlist path; mirror it here so the abstract form reserves guard-ring
            // room for the same devices. `isPmos` defaults to false, so a fixture that
            // omits it keeps the previous all-non-PMOS behaviour bit for bit.
            let (w_ovl, h_ovl) = overlap_dims_for_device(w, h, c.is_pmos, pmos_hard_expand_for_gr);
            cell_widths_keepout.push(w_ovl);
            cell_heights_keepout.push(h_ovl);
            cell_areas_keepout.push(w_ovl * h_ovl);
            is_pmos_mask.push(c.is_pmos);
            boundary_xl = boundary_xl.min(c.bbox.xl);
            boundary_yl = boundary_yl.min(c.bbox.yl);
            boundary_xh = boundary_xh.max(c.bbox.xh);
            boundary_yh = boundary_yh.max(c.bbox.yh);
        }
        if let Some(b) = &bbox_file.boundary {
            boundary_xl = b.xl;
            boundary_yl = b.yl;
            boundary_xh = b.xh;
            boundary_yh = b.yh;
        } else {
            // Enlarge the boundary slightly
            let margin = 100.0;
            boundary_xl -= margin;
            boundary_yl -= margin;
            boundary_xh += margin;
            boundary_yh += margin;
        }

        // === 2. Pin attributes and net mapping ===
        let num_pins = pin_file.pins.len();
        let mut pin_offset_x = Vec::with_capacity(num_pins);
        let mut pin_offset_y = Vec::with_capacity(num_pins);
        let mut pin_owner_cell = Vec::with_capacity(num_pins);
        let mut io_pin_mask = Vec::with_capacity(num_pins);
        let mut pin_net_names: Vec<String> = Vec::with_capacity(num_pins);

        for p in &pin_file.pins {
            pin_offset_x.push(p.offset_x);
            pin_offset_y.push(p.offset_y);
            let cidx = p
                .cell_name
                .is_empty()
                .then_some(-1i32)
                .or_else(|| name_to_idx.get(p.cell_name.as_str()).map(|&i| i as i32))
                .unwrap_or(-1);
            pin_owner_cell.push(cidx);
            io_pin_mask.push(p.is_io);
            pin_net_names.push(p.nets.first().cloned().unwrap_or_default());
        }

        let mut pin_offset = pin_offset_x;
        pin_offset.extend(pin_offset_y);

        // Build net name -> the list of pins on that net; then generate net_pin_offsets / net_pin_index
        let mut net_to_pins: HashMap<String, Vec<i32>> = HashMap::new();
        for (pin_idx, net_name) in pin_net_names.iter().enumerate() {
            net_to_pins
                .entry(net_name.clone())
                .or_default()
                .push(pin_idx as i32);
        }
        // Determinism is a hard invariant: `net_to_pins` is a HashMap, so its iteration
        // order varies run to run. Sort before laying out the net-to-pin index.
        let mut net_names_ordered: Vec<String> = net_to_pins.keys().cloned().collect();
        net_names_ordered.sort_unstable();
        let num_nets = net_names_ordered.len();
        let mut net_pin_offsets = Vec::with_capacity(num_nets + 1);
        let mut net_pin_index = Vec::new();
        let mut start = 0;
        for name in &net_names_ordered {
            net_pin_offsets.push(start);
            let pins = net_to_pins.get(name).unwrap();
            start += pins.len();
            net_pin_index.extend(pins);
        }
        net_pin_offsets.push(start);

        let net_weights: Vec<f64> = (0..num_nets).map(|_| 1.0).collect();

        // === 3. Cell-pin mapping ===
        let mut cell_pin_index: Vec<i32> = Vec::new();
        let mut cell_pin_offsets = Vec::with_capacity(num_cells + 1);
        let mut cell_pins: Vec<Vec<i32>> = (0..num_cells).map(|_| Vec::new()).collect();
        for (pin_idx, &c) in pin_owner_cell.iter().enumerate() {
            if c >= 0 && (c as usize) < num_cells {
                cell_pins[c as usize].push(pin_idx as i32);
            }
        }
        let mut cs = 0usize;
        for pins in &cell_pins {
            cell_pin_offsets.push(cs);
            cs += pins.len();
            cell_pin_index.extend(pins);
        }
        cell_pin_offsets.push(cs);

        // === 4. Overlap constraints (all cell pairs) ===
        let (overlap_pairs, num_overlap_pairs) = build_overlap_pairs(num_cells);

        // === 5. Symmetry constraints ===
        let sym_value: serde_json::Value = serde_json::from_str(&sym_content)
            .map_err(|e| PlaceError::DataInitError(format!("parse sym: {}", e)))?;
        let mut sym_pairs_data = Vec::new();
        let mut sym_pair_group_data = Vec::new();
        let mut sym_group_vertical_data = Vec::new();
        // Declared group names, parallel to `sym_group_vertical_data`; reported in
        // `placement_summary.json` so a reader can name the pair they are looking at.
        let mut sym_group_names_data: Vec<String> = Vec::new();
        let mut grp_idx = 0i32;

        if let Some(obj) = sym_value.as_object() {
            for (top_name, val) in obj {
                let sub = val
                    .as_object()
                    .ok_or_else(|| PlaceError::DataInitError("sym entry is not object".into()))?;
                let is_vert = sym_axis_is_vertical(top_name, sub)?;
                let pairs = sub
                    .get("pairs")
                    .and_then(|v| v.as_object())
                    .ok_or_else(|| {
                        PlaceError::DataInitError(format!(
                            "sym.json block `{top_name}` has no `pairs` object"
                        ))
                    })?;
                let grp_start = sym_pairs_data.len() / 2;
                for (k, v) in pairs {
                    let Some(right) = v.as_str() else {
                        continue;
                    };
                    let left_idx = match short_to_idx.get(k.as_str()) {
                        Some(&i) => i as i32,
                        None => continue,
                    };
                    let right_idx = match short_to_idx.get(right) {
                        Some(&i) => i as i32,
                        None => continue,
                    };
                    sym_pairs_data.push(left_idx);
                    sym_pairs_data.push(right_idx);
                    sym_pair_group_data.push(grp_idx);
                }
                if sym_pairs_data.len() / 2 > grp_start {
                    sym_group_vertical_data.push(is_vert);
                    sym_group_names_data.push(top_name.clone());
                    grp_idx += 1;
                }
            }
        }
        let num_sym_pairs = sym_pairs_data.len() / 2;
        let num_sym_groups = sym_group_vertical_data.len();

        // === 6. Cluster / array left empty ===
        let well_group_offsets = Array1::from_vec(vec![0]);
        let well_group_index = Array1::from_vec(vec![]);
        let cell_well_group = Array1::from_vec(vec![-1; num_cells]);
        let num_clusters = 0;
        let collinear_member_index = Array1::from_vec(vec![]);
        let collinear_member_offsets = Array1::from_vec(vec![0]);
        let collinear_vertical = Array1::from_vec(vec![]);
        let num_collinear_groups = 0;

        log::info!(
            "PlaceData from JSON: cells={}, pins={}, nets={}, sym_pairs={}, sym_grps={}",
            num_cells,
            num_pins,
            num_nets,
            num_sym_pairs,
            num_sym_groups
        );
        let sym_anchor_x: Vec<f64> = cell_widths.iter().map(|w| *w * 0.5).collect();
        let sym_anchor_y: Vec<f64> = cell_heights.iter().map(|h| *h * 0.5).collect();

        Ok(Self {
            num_cells,
            num_pins,
            num_nets,
            cell_widths: Array1::from_vec(cell_widths),
            cell_heights: Array1::from_vec(cell_heights),
            cell_areas: Array1::from_vec(cell_areas),
            cell_widths_keepout: Array1::from_vec(cell_widths_keepout),
            cell_heights_keepout: Array1::from_vec(cell_heights_keepout),
            cell_areas_keepout: Array1::from_vec(cell_areas_keepout),
            is_pmos_mask: Array1::from_vec(is_pmos_mask),
            pin_offset: Array1::from_vec(pin_offset),
            pin_owner_cell: Array1::from_vec(pin_owner_cell),
            io_pin_mask: Array1::from_vec(io_pin_mask),
            net_weights: Array1::from_vec(net_weights),
            net_pin_offsets: Array1::from_vec(net_pin_offsets),
            net_pin_index: Array1::from_vec(net_pin_index),
            cell_pin_offsets: Array1::from_vec(cell_pin_offsets),
            cell_pin_index: Array1::from_vec(cell_pin_index),
            overlap_pairs,
            num_overlap_pairs,
            sym_pairs: Array1::from_vec(sym_pairs_data),
            sym_pair_group: Array1::from_vec(sym_pair_group_data),
            sym_group_vertical: Array1::from_vec(sym_group_vertical_data),
            sym_group_names: sym_group_names_data,
            num_sym_pairs,
            num_sym_groups,
            sym_self_weight: 2.0,
            sym_anchor_x: Array1::from_vec(sym_anchor_x),
            sym_anchor_y: Array1::from_vec(sym_anchor_y),
            well_group_offsets,
            well_group_index,
            cell_well_group,
            num_clusters,
            collinear_member_index,
            collinear_member_offsets,
            collinear_vertical,
            num_collinear_groups,
            boundary_xl,
            boundary_yl,
            boundary_xh,
            boundary_yh,
        })
    }

    /// Validates data consistency
    pub fn validate(&self) -> PlaceResult<()> {
        // Check the array lengths
        if self.cell_widths.len() != self.num_cells {
            return Err(PlaceError::DataInitError(format!(
                "cell_widths length {} != num_cells {}",
                self.cell_widths.len(),
                self.num_cells
            )));
        }

        if self.pin_offset.len() != 2 * self.num_pins {
            return Err(PlaceError::DataInitError(format!(
                "pin_offset length {} != 2 * num_pins {}",
                self.pin_offset.len(),
                2 * self.num_pins
            )));
        }
        if self.cell_widths_keepout.len() != self.num_cells
            || self.cell_heights_keepout.len() != self.num_cells
            || self.cell_areas_keepout.len() != self.num_cells
            || self.is_pmos_mask.len() != self.num_cells
        {
            return Err(PlaceError::DataInitError(format!(
                "overlap geometry length mismatch: w_ovl={}, h_ovl={}, a_ovl={}, pmos_mask={}, num_cells={}",
                self.cell_widths_keepout.len(),
                self.cell_heights_keepout.len(),
                self.cell_areas_keepout.len(),
                self.is_pmos_mask.len(),
                self.num_cells
            )));
        }

        if self.net_pin_offsets.len() != self.num_nets + 1 {
            return Err(PlaceError::DataInitError(format!(
                "net_pin_offsets length {} != num_nets + 1 {}",
                self.net_pin_offsets.len(),
                self.num_nets + 1
            )));
        }

        if self.sym_anchor_x.len() != self.num_cells || self.sym_anchor_y.len() != self.num_cells {
            return Err(PlaceError::DataInitError(format!(
                "sym_anchor length mismatch: x={}, y={}, num_cells={}",
                self.sym_anchor_x.len(),
                self.sym_anchor_y.len(),
                self.num_cells
            )));
        }

        // Check the index ranges
        for &pin_idx in self.net_pin_index.iter() {
            if pin_idx < 0 || pin_idx >= self.num_pins as i32 {
                return Err(PlaceError::DataInitError(format!(
                    "Invalid pin index: {}",
                    pin_idx
                )));
            }
        }

        log::info!("PlaceData validation passed");
        Ok(())
    }
}

// === Helper functions ===

/// Extracts cell attributes
fn extract_cell_properties(
    db: &CircuitDb,
    num_cells: usize,
    pmos_hard_expand_for_gr: f64,
) -> PlaceResult<(
    Array1<f64>,
    Array1<f64>,
    Array1<f64>,
    Array1<f64>,
    Array1<f64>,
    Array1<f64>,
    Array1<bool>,
)> {
    let mut widths = Vec::with_capacity(num_cells);
    let mut heights = Vec::with_capacity(num_cells);
    let mut areas = Vec::with_capacity(num_cells);
    let mut widths_keepout = Vec::with_capacity(num_cells);
    let mut heights_keepout = Vec::with_capacity(num_cells);
    let mut areas_keepout = Vec::with_capacity(num_cells);
    let mut is_pmos = Vec::with_capacity(num_cells);

    for cidx in 0..num_cells {
        let Some(cell) = db.cell(cidx as i32) else {
            return Err(PlaceError::DataInitError(format!(
                "cell {} not found",
                cidx
            )));
        };
        let w = cell.width() as f64;
        let h = cell.height() as f64;
        widths.push(w);
        heights.push(h);
        areas.push(w * h);
        let pmos = is_pmos_model(db, cell.model());
        is_pmos.push(pmos);
        let (w_ovl, h_ovl) = overlap_dims_for_device(w, h, pmos, pmos_hard_expand_for_gr);
        widths_keepout.push(w_ovl);
        heights_keepout.push(h_ovl);
        areas_keepout.push(w_ovl * h_ovl);
    }

    Ok((
        Array1::from_vec(widths),
        Array1::from_vec(heights),
        Array1::from_vec(areas),
        Array1::from_vec(widths_keepout),
        Array1::from_vec(heights_keepout),
        Array1::from_vec(areas_keepout),
        Array1::from_vec(is_pmos),
    ))
}

/// Is this device a PMOS?
///
/// The deck decides, through the `device_types` declaration that
/// `io::apply_device_types` installs on the `CircuitDb`. A deck that declares
/// nothing is refused when it is loaded, so there is no model-name heuristic
/// here: guessing from the spelling is precisely what silently loses the
/// guard-ring reservation on a deck this project was not developed against.
fn is_pmos_model(db: &CircuitDb, model: &str) -> bool {
    db.device_types()
        .map(|dt| dt.is_pmos(model))
        .unwrap_or(false)
}

/// `sym.json` names devices by their short name (`M21`); `bbox.json` names them by the
/// full instance path (`Core_test_flow_M21`). This builds the short → cell-index map.
///
/// Two properties this must hold:
///
/// 1. **Deterministic.** It used to iterate a `HashMap<String, usize>`, so when two
///    devices shared a short name the winner varied run to run — a direct violation of
///    the "same input → byte-identical output" invariant. It now walks the cells in
///    `bbox.json` order.
/// 2. **Explicit on collision.** Silently keeping one of the two colliding devices makes
///    a symmetry constraint quietly bind the wrong device. A collision is now a hard
///    error naming every conflict, because there is no defensible way to pick.
///
/// The four benchmark cases have no collisions, so this changes no output.
fn build_short_to_idx<'a, I>(full_names: I) -> PlaceResult<HashMap<String, usize>>
where
    I: IntoIterator<Item = &'a str>,
{
    // BTreeMap: the conflict report must be identical run to run too.
    let mut short_to_fulls: std::collections::BTreeMap<String, Vec<(usize, String)>> =
        std::collections::BTreeMap::new();
    for (idx, full) in full_names.into_iter().enumerate() {
        let short = full.rsplit('_').next().unwrap_or(full);
        short_to_fulls
            .entry(short.to_string())
            .or_default()
            .push((idx, full.to_string()));
    }

    let conflicts: Vec<String> = short_to_fulls
        .iter()
        .filter(|(_, v)| v.len() > 1)
        .map(|(short, v)| {
            let fulls: Vec<&str> = v.iter().map(|(_, f)| f.as_str()).collect();
            format!("{} <- [{}]", short, fulls.join(", "))
        })
        .collect();
    if !conflicts.is_empty() {
        return Err(PlaceError::DataInitError(format!(
            "bbox.json has {} short-name collision(s); sym.json matches devices by short \
             name, so the constraint would bind an arbitrary one of them. Rename the \
             devices or extend sym.json to full names. Conflicts: {}",
            conflicts.len(),
            conflicts.join("; ")
        )));
    }

    Ok(short_to_fulls
        .into_iter()
        .map(|(short, v)| (short, v[0].0))
        .collect())
}

fn overlap_dims_for_device(
    w: f64,
    h: f64,
    is_pmos: bool,
    pmos_hard_expand_for_gr: f64,
) -> (f64, f64) {
    if is_pmos {
        (
            (w + 2.0 * pmos_hard_expand_for_gr.max(0.0)).max(1e-12),
            (h + 2.0 * pmos_hard_expand_for_gr.max(0.0)).max(1e-12),
        )
    } else {
        (w.max(1e-12), h.max(1e-12))
    }
}

/// Extracts pin attributes
fn extract_pin_properties(
    db: &CircuitDb,
    num_pins: usize,
) -> PlaceResult<(Array1<f64>, Array1<i32>, Array1<bool>)> {
    let mut offset_data = Vec::with_capacity(2 * num_pins);
    let mut pin_owner_cell = Vec::with_capacity(num_pins);
    let mut mask = Vec::with_capacity(num_pins);

    // Extract all x offsets first
    for pidx in 0..num_pins {
        if let Some(pin) = db.pin(pidx as i32) {
            let offset = pin.offset();
            offset_data.push(offset.x as f64);
        } else {
            return Err(PlaceError::DataInitError(format!("Pin {} not found", pidx)));
        }
    }

    // Then extract all y offsets
    for pidx in 0..num_pins {
        if let Some(pin) = db.pin(pidx as i32) {
            let offset = pin.offset();
            offset_data.push(offset.y as f64);
        }
    }

    // Extract the pin_owner_cell mapping and the mask
    for pidx in 0..num_pins {
        if let Some(pin) = db.pin(pidx as i32) {
            pin_owner_cell.push(pin.cell_idx().unwrap_or(-1));

            // The mask is true if the pin is an IO pin or its net is an IO net
            let is_io_pin = pin.is_io();
            let is_io_net = if let Some(net_idx) = pin.net_idx() {
                db.net(net_idx).map(|n| n.is_io()).unwrap_or(false)
            } else {
                false
            };
            mask.push(is_io_pin || is_io_net);
        }
    }

    Ok((
        Array1::from_vec(offset_data),
        Array1::from_vec(pin_owner_cell),
        Array1::from_vec(mask),
    ))
}

/// Extracts net attributes and mappings
fn extract_net_properties(
    db: &CircuitDb,
    num_nets: usize,
) -> PlaceResult<(Array1<f64>, Array1<usize>, Array1<i32>)> {
    let mut weights = Vec::with_capacity(num_nets);
    let mut net_pin_offsets = Vec::with_capacity(num_nets + 1);
    let mut net_pin_index = Vec::new();

    let mut start_idx = 0;
    for nidx in 0..num_nets {
        if let Some(net) = db.net(nidx as i32) {
            // Net weight (IO nets get weight 0)
            if net.is_io() {
                weights.push(0.0);
            } else {
                weights.push(net.placement_weight());
            }

            // Net-pin mapping (CSR format)
            net_pin_offsets.push(start_idx);
            for &pin_idx in net.pin_ids() {
                net_pin_index.push(pin_idx);
            }
            start_idx += net.pin_ids().len();
        } else {
            return Err(PlaceError::DataInitError(format!("Net {} not found", nidx)));
        }
    }
    net_pin_offsets.push(start_idx); // end marker

    Ok((
        Array1::from_vec(weights),
        Array1::from_vec(net_pin_offsets),
        Array1::from_vec(net_pin_index),
    ))
}

/// Builds the cell-pin mapping
fn build_cell_to_pin_map(
    db: &CircuitDb,
    num_cells: usize,
) -> PlaceResult<(Array1<usize>, Array1<i32>)> {
    let mut cell_pin_offsets = Vec::with_capacity(num_cells + 1);
    let mut cell_pin_index = Vec::new();

    let mut start_idx = 0;
    for cidx in 0..num_cells {
        if let Some(cell) = db.cell(cidx as i32) {
            cell_pin_offsets.push(start_idx);
            for &pin_idx in cell.pin_ids() {
                cell_pin_index.push(pin_idx);
            }
            start_idx += cell.pin_ids().len();
        }
    }
    cell_pin_offsets.push(start_idx); // end marker

    Ok((
        Array1::from_vec(cell_pin_offsets),
        Array1::from_vec(cell_pin_index),
    ))
}

/// Builds the overlap constraint pairs (all cell pairs by default)
fn build_overlap_pairs(num_cells: usize) -> (Array1<i32>, usize) {
    let mut pairs = Vec::new();

    // Generate all cell pairs (i, j) where i < j
    for i in 0..num_cells {
        for j in (i + 1)..num_cells {
            pairs.push(i as i32);
            pairs.push(j as i32);
        }
    }

    let num_pairs = pairs.len() / 2;
    (Array1::from_vec(pairs), num_pairs)
}

/// Extracts the symmetry constraints
fn extract_symmetry_constraints(
    db: &CircuitDb,
) -> PlaceResult<(Array1<i32>, Array1<i32>, Array1<bool>, usize, usize)> {
    let mut sym_pairs_data = Vec::new();
    let mut sym_pair_group_data = Vec::new();
    let mut sym_group_vertical_data = Vec::new();

    let mut grp_idx = 0;

    // One group per `sym.json` block, in the order `parse_symmetry` applied them; a
    // block that bound nothing leaves no group.
    for sym_group in db.sym_groups() {
        if sym_group.is_empty() {
            continue;
        }
        // Extract the symmetry pairs
        for &(lidx, ridx) in sym_group.sym_pairs() {
            sym_pairs_data.push(lidx);
            sym_pairs_data.push(ridx);
            sym_pair_group_data.push(grp_idx);
        }
        // Extract the self-symmetric cells
        for &sidx in sym_group.self_syms() {
            sym_pairs_data.push(sidx);
            sym_pairs_data.push(sidx);
            sym_pair_group_data.push(grp_idx);
        }

        // Record the symmetry direction
        sym_group_vertical_data.push(sym_group.is_vert());
        grp_idx += 1;
    }

    let num_pairs = sym_pair_group_data.len();
    let num_grps = sym_group_vertical_data.len();

    Ok((
        Array1::from_vec(sym_pairs_data),
        Array1::from_vec(sym_pair_group_data),
        Array1::from_vec(sym_group_vertical_data),
        num_pairs,
        num_grps,
    ))
}

/// Extracts the well cluster constraints
fn extract_well_groups(
    db: &CircuitDb,
    num_cells: usize,
) -> PlaceResult<(Array1<usize>, Array1<i32>, Array1<i32>, usize)> {
    let mut well_group_offsets = Vec::new();
    let mut well_group_index = Vec::new();
    let mut cell_well_group = vec![-1i32; num_cells];

    let mut start_idx = 0;
    let num_clusters = db.num_well_groups();

    for (cluster_idx, cluster) in db.well_groups().iter().enumerate() {
        well_group_offsets.push(start_idx);

        for &cell_idx in cluster {
            well_group_index.push(cell_idx);
            if (cell_idx as usize) < num_cells {
                cell_well_group[cell_idx as usize] = cluster_idx as i32;
            }
        }

        start_idx += cluster.len();
    }
    well_group_offsets.push(start_idx); // end marker

    Ok((
        Array1::from_vec(well_group_offsets),
        Array1::from_vec(well_group_index),
        Array1::from_vec(cell_well_group),
        num_clusters,
    ))
}

/// Extracts the array constraints
fn extract_array_constraints(
    db: &CircuitDb,
) -> PlaceResult<(Array1<i32>, Array1<usize>, Array1<bool>, usize)> {
    let mut collinear_flat = Vec::new();
    let mut collinear_start = Vec::new();
    let mut directions = Vec::new();

    let mut start_idx = 0;
    let num_groups = db.num_array_groups();

    for group in db.array_groups() {
        collinear_start.push(start_idx);

        for &unit_idx in group.array_members() {
            collinear_flat.push(unit_idx);
        }

        directions.push(group.is_vert());
        start_idx += group.array_members().len();
    }
    collinear_start.push(start_idx); // end marker

    Ok((
        Array1::from_vec(collinear_flat),
        Array1::from_vec(collinear_start),
        Array1::from_vec(directions),
        num_groups,
    ))
}

/// Computes the boundary: a square boundary derived from the total area at a target utilization of 0.7.
/// area(boundary) = total_cell_area / 0.7
fn compute_boundary(total_cell_area: f64) -> (f64, f64, f64, f64) {
    let target_util = 0.7f64;
    let boundary_area = (total_cell_area.max(1.0) / target_util).max(1.0);
    let side = boundary_area.sqrt().max(1000.0);
    (0.0, 0.0, side, side)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_overlap_pairs() {
        let (pairs, num_pairs) = build_overlap_pairs(3);
        assert_eq!(num_pairs, 3); // (0,1), (0,2), (1,2)
        assert_eq!(pairs.len(), 6); // 3 pairs * 2
    }

    /// Every `sym.json` block becomes its own group (see
    /// `io/src/parser.rs::parse_symmetry`). The gradient placer must see one group per
    /// block with its own axis variable -- the same shape the abstract loader builds.
    #[test]
    fn test_extract_symmetry_constraints_covers_every_group() {
        use applace_core::SymGroup;
        let mut db = CircuitDb::new();
        for _ in 0..4 {
            db.allocate_cell("c".to_string());
        }
        let mut first = SymGroup::new(true);
        first.add_sym_pair(0, 1);
        first.add_self_sym(2);
        db.add_sym_group(first);
        db.add_sym_group(SymGroup::new(true));
        let mut second = SymGroup::new(true);
        second.add_sym_pair(2, 3);
        db.add_sym_group(second);

        let (pairs, pair2grp, dirs, num_pairs, num_grps) =
            extract_symmetry_constraints(&db).unwrap();
        assert_eq!(num_grps, 2);
        assert_eq!(num_pairs, 3);
        // first: (0,1) then self-sym 2 as (2,2); the empty group is skipped; second: (2,3)
        assert_eq!(pairs.to_vec(), vec![0, 1, 2, 2, 2, 3]);
        assert_eq!(pair2grp.to_vec(), vec![0, 0, 1]);
        assert_eq!(dirs.to_vec(), vec![true, true]);
    }

    /// With no declaration on the database nothing is a PMOS. The flow cannot
    /// reach this state -- an undeclared deck is refused at load -- and the
    /// point of asserting it is that the answer is a flat `false` rather than a
    /// guess made from the model string.
    #[test]
    fn undeclared_database_classifies_nothing_as_pmos() {
        let db = CircuitDb::new();
        assert!(!is_pmos_model(&db, "pmos_example"));
        assert!(!is_pmos_model(&db, "PMOS"));
        assert!(!is_pmos_model(&db, "nmos_example"));
    }

    /// A deck declaration overrides the hard-wired vendor spellings — the whole point:
    /// a sky130 design gets its PMOS recognised, and a model the heuristic would have
    /// guessed `true` on is not PMOS if the deck says otherwise.
    #[test]
    fn declared_device_types_beat_the_heuristic() {
        use applace_core::{DeviceMatch, DeviceTypes};
        let mut db = CircuitDb::new();
        db.set_device_types(Some(DeviceTypes::new(
            DeviceMatch::Prefix,
            vec!["sky130_fd_pr__pfet".into()],
            vec!["sky130_fd_pr__nfet".into()],
            vec![],
            vec![],
            vec![],
        )));
        assert!(is_pmos_model(&db, "sky130_fd_pr__pfet_01v8"));
        assert!(!is_pmos_model(&db, "sky130_fd_pr__nfet_01v8"));
        // Declared deck, so a model it does not list is not a PMOS, whatever it
        // happens to be spelled.
        assert!(!is_pmos_model(&db, "pmos_example_variant_a"));
    }

    /// The map used to be built by iterating a `HashMap`, so on a short-name
    /// collision the winner varied run to run. It must now be index-order and total.
    #[test]
    fn test_build_short_to_idx_is_index_ordered() {
        let names = ["TOP_M0", "TOP_M21", "TOP_C1", "M7", "TOP_sub_M3"];
        let m = build_short_to_idx(names.iter().copied()).expect("no collision here");
        assert_eq!(m.len(), 5);
        assert_eq!(m["M0"], 0);
        assert_eq!(m["M21"], 1);
        assert_eq!(m["C1"], 2);
        // A name with no '_' is its own short name.
        assert_eq!(m["M7"], 3);
        // Only the last segment counts, however deep the prefix.
        assert_eq!(m["M3"], 4);

        // Determinism: rebuilding the same input must give the same map, and the map
        // must not depend on any hash seed.
        for _ in 0..8 {
            let again = build_short_to_idx(names.iter().copied()).unwrap();
            let mut a: Vec<_> = m.iter().collect();
            let mut b: Vec<_> = again.iter().collect();
            a.sort();
            b.sort();
            assert_eq!(a, b);
        }
    }

    /// A collision is a hard error, not a silent arbitrary pick: `sym.json` binds by short
    /// name, so picking one would quietly constrain the wrong device.
    #[test]
    fn test_build_short_to_idx_rejects_collisions() {
        let names = ["TOP_XP_M1", "TOP_XN_M1", "TOP_M2", "TOP_A_C0", "TOP_B_C0"];
        let err = build_short_to_idx(names.iter().copied())
            .expect_err("two devices share short name M1 (and C0); must not be accepted");
        let msg = format!("{}", err);
        assert!(msg.contains("short-name collision"), "message was: {}", msg);
        // Every conflicting device is named, so the user can act on it.
        for n in ["M1", "TOP_XP_M1", "TOP_XN_M1", "C0", "TOP_A_C0", "TOP_B_C0"] {
            assert!(msg.contains(n), "conflict report omits {}: {}", n, msg);
        }
        assert!(
            msg.contains("2 short-name collision"),
            "message was: {}",
            msg
        );
        // The report itself is deterministic (BTreeMap ordering).
        for _ in 0..8 {
            let again = build_short_to_idx(names.iter().copied()).unwrap_err();
            assert_eq!(format!("{}", again), msg);
        }
    }

    #[test]
    fn test_overlap_dims_for_device() {
        let (w_p, h_p) = overlap_dims_for_device(100.0, 200.0, true, 650.0);
        assert_eq!(w_p, 1400.0);
        assert_eq!(h_p, 1500.0);
        let (w_n, h_n) = overlap_dims_for_device(100.0, 200.0, false, 650.0);
        assert_eq!(w_n, 100.0);
        assert_eq!(h_n, 200.0);
    }
}
