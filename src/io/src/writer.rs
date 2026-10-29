//! Writer - output module
//!
//! Provides:
//! - writing placement as JSON
//! - writing placement as GDS

use crate::parser::{parse_hierarchical_netlist, parse_pdk, PdkRules};
use crate::pcell::PinFile;
use applace_core::CircuitDb;
use gds21::{GdsBoundary, GdsElement, GdsLibrary, GdsPoint, GdsStruct, GdsStructRef, GdsTextElem};
use serde_json::json;
use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::path::Path;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum WriteError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("JSON error: {0}")]
    Json(#[from] serde_json::Error),
    #[error("Write error: {0}")]
    Write(String),
}

/// Writes placement as JSON (global placement)
pub fn write_global_placement_json<P: AsRef<Path>>(
    db: &CircuitDb,
    filename: P,
) -> Result<(), WriteError> {
    let mut gp_json = json!({});

    // Walk all cells and write their raw coordinates
    for idx in 0..db.num_cells() {
        if let Some(cell) = db.cell(idx as applace_core::Index) {
            let name = cell.name().to_string();
            let x_unsnapped = cell.x_unsnapped();
            let y_unsnapped = cell.y_unsnapped();

            let mut cell_data = json!([x_unsnapped, y_unsnapped]);

            // Add the flip flag when the cell is flipped
            if cell.flip_vert() || cell.flip_hori() {
                let mut flipflag = String::new();
                if cell.flip_vert() {
                    flipflag.push_str("vert");
                }
                if cell.flip_hori() {
                    if !flipflag.is_empty() {
                        flipflag.push('_');
                    }
                    flipflag.push_str("hori");
                }
                cell_data = json!([x_unsnapped, y_unsnapped, flipflag]);
            }

            gp_json[name] = cell_data;
        }
    }

    let file = File::create(filename)?;
    serde_json::to_writer_pretty(file, &gp_json)?;
    Ok(())
}

/// Writes placement as JSON (legalized placement)
pub fn write_legalized_placement_json<P: AsRef<Path>>(
    db: &CircuitDb,
    filename: P,
) -> Result<(), WriteError> {
    let mut lp_json = json!({});

    // Walk all cells and write their legalized coordinates
    for idx in 0..db.num_cells() {
        if let Some(cell) = db.cell(idx as applace_core::Index) {
            let name = cell.name().to_string();
            let x = cell.x() as f64;
            let y = cell.y() as f64;

            let mut cell_data = json!([x, y]);

            // Add the flip flag when the cell is flipped
            if cell.flip_vert() || cell.flip_hori() {
                let mut flipflag = String::new();
                if cell.flip_vert() {
                    flipflag.push_str("vert");
                }
                if cell.flip_hori() {
                    if !flipflag.is_empty() {
                        flipflag.push('_');
                    }
                    flipflag.push_str("hori");
                }
                cell_data = json!([x, y, flipflag]);
            }

            lp_json[name] = cell_data;
        }
    }

    let file = File::create(filename)?;
    serde_json::to_writer_pretty(file, &lp_json)?;
    Ok(())
}

fn load_lp_json<P: AsRef<Path>>(
    path: P,
) -> Result<HashMap<String, (i32, i32, Option<String>)>, WriteError> {
    let content = std::fs::read_to_string(path)?;
    let v: serde_json::Value = serde_json::from_str(&content)?;
    let mut map = HashMap::new();
    let obj = v
        .as_object()
        .ok_or_else(|| WriteError::Write("lp.json must be object".into()))?;
    for (name, val) in obj {
        if let Some(arr) = val.as_array() {
            if arr.len() >= 2 {
                let Some(xv) = arr[0].as_i64() else { continue };
                let Some(yv) = arr[1].as_i64() else { continue };
                let x = xv as i32;
                let y = yv as i32;
                let flip = if arr.len() >= 3 {
                    arr[2].as_str().map(|s| s.to_string())
                } else {
                    None
                };
                map.insert(name.clone(), (x, y, flip));
            }
        }
    }
    Ok(map)
}

fn derive_top_info(db: &CircuitDb) -> (String, HashSet<String>) {
    let top_pins = db
        .ports()
        .iter()
        .filter_map(|&net_idx| db.net(net_idx).map(|net| net.name().to_string()))
        .collect();
    (db.name().to_string(), top_pins)
}

fn canonical_net_name(top: &str, top_pins: &HashSet<String>, net_name: &str) -> String {
    if top_pins.contains(net_name) || net_name.starts_with(&format!("{}_", top)) {
        return net_name.to_string();
    }
    let leaf = net_name.rsplit('_').next().unwrap_or(net_name);
    format!("{}_{}", top, leaf)
}

fn locate_gds(
    pcelldir: &Path,
    cell_name: &str,
    top_name: &str,
    model: &str,
) -> Option<std::path::PathBuf> {
    let leaf = cell_name.split('_').next_back().unwrap_or(cell_name);
    let candidates = [
        pcelldir.join(format!("{}__{}.gds", top_name, leaf)),
        pcelldir.join(format!("{}.gds", model)),
        pcelldir.join(format!("{}__{}.gds", model, leaf)),
    ];
    if let Some(found) = candidates.into_iter().find(|p| p.exists()) {
        return Some(found);
    }
    // Fallback matching pcell::gds_path_for: scan for `*_{leaf}.gds` (matched
    // exactly on the `_{leaf}.gds` suffix, so M0 never matches M10; sorting and
    // taking the first keeps this deterministic).
    // Some cases name their layout files with a prefix that does not follow the
    // double-underscore pattern above.
    let suffix = format!("_{}.gds", leaf);
    let mut matches: Vec<std::path::PathBuf> = std::fs::read_dir(pcelldir)
        .ok()?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| {
            p.file_name()
                .and_then(|s| s.to_str())
                .map(|s| s.ends_with(&suffix))
                .unwrap_or(false)
        })
        .collect();
    matches.sort();
    if matches.len() > 1 {
        eprintln!(
            "warning: locate_gds: multiple *_{} matches for cell {}; using {}",
            suffix,
            cell_name,
            matches[0].display()
        );
    }
    matches.into_iter().next()
}

fn referenced_struct_names(s: &GdsStruct) -> Vec<String> {
    let mut out = Vec::new();
    for e in &s.elems {
        match e {
            GdsElement::GdsStructRef(r) => out.push(r.name.clone()),
            GdsElement::GdsArrayRef(a) => out.push(a.name.clone()),
            _ => {}
        }
    }
    out
}

/// Picks the struct that holds the device itself.
///
/// This has to agree with the measuring side
/// ([`crate::pcell::pick_device_struct_name`]): both must land on the same
/// struct, or the coordinate system of the database and pins differs from the
/// one the written SREF uses by a wrapper origin.
fn pick_source_root_struct(lib: &GdsLibrary) -> Option<String> {
    if lib.structs.is_empty() {
        return None;
    }
    crate::pcell::pick_device_struct_name(lib)
}

fn merge_cell_struct(out: &mut GdsLibrary, name: &str, gds_path: &Path) -> Result<(), WriteError> {
    let lib = GdsLibrary::load(gds_path)
        .map_err(|e| WriteError::Write(format!("gds load failed: {e}")))?;
    if lib.structs.is_empty() {
        return Err(WriteError::Write(format!(
            "empty gds: {}",
            gds_path.display()
        )));
    }

    let root_name = pick_source_root_struct(&lib).ok_or_else(|| {
        WriteError::Write(format!("cannot pick root struct: {}", gds_path.display()))
    })?;
    let mut by_name: HashMap<String, GdsStruct> = HashMap::new();
    for s in &lib.structs {
        by_name.insert(s.name.clone(), s.clone());
    }

    let mut stack = vec![root_name.clone()];
    let mut order = Vec::new();
    let mut seen = HashSet::new();
    while let Some(cur) = stack.pop() {
        if !seen.insert(cur.clone()) {
            continue;
        }
        order.push(cur.clone());
        if let Some(s) = by_name.get(&cur) {
            for child in referenced_struct_names(s) {
                if by_name.contains_key(&child) {
                    stack.push(child);
                }
            }
        }
    }

    let mut rename: HashMap<String, String> = HashMap::new();
    let mut dep_idx: usize = 0;
    for orig in &order {
        if *orig == root_name {
            rename.insert(orig.clone(), name.to_string());
        } else {
            dep_idx += 1;
            rename.insert(orig.clone(), format!("{}__d{:03}", name, dep_idx));
        }
    }

    for orig in &order {
        let Some(src) = by_name.get(orig) else {
            continue;
        };
        let mut cell = src.clone();
        if let Some(new_name) = rename.get(orig) {
            cell.name = new_name.clone();
        }
        for elem in &mut cell.elems {
            match elem {
                GdsElement::GdsStructRef(r) => {
                    if let Some(new_child) = rename.get(&r.name) {
                        r.name = new_child.clone();
                    }
                }
                GdsElement::GdsArrayRef(a) => {
                    if let Some(new_child) = rename.get(&a.name) {
                        a.name = new_child.clone();
                    }
                }
                _ => {}
            }
        }
        // Remove all text labels inside pcells; only keep top-level labels.
        cell.elems
            .retain(|e| !matches!(e, GdsElement::GdsTextElem(_)));
        out.structs.push(cell);
    }
    Ok(())
}

/// Extra net labels for LVS, on the LVS text layer of the net's own metal.
///
/// A Calibre LVS deck attaches text per conductor layer (`TEXT LAYER <n>
/// ATTACH <n> metal<k>` / `PORT LAYER TEXT <n>`), so the single annotation
/// layer APPlace normally writes is invisible to it. Enabling this adds one
/// label per net, on `base + (metal_index - 1)`, placed at the centre of a
/// rectangle this writer actually emits for that net
/// ([`collect_lvs_label_sites`]).
///
/// Off by default: `APPLACE_GDS_LVS_TEXT_BASE=<first metal text layer>` turns
/// it on. The base is a per-foundry convention and is therefore never
/// hardcoded here — the caller supplies it.
fn lvs_text_base() -> Option<i16> {
    std::env::var("APPLACE_GDS_LVS_TEXT_BASE")
        .ok()
        .and_then(|v| v.trim().parse::<i16>().ok())
        .filter(|v| *v > 0)
}

/// Which nets [`add_lvs_net_texts`] labels (`APPLACE_GDS_LVS_TEXT_SCOPE`).
///
/// * `ports` (**default**) — the top-level ports only, but now every net a
///   port owns is searched for a label site, so a port whose *first* pin is a
///   metal-less bulk pin no longer falls back to a point on no conductor.
///   Guard-ring / tap / stripe nets are labelled through this route too,
///   because they carry a port's own net name (`VDD` / `GND`): one net, one
///   label.
/// * `all` — additionally labels internal nets, which makes the LVS report and
///   the PEX net list readable. **Cost: on a deck where every text layer that
///   names a net is also declared a port text layer, each internal label becomes
///   a layout port**, and LVS then reports all of them as unmatched layout ports
///   even though the real ports match. Hence it is not the default.
///
/// Unknown value = hard error (never a silent fallback).
#[derive(Clone, Copy, PartialEq, Eq)]
enum LvsTextScope {
    PortsOnly,
    AllNets,
}

fn lvs_text_scope() -> Result<LvsTextScope, WriteError> {
    match std::env::var("APPLACE_GDS_LVS_TEXT_SCOPE") {
        Err(_) => Ok(LvsTextScope::PortsOnly),
        Ok(v) => match v.trim() {
            "" | "ports" => Ok(LvsTextScope::PortsOnly),
            "all" => Ok(LvsTextScope::AllNets),
            other => Err(WriteError::Write(format!(
                "APPLACE_GDS_LVS_TEXT_SCOPE={other} is not one of: ports, all"
            ))),
        },
    }
}

/// Provenance of a candidate label site, and the preference order between them.
///
/// Device pin metal comes first: it is the conductor an LVS run is most likely to
/// accept as the net's own. The guard-ring / tap / stripe rectangles are the
/// fallback, for nets that own no device pin metal. Lower = preferred.
const LVS_SITE_PIN: u8 = 0;
const LVS_SITE_DERIVED: u8 = 1;

/// One rectangle a net owns in the emitted layout, as a candidate label site.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct LvsLabelSite {
    rank: u8,
    metal_idx: i32,
    xl: i32,
    yl: i32,
    xh: i32,
    yh: i32,
}

impl LvsLabelSite {
    /// Deterministic order: preferred source first, then the widest rectangle
    /// (most room around the label point), then a total order on the geometry
    /// itself. No hash iteration, no input-file order.
    fn order_key(&self) -> (u8, i64, i32, i32, i32, i32, i32) {
        let area = (self.xh - self.xl) as i64 * (self.yh - self.yl) as i64;
        (
            self.rank,
            -area,
            self.metal_idx,
            self.xl,
            self.yl,
            self.xh,
            self.yh,
        )
    }

    fn centre(&self) -> (i32, i32) {
        (
            self.xl + (self.xh - self.xl) / 2,
            self.yl + (self.yh - self.yl) / 2,
        )
    }
}

/// GDS layer → metal index (M1 = 0), from the deck's own `layer_map`.
///
/// The text layers a Calibre LVS deck attaches per metal are consecutive from
/// the first one (`base + metal_index`); which number `base` is remains the
/// foundry's business and stays in the environment.
fn metal_index_map(pdk: &PdkRules) -> std::collections::BTreeMap<i32, i32> {
    let mut out = std::collections::BTreeMap::new();
    for (name, layer) in &pdk.layer_map {
        let Some(k) = name.strip_prefix('M').and_then(|s| s.parse::<i32>().ok()) else {
            continue;
        };
        if k >= 1 {
            out.insert(*layer as i32, k - 1);
        }
    }
    out
}

/// A closed axis-aligned rectangle, or `None` for anything else.
fn derived_polygon_rect(
    points: &[applace_core::Point<applace_core::Loc>],
) -> Option<(i32, i32, i32, i32)> {
    if points.len() < 4 || points.len() > 5 {
        return None;
    }
    let xl = points.iter().map(|p| p.x).min()?;
    let yl = points.iter().map(|p| p.y).min()?;
    let xh = points.iter().map(|p| p.x).max()?;
    let yh = points.iter().map(|p| p.y).max()?;
    if xh <= xl || yh <= yl {
        return None;
    }
    if points
        .iter()
        .any(|p| (p.x != xl && p.x != xh) || (p.y != yl && p.y != yh))
    {
        return None;
    }
    Some((xl, yl, xh, yh))
}

/// Every rectangle the emitted layout gives each net, keyed by the **DB net
/// name** (short form — `VDD`, `net07` — which is what the source netlist and
/// therefore the LVS report use).
///
/// Two sources, both of them geometry this writer really emits:
/// device pin metal (`pin.json` shapes translated to the placed
/// origin), and the guard-ring / tap / power-stripe polygons.
fn collect_lvs_label_sites(
    db: &CircuitDb,
    pin_file: &PinFile,
    lp: &HashMap<String, (i32, i32, Option<String>)>,
    pdk: &PdkRules,
    top_name: &str,
) -> std::collections::BTreeMap<String, Vec<LvsLabelSite>> {
    let metals = metal_index_map(pdk);
    let mut out: std::collections::BTreeMap<String, Vec<LvsLabelSite>> =
        std::collections::BTreeMap::new();

    // pin.json spells internal nets with the top-cell prefix (`TOP_net07`)
    // while the DB (and the source netlist) use the leaf name. Resolve both.
    let mut alias: HashMap<String, String> = HashMap::new();
    for idx in 0..db.num_nets() {
        let Some(net) = db.net(idx as applace_core::Index) else {
            continue;
        };
        let name = net.name().to_string();
        alias.insert(format!("{top_name}_{name}"), name.clone());
        alias.insert(name.clone(), name);
    }

    // (1) device pin metal.
    for p in &pin_file.pins {
        if p.cell_name.is_empty() {
            continue;
        }
        let Some((cx, cy, _)) = lp.get(&p.cell_name).cloned() else {
            continue;
        };
        for raw in &p.nets {
            let Some(net) = alias.get(raw.as_str()) else {
                continue;
            };
            for s in &p.shapes {
                let Some(&metal_idx) = metals.get(&s.layer) else {
                    continue;
                };
                if s.shape.xh <= s.shape.xl || s.shape.yh <= s.shape.yl {
                    continue;
                }
                out.entry(net.clone()).or_default().push(LvsLabelSite {
                    rank: LVS_SITE_PIN,
                    metal_idx,
                    xl: cx + s.shape.xl,
                    yl: cy + s.shape.yl,
                    xh: cx + s.shape.xh,
                    yh: cy + s.shape.yh,
                });
            }
        }
    }

    // (2) guard rings / substrate taps / power stripes.
    for shape in db.derived_shapes() {
        let Some(net_idx) = shape.net_idx else {
            continue;
        };
        let Some(net) = db.net(net_idx) else {
            continue;
        };
        let name = net.name().to_string();
        for poly in &shape.polygons {
            let Some(&metal_idx) = metals.get(&{ poly.layer }) else {
                continue;
            };
            let Some((xl, yl, xh, yh)) = derived_polygon_rect(&poly.points) else {
                continue;
            };
            out.entry(name.clone()).or_default().push(LvsLabelSite {
                rank: LVS_SITE_DERIVED,
                metal_idx,
                xl,
                yl,
                xh,
                yh,
            });
        }
    }

    out
}

/// Write one LVS label per net, on the deck's per-metal text layer.
///
/// **Exactly one label per net, never two.** Two texts with the same string on
/// two electrically separate pieces of layout is how a deck is talked into
/// joining nets that the layout does not actually connect; a single label keeps
/// the judge (Calibre) free to report the break.
fn add_lvs_net_texts(
    top: &mut GdsStruct,
    db: &CircuitDb,
    pin_file: &PinFile,
    lp: &HashMap<String, (i32, i32, Option<String>)>,
    pdk: &PdkRules,
    top_name: &str,
    top_pins: &HashSet<String>,
) -> Result<(), WriteError> {
    let Some(base) = lvs_text_base() else {
        return Ok(());
    };
    let scope = lvs_text_scope()?;
    let sites = collect_lvs_label_sites(db, pin_file, lp, pdk, top_name);
    if scope == LvsTextScope::PortsOnly {
        // Guard rings / taps / stripes are labelled through the port whose net
        // they carry (`VDD`, `GND`, …). Say so out loud when one does not — that
        // ring would ship nameless, and silently.
        let mut unnamed: Vec<&str> = db
            .derived_shapes()
            .iter()
            .filter_map(|s| s.net_idx.and_then(|i| db.net(i)).map(|n| n.name()))
            .filter(|n| !top_pins.contains(*n))
            .collect();
        unnamed.sort_unstable();
        unnamed.dedup();
        if !unnamed.is_empty() {
            eprintln!(
                "warning: lvs labels: derived shape net(s) {} are not top-level ports, \
                 so they get no label under APPLACE_GDS_LVS_TEXT_SCOPE=ports",
                unnamed.join(", ")
            );
        }
    }
    for (net, cands) in &sites {
        if scope == LvsTextScope::PortsOnly && !top_pins.contains(net) {
            continue;
        }
        let Some(best) = cands.iter().min_by_key(|s| s.order_key()) else {
            continue;
        };
        let (x, y) = best.centre();
        top.elems.push(GdsElement::GdsTextElem(GdsTextElem {
            string: net.clone(),
            layer: base + best.metal_idx as i16,
            texttype: 0,
            xy: GdsPoint { x, y },
            ..Default::default()
        }));
    }
    Ok(())
}

fn add_top_pin_texts(
    top: &mut GdsStruct,
    pin_file: &PinFile,
    lp: &HashMap<String, (i32, i32, Option<String>)>,
    top_name: &str,
    top_pins: &HashSet<String>,
    text_layer: i16,
) {
    let mut used = HashSet::new();
    for p in &pin_file.pins {
        for net in &p.nets {
            let net = canonical_net_name(top_name, top_pins, net);
            if !top_pins.contains(&net) || used.contains(&net) {
                continue;
            }
            let cell = p.cell_name.clone();
            if cell.is_empty() {
                continue;
            }
            let Some((cx, cy, _)) = lp.get(&cell).cloned() else {
                continue;
            };
            let x = cx + p.offset_x;
            let y = cy + p.offset_y;
            top.elems.push(GdsElement::GdsTextElem(GdsTextElem {
                string: net.clone(),
                layer: text_layer,
                texttype: 0,
                xy: GdsPoint { x, y },
                ..Default::default()
            }));
            used.insert(net);
        }
    }
}

fn append_derived_shape_srefs(db: &CircuitDb, lib: &mut GdsLibrary, top: &mut GdsStruct) {
    let mut emitted = HashSet::new();
    for shape in db.derived_shapes() {
        if shape.polygons.is_empty() || !shape.write_as_sref {
            continue;
        }
        let mut struct_name = shape.name.clone();
        if struct_name.is_empty() {
            struct_name = format!("DERIVED_{}", emitted.len());
        }
        if !emitted.insert(struct_name.clone()) {
            continue;
        }
        let mut derived = GdsStruct {
            name: struct_name.clone(),
            ..Default::default()
        };
        for poly in &shape.polygons {
            derived.elems.push(GdsElement::GdsBoundary(GdsBoundary {
                layer: poly.layer as i16,
                datatype: poly.datatype as i16,
                xy: poly
                    .points
                    .iter()
                    .map(|p| GdsPoint { x: p.x, y: p.y })
                    .collect(),
                ..Default::default()
            }));
        }
        lib.structs.push(derived);
        top.elems.push(GdsElement::GdsStructRef(GdsStructRef {
            name: struct_name,
            xy: GdsPoint { x: 0, y: 0 },
            ..Default::default()
        }));
    }
}

/// Write placement GDS by instantiating pcells at lp.json positions and adding top pin texts.
///
/// Coordinate semantics:
/// - Uses `cell.x()/cell.y()` (integer legal coordinates) from in-memory `CircuitDb`.
/// - No hidden rounding is performed in this writer.
pub fn write_placement_gds_from_db<P: AsRef<Path>>(
    db: &CircuitDb,
    pcelldir: P,
    pin_file: &PinFile,
    pdk: &PdkRules,
    out_gds: P,
) -> Result<(), WriteError> {
    let (mut lib, mut top, place_map, top_name, top_pins) =
        build_placement_gds(db, pcelldir.as_ref(), pin_file, pdk)?;
    add_lvs_net_texts(
        &mut top, db, pin_file, &place_map, pdk, &top_name, &top_pins,
    )?;
    lib.structs.push(top);
    lib.save(out_gds)
        .map_err(|e| WriteError::Write(format!("gds save failed: {e}")))?;
    Ok(())
}

/// Builds the placement layout (device SREFs + top-level pin text + derived
/// shapes) and returns (lib, top), with top not yet pushed into lib.
#[allow(clippy::type_complexity)]
fn build_placement_gds(
    db: &CircuitDb,
    pcelldir: &Path,
    pin_file: &PinFile,
    pdk: &PdkRules,
) -> Result<
    (
        GdsLibrary,
        GdsStruct,
        HashMap<String, (i32, i32, Option<String>)>,
        String,
        HashSet<String>,
    ),
    WriteError,
> {
    let (top_name, top_pins) = derive_top_info(db);
    let text_layer = pdk.textlayers.first().copied().unwrap_or(127) as i16;

    let mut lib = GdsLibrary {
        name: "TOP".into(),
        ..Default::default()
    };
    let mut top = GdsStruct {
        name: "TOP".into(),
        ..Default::default()
    };

    let mut added = HashSet::new();
    let mut place_map: HashMap<String, (i32, i32, Option<String>)> = HashMap::new();
    for idx in 0..db.num_cells() {
        let Some(cell) = db.cell(idx as i32) else {
            continue;
        };
        let cell_name = cell.name().to_string();
        let x = cell.x();
        let y = cell.y();
        place_map.insert(cell_name.clone(), (x, y, None));

        let model = cell.model().to_string();
        let Some(gds_path) = locate_gds(pcelldir, cell.name(), &top_name, &model) else {
            continue;
        };

        if !added.contains(&cell_name) {
            merge_cell_struct(&mut lib, &cell_name, &gds_path)?;
            added.insert(cell_name.clone());
        }

        top.elems.push(GdsElement::GdsStructRef(GdsStructRef {
            name: cell_name,
            xy: GdsPoint { x, y },
            ..Default::default()
        }));
    }

    add_top_pin_texts(
        &mut top, pin_file, &place_map, &top_name, &top_pins, text_layer,
    );
    append_derived_shape_srefs(db, &mut lib, &mut top);
    Ok((lib, top, place_map, top_name, top_pins))
}

/// Write placement GDS by reading netlist/lp/pin files, then delegating to DB-based writer.
pub fn write_placement_gds_from_files<P: AsRef<Path>>(
    netlist: P,
    lp_json: P,
    pcelldir: P,
    pin_json: P,
    pdk_json: P,
    out_gds: P,
) -> Result<(), WriteError> {
    let mut db = CircuitDb::new();
    parse_hierarchical_netlist(netlist, &mut db).map_err(|e| WriteError::Write(e.to_string()))?;
    let pdk: PdkRules = parse_pdk(&pdk_json).map_err(|e| WriteError::Write(e.to_string()))?;
    let lp = load_lp_json(lp_json)?;
    let pin_file: PinFile = serde_json::from_str(&std::fs::read_to_string(pin_json)?)?;

    let (top_name, top_pins) = derive_top_info(&db);
    for idx in 0..db.num_cells() {
        let Some(cell) = db.cell_mut(idx as i32) else {
            continue;
        };
        if let Some((x, y, _flip)) = lp.get(cell.name()).cloned() {
            cell.set_coord(x, y);
            cell.set_coord_unsnapped(x as f64, y as f64);
            continue;
        }
        let leaf = cell.name().rsplit('_').next().unwrap_or(cell.name());
        let canon = format!("{}_{}", top_name, leaf);
        if let Some((x, y, _flip)) = lp.get(&canon).cloned() {
            cell.set_coord(x, y);
            cell.set_coord_unsnapped(x as f64, y as f64);
        }
    }
    let _ = top_pins; // keep derive_top_info behavior for canonical fallback only
    write_placement_gds_from_db(&db, pcelldir, &pin_file, &pdk, out_gds)
}

// ---------------------------------------------------------------------------
// Form two: outline-level placement GDS
//
// Form one writes a real layout: it instantiates the vendor PCell GDS of every device
// and uses the PDK layermap. Form two has neither, on purpose -- it publishes device
// outlines and nothing else. So it gets its own writer, which emits one rectangle per
// device and no vendor geometry whatsoever.
// ---------------------------------------------------------------------------

/// Device outline rectangle layer for [`write_abstract_placement_gds`].
pub const ABSTRACT_GDS_OUTLINE: (i16, i16) = (1, 0);
/// Device-name text layer/texttype.
pub const ABSTRACT_GDS_LABEL: (i16, i16) = (1, 1);
/// Pin-anchor marker layer (the abstract pin model puts every pin at the device centre).
pub const ABSTRACT_GDS_PIN: (i16, i16) = (2, 0);
/// Placement-region outline layer (emitted only when a region is supplied).
pub const ABSTRACT_GDS_REGION: (i16, i16) = (3, 0);
/// Guard-ring reservation band around each PMOS — the room the placer kept free
/// (`pmosHardExpandForGr`), drawn as the band itself, not as a filled box.
pub const ABSTRACT_GDS_GR: (i16, i16) = (4, 0);
/// PMOS marker: a copy of the device outline, on its own layer.
pub const ABSTRACT_GDS_PMOS: (i16, i16) = (5, 0);
/// `PMOS` text, one per PMOS device.
pub const ABSTRACT_GDS_PMOS_LABEL: (i16, i16) = (5, 1);

/// One device of an abstract (form two) placement: the legalized lower-left corner
/// plus the outline size published in `bbox.json`. Coordinates are DBU = nm.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AbstractPlacedDevice {
    pub name: String,
    pub x: i32,
    pub y: i32,
    pub w: i32,
    pub h: i32,
    /// From `bbox.json`'s `isPmos`. Drives the guard-ring reservation the placer
    /// reserved for this device, and the PMOS marker layers.
    pub is_pmos: bool,
}

/// Closed rectangle as a GDS boundary (5 points, first repeated last).
fn abstract_rect(layer: (i16, i16), xl: i32, yl: i32, xh: i32, yh: i32) -> GdsElement {
    GdsElement::GdsBoundary(GdsBoundary {
        layer: layer.0,
        datatype: layer.1,
        xy: vec![
            GdsPoint { x: xl, y: yl },
            GdsPoint { x: xh, y: yl },
            GdsPoint { x: xh, y: yh },
            GdsPoint { x: xl, y: yh },
            GdsPoint { x: xl, y: yl },
        ],
        ..Default::default()
    })
}

/// Half-edge of the pin-anchor marker square, scaled off the device so it stays visible
/// on a 1 µm device and does not swamp one. Integer math only -- deterministic.
fn abstract_anchor_half(w: i32, h: i32) -> i32 {
    (w.min(h) / 20).clamp(20, 500)
}

/// The four rectangles making up the band between `inner` and `inner` grown by `e`.
///
/// Drawn as the band rather than as one grown box on purpose: the band *is* the
/// reserved area, so what a viewer sees is what the placer actually kept free. A filled
/// outer box would cover the device it belongs to and would read as if the device were
/// that big.
fn abstract_band(layer: (i16, i16), xl: i32, yl: i32, xh: i32, yh: i32, e: i32) -> Vec<GdsElement> {
    vec![
        abstract_rect(layer, xl - e, yl - e, xh + e, yl), // bottom
        abstract_rect(layer, xl - e, yh, xh + e, yh + e), // top
        abstract_rect(layer, xl - e, yl, xl, yh),         // left
        abstract_rect(layer, xh, yl, xh + e, yh),         // right
    ]
}

/// Write an outline-level GDS for an abstract (form two) placement.
///
/// What is in the file, and nothing else:
///
/// | layer/datatype | content |
/// |---|---|
/// | 1/0 | one rectangle per device: legalized position + published outline size |
/// | 1/1 | device-name text at the device centre |
/// | 2/0 | pin anchor -- a small square at the device centre |
/// | 3/0 | the placement region from `bbox.json.boundary` (omitted when `region` is `None`) |
/// | 4/0 | guard-ring reservation: the band of width `gr_expand` around each PMOS |
/// | 5/0 | PMOS marker: a copy of the outline rectangle, PMOS devices only |
/// | 5/1 | the text `PMOS`, one per PMOS device |
///
/// Layers 4 and 5 exist because of a maintainer review of the released demo: opened in
/// a viewer, the layout showed boxes and nothing else, so "which of these is a PMOS,
/// and how much room did you keep around it?" had no answer on the screen — and a
/// placement whose PMOS spacing looks unexplained looks wrong. Both are the same two
/// facts form two already publishes (`bbox.json`'s `isPmos`, the `pmosHardExpandForGr`
/// parameter), drawn instead of implied.
///
/// The layer numbers are **demo-local** and are deliberately not any vendor layermap:
/// the abstract form ships no PDK, so there is no layer table to be faithful to, and
/// reusing real numbers would publish a mapping we are not allowed to publish.
///
/// The pin anchor is one marker *per device*, not per pin: form two states that a pin
/// may sit anywhere on its device and seeds every pin at the device centre, so all of a
/// device's anchors coincide. Which pins exist and what nets they are on stays in
/// `pin.json`; drawing N coincident squares would add nothing.
///
/// `gr_expand` is the per-side reservation in DBU (`pmosHardExpandForGr`). Zero or less
/// emits no layer 4 at all — there is then nothing reserved to draw.
///
/// Units are the gds21 default -- DBU = 1 nm, user unit = 1 µm -- matching the rest of
/// the flow, where all placer coordinates are DBU = nm. Element order follows `devices`,
/// so the output is deterministic for a given input order. BGNLIB/BGNSTR timestamps are
/// wall-clock (gds21's default, same as every other writer here), so two runs differ in
/// those header bytes and nowhere else.
pub fn write_abstract_placement_gds<P: AsRef<Path>>(
    devices: &[AbstractPlacedDevice],
    region: Option<(i32, i32, i32, i32)>,
    gr_expand: i32,
    out_gds: P,
) -> Result<(), WriteError> {
    let mut top = GdsStruct {
        name: "ABSTRACT_PLACEMENT".into(),
        ..Default::default()
    };

    if let Some((xl, yl, xh, yh)) = region {
        top.elems
            .push(abstract_rect(ABSTRACT_GDS_REGION, xl, yl, xh, yh));
    }

    for d in devices {
        let (xh, yh) = (d.x + d.w, d.y + d.h);
        if d.is_pmos && gr_expand > 0 {
            top.elems
                .extend(abstract_band(ABSTRACT_GDS_GR, d.x, d.y, xh, yh, gr_expand));
        }
        top.elems
            .push(abstract_rect(ABSTRACT_GDS_OUTLINE, d.x, d.y, xh, yh));
        let cx = d.x + d.w / 2;
        let cy = d.y + d.h / 2;
        let half = abstract_anchor_half(d.w, d.h);
        top.elems.push(abstract_rect(
            ABSTRACT_GDS_PIN,
            cx - half,
            cy - half,
            cx + half,
            cy + half,
        ));
        top.elems.push(GdsElement::GdsTextElem(GdsTextElem {
            string: d.name.clone(),
            layer: ABSTRACT_GDS_LABEL.0,
            texttype: ABSTRACT_GDS_LABEL.1,
            xy: GdsPoint { x: cx, y: cy },
            ..Default::default()
        }));
        if d.is_pmos {
            top.elems
                .push(abstract_rect(ABSTRACT_GDS_PMOS, d.x, d.y, xh, yh));
            // Below the name label, so the two do not sit on top of each other.
            top.elems.push(GdsElement::GdsTextElem(GdsTextElem {
                string: "PMOS".into(),
                layer: ABSTRACT_GDS_PMOS_LABEL.0,
                texttype: ABSTRACT_GDS_PMOS_LABEL.1,
                xy: GdsPoint {
                    x: cx,
                    y: cy - d.h / 4,
                },
                ..Default::default()
            }));
        }
    }

    let lib = GdsLibrary {
        name: "ABSTRACT_PLACEMENT".into(),
        structs: vec![top],
        ..Default::default()
    };
    lib.save(out_gds)
        .map_err(|e| WriteError::Write(format!("gds save failed: {e}")))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use applace_core::CircuitDb;
    use gds21::GdsLibrary;

    #[test]
    fn test_write_global_placement_json() {
        let mut db = CircuitDb::new();

        // Create the test cell
        let cell_idx = db.allocate_cell("test_cell".to_string());
        if let Some(cell) = db.cell_mut(cell_idx) {
            cell.set_coord_unsnapped(10.5, 20.5);
        }

        // Test the JSON write (into a temporary file)
        let temp_file = std::env::temp_dir().join("test_gp.json");
        write_global_placement_json(&db, &temp_file).unwrap();

        // Verify the file exists
        assert!(temp_file.exists());

        // Clean up
        let _ = std::fs::remove_file(&temp_file);
    }

    #[test]
    fn test_write_legalized_placement_json() {
        let mut db = CircuitDb::new();

        // Create the test cell
        let cell_idx = db.allocate_cell("test_cell".to_string());
        if let Some(cell) = db.cell_mut(cell_idx) {
            cell.set_coord(100, 200);
        }

        // Test the JSON write (into a temporary file)
        let temp_file = std::env::temp_dir().join("test_lp.json");
        write_legalized_placement_json(&db, &temp_file).unwrap();

        // Verify the file exists
        assert!(temp_file.exists());

        // Clean up
        let _ = std::fs::remove_file(&temp_file);
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_write_placement_gds_from_db_uses_cell_name_and_db_coords() {
        // The example design's layout is not committed; see docs/INPUT_FORMAT.md.
        if !std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../benchmark/ota2/custom/OTA_2__M0.gds")
            .exists()
        {
            eprintln!("[skip] example design layout is not present (docs/INPUT_FORMAT.md)");
            return;
        }
        let mut db = CircuitDb::new();
        let cidx = db.allocate_cell("custom_instance_name".to_string());
        if let Some(cell) = db.cell_mut(cidx) {
            cell.set_model("nmos_example".to_string());
            cell.set_coord(123, 456);
            cell.set_coord_unsnapped(123.0, 456.0);
        }

        let tmpdir = std::env::temp_dir().join("test_gds_from_db_noncanon");
        let _ = std::fs::remove_dir_all(&tmpdir);
        std::fs::create_dir_all(&tmpdir).unwrap();

        let project_root = {
            let io_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
            io_dir.parent().unwrap().parent().unwrap().to_path_buf()
        };
        let fixture_gds = project_root
            .join("benchmark")
            .join("ota2")
            .join("custom")
            .join("OTA_2__M0.gds");
        std::fs::copy(&fixture_gds, tmpdir.join("nmos_example.gds")).unwrap();

        let pdk_path = { project_root.join("pdk").join("example_lef.json") };
        let pdk = parse_pdk(&pdk_path).unwrap();
        let pin_file = PinFile { pins: vec![] };
        let out = tmpdir.join("out.gds");

        write_placement_gds_from_db(&db, &tmpdir, &pin_file, &pdk, &out).unwrap();

        let lib = GdsLibrary::load(&out).unwrap();
        let top = lib
            .structs
            .iter()
            .find(|s| s.name == "TOP")
            .expect("TOP struct missing");
        let sref = top
            .elems
            .iter()
            .find_map(|e| match e {
                GdsElement::GdsStructRef(r) if r.name == "custom_instance_name" => Some(r),
                _ => None,
            })
            .expect("missing sref for custom_instance_name");
        assert_eq!(sref.xy.x, 123);
        assert_eq!(sref.xy.y, 456);

        let _ = std::fs::remove_dir_all(&tmpdir);
    }

    #[test]
    fn test_write_placement_gds_from_db_writes_derived_shapes_as_srefs() {
        let mut db = CircuitDb::new();
        db.add_derived_shape(applace_core::DerivedShape::new(
            "TEST_DERIVED_SHAPE".to_string(),
            applace_core::DerivedShapeKind::PowerStripe,
            None,
            None,
            None,
            vec![applace_core::DerivedPolygon::rect(
                36,
                0,
                applace_core::Box::new(0, -100, 1_000, 0),
            )],
        ));

        let tmpdir = std::env::temp_dir().join("test_derived_srefs");
        let _ = std::fs::remove_dir_all(&tmpdir);
        std::fs::create_dir_all(&tmpdir).unwrap();

        let project_root = {
            let io_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
            io_dir.parent().unwrap().parent().unwrap().to_path_buf()
        };
        let pdk_path = project_root.join("pdk").join("example_lef.json");
        let pdk = parse_pdk(&pdk_path).unwrap();
        let pin_file = PinFile { pins: vec![] };
        let out = tmpdir.join("out.gds");

        write_placement_gds_from_db(&db, &tmpdir, &pin_file, &pdk, &out).unwrap();

        let lib = GdsLibrary::load(&out).unwrap();
        let top = lib.structs.iter().find(|s| s.name == "TOP").unwrap();
        assert!(top.elems.iter().any(|e| matches!(
            e,
            GdsElement::GdsStructRef(r) if r.name == "TEST_DERIVED_SHAPE"
        )));
        assert!(lib.structs.iter().any(|s| {
            s.name == "TEST_DERIVED_SHAPE"
                && s.elems
                    .iter()
                    .any(|e| matches!(e, GdsElement::GdsBoundary(_)))
        }));
        assert!(!top
            .elems
            .iter()
            .any(|e| matches!(e, GdsElement::GdsBoundary(_))));

        let _ = std::fs::remove_dir_all(&tmpdir);
    }

    #[test]
    fn test_write_placement_gds_from_db_uses_label_carrying_pcell_struct() {
        let mut db = CircuitDb::new();
        let cidx = db.allocate_cell("OTA_2_M28".to_string());
        if let Some(cell) = db.cell_mut(cidx) {
            cell.set_model("pmos_example_variant_a".to_string());
            cell.set_coord(1000, 2000);
            cell.set_coord_unsnapped(1000.0, 2000.0);
        }

        let tmpdir = std::env::temp_dir().join("test_gds_writer_label_struct_root");
        let _ = std::fs::remove_dir_all(&tmpdir);
        std::fs::create_dir_all(&tmpdir).unwrap();

        let child = GdsStruct {
            name: "pmos_example_variant_a_device".to_string(),
            elems: vec![
                GdsElement::GdsBoundary(GdsBoundary {
                    layer: 31,
                    datatype: 0,
                    xy: vec![
                        GdsPoint { x: 0, y: 0 },
                        GdsPoint { x: 780, y: 0 },
                        GdsPoint { x: 780, y: 1690 },
                        GdsPoint { x: 0, y: 1690 },
                        GdsPoint { x: 0, y: 0 },
                    ],
                    ..Default::default()
                }),
                GdsElement::GdsTextElem(GdsTextElem {
                    string: "PCAS".to_string(),
                    layer: 127,
                    texttype: 0,
                    xy: GdsPoint { x: 390, y: 845 },
                    ..Default::default()
                }),
            ],
            ..Default::default()
        };
        let wrapper = GdsStruct {
            name: "M28".to_string(),
            elems: vec![GdsElement::GdsStructRef(GdsStructRef {
                name: "pmos_example_variant_a_device".to_string(),
                xy: GdsPoint { x: 660, y: 440 },
                ..Default::default()
            })],
            ..Default::default()
        };
        GdsLibrary {
            name: "PCell".to_string(),
            structs: vec![child, wrapper],
            ..Default::default()
        }
        .save(tmpdir.join("pmos_example_variant_a.gds"))
        .unwrap();

        let project_root = {
            let io_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
            io_dir.parent().unwrap().parent().unwrap().to_path_buf()
        };
        let pdk = parse_pdk(project_root.join("pdk").join("example_lef.json")).unwrap();
        let out = tmpdir.join("out.gds");

        write_placement_gds_from_db(&db, &tmpdir, &PinFile { pins: vec![] }, &pdk, &out).unwrap();

        let lib = GdsLibrary::load(&out).unwrap();
        let merged = lib
            .structs
            .iter()
            .find(|s| s.name == "OTA_2_M28")
            .expect("merged cell missing");
        assert!(merged
            .elems
            .iter()
            .any(|e| matches!(e, GdsElement::GdsBoundary(_))));
        assert!(!merged
            .elems
            .iter()
            .any(|e| matches!(e, GdsElement::GdsStructRef(_))));

        let _ = std::fs::remove_dir_all(&tmpdir);
    }

    #[test]
    fn abstract_placement_gds_is_outline_only() {
        let devices = vec![
            AbstractPlacedDevice {
                name: "A".into(),
                x: 100,
                y: 200,
                w: 1000,
                h: 2000,
                is_pmos: false,
            },
            AbstractPlacedDevice {
                name: "B".into(),
                x: 5000,
                y: 0,
                w: 400,
                h: 400,
                is_pmos: false,
            },
        ];
        let out = std::env::temp_dir().join("abstract_placement_gds_test.gds");
        write_abstract_placement_gds(&devices, Some((0, 0, 10_000, 10_000)), 650, &out).unwrap();

        let lib = GdsLibrary::load(&out).unwrap();
        assert_eq!(lib.structs.len(), 1, "one flat top struct, no vendor cells");
        let top = &lib.structs[0];
        assert_eq!(top.name, "ABSTRACT_PLACEMENT");
        // No structure references: nothing is instantiated from a PCell here.
        assert!(!top
            .elems
            .iter()
            .any(|e| matches!(e, GdsElement::GdsStructRef(_))));

        let outlines: Vec<&GdsBoundary> = top
            .elems
            .iter()
            .filter_map(|e| match e {
                GdsElement::GdsBoundary(b) if (b.layer, b.datatype) == ABSTRACT_GDS_OUTLINE => {
                    Some(b)
                }
                _ => None,
            })
            .collect();
        assert_eq!(outlines.len(), devices.len());
        // Rectangle 0 is device A at its stated position and size.
        assert_eq!(outlines[0].xy[0], GdsPoint { x: 100, y: 200 });
        assert_eq!(outlines[0].xy[2], GdsPoint { x: 1100, y: 2200 });

        let regions = top
            .elems
            .iter()
            .filter(|e| {
                matches!(e, GdsElement::GdsBoundary(b)
                    if (b.layer, b.datatype) == ABSTRACT_GDS_REGION)
            })
            .count();
        assert_eq!(regions, 1);

        let anchors: Vec<&GdsBoundary> = top
            .elems
            .iter()
            .filter_map(|e| match e {
                GdsElement::GdsBoundary(b) if (b.layer, b.datatype) == ABSTRACT_GDS_PIN => Some(b),
                _ => None,
            })
            .collect();
        assert_eq!(anchors.len(), devices.len(), "one anchor per device");
        // Anchor is centred on the device; half-edge = min(w,h)/20 clamped to [20, 500].
        assert_eq!(
            anchors[0].xy[0],
            GdsPoint {
                x: 600 - 50,
                y: 1200 - 50
            }
        );
        assert_eq!(
            anchors[1].xy[0],
            GdsPoint {
                x: 5200 - 20,
                y: 200 - 20
            }
        );

        let labels: Vec<String> = top
            .elems
            .iter()
            .filter_map(|e| match e {
                GdsElement::GdsTextElem(t) if (t.layer, t.texttype) == ABSTRACT_GDS_LABEL => {
                    Some(t.string.clone())
                }
                _ => None,
            })
            .collect();
        assert_eq!(labels, vec!["A".to_string(), "B".to_string()]);

        // No PMOS here, so neither PMOS layer nor the reservation band appears at all.
        for lt in [ABSTRACT_GDS_GR, ABSTRACT_GDS_PMOS] {
            assert!(
                !top.elems
                    .iter()
                    .any(|e| matches!(e, GdsElement::GdsBoundary(b)
                    if (b.layer, b.datatype) == lt)),
                "layer {lt:?} must be absent when no device is a PMOS"
            );
        }

        // DBU = 1 nm, user unit = 1 um -- the unit the placer works in.
        assert_eq!(lib.units.db_unit(), 1e-9);

        let _ = std::fs::remove_file(&out);
    }

    /// The self-explaining layers a maintainer asked for: which device is a PMOS, and
    /// how much room was kept around it.
    #[test]
    fn abstract_placement_gds_marks_pmos_and_its_reservation() {
        let devices = vec![
            AbstractPlacedDevice {
                name: "P".into(),
                x: 1000,
                y: 1000,
                w: 400,
                h: 800,
                is_pmos: true,
            },
            AbstractPlacedDevice {
                name: "N".into(),
                x: 9000,
                y: 0,
                w: 400,
                h: 800,
                is_pmos: false,
            },
        ];
        let out = std::env::temp_dir().join("abstract_placement_gds_pmos.gds");
        write_abstract_placement_gds(&devices, None, 650, &out).unwrap();
        let top = &GdsLibrary::load(&out).unwrap().structs[0];

        let on = |lt: (i16, i16)| -> Vec<GdsBoundary> {
            top.elems
                .iter()
                .filter_map(|e| match e {
                    GdsElement::GdsBoundary(b) if (b.layer, b.datatype) == lt => Some(b.clone()),
                    _ => None,
                })
                .collect()
        };

        // The PMOS marker is the outline of the one PMOS, and only of that device.
        let marks = on(ABSTRACT_GDS_PMOS);
        assert_eq!(marks.len(), 1);
        assert_eq!(marks[0].xy[0], GdsPoint { x: 1000, y: 1000 });
        assert_eq!(marks[0].xy[2], GdsPoint { x: 1400, y: 1800 });

        // The reservation is the band, four rectangles, none of them covering the
        // device: their union is exactly (outline grown by 650) minus the outline.
        let band = on(ABSTRACT_GDS_GR);
        assert_eq!(band.len(), 4, "one band = four rectangles");
        let area: i64 = band
            .iter()
            .map(|b| {
                let (x0, y0) = (b.xy[0].x as i64, b.xy[0].y as i64);
                let (x1, y1) = (b.xy[2].x as i64, b.xy[2].y as i64);
                (x1 - x0) * (y1 - y0)
            })
            .sum();
        let grown = (400 + 2 * 650i64) * (800 + 2 * 650);
        assert_eq!(area, grown - 400 * 800, "band area = grown minus outline");

        let pmos_labels: Vec<&GdsTextElem> = top
            .elems
            .iter()
            .filter_map(|e| match e {
                GdsElement::GdsTextElem(t) if (t.layer, t.texttype) == ABSTRACT_GDS_PMOS_LABEL => {
                    Some(t)
                }
                _ => None,
            })
            .collect();
        assert_eq!(pmos_labels.len(), 1);
        assert_eq!(pmos_labels[0].string, "PMOS");

        let _ = std::fs::remove_file(&out);
    }

    /// `gr_expand <= 0` means nothing was reserved, so nothing is drawn — the drawing
    /// must never imply a reservation the placer did not make.
    #[test]
    fn abstract_placement_gds_omits_the_band_when_nothing_is_reserved() {
        let devices = vec![AbstractPlacedDevice {
            name: "P".into(),
            x: 0,
            y: 0,
            w: 100,
            h: 100,
            is_pmos: true,
        }];
        let out = std::env::temp_dir().join("abstract_placement_gds_nogr.gds");
        write_abstract_placement_gds(&devices, None, 0, &out).unwrap();
        let top = &GdsLibrary::load(&out).unwrap().structs[0];
        assert!(!top
            .elems
            .iter()
            .any(|e| matches!(e, GdsElement::GdsBoundary(b)
            if (b.layer, b.datatype) == ABSTRACT_GDS_GR)));
        // The PMOS marker still appears: the device is one regardless.
        assert!(top
            .elems
            .iter()
            .any(|e| matches!(e, GdsElement::GdsBoundary(b)
            if (b.layer, b.datatype) == ABSTRACT_GDS_PMOS)));
        let _ = std::fs::remove_file(&out);
    }

    /// The label site is chosen by a total order, not by input-file order:
    /// preferred source first, then the widest rectangle, then geometry.
    #[test]
    fn lvs_label_site_order_is_total_and_source_ranked() {
        let pin_small = LvsLabelSite {
            rank: LVS_SITE_PIN,
            metal_idx: 0,
            xl: 0,
            yl: 0,
            xh: 10,
            yh: 10,
        };
        let pin_big = LvsLabelSite {
            xh: 100,
            ..pin_small
        };
        let derived = LvsLabelSite {
            rank: LVS_SITE_DERIVED,
            xh: 10_000,
            ..pin_small
        };
        let mut v = [derived, pin_small, pin_big];
        v.sort_by_key(|s| s.order_key());
        // Device pin metal first, and the widest one of those wins.
        assert_eq!(v[0], pin_big);
        assert_eq!(v[1], pin_small);
        assert_eq!(v[2], derived);
        // Two sites that differ only in geometry are still ordered.
        let a = LvsLabelSite {
            yl: -5,
            ..pin_small
        };
        assert_ne!(a.order_key(), pin_small.order_key());
        // The centre is inside the rectangle, never on its edge.
        let (cx, cy) = pin_big.centre();
        assert!(cx > pin_big.xl && cx < pin_big.xh);
        assert!(cy > pin_big.yl && cy < pin_big.yh);
    }

    /// Only closed axis-aligned rectangles become label sites — a ring-shaped or
    /// L-shaped polygon's bbox centre can sit in the hole, on no conductor.
    #[test]
    fn derived_polygon_rect_rejects_non_rectangles() {
        use applace_core::Point;
        let rect = vec![
            Point::new(0, 0),
            Point::new(10, 0),
            Point::new(10, 20),
            Point::new(0, 20),
            Point::new(0, 0),
        ];
        assert_eq!(derived_polygon_rect(&rect), Some((0, 0, 10, 20)));
        let l_shape = vec![
            Point::new(0, 0),
            Point::new(10, 0),
            Point::new(10, 5),
            Point::new(5, 5),
            Point::new(5, 20),
        ];
        assert_eq!(derived_polygon_rect(&l_shape), None);
        let degenerate = vec![
            Point::new(0, 0),
            Point::new(10, 0),
            Point::new(10, 0),
            Point::new(0, 0),
        ];
        assert_eq!(derived_polygon_rect(&degenerate), None);
    }

    #[test]
    fn abstract_placement_gds_region_is_optional() {
        let devices = vec![AbstractPlacedDevice {
            name: "only".into(),
            x: 0,
            y: 0,
            w: 100,
            h: 100,
            is_pmos: false,
        }];
        let out = std::env::temp_dir().join("abstract_placement_gds_noregion.gds");
        write_abstract_placement_gds(&devices, None, 650, &out).unwrap();
        let lib = GdsLibrary::load(&out).unwrap();
        assert!(!lib.structs[0].elems.iter().any(|e| {
            matches!(e, GdsElement::GdsBoundary(b)
                if (b.layer, b.datatype) == ABSTRACT_GDS_REGION)
        }));
        let _ = std::fs::remove_file(&out);
    }
}
