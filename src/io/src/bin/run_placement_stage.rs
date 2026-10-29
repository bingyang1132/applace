// Three clippy style lints are allowed for this crate as a whole. Each is a style
// lint whose advice would make this code harder to read, and the reason is written
// out next to it; everything clippy found that was worth fixing was fixed rather
// than allowed.
//
//   type_complexity      the data extractors return tuples of parallel arrays; a type
//                        alias per tuple would name nothing the reader cannot see.
//   too_many_arguments   the gradient entry points take flat parameter lists.
//   needless_range_loop  index loops in the numeric kernel, where the index is the
//                        thing being reasoned about.
#![allow(
    clippy::type_complexity,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]

use applace_core::mfg_grid::{on_grid, snap_nearest};
use applace_core::CircuitDb;
use applace_io::{
    apply_device_types, parse_hierarchical_netlist, parse_pcell, parse_pdk, parse_symmetry,
    write_global_placement_json, write_placement_gds_from_db,
};
use applace_placer::{
    IterRow, PlacementConfig, Placer, TailDiagnosticsIterRow, OUTPUT_SCHEMA_VERSION,
};
use std::collections::HashMap;
use std::f64::consts::PI;
use std::path::{Path, PathBuf};
use std::time::Instant;

/// Device rectangle (used by the grid-snap self-check): `(name, xl, yl, xh, yh)`, in DBU.
type CellRect = (String, i32, i32, i32, i32);

fn cell_rects(db: &CircuitDb) -> Vec<CellRect> {
    db.cells()
        .iter()
        .map(|c| {
            let b = c.bounding_box();
            (c.name().to_string(), b.xl, b.yl, b.xh, b.yh)
        })
        .collect()
}

fn overlapping_pairs(rects: &[CellRect]) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for i in 0..rects.len() {
        for j in (i + 1)..rects.len() {
            let a = &rects[i];
            let b = &rects[j];
            if a.1 < b.3 && b.1 < a.3 && a.2 < b.4 && b.2 < a.4 {
                out.push((a.0.clone(), b.0.clone()));
            }
        }
    }
    out
}

/// Applies the placer's continuous solution to the DB.
///
/// When `snap_grid > 1` the integer coordinates are snapped to the manufacturing grid
/// (device origins were one source of off-grid geometry).
///
/// **Why this is the right moment**: this is the *only* gate where the placer's
/// floating-point solution turns into integer DB coordinates -- the `.gp.json` and `.gds`
/// of every GP/tail/deoverlap checkpoint, and the `local_conflict_resolution.json` that
/// downstream stages read, all flow out of here. Doing it inside the solver
/// would push a quantizer into the gradient loop and perturb convergence; doing it only
/// when writing GDS would let the model the judge sees drift apart from the layout that
/// actually ships. Snapping once at the write-out gate keeps all three artifacts from the
/// same source.
///
/// Snapping rounds to the nearest grid point (displacement <= grid/2). The de-overlap
/// semantics (no overlap) are backed by the caller's `apply_positions_checked`
/// self-check: when device widths and heights are all integer multiples of the grid, new
/// overlap is mathematically impossible (proof and unit tests in `core::mfg_grid`); the
/// self-check exists to stop a PDK/PCell change whose widths or heights are not grid
/// multiples from quietly breaking that precondition.
fn apply_positions(db: &mut CircuitDb, positions: &[f64], snap_grid: i32) {
    let n = db.num_cells();
    let num_cells = n;
    for i in 0..num_cells {
        if i + num_cells >= positions.len() {
            break;
        }
        if let Some(cell) = db.cell_mut(i as i32) {
            let x = positions[i];
            let y = positions[i + num_cells];
            if snap_grid > 1 {
                let xs = snap_nearest(x.round() as i32, snap_grid);
                let ys = snap_nearest(y.round() as i32, snap_grid);
                cell.set_coord_unsnapped(xs as f64, ys as f64);
                cell.set_coord(xs, ys);
            } else {
                cell.set_coord_unsnapped(x, y);
                cell.set_coord(x.round() as i32, y.round() as i32);
            }
        }
    }
}

/// `apply_positions` plus a "snapping created no new overlap" self-check.
///
/// The comparison only runs when `snap_grid > 1`: positions are first applied with the old
/// semantics and the overlap set is recorded, then snapped and recomputed; any new
/// overlapping pair is a **hard error** (never a silent downgrade). It also checks whether
/// device widths and heights are integer multiples of the grid -- that is the precondition
/// for "snapping adds no overlap".
fn apply_positions_checked(
    db: &mut CircuitDb,
    positions: &[f64],
    snap_grid: i32,
    label: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    if snap_grid <= 1 {
        apply_positions(db, positions, snap_grid);
        return Ok(());
    }
    apply_positions(db, positions, 0);
    let before = overlapping_pairs(&cell_rects(db));
    apply_positions(db, positions, snap_grid);
    let after_rects = cell_rects(db);

    let off_size: Vec<&CellRect> = after_rects
        .iter()
        .filter(|r| !on_grid(r.3 - r.1, snap_grid) || !on_grid(r.4 - r.2, snap_grid))
        .collect();
    if !off_size.is_empty() {
        return Err(format!(
            "[mfg-grid] {label}: {} devices have a width/height that is not a multiple of the manufacturing grid {snap_grid} DBU \
             (first: {} {}x{}); the precondition that snapping adds no overlap is broken, refusing to continue",
            off_size.len(),
            off_size[0].0,
            off_size[0].3 - off_size[0].1,
            off_size[0].4 - off_size[0].2,
        )
        .into());
    }

    let before_set: std::collections::BTreeSet<(String, String)> = before.into_iter().collect();
    let new_pairs: Vec<(String, String)> = overlapping_pairs(&after_rects)
        .into_iter()
        .filter(|p| !before_set.contains(p))
        .collect();
    if !new_pairs.is_empty() {
        return Err(format!(
            "[mfg-grid] {label}: snapping to the manufacturing grid {snap_grid} DBU added {} new overlap pairs (first: {} / {})",
            new_pairs.len(),
            new_pairs[0].0,
            new_pairs[0].1,
        )
        .into());
    }
    Ok(())
}

fn build_ap_json_by_net(db: &CircuitDb, phi_radians: &[f64]) -> serde_json::Value {
    let mut ap_by_net = serde_json::Map::new();
    for (pidx, phi) in phi_radians.iter().copied().enumerate() {
        let Some(pin) = db.pin(pidx as i32) else {
            continue;
        };
        let pin_name = pin.name().to_string();
        let device_name = pin
            .cell_idx()
            .and_then(|cidx| db.cell(cidx))
            .map(|c| c.name().to_string())
            .unwrap_or_else(|| "TOP".to_string());
        if pin.net_ids().is_empty() {
            let e = ap_by_net
                .entry("__UNCONNECTED__".to_string())
                .or_insert_with(|| serde_json::Value::Array(Vec::new()));
            if let Some(arr) = e.as_array_mut() {
                arr.push(serde_json::json!({
                    "pin_name": pin_name,
                    "device_name": device_name,
                    "phi": phi,
                }));
            }
            continue;
        }
        for net_idx in pin.net_ids() {
            let net_name = db
                .net(*net_idx)
                .map(|n| n.name().to_string())
                .unwrap_or_else(|| format!("__NET_{}__", net_idx));
            let e = ap_by_net
                .entry(net_name)
                .or_insert_with(|| serde_json::Value::Array(Vec::new()));
            if let Some(arr) = e.as_array_mut() {
                arr.push(serde_json::json!({
                    "pin_name": pin_name,
                    "device_name": device_name,
                    "phi": phi,
                }));
            }
        }
    }
    serde_json::Value::Object(ap_by_net)
}

fn write_tail_diag_csv(
    path: &Path,
    rows: &[TailDiagnosticsIterRow],
) -> Result<(), Box<dyn std::error::Error>> {
    let mut w = String::new();
    w.push_str("iter,phase,wirelength_main,overlap_loss,symmetry_loss,anchor_loss,total_loss,lambda_overlap,lambda_symmetry,lambda_boundary,anchor_w_strong,anchor_w_mid,anchor_w_weak,mobility_min,mobility_max,grad_norm_total,grad_norm_wirelength,grad_norm_overlap,grad_norm_symmetry,grad_norm_anchor,update_norm_pre_damping,update_norm_post_damping,applied_step_norm,disp_all_max,disp_all_rms,disp_all_median,disp_conflict_max,disp_conflict_rms,disp_conflict_median,disp_nonconflict_max,disp_nonconflict_rms,disp_nonconflict_median,disp_top10_max,disp_top10_rms,disp_top10_median,disp_mid40_max,disp_mid40_rms,disp_mid40_median,disp_bot50_max,disp_bot50_rms,disp_bot50_median,overlap_count,overlap_total_area,conflict_component_count,largest_component_size,full_variable_count,reduced_variable_count,pair_tie_count,self_sym_tie_count,movable_object_count,adam_m_norm,adam_v_norm,reduced_state_dim,expanded_dim,shape_consistent,inflation_scale_current,inflated_component_count,inflation_object_count,dist_to_final_max,dist_to_final_rms\n");
    for r in rows {
        w.push_str(&format!(
            "{},{},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{},{:.6e},{},{},{},{},{},{},{},{:.6e},{:.6e},{},{},{},{:.6e},{},{},{:.6e},{:.6e}\n",
            r.iter,
            r.phase,
            r.wirelength_main,
            r.overlap_loss,
            r.symmetry_loss,
            r.anchor_loss,
            r.total_loss,
            r.lambda_overlap,
            r.lambda_symmetry,
            r.lambda_boundary,
            r.anchor_w_strong,
            r.anchor_w_mid,
            r.anchor_w_weak,
            r.mobility_min,
            r.mobility_max,
            r.grad_norm_total,
            r.grad_norm_wirelength,
            r.grad_norm_overlap,
            r.grad_norm_symmetry,
            r.grad_norm_anchor,
            r.update_norm_pre_damping,
            r.update_norm_post_damping,
            r.applied_step_norm,
            r.disp_all_max,
            r.disp_all_rms,
            r.disp_all_median,
            r.disp_conflict_max,
            r.disp_conflict_rms,
            r.disp_conflict_median,
            r.disp_nonconflict_max,
            r.disp_nonconflict_rms,
            r.disp_nonconflict_median,
            r.disp_top10_max,
            r.disp_top10_rms,
            r.disp_top10_median,
            r.disp_mid40_max,
            r.disp_mid40_rms,
            r.disp_mid40_median,
            r.disp_bot50_max,
            r.disp_bot50_rms,
            r.disp_bot50_median,
            r.overlap_count,
            r.overlap_total_area,
            r.conflict_component_count,
            r.largest_component_size,
            r.full_variable_count,
            r.reduced_variable_count,
            r.pair_tie_count,
            r.self_sym_tie_count,
            r.movable_object_count,
            r.adam_m_norm,
            r.adam_v_norm,
            r.reduced_state_dim,
            r.expanded_dim,
            r.shape_consistent,
            r.inflation_scale_current,
            r.inflated_component_ids.len(),
            r.inflation_object_count,
            r.dist_to_final_max.unwrap_or(0.0),
            r.dist_to_final_rms.unwrap_or(0.0),
        ));
    }
    std::fs::write(path, w)?;
    Ok(())
}

fn write_tail_diag_csv_from_phases(
    path: &Path,
    rows: &[IterRow],
) -> Result<(), Box<dyn std::error::Error>> {
    let mut w = String::new();
    w.push_str("iter,phase,disp_all_max,disp_all_rms,disp_all_median,dist_to_final_max,dist_to_final_rms,overlap_total_area,conflict_component_count,applied_step_norm\n");
    for r in rows {
        w.push_str(&format!(
            "{},multiphase,{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{},{:.6e}\n",
            r.iter,
            r.disp_to_ckpt_max,
            r.disp_to_ckpt_rms,
            r.disp_to_ckpt_rms,
            0.0f64,
            0.0f64,
            r.ovl_raw_real,
            r.num_conflict_components_real,
            r.step_norm,
        ));
    }
    std::fs::write(path, w)?;
    Ok(())
}

fn update_run_summary(
    multiphase_dir: &Path,
    out_gp: &Path,
    out_gds: &Path,
    base_gp: &Path,
    base_gds: &Path,
    tail_gp: &Path,
    tail_gds: &Path,
    best_gp: &Path,
    best_gds: &Path,
    deoverlap_gp: &Path,
    deoverlap_gds: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let summary_path = multiphase_dir.join("run_summary.json");
    let mut v: serde_json::Value = serde_json::from_slice(&std::fs::read(&summary_path)?)?;
    let got = v
        .get("schema_version")
        .and_then(serde_json::Value::as_u64)
        .or_else(|| v.get("schemaVersion").and_then(serde_json::Value::as_u64))
        .unwrap_or(0);
    if got != OUTPUT_SCHEMA_VERSION as u64 {
        return Err(format!(
            "run_summary schema mismatch: expected {}, got {}",
            OUTPUT_SCHEMA_VERSION, got
        )
        .into());
    }
    for key in [
        "base_end_iter",
        "tail_end_iter",
        "selected_final_candidate_label",
        "candidate_summary",
    ] {
        if v.get(key).is_none() {
            return Err(format!("run_summary missing required key `{key}`").into());
        }
    }

    if let Some(obj) = v.as_object_mut() {
        obj.insert(
            "schemaVersion".to_string(),
            serde_json::json!(OUTPUT_SCHEMA_VERSION),
        );
        obj.insert(
            "output_semantics".to_string(),
            serde_json::json!({
                "out_binding": "base_end",
                "gp_end_label": "base_end",
                "base_end_role": "global_placement",
                "tail_end_role": "reference_guided_expansion",
                "best_composite_role": "internal_candidate",
                "deoverlap_end_role": "local_conflict_resolution"
            }),
        );
        obj.insert(
            "artifacts".to_string(),
            serde_json::json!({
                "base_end": { "gp": base_gp.to_string_lossy(), "gds": base_gds.to_string_lossy() },
                "gp_end": { "gp": base_gp.to_string_lossy(), "gds": base_gds.to_string_lossy() },
                "tail_end": { "gp": tail_gp.to_string_lossy(), "gds": tail_gds.to_string_lossy() },
                "best_composite": { "gp": best_gp.to_string_lossy(), "gds": best_gds.to_string_lossy() },
                "deoverlap_end": { "gp": deoverlap_gp.to_string_lossy(), "gds": deoverlap_gds.to_string_lossy() },
                "out": { "gp": out_gp.to_string_lossy(), "gds": out_gds.to_string_lossy() }
            }),
        );
    } else {
        return Err("run_summary.json must be a JSON object".into());
    }

    std::fs::write(summary_path, serde_json::to_string_pretty(&v)?)?;
    Ok(())
}

fn env_flag_on(name: &str) -> bool {
    matches!(
        std::env::var(name).ok().as_deref(),
        Some("1" | "true" | "TRUE" | "yes" | "YES")
    )
}

/// Resolve a per-cell routability weight map (`APPLACE_HALO_WEIGHTS`) into a weight
/// vector indexed like every other placer per-cell array.
///
/// The file is supplied by the caller: it is how an outside congestion estimate is
/// fed back into placement. Only its `cells` object is read here; the `grid` and
/// `bins` sections are accepted and ignored.
///
/// Returns `None` when the knob is unset, which is the default and leaves the placer's
/// own all-zero weights (and therefore byte-identical placement) in force.
fn load_halo_weights(
    db: &CircuitDb,
) -> Result<Option<(Vec<f64>, f64)>, Box<dyn std::error::Error>> {
    let Some(path) = std::env::var_os("APPLACE_HALO_WEIGHTS") else {
        return Ok(None);
    };
    let path = PathBuf::from(path);
    let gain: f64 = match std::env::var("APPLACE_HALO_GAIN") {
        Ok(v) => v.trim().parse().map_err(|e| {
            format!("APPLACE_HALO_GAIN is not a number: {v:?} ({e}); expected a float >= 0")
        })?,
        Err(_) => 1.0,
    };

    let text = std::fs::read_to_string(&path)
        .map_err(|e| format!("cannot read APPLACE_HALO_WEIGHTS={}: {e}", path.display()))?;
    let doc: serde_json::Value = serde_json::from_str(&text).map_err(|e| {
        format!(
            "APPLACE_HALO_WEIGHTS={} is not valid JSON: {e}",
            path.display()
        )
    })?;
    let schema = doc.get("schema").and_then(|v| v.as_str()).unwrap_or("");
    if schema != HALO_WEIGHTS_SCHEMA {
        return Err(format!(
            "APPLACE_HALO_WEIGHTS={}: schema is {schema:?}, expected {HALO_WEIGHTS_SCHEMA:?}",
            path.display()
        )
        .into());
    }
    let cells = doc
        .get("cells")
        .and_then(|v| v.as_object())
        .ok_or_else(|| {
            format!(
                "APPLACE_HALO_WEIGHTS={}: missing `cells` object",
                path.display()
            )
        })?;

    // Name -> index over the cells.
    let n = db.num_cells();
    let mut name_to_idx: HashMap<&str, usize> = HashMap::new();
    for (i, cell) in db.cells().iter().enumerate() {
        name_to_idx.insert(cell.name(), i);
    }

    let mut weights = vec![0.0f64; n];
    // BTreeMap iteration order (serde_json preserves insertion order without the
    // `preserve_order` feature, but sorting makes the log and the error path
    // order-independent regardless of how the producer emitted the file).
    let mut entries: Vec<(&String, &serde_json::Value)> = cells.iter().collect();
    entries.sort_by(|a, b| a.0.cmp(b.0));
    let mut matched = 0usize;
    let mut unmatched: Vec<&str> = Vec::new();
    for (name, val) in entries {
        let w = val.as_f64().ok_or_else(|| {
            format!("APPLACE_HALO_WEIGHTS: cells[{name:?}] is not a number: {val}")
        })?;
        if !w.is_finite() || w < 0.0 {
            return Err(
                format!("APPLACE_HALO_WEIGHTS: cells[{name:?}] = {w}, expected >= 0").into(),
            );
        }
        match name_to_idx.get(name.as_str()) {
            Some(&i) => {
                weights[i] = w;
                matched += 1;
            }
            None => unmatched.push(name.as_str()),
        }
    }
    if !unmatched.is_empty() {
        // A stale map (wrong case, renamed cells) must not silently degrade into a
        // no-op: that would look like "the loop did nothing" instead of "the input
        // was wrong".
        return Err(format!(
            "APPLACE_HALO_WEIGHTS={}: {} of {} cell names do not exist in this design \
             (first few: {:?}); the map does not belong to this case",
            path.display(),
            unmatched.len(),
            matched + unmatched.len(),
            &unmatched[..unmatched.len().min(5)]
        )
        .into());
    }
    let nonzero = weights.iter().filter(|w| **w > 0.0).count();
    let wmax = weights.iter().cloned().fold(0.0f64, f64::max);
    eprintln!(
        "[halo] APPLACE_HALO_WEIGHTS={} gain={gain} matched_cells={matched} \
         nonzero={nonzero} max_weight={wmax}",
        path.display()
    );
    Ok(Some((weights, gain)))
}

const HALO_WEIGHTS_SCHEMA: &str = "applace.congestion/1";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let t_total = Instant::now();
    // usage:
    // run_placement_stage <netlist> <pcelldir> <pdk.json> <params.json> <out_gp.json> <out_gds> [sym.json] [--set key=value]...
    let args: Vec<String> = std::env::args().collect();
    let mut positional: Vec<String> = Vec::new();
    let mut cli_overrides: Vec<String> = Vec::new();
    let mut i = 1usize;
    while i < args.len() {
        let arg = &args[i];
        if arg == "--set" {
            if i + 1 >= args.len() {
                eprintln!("usage error: `--set` requires `key=value`");
                std::process::exit(2);
            }
            cli_overrides.push(args[i + 1].clone());
            i += 2;
            continue;
        }
        if let Some(rest) = arg.strip_prefix("--set=") {
            cli_overrides.push(rest.to_string());
            i += 1;
            continue;
        }
        positional.push(arg.clone());
        i += 1;
    }
    if positional.len() != 6 && positional.len() != 7 {
        eprintln!(
            "usage: {} <netlist> <pcelldir> <pdk.json> <params.json> <out_gp.json> <out_gds> [sym.json] [--set key=value]...",
            args[0]
        );
        std::process::exit(2);
    }

    let netlist = PathBuf::from(&positional[0]);
    let pcelldir = PathBuf::from(&positional[1]);
    let pdk = PathBuf::from(&positional[2]);
    let params_path = PathBuf::from(&positional[3]);
    let out_gp = PathBuf::from(&positional[4]);
    let out_gds = PathBuf::from(&positional[5]);
    let sym = positional.get(6).map(PathBuf::from);

    let t_load = Instant::now();
    let mut db = CircuitDb::new();
    parse_hierarchical_netlist(&netlist, &mut db)?;
    if let Some(sym_path) = &sym {
        if sym_path.exists() {
            let _ = parse_symmetry(sym_path, &mut db)?;
        }
    }

    let pcell = parse_pcell(&pcelldir, None::<&std::path::Path>, Some(&pdk), &mut db)?;
    let pdk_rules = parse_pdk(&pdk)?;
    // The deck's device-class declaration, checked against this netlist. No section in
    // the deck = nothing installed = the historical heuristics, byte-identical.
    apply_device_types(&mut db, &pdk_rules)?;
    eprintln!(
        "[timing] placement.load_inputs_ms={}",
        t_load.elapsed().as_millis()
    );

    let out_dir = out_gp
        .parent()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."));
    let multiphase_dir =
        if out_dir.file_name().and_then(|x| x.to_str()) == Some("multiphase_controller") {
            out_dir.clone()
        } else {
            out_dir.join("multiphase_controller")
        };
    std::fs::create_dir_all(&multiphase_dir)?;

    let t_params = Instant::now();
    let (mut params, resolve_meta) =
        PlacementConfig::from_case_params_json_with_overrides(&params_path, &cli_overrides)?;

    // Derive the DRC-aware tail inflation floor from PDK spacing so legalization keeps a
    // minimum clean gap between abutting devices. Computed in placement DBU as
    //   hard_inflation = ceil_to_grid(0.5 * max_pairwise_clean_spacing + safety_margin).
    // PMOS devices already carry a static guard-ring expansion, so this floor stacks on
    // top of it; non-PMOS devices get the floor directly. Falls back to `crit_sp` when the
    // PDK omits `max_pairwise_clean_spacing`.
    {
        let um_to_dbu = if pdk_rules.unit_r > 0.0 && pdk_rules.unit_n > 0.0 {
            pdk_rules.unit_r / pdk_rules.unit_n
        } else {
            1000.0
        };
        let clean_sp_um = pdk_rules
            .max_pairwise_clean_spacing
            .unwrap_or(pdk_rules.crit_sp);
        let safety_um = params.tail_inflation_safety_margin;
        // Convert each length to integer DBU before combining so floating-point drift in
        // `unit_r / unit_n` does not leak into the floor (DBU are integer nanometers).
        let clean_sp_dbu = (clean_sp_um * um_to_dbu).round();
        let safety_dbu = (safety_um * um_to_dbu).round();
        let grid_dbu = (pdk_rules.grid * um_to_dbu).round().max(1.0);
        let raw_dbu = 0.5 * clean_sp_dbu + safety_dbu;
        let floor_dbu = (raw_dbu / grid_dbu).ceil() * grid_dbu;
        params.tail_hard_inflation_floor = floor_dbu;
        eprintln!(
            "[drc] max_pairwise_clean_spacing_um={} safety_um={} grid_dbu={} tail_hard_inflation_floor_dbu={}",
            clean_sp_um, safety_um, grid_dbu, floor_dbu
        );

        // PMOS guard-ring expansion: the expansion a guard ring actually needs is
        // od_w + 2*nw_od (guard_ring_bbox), and the placer's overlap reservation must come
        // from the same source; otherwise a PDK swap silently mismatches (reservation too
        // small -> the ring hits a neighbour; too large -> wasted area). When params does
        // not set it explicitly it is derived from the PDK; an explicit setting is kept as
        // an override channel (for experiments such as an area sweep).
        if resolve_meta
            .sources
            .get("pmosHardExpandForGr")
            .map(|s| s == "default")
            .unwrap_or(false)
        {
            let derived_um = pdk_rules.od_w + 2.0 * pdk_rules.nw_od;
            let derived_dbu = (derived_um * um_to_dbu).round();
            params.pmos_hard_expand_for_gr = derived_dbu;
            eprintln!(
                "[drc] pmosHardExpandForGr derived from PDK (od_w + 2*nw_od): {} DBU",
                derived_dbu
            );
        } else {
            eprintln!(
                "[drc] pmosHardExpandForGr explicitly set ({} DBU), PDK derivation skipped",
                params.pmos_hard_expand_for_gr
            );
        }
    }

    let effective_params_path = multiphase_dir.join("effective_params.json");
    let effective_sources_path = multiphase_dir.join("effective_params_sources.json");
    std::fs::write(
        &effective_params_path,
        serde_json::to_string_pretty(&params)?,
    )?;
    std::fs::write(
        &effective_sources_path,
        serde_json::to_string_pretty(&serde_json::json!({
            "schemaVersion": resolve_meta.schema_version,
            "mode": resolve_meta.mode,
            "cliOverrides": resolve_meta.cli_overrides,
            "sources": resolve_meta.sources
        }))?,
    )?;
    eprintln!("[params] params={}", params_path.display());
    eprintln!("[params] mode={}", resolve_meta.mode);
    eprintln!("[params] cli_overrides={:?}", cli_overrides);
    eprintln!("[params] maxIterations={}", params.max_iterations);
    eprintln!(
        "[params] resolved PlacementConfig:\n{}",
        serde_json::to_string_pretty(&params)?
    );
    eprintln!(
        "[timing] placement.load_params_ms={}",
        t_params.elapsed().as_millis()
    );
    let tail_infl_ratio = params.tail_infl_ratio;
    let tail_infl_ramp_iters = params.tail_infl_ramp_iters;
    let tail_tau_o_ratio = params.tail_tau_o_ratio;
    let tail_ref_dilate_beta = params.tail_ref_dilate_beta;
    let tail_cover_tau = params.tail_cover_tau;
    let tail_cover_softness = params.tail_cover_softness;
    let tail_cover_lambda_ratio = params.tail_cover_lambda_ratio;
    let tail_deoverlap_final_phi_iters = params.tail_deoverlap_final_phi_iters;
    let use_ap_wl = params.use_ap_wl;
    let t_gp = Instant::now();
    let mut placer = Placer::new(&db, params)?;
    if let Some((weights, gain)) = load_halo_weights(&db)? {
        placer.set_halo_weights(&weights, gain)?;
    }
    let _ = placer.run()?;
    eprintln!(
        "[timing] placement.gp_optimize_ms={}",
        t_gp.elapsed().as_millis()
    );
    let t_final_phi = Instant::now();
    let _ = placer.deoverlap_finalize_phi_apwl_fixed_placement(tail_deoverlap_final_phi_iters)?;
    eprintln!(
        "[timing] placement.deoverlap_final_phi_ms={}",
        t_final_phi.elapsed().as_millis()
    );
    let csv_only = env_flag_on("HIER_MULTIPHASE_CSV_ONLY");
    if csv_only {
        let t_csv_only = Instant::now();
        placer.export_iter_csv_only(&multiphase_dir)?;
        if env_flag_on("HIER_BATCH_ORACLE_APWL") {
            let oracle_path = multiphase_dir.join("oracle_apwl_fixed_placement.json");
            let oracle_json = if use_ap_wl {
                serde_json::json!({
                    "applied": false,
                    "reason": "useApWl_true",
                    "oracle_iters_budget": 300,
                    "iters_used": 0,
                    "wl_ap_initial": serde_json::Value::Null,
                    "wl_ap_optimal": serde_json::Value::Null,
                    "improve_abs": serde_json::Value::Null,
                    "improve_pct": serde_json::Value::Null
                })
            } else {
                placer.batch_oracle_apwl_fixed_placement(300)?
            };
            std::fs::write(oracle_path, serde_json::to_string_pretty(&oracle_json)?)?;
        }
        eprintln!(
            "[timing] placement.csv_only_export_ms={}",
            t_csv_only.elapsed().as_millis()
        );
        eprintln!(
            "[timing] placement.total_ms={}",
            t_total.elapsed().as_millis()
        );
        return Ok(());
    }

    if let Some(parent) = out_gp.parent() {
        std::fs::create_dir_all(parent)?;
    }
    if let Some(parent) = out_gds.parent() {
        std::fs::create_dir_all(parent)?;
    }

    let base_end = placer
        .base_end_positions()
        .or_else(|| placer.get_checkpoint_positions())
        .unwrap_or_else(|| placer.get_positions());
    let tail_end = placer
        .tail_end_positions()
        .unwrap_or_else(|| placer.get_positions());
    let best_composite = placer
        .best_composite_positions()
        .unwrap_or_else(|| placer.get_positions());
    let deoverlap_end = placer
        .deoverlap_end_positions()
        .unwrap_or_else(|| placer.get_positions());

    let t_multiphase_export = Instant::now();
    placer.export_multiphase_artifacts(&multiphase_dir)?;
    eprintln!(
        "[timing] placement.export_multiphase_artifacts_ms={}",
        t_multiphase_export.elapsed().as_millis()
    );
    let checkpoints_dir = multiphase_dir.join("checkpoints");
    std::fs::create_dir_all(&checkpoints_dir)?;

    // Manufacturing-grid snapping (master switch APPLACE_MFG_GRID_SNAP, default follows the
    // deck declaration; =0 restores the old behaviour).
    // The grid value comes only from the PDK's `grid` field and must never be hardcoded;
    // whether that `grid` is the real manufacturing grid is self-reported by the deck's
    // `_meta.physical_grid` (default = yes).
    let mfg_um_to_dbu = if pdk_rules.unit_r > 0.0 && pdk_rules.unit_n > 0.0 {
        pdk_rules.unit_r / pdk_rules.unit_n
    } else {
        1000.0
    };
    let mfg_physical_grid = pdk_rules.physical_grid();
    let mfg_snap_grid = applace_core::mfg_grid::effective_snap_grid_declared(
        (pdk_rules.grid * mfg_um_to_dbu).round() as i32,
        mfg_physical_grid,
    );
    eprintln!(
        "[mfg-grid] placement snap_grid_dbu={mfg_snap_grid} physical_grid={mfg_physical_grid}"
    );
    let base_json = checkpoints_dir.join("base_end.gp.json");
    let base_gds = checkpoints_dir.join("base_end.gds");
    let tail_json = checkpoints_dir.join("tail_end.gp.json");
    let tail_gds = checkpoints_dir.join("tail_end.gds");
    let best_json = checkpoints_dir.join("best_composite.gp.json");
    let best_gds = checkpoints_dir.join("best_composite.gds");
    let deoverlap_json = checkpoints_dir.join("deoverlap_end.gp.json");
    let deoverlap_gds = checkpoints_dir.join("deoverlap_end.gds");
    let tail_metrics = out_dir.join("tail_metrics.json");
    let mobility_stats = out_dir.join("mobility_stats.json");
    let conflict_components = out_dir.join("conflict_components.json");
    let inflation_schedule = out_dir.join("tail_inflation_schedule.json");
    let tail_diag_csv = out_dir.join("tail_diagnostics_per_iter.csv");
    let tail_diag_json = out_dir.join("tail_diagnostics.json");
    let tail_object_disp = out_dir.join("tail_object_displacement.json");
    let deoverlap_summary_json = multiphase_dir.join("deoverlap_summary.json");
    let deoverlap_moves_csv = multiphase_dir.join("deoverlap_moves.csv");
    let deoverlap_components_before_json = multiphase_dir.join("deoverlap_components_before.json");
    let deoverlap_components_after_json = multiphase_dir.join("deoverlap_components_after.json");
    let deoverlap_infl_tail_snapshot_json =
        multiphase_dir.join("deoverlap_infl_tail_snapshot.json");
    let deoverlap_final_ap_json = multiphase_dir.join("deoverlap_final_ap.json");

    let t_artifacts = Instant::now();
    apply_positions_checked(
        &mut db,
        base_end.as_slice().unwrap_or(&[]),
        mfg_snap_grid,
        "base_end",
    )?;
    write_global_placement_json(&db, &base_json)?;
    write_placement_gds_from_db(
        &db,
        &pcell.resolved_pcelldir,
        &pcell.pins,
        &pdk_rules,
        &base_gds,
    )?;

    apply_positions_checked(
        &mut db,
        tail_end.as_slice().unwrap_or(&[]),
        mfg_snap_grid,
        "tail_end",
    )?;
    write_global_placement_json(&db, &tail_json)?;
    write_placement_gds_from_db(
        &db,
        &pcell.resolved_pcelldir,
        &pcell.pins,
        &pdk_rules,
        &tail_gds,
    )?;

    apply_positions_checked(
        &mut db,
        best_composite.as_slice().unwrap_or(&[]),
        mfg_snap_grid,
        "best_composite",
    )?;
    write_global_placement_json(&db, &best_json)?;
    write_placement_gds_from_db(
        &db,
        &pcell.resolved_pcelldir,
        &pcell.pins,
        &pdk_rules,
        &best_gds,
    )?;
    apply_positions_checked(
        &mut db,
        deoverlap_end.as_slice().unwrap_or(&[]),
        mfg_snap_grid,
        "deoverlap_end",
    )?;
    write_global_placement_json(&db, &deoverlap_json)?;
    write_placement_gds_from_db(
        &db,
        &pcell.resolved_pcelldir,
        &pcell.pins,
        &pdk_rules,
        &deoverlap_gds,
    )?;
    let deoverlap_final_phi = placer.current_phi_radians();
    std::fs::write(
        &deoverlap_final_ap_json,
        serde_json::to_string_pretty(&build_ap_json_by_net(&db, &deoverlap_final_phi))?,
    )?;
    std::fs::write(
        &tail_metrics,
        serde_json::to_string_pretty(&placer.tail_metrics_json())?,
    )?;
    let tail_rows = placer.tail_diagnostics_iter_row();
    if tail_rows.is_empty() {
        let iter_rows = placer.iter_rows();
        if !iter_rows.is_empty() {
            write_tail_diag_csv_from_phases(&tail_diag_csv, &iter_rows)?;
        } else {
            write_tail_diag_csv(&tail_diag_csv, &tail_rows)?;
        }
    } else {
        write_tail_diag_csv(&tail_diag_csv, &tail_rows)?;
    }
    std::fs::write(
        &tail_diag_json,
        serde_json::to_string_pretty(&placer.tail_diagnostics_snapshot())?,
    )?;
    std::fs::write(
        &tail_object_disp,
        serde_json::to_string_pretty(&placer.object_displacement_json())?,
    )?;
    std::fs::write(
        &mobility_stats,
        serde_json::to_string_pretty(&placer.mobility_stats_json())?,
    )?;
    std::fs::write(
        &conflict_components,
        serde_json::to_string_pretty(&placer.conflict_components_json())?,
    )?;
    std::fs::write(
        &inflation_schedule,
        serde_json::to_string_pretty(&serde_json::json!({
            "mode": "tail_v2_2_inflation_plus_dilated_reference_plus_cover",
            "r_infl": tail_infl_ratio,
            "n_infl_ramp": tail_infl_ramp_iters,
            "tau_o_tail_ratio": tail_tau_o_ratio,
            "beta": tail_ref_dilate_beta,
            "tau_cover": tail_cover_tau,
            "s_cover": tail_cover_softness,
            "lambda_cover_tail_ratio": tail_cover_lambda_ratio
        }))?,
    )?;
    std::fs::write(
        &deoverlap_summary_json,
        serde_json::to_string_pretty(&placer.deoverlap_summary_json())?,
    )?;
    std::fs::write(&deoverlap_moves_csv, placer.deoverlap_moves_csv())?;
    std::fs::write(
        &deoverlap_components_before_json,
        serde_json::to_string_pretty(&placer.deoverlap_components_before())?,
    )?;
    std::fs::write(
        &deoverlap_components_after_json,
        serde_json::to_string_pretty(&placer.deoverlap_components_after())?,
    )?;
    std::fs::write(
        &deoverlap_infl_tail_snapshot_json,
        serde_json::to_string_pretty(&placer.deoverlap_infl_tail_snapshot())?,
    )?;
    // Only written when the LCR retry loop actually fired, so a run whose
    // local conflict resolution converged keeps exactly the file set it had before.
    if let Some(v) = placer.lcr_retry_summary_json() {
        std::fs::write(
            multiphase_dir.join("lcr_retry_summary.json"),
            serde_json::to_string_pretty(&v)?,
        )?;
    }
    eprintln!(
        "[timing] placement.write_artifacts_ms={}",
        t_artifacts.elapsed().as_millis()
    );

    // Public APPlace Global Placement output binds to the pre-tail multi-phase result.
    let t_final_out = Instant::now();
    apply_positions_checked(
        &mut db,
        base_end.as_slice().unwrap_or(&[]),
        mfg_snap_grid,
        "gp_public_output",
    )?;
    write_global_placement_json(&db, &out_gp)?;
    write_placement_gds_from_db(
        &db,
        &pcell.resolved_pcelldir,
        &pcell.pins,
        &pdk_rules,
        &out_gds,
    )?;
    if placer.run_summary().is_some() {
        update_run_summary(
            &multiphase_dir,
            &out_gp,
            &out_gds,
            &base_json,
            &base_gds,
            &tail_json,
            &tail_gds,
            &best_json,
            &best_gds,
            &deoverlap_json,
            &deoverlap_gds,
        )?;
    }
    eprintln!(
        "[timing] placement.write_final_outputs_ms={}",
        t_final_out.elapsed().as_millis()
    );

    // Export labeled multi-phase candidates as GDS for quick visual comparison.
    let t_candidates = Instant::now();
    if let Some(summary) = placer.run_summary() {
        let candidates = placer.candidates();
        let cand_dir = multiphase_dir.join("candidates");
        if cand_dir.exists() {
            std::fs::remove_dir_all(&cand_dir)?;
        }
        std::fs::create_dir_all(&cand_dir)?;
        let mut by_iter = HashMap::new();
        for c in &candidates {
            by_iter.insert(c.iter, c);
        }
        let mut manifest = Vec::new();
        for ent in &summary.candidate_summary {
            let c = by_iter.get(&ent.iter).ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    format!(
                        "candidate summary entry `{}` at iter {} has no matching candidate",
                        ent.label, ent.iter
                    ),
                )
            })?;
            apply_positions(&mut db, &c.full_positions, mfg_snap_grid);
            let safe_label = ent
                .label
                .chars()
                .map(|ch| {
                    if ch.is_ascii_alphanumeric() || ch == '_' || ch == '-' {
                        ch
                    } else {
                        '_'
                    }
                })
                .collect::<String>();
            let gds_path =
                cand_dir.join(format!("candidate_{}_iter_{:05}.gds", safe_label, ent.iter));
            write_placement_gds_from_db(
                &db,
                &pcell.resolved_pcelldir,
                &pcell.pins,
                &pdk_rules,
                &gds_path,
            )?;
            let gp_json_path = cand_dir.join(format!(
                "candidate_{}_iter_{:05}.gp.json",
                safe_label, ent.iter
            ));
            write_global_placement_json(&db, &gp_json_path)?;
            let ap_json_path = cand_dir.join(format!(
                "candidate_{}_iter_{:05}.ap.json",
                safe_label, ent.iter
            ));
            let cand_phi: Vec<f64> = c.phi_tilde.iter().map(|v| v * PI).collect();
            std::fs::write(
                &ap_json_path,
                serde_json::to_string_pretty(&build_ap_json_by_net(&db, &cand_phi))?,
            )?;
            manifest.push(serde_json::json!({
                "label": ent.label,
                "iter": ent.iter,
                "gds": gds_path.to_string_lossy(),
                "gp_json": gp_json_path.to_string_lossy(),
                "ap_json": ap_json_path.to_string_lossy(),
            }));
        }
        let manifest_path = multiphase_dir.join("candidates_manifest.json");
        std::fs::write(&manifest_path, serde_json::to_string_pretty(&manifest)?)?;
    }
    eprintln!(
        "[timing] placement.export_candidate_gds_ms={}",
        t_candidates.elapsed().as_millis()
    );
    eprintln!(
        "[timing] placement.total_ms={}",
        t_total.elapsed().as_millis()
    );

    Ok(())
}

#[cfg(test)]
mod mfg_grid_tests {
    use super::*;

    /// Builds a DB holding cells only; `positions` uses the `apply_positions` layout
    /// (the first n entries are x, the next n are y, index = node index).
    fn db_with_cells(sizes: &[(i32, i32)]) -> CircuitDb {
        let mut db = CircuitDb::new();
        for (i, (w, h)) in sizes.iter().enumerate() {
            let idx = db.allocate_cell(format!("C{i}"));
            if let Some(c) = db.cell_mut(idx) {
                c.set_width(*w);
                c.set_height(*h);
            }
        }
        db
    }

    fn positions(xy: &[(f64, f64)]) -> Vec<f64> {
        let mut v: Vec<f64> = xy.iter().map(|p| p.0).collect();
        v.extend(xy.iter().map(|p| p.1));
        v
    }

    fn coords(db: &CircuitDb) -> Vec<(i32, i32)> {
        db.cells().iter().map(|c| (c.x(), c.y())).collect()
    }

    /// The grid step used in the unit tests is a **synthetic value**, unrelated to the real value of any PDK (real vendor values never enter the repository).
    const G: i32 = 8;

    /// Fix point 1: device origins land on the manufacturing grid, and the integer
    /// coordinates share the same source as the raw ones (downstream
    /// `local_conflict_resolution.json` reads the raw coordinates).
    #[test]
    fn apply_positions_snaps_origins_to_grid() {
        let mut db = db_with_cells(&[(96, 200), (96, 200)]);
        let p = positions(&[(1003.4, -7.6), (2222.0, 9.0)]);
        apply_positions(&mut db, &p, G);
        assert_eq!(coords(&db), vec![(1000, -8), (2224, 8)]);
        for i in 0..db.num_cells() as i32 {
            if let Some(c) = db.cell(i) {
                assert_eq!(
                    c.x() as f64,
                    c.x_unsnapped(),
                    "int/raw must share one source"
                );
                assert_eq!(c.y() as f64, c.y_unsnapped());
            }
        }
    }

    /// With the master switch off (`snap_grid = 0`), every field returns to the pre-fix
    /// behaviour: round-to-nearest, raw stays floating point.
    #[test]
    fn apply_positions_is_unchanged_when_snap_is_off() {
        let mut db = db_with_cells(&[(100, 200)]);
        let p = positions(&[(1003.4, -7.6)]);
        apply_positions(&mut db, &p, 0);
        assert_eq!(coords(&db), vec![(1003, -8)]);
        if let Some(c) = db.cell(0) {
            assert_eq!(c.x_unsnapped(), 1003.4);
        }
    }

    /// Self-check: snapping must not manufacture a new overlap. When device widths and
    /// heights are integer multiples of the grid, the spacing between adjacent (even
    /// touching) devices after snapping only stays the same or becomes another integer
    /// multiple of the grid; it never turns negative.
    #[test]
    fn snapping_never_creates_a_new_overlap() {
        // Two devices whose widths are grid multiples, sweeping the gap from 0 (touching)
        // to 3 grid steps, with deliberately off-grid start positions.
        let w = 12 * G;
        for gap in 0..(3 * G) {
            for x0 in [-13.0f64, -1.0, 0.0, 3.0, 7.4, 101.6] {
                let mut db = db_with_cells(&[(w, w), (w, w)]);
                let p = positions(&[(x0, 0.0), (x0 + w as f64 + gap as f64, 0.0)]);
                apply_positions_checked(&mut db, &p, G, "unit").unwrap_or_else(|e| {
                    panic!("gap={gap} x0={x0}: {e}");
                });
                let c = coords(&db);
                assert!(c[1].0 >= c[0].0 + w, "gap={gap} x0={x0} -> {c:?}");
                assert_eq!(c[0].0.rem_euclid(G), 0);
                assert_eq!(c[1].0.rem_euclid(G), 0);
            }
        }
    }

    /// When the precondition is broken (device width/height not a grid multiple) this must
    /// be a hard error, never a silent downgrade.
    #[test]
    fn non_grid_sized_devices_are_rejected() {
        let mut db = db_with_cells(&[(103, 200)]);
        let p = positions(&[(1000.0, 0.0)]);
        let err = apply_positions_checked(&mut db, &p, G, "unit").unwrap_err();
        assert!(
            err.to_string().contains("manufacturing grid"),
            "unexpected error: {err}"
        );
    }

    /// `overlapping_pairs` uses half-open intervals: touching does not count as overlap.
    #[test]
    fn touching_rects_are_not_overlapping() {
        let rects = vec![
            ("a".to_string(), 0, 0, 100, 100),
            ("b".to_string(), 100, 0, 200, 100),
            ("c".to_string(), 99, 0, 199, 100),
        ];
        let pairs = overlapping_pairs(&rects);
        // a|b only touch -> not an overlap; c genuinely overlaps both a and b.
        assert_eq!(
            pairs,
            vec![
                ("a".to_string(), "c".to_string()),
                ("b".to_string(), "c".to_string())
            ]
        );
    }
}
