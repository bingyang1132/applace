//! Form two — placement from a purely geometric abstract case.
//!
//! Form one (`run_placement_stage`) reads a device flow: a SPICE
//! netlist plus one GDS per PCell plus a PDK deck. Form two reads none of that. Its
//! whole input is:
//!
//!   * `bbox.json` — one outline box per device, plus an `isPmos` flag
//!   * `pin.json`  — which device each pin belongs to and which net it is on
//!   * `sym.json`  — symmetry pairs
//!   * `params.json` — the same optimizer parameters form one uses
//!
//! No vendor geometry, no PDK rule values, no GDS. That makes it the cleanest thing to
//! publish, and it is enough to run the whole gradient placer and emit a legalized
//! placement.
//!
//! Pin positions are deliberately *not* given. Form two says a pin may sit anywhere
//! on its device, and that is exactly what the placer's access-point model already
//! expresses: `phi` is a free per-pin variable placing the pin on its device's
//! outline, solved along with the placement. So form two keeps `useApWl` on and
//! simply supplies no informative offset -- the abstract `pin.json` writes the device
//! centre, which projects to a neutral starting angle that the optimizer then moves.
//!
//! Outputs, next to `<out.json>`:
//!
//!   * `<out>.json` — `{device_name: [x, y]}`, the same format form one's
//!     `local_conflict_resolution.json` uses (lower-left corner, DBU = nm)
//!   * `<out>.gds`  — the same placement as an outline-level layout you can open in a
//!     GDS viewer: one rectangle per device, a name label, a pin anchor at the device
//!     centre, the declared placement region, and — so the picture explains itself —
//!     a PMOS marker and the guard-ring reservation band around each PMOS. Demo-local
//!     layer numbers, no vendor layermap (see `writer::write_abstract_placement_gds`).
//!   * `<out>_summary.json` — the same placement as data: per device its box, whether
//!     it is a PMOS, how much guard-ring room was reserved for it and which symmetry
//!     pair it belongs to, plus the global figures (region, layout bbox, HPWL, residual
//!     overlaps, spacing floor). This is what `scripts/plot_placement.py` reads, and
//!     what answers "is this placement reasonable?" without rerunning anything.
//!   * `<out>.lcr_retry.json` — only when the LCR retry loop fired.
//!
//! Usage:
//!   run_abstract_placement <bbox.json> <pin.json> <sym.json> <params.json> <out.json>
//!                          [--set key=value]...

use applace_io::writer::{write_abstract_placement_gds, AbstractPlacedDevice};
use applace_placer::{PlaceData, PlacementConfig, Placer};
use std::path::PathBuf;

/// Which symmetry pair a device belongs to, for reporting only.
#[derive(Debug, Clone, PartialEq, Eq)]
struct SymMembership {
    /// The group's declared name in `sym.json`.
    group: String,
    /// The other device's cell index, or `None` for a self-symmetric device (one
    /// declared symmetric about the axis by itself, which the placer stores as the
    /// degenerate pair `(i, i)`).
    partner: Option<usize>,
    /// `true` = the group's axis is vertical.
    is_vert: bool,
}

/// Per-device symmetry membership, indexed by cell index.
///
/// Read straight off what the placer solved (`sym_pairs` / `sym_pair_group` /
/// `sym_group_vertical`) plus the declared group names the loader carried through, so the
/// report cannot drift from the constraint that was actually applied. A device in more
/// than one pair keeps the first — the report names one partner, and the full relation
/// is still in `sym.json`.
fn sym_membership(data: &PlaceData, num_cells: usize) -> Vec<Option<SymMembership>> {
    let mut out: Vec<Option<SymMembership>> = vec![None; num_cells];
    for p in 0..data.num_sym_pairs {
        let a = data.sym_pairs[2 * p] as usize;
        let b = data.sym_pairs[2 * p + 1] as usize;
        let g = data.sym_pair_group[p] as usize;
        let name = data
            .sym_group_names
            .get(g)
            .cloned()
            .unwrap_or_else(|| format!("group_{g}"));
        let is_vert = data.sym_group_vertical.get(g).copied().unwrap_or(true);
        for (self_idx, other) in [(a, b), (b, a)] {
            if self_idx >= num_cells || out[self_idx].is_some() {
                continue;
            }
            out[self_idx] = Some(SymMembership {
                group: name.clone(),
                partner: (other != self_idx).then_some(other),
                is_vert,
            });
        }
    }
    out
}

/// Net membership as device-index lists — the topology the HPWL figure is computed on.
struct NetTopology {
    /// One entry per net: the distinct devices it touches, in ascending index order.
    nets: Vec<Vec<usize>>,
}

impl NetTopology {
    fn from_data(data: &PlaceData) -> Self {
        let mut nets = Vec::with_capacity(data.num_nets);
        for n in 0..data.num_nets {
            let (lo, hi) = (data.net_pin_offsets[n], data.net_pin_offsets[n + 1]);
            let mut cells: Vec<usize> = (lo..hi)
                .map(|k| data.pin_owner_cell[data.net_pin_index[k] as usize] as usize)
                .collect();
            cells.sort_unstable();
            cells.dedup();
            nets.push(cells);
        }
        Self { nets }
    }

    /// Half-perimeter wire length over **device centres**.
    ///
    /// Centres, not pins, on purpose: form two publishes no real pin offsets — every
    /// pin is seeded at its device's centre and the placer slides it around the outline
    /// — so a pin-based HPWL would report the optimizer's internal `phi` variables
    /// rather than anything about the placement. Nets touching fewer than two devices
    /// contribute nothing.
    fn hpwl(&self, cx: &[f64], cy: &[f64]) -> f64 {
        let mut total = 0.0;
        for cells in &self.nets {
            if cells.len() < 2 {
                continue;
            }
            let (mut xl, mut xh) = (f64::MAX, f64::MIN);
            let (mut yl, mut yh) = (f64::MAX, f64::MIN);
            for &c in cells {
                xl = xl.min(cx[c]);
                xh = xh.max(cx[c]);
                yl = yl.min(cy[c]);
                yh = yh.max(cy[c]);
            }
            total += (xh - xl) + (yh - yl);
        }
        total
    }
}

/// The furthest any symmetric device sits from where its group's axis puts it, in DBU.
///
/// Every member of a group contributes an estimate of that group's axis: the midpoint
/// of a mirrored pair, or its own centre if it is self-symmetric. The axis is the mean
/// of those estimates, and the deviation is the worst of two things -- how far any one
/// estimate sits from the mean, and how far a pair's two devices sit apart along the
/// other axis, which mirroring has to leave equal.
///
/// Zero is the expected answer: the legalizer resolves one axis per group and snaps its
/// members onto it. A non-zero figure means a symmetry constraint was not met.
fn symmetry_deviation(sym: &[Option<SymMembership>], cx: &[f64], cy: &[f64]) -> f64 {
    let mut groups: std::collections::BTreeMap<&str, Vec<usize>> =
        std::collections::BTreeMap::new();
    for (i, m) in sym.iter().enumerate() {
        if let Some(m) = m {
            groups.entry(m.group.as_str()).or_default().push(i);
        }
    }

    let mut worst: f64 = 0.0;
    for members in groups.values() {
        let mut axis_estimates: Vec<f64> = Vec::with_capacity(members.len());
        for &i in members {
            let Some(m) = sym[i].as_ref() else {
                continue;
            };
            // Along the axis the pair straddles; across the one it must share.
            let (along, across) = if m.is_vert { (cx, cy) } else { (cy, cx) };
            match m.partner {
                Some(p) => {
                    axis_estimates.push(0.5 * (along[i] + along[p]));
                    worst = worst.max((across[i] - across[p]).abs());
                }
                None => axis_estimates.push(along[i]),
            }
        }
        if axis_estimates.is_empty() {
            continue;
        }
        let axis = axis_estimates.iter().sum::<f64>() / axis_estimates.len() as f64;
        for estimate in &axis_estimates {
            worst = worst.max((estimate - axis).abs());
        }
    }
    worst
}

/// Number of device pairs whose outlines overlap with positive area.
///
/// The honest residual-overlap count for the placement as published: computed off the
/// boxes in `placement_summary.json` itself, not off any internal legalizer bookkeeping.
fn overlap_pair_count(x: &[f64], y: &[f64], w: &[f64], h: &[f64]) -> usize {
    let n = x.len();
    let mut count = 0;
    for i in 0..n {
        for j in (i + 1)..n {
            let ox = (x[i] + w[i]).min(x[j] + w[j]) - x[i].max(x[j]);
            let oy = (y[i] + h[i]).min(y[j] + h[j]) - y[i].max(y[j]);
            if ox > 1e-9 && oy > 1e-9 {
                count += 1;
            }
        }
    }
    count
}

/// Assemble `placement_summary.json`.
///
/// The schema is documented in `docs/OUTPUT.md`; keep the two in step.
#[allow(clippy::too_many_arguments)]
fn build_summary(
    names: &[String],
    cells: &[serde_json::Value],
    positions: &[f64],
    num_cells: usize,
    sym: &[Option<SymMembership>],
    nets: &NetTopology,
    region: Option<(i32, i32, i32, i32)>,
    gr_reservation: f64,
    spacing_floor: f64,
) -> serde_json::Value {
    let size = |c: &serde_json::Value, lo: &str, hi: &str| -> f64 {
        (c["bbox"][hi].as_f64().unwrap_or(0.0) - c["bbox"][lo].as_f64().unwrap_or(0.0)).abs()
    };
    let xs: Vec<f64> = (0..num_cells).map(|i| positions[i]).collect();
    let ys: Vec<f64> = (0..num_cells).map(|i| positions[i + num_cells]).collect();
    let ws: Vec<f64> = cells.iter().map(|c| size(c, "xl", "xh")).collect();
    let hs: Vec<f64> = cells.iter().map(|c| size(c, "yl", "yh")).collect();
    let pmos: Vec<bool> = cells
        .iter()
        .map(|c| c["isPmos"].as_bool().unwrap_or(false))
        .collect();

    let devices: Vec<serde_json::Value> = (0..num_cells)
        .map(|i| {
            let s = sym[i].as_ref();
            serde_json::json!({
                "name": names[i],
                "x": xs[i],
                "y": ys[i],
                "w": ws[i],
                "h": hs[i],
                "isPmos": pmos[i],
                // What the placer kept free around this device, per side. 0 for a
                // non-PMOS: the reservation is a PMOS guard-ring allowance.
                "gr_reservation": if pmos[i] { gr_reservation } else { 0.0 },
                "symmetry_group": s.map(|m| m.group.clone()),
                "symmetry_partner": s.and_then(|m| m.partner).map(|p| names[p].clone()),
                "symmetry_axis": s.map(|m| if m.is_vert { "vertical" } else { "horizontal" }),
            })
        })
        .collect();

    let cx: Vec<f64> = (0..num_cells).map(|i| xs[i] + ws[i] / 2.0).collect();
    let cy: Vec<f64> = (0..num_cells).map(|i| ys[i] + hs[i] / 2.0).collect();
    let bbox = if num_cells == 0 {
        serde_json::Value::Null
    } else {
        serde_json::json!({
            "xl": xs.iter().cloned().fold(f64::MAX, f64::min),
            "yl": ys.iter().cloned().fold(f64::MAX, f64::min),
            "xh": (0..num_cells).map(|i| xs[i] + ws[i]).fold(f64::MIN, f64::max),
            "yh": (0..num_cells).map(|i| ys[i] + hs[i]).fold(f64::MIN, f64::max),
        })
    };
    let sym_pairs = {
        // One entry per unordered pair actually reported above.
        let mut seen = std::collections::BTreeSet::new();
        for (i, m) in sym.iter().enumerate() {
            if let Some(m) = m {
                let p = m.partner.unwrap_or(i);
                seen.insert((i.min(p), i.max(p)));
            }
        }
        seen.len()
    };

    serde_json::json!({
        "schema": "applace.placement_summary/1",
        "form": "abstract",
        "devices": devices,
        "global": {
            "device_count": num_cells,
            "pmos_count": pmos.iter().filter(|p| **p).count(),
            "net_count": nets.nets.len(),
            "sym_pairs": sym_pairs,
            // How far the worst symmetric device sits from its group's axis, in DBU.
            // Zero means every declared symmetry came out exact.
            "sym_deviation": symmetry_deviation(sym, &cx, &cy),
            // Per-side guard-ring room reserved around every PMOS (pmosHardExpandForGr).
            "gr_reservation": gr_reservation,
            // Minimum clean gap legalization keeps between any two devices
            // (tailHardInflationFloor). 0 = no floor was configured, which is the
            // abstract form's default: it ships no deck to derive one from.
            "spacing_floor": spacing_floor,
            // The declared placement region from bbox.json (null when none was given).
            "region": region.map(|(xl, yl, xh, yh)| serde_json::json!(
                {"xl": xl, "yl": yl, "xh": xh, "yh": yh})),
            "layout_bbox": bbox,
            "hpwl": nets.hpwl(&cx, &cy),
            "overlap_pairs": overlap_pair_count(&xs, &ys, &ws, &hs),
            "unit": "DBU = nm",
        }
    })
}

fn usage(argv0: &str) -> String {
    format!(
        "usage: {argv0} <bbox.json> <pin.json> <sym.json> <params.json> <out.json> \
         [--set key=value]..."
    )
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let mut positional: Vec<String> = Vec::new();
    let mut overrides: Vec<String> = Vec::new();
    let mut i = 1;
    while i < args.len() {
        if args[i] == "--set" {
            if i + 1 >= args.len() {
                return Err(
                    format!("--set needs a key=value argument\n{}", usage(&args[0])).into(),
                );
            }
            overrides.push(args[i + 1].clone());
            i += 2;
        } else {
            positional.push(args[i].clone());
            i += 1;
        }
    }
    if positional.len() != 5 {
        return Err(usage(&args[0]).into());
    }

    let bbox = PathBuf::from(&positional[0]);
    let pin = PathBuf::from(&positional[1]);
    let sym = PathBuf::from(&positional[2]);
    let params_path = PathBuf::from(&positional[3]);
    let out = PathBuf::from(&positional[4]);

    let (params, _resolve_meta) =
        PlacementConfig::from_case_params_json_with_overrides(&params_path, &overrides)?;
    params.validate()?;

    let data =
        PlaceData::from_json_files_with_gr(&bbox, &pin, &sym, params.pmos_hard_expand_for_gr)?;
    let num_cells = data.num_cells;
    let num_nets = data.num_nets;
    eprintln!(
        "[abstract] devices={num_cells} pins={} nets={num_nets} sym_pairs={}",
        data.num_pins, data.num_sym_pairs
    );

    // Everything the summary needs, taken before `data` and `params` move into the
    // placer. None of it is an optimizer input; it is what the run is reported with.
    let gr_reservation = params.pmos_hard_expand_for_gr;
    let spacing_floor = params.tail_hard_inflation_floor;
    let sym_index = sym_membership(&data, num_cells);
    let nets = NetTopology::from_data(&data);

    // Diagnostics only (`APPLACE_ABSTRACT_STAGES=1`): the symmetry relation exactly as
    // the placer will solve it, captured before `data` moves into the placer, so a stage
    // dump can measure a mirror pair's separation without re-deriving it from names.
    let sym_relation: Vec<serde_json::Value> = (0..data.num_sym_pairs)
        .map(|p| {
            let a = data.sym_pairs[2 * p] as usize;
            let b = data.sym_pairs[2 * p + 1] as usize;
            let g = data.sym_pair_group[p] as usize;
            serde_json::json!({
                "a": a,
                "b": b,
                "group": g,
                "is_vert": data.sym_group_vertical.get(g).copied().unwrap_or(true),
            })
        })
        .collect();

    let mut placer = Placer::from_data(data, params)?;
    placer.run()?;

    let positions = placer
        .deoverlap_end_positions()
        .ok_or("placer produced no deoverlap_end (LCR) positions")?;

    // Names and sizes come back in bbox order, which is the cell index order the placer
    // uses.
    let bbox_json: serde_json::Value = serde_json::from_slice(&std::fs::read(&bbox)?)?;
    let cells = bbox_json["cells"]
        .as_array()
        .ok_or("bbox.json has no `cells` array")?;
    let names: Vec<String> = cells
        .iter()
        .map(|c| c["cell_name"].as_str().unwrap_or_default().to_string())
        .collect();

    let mut map = serde_json::Map::new();
    for (idx, name) in names.iter().enumerate() {
        let x = positions[idx];
        let y = positions[idx + num_cells];
        map.insert(name.clone(), serde_json::json!([x, y]));
    }
    if let Some(parent) = out.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&out, serde_json::to_string_pretty(&map)?)?;

    // The same placement as an outline-level layout, so it can be opened in a GDS
    // viewer. GDS coordinates are integers, so the placer's f64 positions are rounded
    // here; `placement.json` above keeps the unrounded values.
    let devices: Vec<AbstractPlacedDevice> = cells
        .iter()
        .enumerate()
        .map(|(idx, c)| {
            let b = &c["bbox"];
            let size = |lo: &str, hi: &str| -> i32 {
                (b[hi].as_f64().unwrap_or(0.0) - b[lo].as_f64().unwrap_or(0.0))
                    .abs()
                    .round() as i32
            };
            AbstractPlacedDevice {
                name: names[idx].clone(),
                x: positions[idx].round() as i32,
                y: positions[idx + num_cells].round() as i32,
                w: size("xl", "xh"),
                h: size("yl", "yh"),
                is_pmos: c["isPmos"].as_bool().unwrap_or(false),
            }
        })
        .collect();
    let region = bbox_json.get("boundary").and_then(|r| {
        let g = |k: &str| r[k].as_f64().map(|v| v.round() as i32);
        Some((g("xl")?, g("yl")?, g("xh")?, g("yh")?))
    });
    let gr_expand = gr_reservation.round() as i32;
    let out_gds = out.with_extension("gds");
    write_abstract_placement_gds(&devices, region, gr_expand, &out_gds)?;
    eprintln!("[abstract] layout -> {}", out_gds.display());

    // The same placement as data. Written next to `placement.json` as
    // `placement_summary.json`.
    let summary_path = out.with_file_name(format!(
        "{}_summary.json",
        out.file_stem().unwrap_or_default().to_string_lossy()
    ));
    let summary = build_summary(
        &names,
        cells,
        positions.as_slice().ok_or("positions are not contiguous")?,
        num_cells,
        &sym_index,
        &nets,
        region,
        gr_reservation,
        spacing_floor,
    );
    std::fs::write(
        &summary_path,
        serde_json::to_string_pretty(&summary)? + "\n",
    )?;
    eprintln!("[abstract] summary -> {}", summary_path.display());

    // Diagnostics only, off by default: the three multi-phase checkpoints side by side, so a
    // degenerate placement can be traced to the stage that produced it (which mirror
    // pair had already collapsed onto the axis at base_end, what the tail did to it,
    // what LCR did to it). Writing it changes nothing about the published outputs.
    if std::env::var("APPLACE_ABSTRACT_STAGES").ok().as_deref() == Some("1") {
        let dump = serde_json::json!({
            "schema": "applace.abstract_stage_dump/1",
            "num_cells": num_cells,
            "names": names,
            "w": cells.iter().map(|c| (c["bbox"]["xh"].as_f64().unwrap_or(0.0)
                - c["bbox"]["xl"].as_f64().unwrap_or(0.0)).abs()).collect::<Vec<f64>>(),
            "h": cells.iter().map(|c| (c["bbox"]["yh"].as_f64().unwrap_or(0.0)
                - c["bbox"]["yl"].as_f64().unwrap_or(0.0)).abs()).collect::<Vec<f64>>(),
            "sym_pairs": sym_relation,
            "region": region.map(|(xl, yl, xh, yh)| serde_json::json!(
                {"xl": xl, "yl": yl, "xh": xh, "yh": yh})),
            // Positions are flat [x0..x_{n-1}, y0..y_{n-1}], the placer's own layout.
            "base_end": placer.base_end_positions().map(|v| v.to_vec()),
            "tail_end": placer.tail_end_positions().map(|v| v.to_vec()),
            "deoverlap_end": placer.deoverlap_end_positions().map(|v| v.to_vec()),
            "deoverlap_summary": placer.deoverlap_summary_json(),
            "deoverlap_components_before": placer.deoverlap_components_before(),
            "deoverlap_components_after": placer.deoverlap_components_after(),
            "deoverlap_unresolved_cells": placer.deoverlap_unresolved_cells(),
            "lcr_retry": placer.lcr_retry_summary_json(),
        });
        let stages_path = out.with_file_name(format!(
            "{}_stages.json",
            out.file_stem().unwrap_or_default().to_string_lossy()
        ));
        std::fs::write(&stages_path, serde_json::to_string(&dump)? + "\n")?;
        eprintln!("[abstract] stage dump -> {}", stages_path.display());
    }

    if let Some(s) = placer.lcr_retry_summary_json() {
        let retry = out.with_file_name(format!(
            "{}.lcr_retry.json",
            out.file_stem().unwrap_or_default().to_string_lossy()
        ));
        std::fs::write(&retry, serde_json::to_string_pretty(&s)?)?;
        eprintln!("[abstract] LCR retry fired; summary -> {}", retry.display());
    }

    eprintln!("[abstract] placement -> {}", out.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hpwl_is_over_device_centres_and_ignores_one_device_nets() {
        let nets = NetTopology {
            nets: vec![vec![0, 1], vec![2], vec![0, 1, 2]],
        };
        let cx = [0.0, 30.0, 10.0];
        let cy = [0.0, 40.0, 100.0];
        // net 0: (30-0) + (40-0) = 70; net 1: skipped; net 2: 30 + 100 = 130.
        assert_eq!(nets.hpwl(&cx, &cy), 200.0);
    }

    #[test]
    fn overlap_count_needs_positive_area_so_abutting_is_not_an_overlap() {
        let x = [0.0, 10.0, 5.0];
        let y = [0.0, 0.0, 0.0];
        let w = [10.0, 10.0, 10.0];
        let h = [10.0, 10.0, 10.0];
        // 0 and 1 touch edge-to-edge -> not an overlap; 2 straddles both -> two pairs.
        assert_eq!(overlap_pair_count(&x, &y, &w, &h), 2);
    }
}
