//! Write-back consistency test for the pin shapes produced by parse_pcell:
//! - the union bbox of `shapes` in pin.json must equal the existing `shape` field;
//! - DB `Pin::shapes` and pin.json `shapes` must agree in count;
//! - the `layers` field equals the shape layer numbers, sorted and deduplicated.
//!
//! Reuses the example design inputs. If the normalized PCell directory already exists,
//! parse_pcell reuses it, which avoids contending with other tests running in parallel.

use applace_core::CircuitDb;
use applace_io::{parse_hierarchical_netlist, parse_pcell};
use std::path::PathBuf;

fn project_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

#[test]
#[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
fn pin_json_shapes_union_matches_shape_bbox() {
    let root = project_root();
    let netlist = root.join("benchmark/ota2/ota2.sp");
    let pcelldir = root.join("benchmark/ota2/custom");
    // The deck that sits next to the design, which is the one the device flow
    // picks (run/_common.sh) and the only one that names this design's device
    // models. Like the design itself, it is not distributed, so a clone skips.
    let pdk = root.join("benchmark/ota2/lef.json");
    if !netlist.exists() || !pcelldir.exists() || !pdk.exists() {
        eprintln!("benchmark inputs missing; skipping");
        return;
    }

    let mut db = CircuitDb::new();
    parse_hierarchical_netlist(&netlist, &mut db).expect("netlist parses");
    let out = parse_pcell(&pcelldir, None::<&std::path::Path>, Some(&pdk), &mut db)
        .expect("parse_pcell succeeds");

    let mut checked = 0usize;
    for pin in &out.pins.pins {
        if pin.shapes.is_empty() {
            continue;
        }
        let mut xl = i32::MAX;
        let mut yl = i32::MAX;
        let mut xh = i32::MIN;
        let mut yh = i32::MIN;
        let mut layers: Vec<i32> = Vec::new();
        for s in &pin.shapes {
            xl = xl.min(s.shape.xl);
            yl = yl.min(s.shape.yl);
            xh = xh.max(s.shape.xh);
            yh = yh.max(s.shape.yh);
            layers.push(s.layer);
        }
        layers.sort_unstable();
        layers.dedup();
        assert_eq!(
            (xl, yl, xh, yh),
            (pin.shape.xl, pin.shape.yl, pin.shape.xh, pin.shape.yh),
            "shapes union bbox mismatch for pin {}",
            pin.name
        );
        assert_eq!(layers, pin.layers, "layers mismatch for pin {}", pin.name);
        // offset = center of the union bbox (existing semantics kept unchanged).
        assert_eq!(
            (pin.offset_x, pin.offset_y),
            ((xl + xh) / 2, (yl + yh) / 2),
            "offset mismatch for pin {}",
            pin.name
        );
        checked += 1;
    }
    assert!(
        checked > 0,
        "no pin with shapes found in the example design's pin.json"
    );

    // DB side: at least one pin carries shapes, and for every pin that does, its layer_ids
    // agree with the layers of those shapes.
    let mut db_pins_with_shapes = 0usize;
    for pidx in 0..db.num_pins() {
        let Some(pin) = db.pin(pidx as i32) else {
            continue;
        };
        if pin.shapes().is_empty() {
            continue;
        }
        db_pins_with_shapes += 1;
        let mut layers: Vec<i32> = pin.shapes().iter().map(|s| s.gds_layer).collect();
        layers.sort_unstable();
        layers.dedup();
        assert_eq!(layers, pin.layer_ids(), "db pin {} layers", pin.name());
    }
    assert!(db_pins_with_shapes > 0, "no db pin carries shapes");

    // Regression guard: **no pin may be a pile of copies of the same rectangle**.
    //
    // The old bug (the MOSFET branch of `compute_pin_shapes` was missing its
    // `HashSet<usize>` guard): one horizontal M1 gate bar overlaps **every single** gate CO
    // of that device, so it was pushed once per CO -- one device ended up with
    // 319 shapes and **only 1** distinct geometry. Everything downstream iterates over
    // shapes: the same gap was reported N times, `pin_metal_area` inflated
    // several-fold, and the pin enumeration did N times the useless work.
    //
    // The criterion is "collapses to a single rectangle" rather than "no duplicates",
    // because **another kind** of duplicate also exists: in the source GDS of a few PCells
    // there are distinct polygons that share a bbox (one MOM capacitor, for instance,
    // has 767 polygons and 729 distinct bboxes), and after `bbox_from_points` they can
    // no longer be told apart. That kind has a ratio of about 1.09, originates in the source
    // data, and is out of scope for this fix.
    for pidx in 0..db.num_pins() {
        let Some(pin) = db.pin(pidx as i32) else {
            continue;
        };
        let total = pin.shapes().len();
        let distinct = pin
            .shapes()
            .iter()
            .map(|s| (s.gds_layer, s.rect.xl, s.rect.yl, s.rect.xh, s.rect.yh))
            .collect::<std::collections::HashSet<_>>()
            .len();
        assert!(
            distinct > 1 || total <= 1,
            "pin {} is {} copies of a single rectangle (regression)",
            pin.name(),
            total
        );
    }
}
