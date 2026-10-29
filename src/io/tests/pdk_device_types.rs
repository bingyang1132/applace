//! `device_types` — "the deck declares which model names are which device class".
//!
//! Background: the answer used to be derived from the model string itself, by
//! testing it against a fixed list of spellings. Anyone bringing their own PDK --
//! sky130's `sky130_fd_pr__pfet_01v8`, or any deck whose names were not on that
//! list -- silently got `isPmos = false` for every device, the guard-ring
//! reservation (`pmosHardExpandForGr`) was reserved for nothing, and nothing said
//! a word.
//!
//! The rule in this build: the deck declares it, a deck that declares nothing is
//! refused, and a declaration that does not cover the netlist is a hard error
//! rather than a quiet `false`. This file is the regression for all three states,
//! over the real `parse_pdk` path and the real shipped deck. The matching rules
//! themselves are unit tested in `core::device_types`.

use applace_core::{CircuitDb, DeviceClass};
use applace_io::{apply_device_types, parse_hierarchical_netlist, PdkRules};

/// The placeholder deck that ships as `pdk/example_lef.json`.
const EXAMPLE_DECK: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../pdk/example_lef.json");

fn deck_json() -> serde_json::Value {
    serde_json::from_str(&std::fs::read_to_string(EXAMPLE_DECK).expect("read example deck"))
        .expect("example deck is json")
}

fn deck_from(v: serde_json::Value) -> PdkRules {
    serde_json::from_value(v).expect("deck parses back into PdkRules")
}

/// A one-device netlist with the given model string, parsed into a real `CircuitDb`.
fn db_with_models(models: &[&str]) -> CircuitDb {
    let mut body = String::from("topckt T A B VSS\n");
    for (i, m) in models.iter().enumerate() {
        body.push_str(&format!("M{i} (A B VSS VSS ) {m} w=1u l=40n m=1\n"));
    }
    body.push_str("ends T\n");

    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let path = std::env::temp_dir().join(format!("applace_device_types_{stamp}.sp"));
    std::fs::write(&path, body).expect("write temp netlist");
    let mut db = CircuitDb::new();
    parse_hierarchical_netlist(&path, &mut db).expect("parse netlist");
    let _ = std::fs::remove_file(&path);
    db
}

fn classes(db: &CircuitDb) -> Vec<Option<DeviceClass>> {
    let dt = db.device_types().expect("declaration installed");
    db.cells().iter().map(|c| dt.classify(c.model())).collect()
}

/// The shipped deck declares, and its declaration covers the benchmark model families.
#[test]
fn shipped_deck_declares_device_types() {
    let pdk = deck_from(deck_json());
    let dt = pdk
        .device_types()
        .expect("declaration is well-formed")
        .expect("pdk/example_lef.json must declare device_types");
    // Every model string the four benchmark netlists actually use.
    for m in [
        "pmos_example",
        "pmos_example_variant_a",
        "pmos_example_variant_b",
        "pmos_example_variant_c",
        "nmos_example",
        "nmos_example_variant_a",
        "nmos_example_variant_c",
        "nmos_example_variant_b",
        "nmos_example_variant_d",
        "cap_example",
        "cap_example_variant_a",
        "res_example",
        "res_example_variant_a",
    ] {
        assert!(
            dt.classify(m).is_some(),
            "shipped deck must classify benchmark model {m}"
        );
    }
    assert!(dt.is_pmos("pmos_example_variant_c"));
    assert!(!dt.is_pmos("nmos_example_variant_c"));
}

/// Declaration hit: the deck's names decide, including for a deck whose spellings the
/// built-in heuristic knows nothing about.
#[test]
fn a_foreign_deck_gets_its_pmos_recognised() {
    let mut v = deck_json();
    v["device_types"] = serde_json::json!({
        "pmos": ["sky130_fd_pr__pfet"],
        "nmos": ["sky130_fd_pr__nfet"],
    });
    let pdk = deck_from(v);
    let mut db = db_with_models(&["sky130_fd_pr__pfet_01v8", "sky130_fd_pr__nfet_01v8"]);
    apply_device_types(&mut db, &pdk).expect("declaration covers the netlist");
    assert_eq!(
        classes(&db),
        vec![Some(DeviceClass::Pmos), Some(DeviceClass::Nmos)]
    );
}

/// A deck that declares nothing is refused. There is no model-name heuristic in
/// this build, and answering "not a PMOS" for every device is the failure the
/// declaration exists to prevent.
#[test]
fn an_undeclared_deck_is_refused() {
    let mut v = deck_json();
    v.as_object_mut().unwrap().remove("device_types");
    let pdk = deck_from(v);
    assert!(pdk.device_types().unwrap().is_none());

    let mut db = db_with_models(&["pmos_example_variant_a", "some_unheard_of_device"]);
    let err = apply_device_types(&mut db, &pdk)
        .expect_err("an undeclared deck must not run")
        .to_string();
    assert!(err.contains("device_types"), "{err}");
    assert!(
        err.contains("INPUT_FORMAT"),
        "the message must say where to read: {err}"
    );
    assert!(db.device_types().is_none());
}

/// A section that lists no names at all is not a declaration, and is refused the
/// same way an absent one is -- rather than turning every device in the netlist
/// into an "unknown model" error, which says nothing about what to fix.
#[test]
fn an_empty_section_is_refused_like_an_absent_one() {
    let mut v = deck_json();
    v["device_types"] = serde_json::json!({ "match": "prefix" });
    let pdk = deck_from(v);
    let mut db = db_with_models(&["pmos_example_variant_a"]);
    assert!(apply_device_types(&mut db, &pdk).is_err());
    assert!(db.device_types().is_none());
}

/// The P1 fix proper: a declaring deck that does not cover a model in the netlist is a
/// hard error naming every uncovered model — never a silent `isPmos = false`.
#[test]
fn an_uncovered_model_is_a_hard_error() {
    let mut v = deck_json();
    v["device_types"] = serde_json::json!({ "pmos": ["pmos_example"], "nmos": ["nmos_example"] });
    let pdk = deck_from(v);
    let mut db = db_with_models(&[
        "pmos_example_variant_a",
        "xfet_weird",
        "nmos_example",
        "another_unknown",
    ]);
    let err = apply_device_types(&mut db, &pdk)
        .expect_err("an uncovered model must not pass silently")
        .to_string();
    assert!(err.contains("another_unknown"), "{err}");
    assert!(err.contains("xfet_weird"), "{err}");
    assert!(
        !err.contains("pmos_example_variant_a"),
        "covered models must not be listed: {err}"
    );
    // Sorted, so the message is identical run to run.
    assert!(
        err.find("another_unknown").unwrap() < err.find("xfet_weird").unwrap(),
        "{err}"
    );
}

/// `other` is the escape hatch: a device deliberately outside the four classes is
/// declared, so it neither errors nor pretends to be a transistor.
#[test]
fn other_covers_a_device_that_is_none_of_the_four() {
    let mut v = deck_json();
    v["device_types"] = serde_json::json!({ "pmos": ["pmos_example"], "other": ["diode"] });
    let pdk = deck_from(v);
    let mut db = db_with_models(&["pmos_example", "diode_pw"]);
    apply_device_types(&mut db, &pdk).expect("`other` covers it");
    assert_eq!(
        classes(&db),
        vec![Some(DeviceClass::Pmos), Some(DeviceClass::Other)]
    );
}

/// A self-contradictory declaration is refused at load; picking a winner would make the
/// classification depend on nothing the deck author can see.
#[test]
fn a_model_claimed_by_two_classes_is_refused() {
    let mut v = deck_json();
    v["device_types"] = serde_json::json!({ "pmos": ["pmos_example"], "nmos": ["PMOS_EXAMPLE"] });
    let pdk = deck_from(v);
    let err = pdk.device_types().unwrap_err();
    assert!(err.contains("pmos") && err.contains("nmos"), "{err}");

    let mut db = db_with_models(&["pmos_example"]);
    assert!(apply_device_types(&mut db, &pdk).is_err());
}

/// `"match": "exact"` is honoured end to end, so a deck that wants per-flavour control
/// can have it.
#[test]
fn exact_match_mode_is_honoured() {
    let mut v = deck_json();
    v["device_types"] = serde_json::json!({ "match": "exact", "pmos": ["pmos_example"] });
    let pdk = deck_from(v);
    let mut db = db_with_models(&["pmos_example_variant_a"]);
    let err = apply_device_types(&mut db, &pdk)
        .expect_err("exact mode must not match a flavour")
        .to_string();
    assert!(err.contains("pmos_example_variant_a"), "{err}");

    let mut db = db_with_models(&["pmos_example"]);
    apply_device_types(&mut db, &pdk).expect("the exact name matches");
    assert_eq!(classes(&db), vec![Some(DeviceClass::Pmos)]);
}
