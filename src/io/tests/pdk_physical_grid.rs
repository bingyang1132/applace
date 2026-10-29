//! Regression: the deck declares whether its `grid` is a physical manufacturing
//! grid, and that declaration has three states -- declared true, declared false,
//! and absent. Absent means physical, so a deck written without the field keeps
//! the conservative behaviour.
//!
//! This goes through the real deck parser. The snapping rule itself is unit-tested
//! in `core::mfg_grid`.

use applace_io::{parse_pdk, PdkRules};

/// The deck shipped with this repository, which declares `physical_grid: false`.
const EXAMPLE_DECK: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../pdk/example_lef.json");

/// Rewrite `_meta` and deserialize again, to get the same deck in each of the three
/// declaration states. `None` means `_meta` is absent entirely -- the most
/// conservative state, and what a hand-written deck looks like.
fn deck_with(physical_grid: Option<bool>) -> PdkRules {
    let mut v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(EXAMPLE_DECK).expect("read example deck"))
            .expect("example deck is json");
    match physical_grid {
        None => {
            v.as_object_mut()
                .expect("deck is an object")
                .remove("_meta");
        }
        Some(b) => {
            v["_meta"] = serde_json::json!({ "physical_grid": b });
        }
    }
    serde_json::from_value(v).expect("deck deserializes")
}

#[test]
fn physical_grid_declaration_is_tri_state() {
    assert!(
        !deck_with(Some(false)).physical_grid(),
        "declared false must disable snapping"
    );
    assert!(
        deck_with(Some(true)).physical_grid(),
        "declared true must enable snapping"
    );
    assert!(
        deck_with(None).physical_grid(),
        "an absent declaration must default to physical"
    );
}

#[test]
fn shipped_deck_declares_non_physical_grid() {
    let shipped = parse_pdk(EXAMPLE_DECK).expect("parse shipped example deck");
    assert!(
        !shipped.physical_grid(),
        "the shipped deck carries placeholder values, so its grid is not a real \
                manufacturing grid and it must say so"
    );
}
