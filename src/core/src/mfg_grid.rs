//! Manufacturing-grid snapping.
//!
//! Background: off-grid geometry is a common sign-off failure, and in this flow it
//! came from the geometry this flow produces itself, starting with the device
//! origins that placement writes. The fix snaps on the producing side; it does
//! not snap the layout afterwards.
//!
//! The grid value always comes from the PDK `grid` field (DBU = nm) and is **never
//! hardcoded**: this module only accepts the `grid` its caller passes in, and any
//! `grid <= 1` means "do not snap", so with the master switch off, passing 0 reproduces
//! the pre-fix behaviour byte for byte.
//!
//! **Snapping is only meaningful against a physical manufacturing grid.** If a deck's
//! `grid` is not a real manufacturing grid — the placeholder values in
//! `pdk/example_lef.json` are exactly that case: after the confidentiality pass they are
//! merely non-disclosing fakes, coarser than the real values, and device footprints are
//! not integer multiples of them — then snapping the coordinates neither improves
//! manufacturability nor survives the `apply_positions_checked` self-check, whose
//! precondition is that device width and height are integer multiples of the grid. A
//! deck therefore declares its own nature through `_meta.physical_grid`:
//!
//! * field absent = **physical** (conservative: real decks and external users' decks get
//!   snapping without any change);
//! * `physical_grid: false` → snapping is switched off entirely, exactly as with
//!   `APPLACE_MFG_GRID_SNAP=0`;
//! * the environment variable `APPLACE_MFG_GRID_SNAP` has **the highest priority** and
//!   can force either direction. Forcing it on while the precondition does not hold
//!   still produces a hard error — that is the self-check doing its job.

use crate::geometry::Box as GeoBox;
use crate::types::Loc;
use std::sync::OnceLock;

/// The three states of the master switch `APPLACE_MFG_GRID_SNAP`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapSwitch {
    /// Not set — the deck's `_meta.physical_grid` decides.
    Unset,
    /// Explicitly forced on (any value outside the off-word list counts, preserving the historical behaviour).
    ForceOn,
    /// Explicitly forced off (`0`/`false`/`off`/`no`).
    ForceOff,
}

/// Pure-function switch parsing, so it can be unit tested (`None` = the environment variable is not set).
fn parse_snap_switch(v: Option<&str>) -> SnapSwitch {
    match v {
        None => SnapSwitch::Unset,
        Some(v) => {
            if matches!(
                v.trim().to_ascii_lowercase().as_str(),
                "0" | "false" | "off" | "no"
            ) {
                SnapSwitch::ForceOff
            } else {
                SnapSwitch::ForceOn
            }
        }
    }
}

/// The master switch state of this process. The environment variable is read exactly once per process (determinism: the same process always gives the same answer).
pub fn snap_switch() -> SnapSwitch {
    static SWITCH: OnceLock<SnapSwitch> = OnceLock::new();
    *SWITCH
        .get_or_init(|| parse_snap_switch(std::env::var("APPLACE_MFG_GRID_SNAP").ok().as_deref()))
}

/// Three-state switch × deck declaration → the snap step actually in effect (0 = no snapping). A pure function, unit-testable.
pub fn resolve_snap_grid(pdk_grid_dbu: Loc, physical_grid: bool, sw: SnapSwitch) -> Loc {
    match sw {
        SnapSwitch::ForceOff => 0,
        SnapSwitch::ForceOn => pdk_grid_dbu.max(0),
        SnapSwitch::Unset => {
            if physical_grid {
                pdk_grid_dbu.max(0)
            } else {
                0
            }
        }
    }
}

/// Whether the master switch lets snapping take effect (the reading used when `physical_grid` defaults to true).
pub fn mfg_grid_snap_enabled() -> bool {
    snap_switch() != SnapSwitch::ForceOff
}

/// Converts the PDK grid into the snap step actually in effect, consuming the deck's
/// `_meta.physical_grid` declaration (`false` = this deck's `grid` is not a physical manufacturing grid).
pub fn effective_snap_grid_declared(pdk_grid_dbu: Loc, physical_grid: bool) -> Loc {
    resolve_snap_grid(pdk_grid_dbu, physical_grid, snap_switch())
}

/// The "deck declares nothing" shortcut for `effective_snap_grid_declared` (= treat the grid as physical).
pub fn effective_snap_grid(pdk_grid_dbu: Loc) -> Loc {
    effective_snap_grid_declared(pdk_grid_dbu, true)
}

/// Snaps to the nearest grid point (round-half-**up**: exactly half a step goes towards +∞).
///
/// `grid <= 1` → identity. The displacement is ≤ grid/2, the smallest of the three
/// rounding modes used here.
///
/// **Key invariant**: `snap_nearest(v) - v` depends only on `v mod grid`. So for two
/// objects whose sizes are integer multiples of the grid, a gap that was already an
/// integer multiple of the grid is **byte-identical** after snapping (abutting devices
/// and arrays are not pulled apart); otherwise the snapped gap is still an integer
/// multiple of the grid and `> gap - grid`, so a non-negative gap can never become
/// negative — snapping cannot create a new overlap (precondition: device width and
/// height are integer multiples of the grid, which the caller's
/// `apply_positions_checked` self-check enforces).
///
/// Half a step goes towards +∞ rather than away from 0 precisely because of this
/// invariant: rounding away from 0 gives different displacements for the same residue
/// on either side of 0, which on an even grid squeezes abutting devices into an overlap.
pub fn snap_nearest(v: Loc, grid: Loc) -> Loc {
    if grid <= 1 {
        return v;
    }
    let g = grid as i64;
    ((v as i64 + g / 2).div_euclid(g) * g) as Loc
}

/// Snaps down (towards -∞).
pub fn snap_floor(v: Loc, grid: Loc) -> Loc {
    if grid <= 1 {
        return v;
    }
    ((v as i64).div_euclid(grid as i64) * grid as i64) as Loc
}

/// Snaps up (towards +∞).
pub fn snap_ceil(v: Loc, grid: Loc) -> Loc {
    if grid <= 1 {
        return v;
    }
    (-((-(v as i64)).div_euclid(grid as i64)) * grid as i64) as Loc
}

/// Snaps a rectangle **outward**: low edges down, high edges up.
///
/// Used on the geometry-producing side: the box can only
/// grow, never shrink, so any min-width / min-area / enclosure / coverage relation that
/// held before still holds after snapping. The price is that the gap to a neighbour can
/// shrink by at most `grid - 1`, which the evaluator accounts for honestly.
pub fn snap_box_outward(b: GeoBox<Loc>, grid: Loc) -> GeoBox<Loc> {
    if grid <= 1 {
        return b;
    }
    GeoBox::new(
        snap_floor(b.xl, grid),
        snap_floor(b.yl, grid),
        snap_ceil(b.xh, grid),
        snap_ceil(b.yh, grid),
    )
}

/// Whether the value is already on the grid (always true when `grid <= 1`).
pub fn on_grid(v: Loc, grid: Loc) -> bool {
    grid <= 1 || v.rem_euclid(grid) == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The grid step used in the unit tests is a **synthetic value**, unrelated to the real value of any PDK (real vendor values never enter the repository).
    const G: Loc = 8;

    /// The three states of switch parsing (including the historical behaviour that any value which is not an off word counts as "forced on").
    #[test]
    fn snap_switch_is_tri_state() {
        assert_eq!(parse_snap_switch(None), SnapSwitch::Unset);
        for v in ["0", "false", "off", "no", "  OFF  ", "False"] {
            assert_eq!(parse_snap_switch(Some(v)), SnapSwitch::ForceOff, "v={v}");
        }
        for v in ["1", "true", "on", "yes", "", "whatever"] {
            assert_eq!(parse_snap_switch(Some(v)), SnapSwitch::ForceOn, "v={v}");
        }
    }

    /// The full truth table of deck declaration × three-state switch.
    ///
    /// * declaration absent (= `true`, see `PdkRules::physical_grid`) → snapping as usual;
    /// * `physical_grid: false` → snapping switched off entirely, same as `APPLACE_MFG_GRID_SNAP=0`;
    /// * the environment variable has the highest priority: it can override the declaration in either direction.
    #[test]
    fn declaration_and_env_switch_truth_table() {
        // Environment variable unset: the deck decides.
        assert_eq!(resolve_snap_grid(G, true, SnapSwitch::Unset), G);
        assert_eq!(resolve_snap_grid(G, false, SnapSwitch::Unset), 0);
        // Environment variable forces off: neither declaration snaps.
        assert_eq!(resolve_snap_grid(G, true, SnapSwitch::ForceOff), 0);
        assert_eq!(resolve_snap_grid(G, false, SnapSwitch::ForceOff), 0);
        // Environment variable forces on: overrides `physical_grid: false` (when the precondition does not hold, the caller's self-check raises a hard error).
        assert_eq!(resolve_snap_grid(G, true, SnapSwitch::ForceOn), G);
        assert_eq!(resolve_snap_grid(G, false, SnapSwitch::ForceOn), G);
        // A negative grid is always clamped to 0 (= no snapping); a negative step must never reach the rounding functions.
        assert_eq!(resolve_snap_grid(-4, true, SnapSwitch::Unset), 0);
        assert_eq!(resolve_snap_grid(-4, false, SnapSwitch::ForceOn), 0);
    }

    #[test]
    fn grid_le_one_is_identity() {
        for v in [-7, -1, 0, 1, 13] {
            assert_eq!(snap_nearest(v, 0), v);
            assert_eq!(snap_nearest(v, 1), v);
            assert_eq!(snap_floor(v, 0), v);
            assert_eq!(snap_ceil(v, 1), v);
        }
        let b = GeoBox::new(1, 2, 3, 4);
        assert_eq!(snap_box_outward(b, 0), b);
    }

    #[test]
    fn nearest_rounds_half_up_and_delta_depends_only_on_residue() {
        assert_eq!(snap_nearest(0, G), 0);
        assert_eq!(snap_nearest(3, G), 0);
        assert_eq!(snap_nearest(4, G), G); // exactly half a step rounds up
        assert_eq!(snap_nearest(11, G), G);
        assert_eq!(snap_nearest(12, G), 2 * G);
        assert_eq!(snap_nearest(-3, G), 0);
        assert_eq!(snap_nearest(-4, G), 0); // half a step goes towards +∞, not away from 0
        assert_eq!(snap_nearest(-5, G), -G);
        assert_eq!(snap_nearest(-12, G), -G);
        for v in -200..200 {
            assert!(on_grid(snap_nearest(v, G), G));
            assert!((snap_nearest(v, G) - v).abs() <= G / 2);
            // The displacement depends only on the residue — this is the entire basis for "snapping adds no overlap".
            assert_eq!(
                snap_nearest(v, G) - v,
                snap_nearest(v + 7 * G, G) - (v + 7 * G)
            );
        }
    }

    #[test]
    fn floor_ceil_bracket_the_value() {
        for v in -200..200 {
            let f = snap_floor(v, G);
            let c = snap_ceil(v, G);
            assert!(on_grid(f, G) && on_grid(c, G));
            assert!(f <= v && v <= c, "v={v} f={f} c={c}");
            assert!(v - f < G && c - v < G);
        }
    }

    #[test]
    fn outward_snap_never_shrinks() {
        for xl in -30..30 {
            for w in 1..17 {
                let b = GeoBox::new(xl, xl, xl + w, xl + w);
                let s = snap_box_outward(b, G);
                assert!(s.xl <= b.xl && s.yl <= b.yl);
                assert!(s.xh >= b.xh && s.yh >= b.yh);
                assert!(on_grid(s.xl, G) && on_grid(s.xh, G));
                assert!(on_grid(s.yl, G) && on_grid(s.yh, G));
            }
        }
    }

    /// Snapping cannot create a new overlap — the mechanized form of the core argument.
    ///
    /// Assume both objects have widths that are integer multiples of the grid. If the
    /// original gap is `>= 0`, the gap after `snap_nearest` is still `>= 0` (and still an
    /// integer multiple of the grid); if the original gap was already an integer multiple
    /// of the grid, the gap is **byte-identical** (abutting devices and arrays are not
    /// pulled apart).
    #[test]
    fn nearest_snap_cannot_create_overlap_for_grid_sized_objects() {
        for w1 in [G, 2 * G, 5 * G, 13 * G] {
            for x1 in -37..37 {
                for gap in 0..(3 * G) {
                    let x2 = x1 + w1 + gap;
                    let a = snap_nearest(x1, G);
                    let b = snap_nearest(x2, G);
                    let new_gap = b - (a + w1);
                    assert!(
                        new_gap >= 0,
                        "x1={x1} w1={w1} gap={gap} -> new_gap={new_gap}"
                    );
                    assert!(on_grid(new_gap, G));
                    if gap % G == 0 {
                        assert_eq!(
                            new_gap, gap,
                            "a grid-multiple gap must be preserved byte for byte"
                        );
                    }
                }
            }
        }
    }
}
