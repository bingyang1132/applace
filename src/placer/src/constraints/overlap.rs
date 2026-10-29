//! The overlap penalty.
//!
//! Two devices that sit on top of each other cost the area they share:
//!
//! ```text
//! overlap(i, j) = max(0, min(xi + wi, xj + wj) - max(xi, xj))
//!               * max(0, min(yi + hi, yj + hj) - max(yi, yj))
//! ```
//!
//! which is exactly the quantity the placer wants at zero, and exactly the quantity it
//! cannot differentiate: the `min`, the `max` and the clamp all have corners. Each axis
//! is therefore evaluated through [`smooth_overlap_extent`], a log-sum-exp whose radius
//! `gamma` the controller shrinks as the placement settles -- smooth enough to descend
//! early, sharp enough to mean the real area by the end.
//!
//! The penalty is charged on each device's keep-out outline, not its drawn outline: see
//! [`effective_lower_and_size`].

use crate::data::PlaceData;
use crate::error::PlaceResult;
use ndarray::Array1;

/// One device's extent along one axis, as the penalty sees it.
///
/// The optimization variable is the lower corner of the device's drawn outline. What
/// must not touch its neighbour is the keep-out outline: `ovl_size` scaled by `inf` and
/// grown by `halo` on both sides. That box stays centred on the drawn one, so its lower
/// edge sits half the difference further out. The `max(1e-12)` keeps a degenerate
/// device from collapsing the log-sum-exp.
#[inline]
fn effective_lower_and_size(
    anchor_ll: f64,
    real_size: f64,
    ovl_size: f64,
    inf: f64,
    halo: f64,
) -> (f64, f64) {
    let eff = (ovl_size * inf + 2.0 * halo).max(1e-12);
    let delta = 0.5 * (eff - real_size.max(0.0));
    (anchor_ll - delta, eff)
}

/// How far two intervals overlap along one axis, before smoothing.
///
/// Intervals `[lo_a, lo_a + size_a]` and `[lo_b, lo_b + size_b]` share
/// `min(hi_a - lo_b, hi_b - lo_a, size_a, size_b)`: either one interval's far edge
/// stops short inside the other, or one interval lies wholly within the other and its
/// own length is the answer. Not positive means they do not meet at all.
#[inline]
fn overlap_bound(lo_a: f64, size_a: f64, lo_b: f64, size_b: f64) -> f64 {
    (lo_a + size_a - lo_b)
        .min(lo_b + size_b - lo_a)
        .min(size_a)
        .min(size_b)
}

/// The same four candidates, folded into something differentiable.
///
/// A hard `min` has no derivative where two candidates cross, so the four are combined
/// with a log-sum-exp of radius `gamma`: the smallest candidate, less `gamma * ln` of
/// the summed exponentials. It approaches the true minimum as `gamma` goes to zero, and
/// it is smooth everywhere in between.
///
/// Shifting every candidate by the smallest before exponentiating is what keeps the
/// arithmetic in range: each shifted exponent is then at most zero, the smallest
/// candidate contributes exactly one, and the sum stays inside `[1, 4]` however far
/// apart the intervals are.
///
/// Only two of the four candidates move when a lower corner moves -- an interval's own
/// length does not -- so those two weights come back with the sum, which is all the
/// caller needs to differentiate the extent with respect to `lo_a` and `lo_b`.
#[derive(Clone, Copy)]
struct AxisOverlap {
    /// The smoothed overlap extent.
    extent: f64,
    /// Weight of the candidate that moves with `lo_a`.
    weight_a: f64,
    /// Weight of the candidate that moves with `lo_b`.
    weight_b: f64,
    /// All four weights summed: the derivative's denominator.
    weight_sum: f64,
}

/// A pair that cannot overlap on this axis: no extent, and no gradient to give.
const NO_AXIS_OVERLAP: AxisOverlap = AxisOverlap {
    extent: 0.0,
    weight_a: 0.0,
    weight_b: 0.0,
    weight_sum: 1.0,
};

#[inline]
fn smooth_overlap_extent(
    lo_a: f64,
    size_a: f64,
    lo_b: f64,
    size_b: f64,
    gamma: f64,
) -> AxisOverlap {
    let sharpness = 1.0 / gamma;
    let reach_a = lo_a + size_a - lo_b;
    let reach_b = lo_b + size_b - lo_a;
    let bound = overlap_bound(lo_a, size_a, lo_b, size_b);
    let weight_a = (-(reach_a - bound) * sharpness).exp();
    let weight_b = (-(reach_b - bound) * sharpness).exp();
    let weight_sum = weight_a
        + weight_b
        + (-(size_a - bound) * sharpness).exp()
        + (-(size_b - bound) * sharpness).exp();
    AxisOverlap {
        extent: bound - gamma * weight_sum.ln(),
        weight_a,
        weight_b,
        weight_sum,
    }
}

/// Total smoothed overlap over every declared pair, with no inflation and no halo.
///
/// # Arguments
///
/// - `pos`: the optimization variables, laid out `[x0..x_{N-1}, y0..y_{N-1}]`
/// - `data`: the placement problem
/// - `gamma`: the log-sum-exp radius
/// - `grad`: receives the gradient; accumulated into, not cleared
pub fn compute_overlap(
    pos: &Array1<f64>,
    data: &PlaceData,
    gamma: f64,
    grad: &mut Array1<f64>,
) -> PlaceResult<f64> {
    compute_overlap_with_inflation_and_halo(pos, data, gamma, grad, None, None)
}

/// As [`compute_overlap`], with a per-device keep-out scale factor.
pub fn compute_overlap_with_inflation(
    pos: &Array1<f64>,
    data: &PlaceData,
    gamma: f64,
    grad: &mut Array1<f64>,
    inflation: Option<&[f64]>,
) -> PlaceResult<f64> {
    compute_overlap_with_inflation_and_halo(pos, data, gamma, grad, inflation, None)
}

/// As [`compute_overlap`], with a per-device keep-out scale factor and a per-device
/// margin added on every side.
pub fn compute_overlap_with_inflation_and_halo(
    pos: &Array1<f64>,
    data: &PlaceData,
    gamma: f64,
    grad: &mut Array1<f64>,
    inflation: Option<&[f64]>,
    halo: Option<&[f64]>,
) -> PlaceResult<f64> {
    let num_cells = data.num_cells;
    let num_pairs = data.num_overlap_pairs;

    let mut total_overlap = 0.0;

    // One smoothed extent per pair per axis, kept for the gradient pass below.
    let mut along_x = vec![NO_AXIS_OVERLAP; num_pairs];
    let mut along_y = vec![NO_AXIS_OVERLAP; num_pairs];

    for i in 0..num_pairs {
        let idx_a = data.overlap_pairs[2 * i] as usize;
        let idx_b = data.overlap_pairs[2 * i + 1] as usize;

        if idx_a >= num_cells || idx_b >= num_cells {
            continue;
        }

        let inf_a = inflation.and_then(|f| f.get(idx_a)).copied().unwrap_or(1.0);
        let inf_b = inflation.and_then(|f| f.get(idx_b)).copied().unwrap_or(1.0);
        let halo_a = halo.and_then(|h| h.get(idx_a)).copied().unwrap_or(0.0);
        let halo_b = halo.and_then(|h| h.get(idx_b)).copied().unwrap_or(0.0);

        let (left_a, width_a) = effective_lower_and_size(
            pos[idx_a],
            data.cell_widths[idx_a],
            data.cell_widths_keepout[idx_a],
            inf_a,
            halo_a,
        );
        let (bottom_a, height_a) = effective_lower_and_size(
            pos[idx_a + num_cells],
            data.cell_heights[idx_a],
            data.cell_heights_keepout[idx_a],
            inf_a,
            halo_a,
        );
        let (left_b, width_b) = effective_lower_and_size(
            pos[idx_b],
            data.cell_widths[idx_b],
            data.cell_widths_keepout[idx_b],
            inf_b,
            halo_b,
        );
        let (bottom_b, height_b) = effective_lower_and_size(
            pos[idx_b + num_cells],
            data.cell_heights[idx_b],
            data.cell_heights_keepout[idx_b],
            inf_b,
            halo_b,
        );

        // The bail-out is on the unsmoothed bound, not on the smoothed extent: a pair
        // that cannot touch contributes nothing at all, whereas a pair that can touch
        // but currently does not contributes the (negative) smoothed extent.
        if overlap_bound(left_a, width_a, left_b, width_b) <= 0.0
            || overlap_bound(bottom_a, height_a, bottom_b, height_b) <= 0.0
        {
            continue;
        }

        let x = smooth_overlap_extent(left_a, width_a, left_b, width_b, gamma);
        let y = smooth_overlap_extent(bottom_a, height_a, bottom_b, height_b, gamma);
        along_x[i] = x;
        along_y[i] = y;

        total_overlap += x.extent * y.extent;
    }

    for i in 0..num_pairs {
        let idx_a = data.overlap_pairs[2 * i] as usize;
        let idx_b = data.overlap_pairs[2 * i + 1] as usize;

        if idx_a >= num_cells || idx_b >= num_cells {
            continue;
        }

        let (x, y) = (along_x[i], along_y[i]);
        if x.extent <= 0.0 || y.extent <= 0.0 {
            continue;
        }

        // The area is a product, so each axis is scaled by the other's extent; the
        // two devices of a pair always get equal and opposite pushes.
        grad[idx_a] += y.extent * (x.weight_a - x.weight_b) / x.weight_sum;
        grad[idx_b] += y.extent * (x.weight_b - x.weight_a) / x.weight_sum;

        grad[idx_a + num_cells] += x.extent * (y.weight_a - y.weight_b) / y.weight_sum;
        grad[idx_b + num_cells] += x.extent * (y.weight_b - y.weight_a) / y.weight_sum;
    }

    Ok(total_overlap)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_overlap_no_overlap() {
        // Two cells that do not overlap
        let num_cells = 2;

        let data = PlaceData {
            num_cells,
            num_pins: 0,
            num_nets: 0,
            cell_widths: Array1::from_vec(vec![10.0, 10.0]),
            cell_heights: Array1::from_vec(vec![10.0, 10.0]),
            cell_areas: Array1::from_vec(vec![100.0, 100.0]),
            cell_widths_keepout: Array1::from_vec(vec![10.0, 10.0]),
            cell_heights_keepout: Array1::from_vec(vec![10.0, 10.0]),
            cell_areas_keepout: Array1::from_vec(vec![100.0, 100.0]),
            is_pmos_mask: Array1::from_vec(vec![false, false]),
            pin_offset: Array1::zeros(0),
            pin_owner_cell: Array1::zeros(0),
            io_pin_mask: Array1::from_vec(vec![]),
            net_weights: Array1::zeros(0),
            net_pin_offsets: Array1::from_vec(vec![0]),
            net_pin_index: Array1::zeros(0),
            cell_pin_offsets: Array1::from_vec(vec![0, 0, 0]),
            cell_pin_index: Array1::zeros(0),
            overlap_pairs: Array1::from_vec(vec![0, 1]),
            num_overlap_pairs: 1,
            sym_pairs: Array1::zeros(0),
            sym_pair_group: Array1::zeros(0),
            sym_group_vertical: Array1::from_vec(vec![]),
            sym_group_names: Vec::new(),
            num_sym_pairs: 0,
            num_sym_groups: 0,
            sym_self_weight: 2.0,
            sym_anchor_x: Array1::from_vec(vec![5.0; num_cells]),
            sym_anchor_y: Array1::from_vec(vec![5.0; num_cells]),
            well_group_offsets: Array1::zeros(1),
            well_group_index: Array1::zeros(0),
            cell_well_group: Array1::from_vec(vec![-1, -1]),
            num_clusters: 0,
            collinear_member_index: Array1::zeros(0),
            collinear_member_offsets: Array1::zeros(1),
            collinear_vertical: Array1::from_vec(vec![]),
            num_collinear_groups: 0,
            boundary_xl: 0.0,
            boundary_yl: 0.0,
            boundary_xh: 1000.0,
            boundary_yh: 1000.0,
        };

        // cell 0 at (0, 0), cell 1 at (20, 0) - no overlap
        let pos = Array1::from_vec(vec![0.0, 20.0, 0.0, 0.0]);
        let mut grad = Array1::zeros(4);

        let overlap = compute_overlap(&pos, &data, 1.0, &mut grad).unwrap();

        // There should be no overlap
        assert!(overlap < 0.1, "overlap = {}", overlap);
    }

    #[test]
    fn test_overlap_with_overlap() {
        // Two cells that overlap
        let num_cells = 2;

        let data = PlaceData {
            num_cells,
            num_pins: 0,
            num_nets: 0,
            cell_widths: Array1::from_vec(vec![10.0, 10.0]),
            cell_heights: Array1::from_vec(vec![10.0, 10.0]),
            cell_areas: Array1::from_vec(vec![100.0, 100.0]),
            cell_widths_keepout: Array1::from_vec(vec![10.0, 10.0]),
            cell_heights_keepout: Array1::from_vec(vec![10.0, 10.0]),
            cell_areas_keepout: Array1::from_vec(vec![100.0, 100.0]),
            is_pmos_mask: Array1::from_vec(vec![false, false]),
            pin_offset: Array1::zeros(0),
            pin_owner_cell: Array1::zeros(0),
            io_pin_mask: Array1::from_vec(vec![]),
            net_weights: Array1::zeros(0),
            net_pin_offsets: Array1::from_vec(vec![0]),
            net_pin_index: Array1::zeros(0),
            cell_pin_offsets: Array1::from_vec(vec![0, 0, 0]),
            cell_pin_index: Array1::zeros(0),
            overlap_pairs: Array1::from_vec(vec![0, 1]),
            num_overlap_pairs: 1,
            sym_pairs: Array1::zeros(0),
            sym_pair_group: Array1::zeros(0),
            sym_group_vertical: Array1::from_vec(vec![]),
            sym_group_names: Vec::new(),
            num_sym_pairs: 0,
            num_sym_groups: 0,
            sym_self_weight: 2.0,
            sym_anchor_x: Array1::from_vec(vec![5.0; num_cells]),
            sym_anchor_y: Array1::from_vec(vec![5.0; num_cells]),
            well_group_offsets: Array1::zeros(1),
            well_group_index: Array1::zeros(0),
            cell_well_group: Array1::from_vec(vec![-1, -1]),
            num_clusters: 0,
            collinear_member_index: Array1::zeros(0),
            collinear_member_offsets: Array1::zeros(1),
            collinear_vertical: Array1::from_vec(vec![]),
            num_collinear_groups: 0,
            boundary_xl: 0.0,
            boundary_yl: 0.0,
            boundary_xh: 1000.0,
            boundary_yh: 1000.0,
        };

        // cell 0 at (0, 0), cell 1 at (5, 0) - overlap 5x10
        let pos = Array1::from_vec(vec![0.0, 5.0, 0.0, 0.0]);
        let mut grad = Array1::zeros(4);

        let overlap = compute_overlap(&pos, &data, 1.0, &mut grad).unwrap();

        // There should be an overlap
        assert!(overlap > 0.0, "overlap = {}", overlap);

        // The gradient should be non-zero
        assert!(grad.iter().any(|&g| g.abs() > 0.0));
    }

    #[test]
    fn test_overlap_pmos_ovl_geometry_increases_overlap() {
        let num_cells = 2;
        let pos = Array1::from_vec(vec![0.0, 20.0, 0.0, 0.0]);
        let mut grad = Array1::zeros(4);

        let base_data = PlaceData {
            num_cells,
            num_pins: 0,
            num_nets: 0,
            cell_widths: Array1::from_vec(vec![10.0, 10.0]),
            cell_heights: Array1::from_vec(vec![10.0, 10.0]),
            cell_areas: Array1::from_vec(vec![100.0, 100.0]),
            cell_widths_keepout: Array1::from_vec(vec![10.0, 10.0]),
            cell_heights_keepout: Array1::from_vec(vec![10.0, 10.0]),
            cell_areas_keepout: Array1::from_vec(vec![100.0, 100.0]),
            is_pmos_mask: Array1::from_vec(vec![false, false]),
            pin_offset: Array1::zeros(0),
            pin_owner_cell: Array1::zeros(0),
            io_pin_mask: Array1::from_vec(vec![]),
            net_weights: Array1::zeros(0),
            net_pin_offsets: Array1::from_vec(vec![0]),
            net_pin_index: Array1::zeros(0),
            cell_pin_offsets: Array1::from_vec(vec![0, 0, 0]),
            cell_pin_index: Array1::zeros(0),
            overlap_pairs: Array1::from_vec(vec![0, 1]),
            num_overlap_pairs: 1,
            sym_pairs: Array1::zeros(0),
            sym_pair_group: Array1::zeros(0),
            sym_group_vertical: Array1::from_vec(vec![]),
            sym_group_names: Vec::new(),
            num_sym_pairs: 0,
            num_sym_groups: 0,
            sym_self_weight: 2.0,
            sym_anchor_x: Array1::from_vec(vec![5.0; num_cells]),
            sym_anchor_y: Array1::from_vec(vec![5.0; num_cells]),
            well_group_offsets: Array1::zeros(1),
            well_group_index: Array1::zeros(0),
            cell_well_group: Array1::from_vec(vec![-1, -1]),
            num_clusters: 0,
            collinear_member_index: Array1::zeros(0),
            collinear_member_offsets: Array1::zeros(1),
            collinear_vertical: Array1::from_vec(vec![]),
            num_collinear_groups: 0,
            boundary_xl: 0.0,
            boundary_yl: 0.0,
            boundary_xh: 1000.0,
            boundary_yh: 1000.0,
        };

        let base_ovl = compute_overlap(&pos, &base_data, 1.0, &mut grad).unwrap();
        grad.fill(0.0);

        let mut pmos_data = base_data.clone();
        pmos_data.cell_widths_keepout = Array1::from_vec(vec![1310.0, 10.0]);
        pmos_data.cell_heights_keepout = Array1::from_vec(vec![1310.0, 10.0]);
        pmos_data.cell_areas_keepout = Array1::from_vec(vec![1310.0 * 1310.0, 100.0]);
        pmos_data.is_pmos_mask = Array1::from_vec(vec![true, false]);

        let pmos_ovl = compute_overlap(&pos, &pmos_data, 1.0, &mut grad).unwrap();
        assert!(
            pmos_ovl >= base_ovl,
            "pmos_ovl={} base_ovl={}",
            pmos_ovl,
            base_ovl
        );
        assert!(pmos_ovl > 1.0, "pmos_ovl should be clearly positive");
    }

    #[test]
    fn test_overlap_symmetric_expand_detects_left_side_keepout() {
        let num_cells = 2;
        let mut grad = Array1::zeros(4);
        let data = PlaceData {
            num_cells,
            num_pins: 0,
            num_nets: 0,
            cell_widths: Array1::from_vec(vec![10.0, 10.0]),
            cell_heights: Array1::from_vec(vec![10.0, 10.0]),
            cell_areas: Array1::from_vec(vec![100.0, 100.0]),
            // cell0 is PMOS-expanded geometry (10 + 2*650), cell1 is normal.
            cell_widths_keepout: Array1::from_vec(vec![1310.0, 10.0]),
            cell_heights_keepout: Array1::from_vec(vec![1310.0, 10.0]),
            cell_areas_keepout: Array1::from_vec(vec![1310.0 * 1310.0, 100.0]),
            is_pmos_mask: Array1::from_vec(vec![true, false]),
            pin_offset: Array1::zeros(0),
            pin_owner_cell: Array1::zeros(0),
            io_pin_mask: Array1::from_vec(vec![]),
            net_weights: Array1::zeros(0),
            net_pin_offsets: Array1::from_vec(vec![0]),
            net_pin_index: Array1::zeros(0),
            cell_pin_offsets: Array1::from_vec(vec![0, 0, 0]),
            cell_pin_index: Array1::zeros(0),
            overlap_pairs: Array1::from_vec(vec![0, 1]),
            num_overlap_pairs: 1,
            sym_pairs: Array1::zeros(0),
            sym_pair_group: Array1::zeros(0),
            sym_group_vertical: Array1::from_vec(vec![]),
            sym_group_names: Vec::new(),
            num_sym_pairs: 0,
            num_sym_groups: 0,
            sym_self_weight: 2.0,
            sym_anchor_x: Array1::from_vec(vec![5.0; num_cells]),
            sym_anchor_y: Array1::from_vec(vec![5.0; num_cells]),
            well_group_offsets: Array1::zeros(1),
            well_group_index: Array1::zeros(0),
            cell_well_group: Array1::from_vec(vec![-1, -1]),
            num_clusters: 0,
            collinear_member_index: Array1::zeros(0),
            collinear_member_offsets: Array1::zeros(1),
            collinear_vertical: Array1::from_vec(vec![]),
            num_collinear_groups: 0,
            boundary_xl: -2000.0,
            boundary_yl: -2000.0,
            boundary_xh: 2000.0,
            boundary_yh: 2000.0,
        };

        // cell1 is placed to the left of cell0 real bbox.
        // Symmetric expansion must still detect overlap with PMOS keepout.
        let pos = Array1::from_vec(vec![0.0, -20.0, 0.0, 0.0]);
        let ovl = compute_overlap(&pos, &data, 1.0, &mut grad).unwrap();
        assert!(
            ovl > 1.0,
            "expected positive overlap from left-side keepout, got {ovl}"
        );
    }
}
