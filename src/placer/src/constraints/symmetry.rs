//! Symmetry constraint
//!
//! Symmetry constraint: forces a symmetry pair to be symmetric about the symmetry axis
//!
//! ## Mathematics
//!
//! For a symmetry pair (c1, c2) and a symmetry axis `axis`:
//! - center1 = pos1 + width1/2 (or height1/2)
//! - center2 = pos2 + width2/2 (or height2/2)
//! - loss = (center1 + center2 - 2*axis)² + (y1 - y2)²

use crate::data::PlaceData;
use crate::error::PlaceResult;
use ndarray::Array1;

/// Computes the symmetry constraint
///
/// # Parameters
///
/// - pos: position array [x0...x_{N-1}, y0...y_{N-1}]
/// - data: placement data
/// - axes: symmetry axis positions (one per symmetry group)
/// - grad: gradient array (output)
///
/// # Returns
///
/// The total symmetry loss
pub fn compute_symmetry(
    pos: &Array1<f64>,
    data: &PlaceData,
    axes: &Array1<f64>,
    grad: &mut Array1<f64>,
) -> PlaceResult<f64> {
    let num_cells = data.num_cells;
    let num_pairs = data.num_sym_pairs;

    if num_pairs == 0 {
        return Ok(0.0);
    }

    let mut total_loss = 0.0;

    // Compute the loss for each symmetry pair
    for i in 0..num_pairs {
        let gidx = data.sym_pair_group[i] as usize;
        if gidx >= axes.len() {
            continue;
        }

        let axis = axes[gidx];
        let is_vert = data.sym_group_vertical[gidx]; // true = vertical symmetry axis (x=axis)

        let cidx1 = data.sym_pairs[2 * i] as usize;
        let cidx2 = data.sym_pairs[2 * i + 1] as usize;

        if cidx1 >= num_cells || cidx2 >= num_cells {
            continue;
        }

        // Symmetry direction: 0 = x, 1 = y
        let dir_offset = if is_vert { 0 } else { num_cells };
        let other_offset = if is_vert { num_cells } else { 0 };

        // Center position along the symmetry direction
        let x1 = pos[cidx1 + dir_offset]
            + if is_vert {
                data.sym_anchor_x[cidx1]
            } else {
                data.sym_anchor_y[cidx1]
            };

        let x2 = pos[cidx2 + dir_offset]
            + if is_vert {
                data.sym_anchor_x[cidx2]
            } else {
                data.sym_anchor_y[cidx2]
            };

        // Position along the direction perpendicular to the symmetry axis
        let y1 = pos[cidx1 + other_offset]
            + if is_vert {
                data.sym_anchor_y[cidx1]
            } else {
                data.sym_anchor_x[cidx1]
            };

        let y2 = pos[cidx2 + other_offset]
            + if is_vert {
                data.sym_anchor_y[cidx2]
            } else {
                data.sym_anchor_x[cidx2]
            };

        // Loss: the two centers should be symmetric about the symmetry axis
        let mut sym_loss = (x1 + x2 - 2.0 * axis).powi(2);
        let mut align_loss = (y1 - y2).powi(2);
        let self_w = if cidx1 == cidx2 {
            data.sym_self_weight
        } else {
            1.0
        };
        sym_loss *= self_w;
        align_loss *= self_w;

        total_loss += sym_loss + align_loss;

        // Gradient
        let grad_sym = 2.0 * (x1 + x2 - 2.0 * axis) * self_w;
        let grad_align1 = 2.0 * (y1 - y2) * self_w;
        let grad_align2 = -grad_align1;

        grad[cidx1 + dir_offset] += grad_sym;
        grad[cidx2 + dir_offset] += grad_sym;
        grad[cidx1 + other_offset] += grad_align1;
        grad[cidx2 + other_offset] += grad_align2;
    }

    Ok(total_loss)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_symmetry_perfect() {
        // 2 perfectly symmetric cells
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
            overlap_pairs: Array1::zeros(0),
            num_overlap_pairs: 0,
            sym_pairs: Array1::from_vec(vec![0, 1]), // (cell0, cell1)
            sym_pair_group: Array1::from_vec(vec![0]), // group 0
            sym_group_vertical: Array1::from_vec(vec![true]), // vertical symmetry
            sym_group_names: Vec::new(),
            num_sym_pairs: 1,
            num_sym_groups: 1,
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

        // Symmetry axis at x=50
        let axes = Array1::from_vec(vec![50.0]);

        // cell 0 at (0, 0), cell 1 at (90, 0)
        // center0 = 0 + 5 = 5, center1 = 90 + 5 = 95
        // (5 + 95) / 2 = 50 -> perfectly symmetric
        let pos = Array1::from_vec(vec![0.0, 90.0, 0.0, 0.0]);
        let mut grad = Array1::zeros(4);

        let loss = compute_symmetry(&pos, &data, &axes, &mut grad).unwrap();

        // Should be close to 0 (perfectly symmetric)
        assert!(loss < 1.0, "loss = {}", loss);
    }

    #[test]
    fn test_symmetry_imperfect() {
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
            overlap_pairs: Array1::zeros(0),
            num_overlap_pairs: 0,
            sym_pairs: Array1::from_vec(vec![0, 1]),
            sym_pair_group: Array1::from_vec(vec![0]),
            sym_group_vertical: Array1::from_vec(vec![true]),
            sym_group_names: Vec::new(),
            num_sym_pairs: 1,
            num_sym_groups: 1,
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

        let axes = Array1::from_vec(vec![50.0]);

        // cell 0 at (0, 0), cell 1 at (80, 10) - not symmetric
        let pos = Array1::from_vec(vec![0.0, 80.0, 0.0, 10.0]);
        let mut grad = Array1::zeros(4);

        let loss = compute_symmetry(&pos, &data, &axes, &mut grad).unwrap();

        // There should be a loss
        assert!(loss > 1.0, "loss = {}", loss);

        // The gradient should be non-zero
        assert!(grad.iter().any(|&g| g.abs() > 0.0));
    }

    #[test]
    fn test_symmetry_ignores_overlap_only_geometry() {
        let num_cells = 2;
        let mut data = PlaceData {
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
            overlap_pairs: Array1::zeros(0),
            num_overlap_pairs: 0,
            sym_pairs: Array1::from_vec(vec![0, 1]),
            sym_pair_group: Array1::from_vec(vec![0]),
            sym_group_vertical: Array1::from_vec(vec![true]),
            sym_group_names: Vec::new(),
            num_sym_pairs: 1,
            num_sym_groups: 1,
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

        let axes = Array1::from_vec(vec![50.0]);
        let pos = Array1::from_vec(vec![0.0, 80.0, 0.0, 10.0]);
        let mut grad_a = Array1::zeros(4);
        let loss_a = compute_symmetry(&pos, &data, &axes, &mut grad_a).unwrap();

        data.cell_widths_keepout = Array1::from_vec(vec![1310.0, 1410.0]);
        data.cell_heights_keepout = Array1::from_vec(vec![1310.0, 1410.0]);
        data.cell_areas_keepout = Array1::from_vec(vec![1310.0 * 1310.0, 1410.0 * 1410.0]);
        data.is_pmos_mask = Array1::from_vec(vec![true, true]);

        let mut grad_b = Array1::zeros(4);
        let loss_b = compute_symmetry(&pos, &data, &axes, &mut grad_b).unwrap();
        assert!(
            (loss_a - loss_b).abs() < 1e-12,
            "loss_a={} loss_b={}",
            loss_a,
            loss_b
        );
        for i in 0..grad_a.len() {
            assert!(
                (grad_a[i] - grad_b[i]).abs() < 1e-12,
                "grad differs at {}: {} vs {}",
                i,
                grad_a[i],
                grad_b[i]
            );
        }
    }
}
