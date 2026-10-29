//! Boundary constraint
//!
//! Boundary constraint: keeps cells from leaving the placement region
//!
//! ## Mathematical background
//!
//! For each cell, compute how far it oversteps the boundary:
//! - overstep_left = max(0, boundary_xl - x)
//! - overstep_right = max(0, x + width - boundary_xh)
//! - loss = Σ overstep²

use crate::data::PlaceData;
use crate::error::PlaceResult;
use ndarray::Array1;

/// Computes the boundary constraint
///
/// # Parameters
///
/// - pos: position array [x0...x_{N-1}, y0...y_{N-1}]
/// - data: placement data
/// - grad: gradient array (output)
///
/// # Returns
///
/// The total boundary violation loss
pub fn compute_boundary(
    pos: &Array1<f64>,
    data: &PlaceData,
    grad: &mut Array1<f64>,
) -> PlaceResult<f64> {
    let num_cells = data.num_cells;
    let mut total_loss = 0.0;

    for cidx in 0..num_cells {
        let x = pos[cidx];
        let y = pos[cidx + num_cells];
        let w = data.cell_widths[cidx];
        let h = data.cell_heights[cidx];

        // Boundary check in X
        // Left boundary
        if x < data.boundary_xl {
            let overstep = data.boundary_xl - x;
            total_loss += overstep * overstep;
            grad[cidx] += -2.0 * overstep;
        }

        // Right boundary
        if x + w > data.boundary_xh {
            let overstep = x + w - data.boundary_xh;
            total_loss += overstep * overstep;
            grad[cidx] += 2.0 * overstep;
        }

        // Boundary check in Y
        // Lower boundary
        if y < data.boundary_yl {
            let overstep = data.boundary_yl - y;
            total_loss += overstep * overstep;
            grad[cidx + num_cells] += -2.0 * overstep;
        }

        // Upper boundary
        if y + h > data.boundary_yh {
            let overstep = y + h - data.boundary_yh;
            total_loss += overstep * overstep;
            grad[cidx + num_cells] += 2.0 * overstep;
        }
    }

    Ok(total_loss)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_boundary_inside() {
        // The cell is inside the boundary
        let num_cells = 1;

        let data = PlaceData {
            num_cells,
            num_pins: 0,
            num_nets: 0,
            cell_widths: Array1::from_vec(vec![10.0]),
            cell_heights: Array1::from_vec(vec![10.0]),
            cell_areas: Array1::from_vec(vec![100.0]),
            cell_widths_keepout: Array1::from_vec(vec![10.0]),
            cell_heights_keepout: Array1::from_vec(vec![10.0]),
            cell_areas_keepout: Array1::from_vec(vec![100.0]),
            is_pmos_mask: Array1::from_vec(vec![false]),
            pin_offset: Array1::zeros(0),
            pin_owner_cell: Array1::zeros(0),
            io_pin_mask: Array1::from_vec(vec![]),
            net_weights: Array1::zeros(0),
            net_pin_offsets: Array1::from_vec(vec![0]),
            net_pin_index: Array1::zeros(0),
            cell_pin_offsets: Array1::from_vec(vec![0, 0]),
            cell_pin_index: Array1::zeros(0),
            overlap_pairs: Array1::zeros(0),
            num_overlap_pairs: 0,
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
            cell_well_group: Array1::from_vec(vec![-1]),
            num_clusters: 0,
            collinear_member_index: Array1::zeros(0),
            collinear_member_offsets: Array1::zeros(1),
            collinear_vertical: Array1::from_vec(vec![]),
            num_collinear_groups: 0,
            boundary_xl: 0.0,
            boundary_yl: 0.0,
            boundary_xh: 100.0,
            boundary_yh: 100.0,
        };

        // Cell at (10, 10), entirely inside the boundary
        let pos = Array1::from_vec(vec![10.0, 10.0]);
        let mut grad = Array1::zeros(2);

        let loss = compute_boundary(&pos, &data, &mut grad).unwrap();

        // There should be no loss
        assert_eq!(loss, 0.0);
        assert_eq!(grad[0], 0.0);
        assert_eq!(grad[1], 0.0);
    }

    #[test]
    fn test_boundary_outside_left() {
        let num_cells = 1;

        let data = PlaceData {
            num_cells,
            num_pins: 0,
            num_nets: 0,
            cell_widths: Array1::from_vec(vec![10.0]),
            cell_heights: Array1::from_vec(vec![10.0]),
            cell_areas: Array1::from_vec(vec![100.0]),
            cell_widths_keepout: Array1::from_vec(vec![10.0]),
            cell_heights_keepout: Array1::from_vec(vec![10.0]),
            cell_areas_keepout: Array1::from_vec(vec![100.0]),
            is_pmos_mask: Array1::from_vec(vec![false]),
            pin_offset: Array1::zeros(0),
            pin_owner_cell: Array1::zeros(0),
            io_pin_mask: Array1::from_vec(vec![]),
            net_weights: Array1::zeros(0),
            net_pin_offsets: Array1::from_vec(vec![0]),
            net_pin_index: Array1::zeros(0),
            cell_pin_offsets: Array1::from_vec(vec![0, 0]),
            cell_pin_index: Array1::zeros(0),
            overlap_pairs: Array1::zeros(0),
            num_overlap_pairs: 0,
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
            cell_well_group: Array1::from_vec(vec![-1]),
            num_clusters: 0,
            collinear_member_index: Array1::zeros(0),
            collinear_member_offsets: Array1::zeros(1),
            collinear_vertical: Array1::from_vec(vec![]),
            num_collinear_groups: 0,
            boundary_xl: 0.0,
            boundary_yl: 0.0,
            boundary_xh: 100.0,
            boundary_yh: 100.0,
        };

        // Cell at (-5, 10), 5 units past the left boundary
        let pos = Array1::from_vec(vec![-5.0, 10.0]);
        let mut grad = Array1::zeros(2);

        let loss = compute_boundary(&pos, &data, &mut grad).unwrap();

        // The loss should be 5² = 25
        assert_eq!(loss, 25.0);

        // The gradient should be negative (pushing to the right, back inside the boundary)
        // because the optimizer does pos -= lr * grad, so a negative gradient increases x
        assert!(grad[0] < 0.0);
    }

    #[test]
    fn test_boundary_outside_right() {
        let num_cells = 1;

        let data = PlaceData {
            num_cells,
            num_pins: 0,
            num_nets: 0,
            cell_widths: Array1::from_vec(vec![10.0]),
            cell_heights: Array1::from_vec(vec![10.0]),
            cell_areas: Array1::from_vec(vec![100.0]),
            cell_widths_keepout: Array1::from_vec(vec![10.0]),
            cell_heights_keepout: Array1::from_vec(vec![10.0]),
            cell_areas_keepout: Array1::from_vec(vec![100.0]),
            is_pmos_mask: Array1::from_vec(vec![false]),
            pin_offset: Array1::zeros(0),
            pin_owner_cell: Array1::zeros(0),
            io_pin_mask: Array1::from_vec(vec![]),
            net_weights: Array1::zeros(0),
            net_pin_offsets: Array1::from_vec(vec![0]),
            net_pin_index: Array1::zeros(0),
            cell_pin_offsets: Array1::from_vec(vec![0, 0]),
            cell_pin_index: Array1::zeros(0),
            overlap_pairs: Array1::zeros(0),
            num_overlap_pairs: 0,
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
            cell_well_group: Array1::from_vec(vec![-1]),
            num_clusters: 0,
            collinear_member_index: Array1::zeros(0),
            collinear_member_offsets: Array1::zeros(1),
            collinear_vertical: Array1::from_vec(vec![]),
            num_collinear_groups: 0,
            boundary_xl: 0.0,
            boundary_yl: 0.0,
            boundary_xh: 100.0,
            boundary_yh: 100.0,
        };

        // Cell at (95, 10), 5 units past the right boundary (95+10=105 > 100)
        let pos = Array1::from_vec(vec![95.0, 10.0]);
        let mut grad = Array1::zeros(2);

        let loss = compute_boundary(&pos, &data, &mut grad).unwrap();

        // The loss should be 5² = 25
        assert_eq!(loss, 25.0);

        // The gradient should point inward (negative direction)
        assert!(grad[0] > 0.0); // should push to the left (the gradient is positive because the cell oversteps on the right)
    }
}
