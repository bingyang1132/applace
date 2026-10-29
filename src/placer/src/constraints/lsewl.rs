//! Log-sum-exp wirelength.
//!
//! A net costs the half-perimeter of the box that encloses its pins, and that box is
//! built out of `max` and `min` -- neither of which a gradient can see through. Each
//! extreme is therefore replaced by its log-sum-exp, of radius `gamma`:
//!
//! ```text
//! max(x) ~=  gamma * ln(sum_i exp( x_i / gamma))
//! min(x) ~= -gamma * ln(sum_i exp(-x_i / gamma))
//! ```
//!
//! and the net contributes `(max_x - min_x) + (max_y - min_y)`, times its weight. As
//! the controller shrinks `gamma` the estimate tightens onto the true half-perimeter.
//!
//! The two axes are one computation done twice, so everything here is indexed by axis:
//! `0` is x, `1` is y, and the entry for axis `a` of pin `p` lives at `p + a * num_pins`
//! (likewise for nets). The axis loop sits *inside* the pin loop rather than outside it,
//! which keeps each accumulator receiving its terms in pin order.

use crate::data::PlaceData;
use crate::error::PlaceResult;
use ndarray::Array1;

/// x and y.
const AXES: usize = 2;

/// Total smoothed wirelength, and its gradient.
///
/// # Arguments
///
/// - `pos`: the optimization variables, laid out `[x0..x_{N-1}, y0..y_{N-1}]`
/// - `data`: the placement problem
/// - `gamma`: the log-sum-exp radius
/// - `grad`: receives the gradient; accumulated into, not cleared
pub fn compute_lsewl(
    pos: &Array1<f64>,
    data: &PlaceData,
    gamma: f64,
    grad: &mut Array1<f64>,
) -> PlaceResult<f64> {
    let num_cells = data.num_cells;
    let num_pins = data.num_pins;
    let num_nets = data.num_nets;
    let sharpness = 1.0 / gamma;

    // Each pin, at its device's position plus its own offset.
    let mut pin_pos = Array1::<f64>::zeros(AXES * num_pins);
    for pidx in 0..num_pins {
        let cell_idx = data.pin_owner_cell[pidx];
        if cell_idx >= 0 {
            let cidx = cell_idx as usize;
            pin_pos[pidx] = pos[cidx] + data.pin_offset[pidx];
            pin_pos[pidx + num_pins] = pos[cidx + num_cells] + data.pin_offset[pidx + num_pins];
        }
    }

    // Per-pin exponential weights and their per-net sums. `hi` is the smoothed upper
    // extreme of the bounding box, `lo` the lower one.
    let mut exp_hi = Array1::<f64>::zeros(AXES * num_pins);
    let mut exp_lo = Array1::<f64>::zeros(AXES * num_pins);
    let mut net_sum_hi = Array1::<f64>::zeros(AXES * num_nets);
    let mut net_sum_lo = Array1::<f64>::zeros(AXES * num_nets);
    let mut net_valid_pins = Array1::<usize>::zeros(num_nets);

    let mut total_wl = 0.0;

    for nidx in 0..num_nets {
        let start = data.net_pin_offsets[nidx];
        let end = data.net_pin_offsets[nidx + 1];

        // The true extremes first: every exponent below is taken relative to one of
        // them, so that no exponent is ever positive and `exp` cannot overflow.
        let mut valid_pins = 0;
        let mut hi = [f64::NEG_INFINITY; AXES];
        let mut lo = [f64::INFINITY; AXES];

        for j in start..end {
            let pidx = data.net_pin_index[j] as usize;
            // An IO pin belongs to no device this run can move.
            if data.io_pin_mask[pidx] {
                continue;
            }
            valid_pins += 1;
            for axis in 0..AXES {
                let v = pin_pos[pidx + axis * num_pins];
                hi[axis] = hi[axis].max(v);
                lo[axis] = lo[axis].min(v);
            }
        }

        net_valid_pins[nidx] = valid_pins;

        // A net reaching one movable pin or none has no length to shorten.
        if valid_pins <= 1 {
            continue;
        }

        for j in start..end {
            let pidx = data.net_pin_index[j] as usize;
            if data.io_pin_mask[pidx] {
                continue;
            }
            for axis in 0..AXES {
                let p = pidx + axis * num_pins;
                let n = nidx + axis * num_nets;
                let v = pin_pos[p];
                exp_hi[p] = ((v - hi[axis]) * sharpness).exp();
                exp_lo[p] = (-(v - lo[axis]) * sharpness).exp();
                net_sum_hi[n] += exp_hi[p];
                net_sum_lo[n] += exp_lo[p];
            }
        }

        let mut span = [0.0f64; AXES];
        for (axis, s) in span.iter_mut().enumerate() {
            let n = nidx + axis * num_nets;
            *s = (hi[axis] + gamma * net_sum_hi[n].ln()) - (lo[axis] - gamma * net_sum_lo[n].ln());
        }
        total_wl += (span[0] + span[1]) * data.net_weights[nidx];
    }

    // The gradient lands on pins first and is folded onto devices afterwards, because
    // several pins of one device can belong to different nets.
    let mut pin_grad = Array1::<f64>::zeros(AXES * num_pins);

    for pidx in 0..num_pins {
        if data.io_pin_mask[pidx] {
            continue;
        }

        let cell_idx = data.pin_owner_cell[pidx];
        if cell_idx < 0 {
            continue;
        }

        // There is no pin-to-net index, so the net is found by scanning. Nets are few
        // and this runs once per iteration; an inverse map would be the optimization.
        for nidx in 0..num_nets {
            if net_valid_pins[nidx] <= 1 {
                continue;
            }

            let start = data.net_pin_offsets[nidx];
            let end = data.net_pin_offsets[nidx + 1];
            if !(start..end).any(|j| data.net_pin_index[j] as usize == pidx) {
                continue;
            }

            let weight = data.net_weights[nidx];
            for axis in 0..AXES {
                let p = pidx + axis * num_pins;
                let n = nidx + axis * num_nets;
                pin_grad[p] = (exp_hi[p] / net_sum_hi[n] - exp_lo[p] / net_sum_lo[n]) * weight;
            }
        }
    }

    for pidx in 0..num_pins {
        let cell_idx = data.pin_owner_cell[pidx];
        if cell_idx >= 0 {
            let cidx = cell_idx as usize;
            grad[cidx] += pin_grad[pidx];
            grad[cidx + num_cells] += pin_grad[pidx + num_pins];
        }
    }

    Ok(total_wl)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lsewl_simple() {
        // Build simple test data
        // 2 cells, 2 pins, 1 net
        let num_cells = 2;
        let num_pins = 2;

        let data = PlaceData {
            num_cells,
            num_pins,
            num_nets: 1,
            cell_widths: Array1::from_vec(vec![10.0, 10.0]),
            cell_heights: Array1::from_vec(vec![10.0, 10.0]),
            cell_areas: Array1::from_vec(vec![100.0, 100.0]),
            cell_widths_keepout: Array1::from_vec(vec![10.0, 10.0]),
            cell_heights_keepout: Array1::from_vec(vec![10.0, 10.0]),
            cell_areas_keepout: Array1::from_vec(vec![100.0, 100.0]),
            is_pmos_mask: Array1::from_vec(vec![false, false]),
            pin_offset: Array1::from_vec(vec![5.0, 5.0, 5.0, 5.0]), // center of cells
            pin_owner_cell: Array1::from_vec(vec![0, 1]),
            io_pin_mask: Array1::from_vec(vec![false, false]),
            net_weights: Array1::from_vec(vec![1.0]),
            net_pin_offsets: Array1::from_vec(vec![0, 2]),
            net_pin_index: Array1::from_vec(vec![0, 1]),
            cell_pin_offsets: Array1::from_vec(vec![0, 1, 2]),
            cell_pin_index: Array1::from_vec(vec![0, 1]),
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

        // Positions: cell 0 at (0, 0), cell 1 at (100, 0)
        let pos = Array1::from_vec(vec![0.0, 100.0, 0.0, 0.0]);
        let mut grad = Array1::zeros(4);

        let wl = compute_lsewl(&pos, &data, 1.0, &mut grad).unwrap();

        // The wirelength should be about 100 (the distance between the two pins)
        assert!(wl > 90.0 && wl < 110.0, "wl = {}", wl);

        // The gradient should be non-zero
        assert!(grad[0].abs() > 0.0 || grad[1].abs() > 0.0);
    }
}
