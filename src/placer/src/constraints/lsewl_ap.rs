//! AP-aware LSEWL (superellipse-direction pins)
//!
//! This term augments pin coordinates with per-pin direction variable `phi`
//! on a superellipse boundary and computes LSE wirelength + gradients.

use crate::data::PlaceData;
use crate::error::{PlaceError, PlaceResult};

/// Compute AP-aware LSEWL and gradients.
///
/// `pos` layout: `[x0..x_{N-1}, y0..y_{N-1}]`
/// `phi` layout: one angle per pin.
///
/// Returns raw wirelength loss (no external scaling/lambda here).
pub fn compute_lsewl_ap(
    pos: &[f64],
    data: &PlaceData,
    phi: &[f64],
    phi_grad: &mut [f64],
    grad: &mut [f64],
    gamma: f64,
    m: f64,
    eps: f64,
) -> PlaceResult<f64> {
    let num_cells = data.num_cells;
    let num_pins = data.num_pins;
    let num_nets = data.num_nets;

    if pos.len() != 2 * num_cells {
        return Err(PlaceError::ShapeError(format!(
            "pos length {} != 2*num_cells {}",
            pos.len(),
            2 * num_cells
        )));
    }
    if grad.len() != 2 * num_cells {
        return Err(PlaceError::ShapeError(format!(
            "grad length {} != 2*num_cells {}",
            grad.len(),
            2 * num_cells
        )));
    }
    if phi.len() != num_pins {
        return Err(PlaceError::ShapeError(format!(
            "phi length {} != num_pins {}",
            phi.len(),
            num_pins
        )));
    }
    if phi_grad.len() != num_pins {
        return Err(PlaceError::ShapeError(format!(
            "phi_grad length {} != num_pins {}",
            phi_grad.len(),
            num_pins
        )));
    }
    if gamma <= 0.0 {
        return Err(PlaceError::InvalidParameter(
            "gamma must be positive".to_string(),
        ));
    }
    if m <= 0.0 {
        return Err(PlaceError::InvalidParameter(
            "m must be positive".to_string(),
        ));
    }
    if eps <= 0.0 {
        return Err(PlaceError::InvalidParameter(
            "eps must be positive".to_string(),
        ));
    }

    let k = (2.0 / m - 1.0) * 0.5;
    let inv_gamma = 1.0 / gamma;

    // AP-aware pin coordinates and Jacobian wrt phi.
    let mut x_pin = vec![0.0; num_pins];
    let mut y_pin = vec![0.0; num_pins];
    let mut ddx_dphi = vec![0.0; num_pins];
    let mut ddy_dphi = vec![0.0; num_pins];

    for p in 0..num_pins {
        let cidx_i32 = data.pin_owner_cell[p];
        if cidx_i32 < 0 {
            continue;
        }
        let cidx = cidx_i32 as usize;
        if cidx >= num_cells {
            continue;
        }

        let a = data.cell_widths[cidx] * 0.5;
        let b = data.cell_heights[cidx] * 0.5;
        let ph = phi[p];
        let c = ph.cos();
        let s = ph.sin();
        let c2 = c * c;
        let s2 = s * s;
        let base_c = c2 + eps;
        let base_s = s2 + eps;
        let pow_c = base_c.powf(k);
        let pow_s = base_s.powf(k);

        let dx = a * c * pow_c;
        let dy = b * s * pow_s;
        x_pin[p] = pos[cidx] + dx;
        y_pin[p] = pos[cidx + num_cells] + dy;

        // Chain rule pieces
        let c_prime = -s;
        let s_prime = c;
        let dpow_c_dphi = k * base_c.powf(k - 1.0) * (2.0 * c * c_prime);
        let dpow_s_dphi = k * base_s.powf(k - 1.0) * (2.0 * s * s_prime);
        ddx_dphi[p] = a * (c_prime * pow_c + c * dpow_c_dphi);
        ddy_dphi[p] = b * (s_prime * pow_s + s * dpow_s_dphi);
    }

    let mut total_wl = 0.0;

    // Same CSR traversal style as existing LSEWL.
    for nidx in 0..num_nets {
        let start = data.net_pin_offsets[nidx];
        let end = data.net_pin_offsets[nidx + 1];

        let mut pins = Vec::with_capacity(end.saturating_sub(start));
        let mut max_x = f64::NEG_INFINITY;
        let mut min_x = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;

        for j in start..end {
            let p = data.net_pin_index[j] as usize;
            if p >= num_pins {
                continue;
            }
            if data.io_pin_mask[p] {
                continue;
            }
            let cidx_i32 = data.pin_owner_cell[p];
            if cidx_i32 < 0 || (cidx_i32 as usize) >= num_cells {
                continue;
            }
            pins.push(p);
            max_x = max_x.max(x_pin[p]);
            min_x = min_x.min(x_pin[p]);
            max_y = max_y.max(y_pin[p]);
            min_y = min_y.min(y_pin[p]);
        }

        if pins.len() <= 1 {
            continue;
        }

        let mut exp_pos_x = vec![0.0; pins.len()];
        let mut exp_neg_x = vec![0.0; pins.len()];
        let mut exp_pos_y = vec![0.0; pins.len()];
        let mut exp_neg_y = vec![0.0; pins.len()];

        let mut sum_pos_x = 0.0;
        let mut sum_neg_x = 0.0;
        let mut sum_pos_y = 0.0;
        let mut sum_neg_y = 0.0;

        for (idx, &p) in pins.iter().enumerate() {
            let ex_p = ((x_pin[p] - max_x) * inv_gamma).exp();
            let ex_n = (-(x_pin[p] - min_x) * inv_gamma).exp();
            let ey_p = ((y_pin[p] - max_y) * inv_gamma).exp();
            let ey_n = (-(y_pin[p] - min_y) * inv_gamma).exp();
            exp_pos_x[idx] = ex_p;
            exp_neg_x[idx] = ex_n;
            exp_pos_y[idx] = ey_p;
            exp_neg_y[idx] = ey_n;
            sum_pos_x += ex_p;
            sum_neg_x += ex_n;
            sum_pos_y += ey_p;
            sum_neg_y += ey_n;
        }

        let x_max_lse = max_x + gamma * sum_pos_x.ln();
        let x_min_lse = min_x - gamma * sum_neg_x.ln();
        let y_max_lse = max_y + gamma * sum_pos_y.ln();
        let y_min_lse = min_y - gamma * sum_neg_y.ln();

        let weight = data.net_weights[nidx];
        total_wl += ((x_max_lse - x_min_lse) + (y_max_lse - y_min_lse)) * weight;

        // dL/d(pin) then chain to cell-pos and phi.
        for (idx, &p) in pins.iter().enumerate() {
            let dldx = (exp_pos_x[idx] / sum_pos_x - exp_neg_x[idx] / sum_neg_x) * weight;
            let dldy = (exp_pos_y[idx] / sum_pos_y - exp_neg_y[idx] / sum_neg_y) * weight;

            let cidx = data.pin_owner_cell[p] as usize;
            grad[cidx] += dldx;
            grad[cidx + num_cells] += dldy;
            phi_grad[p] += dldx * ddx_dphi[p] + dldy * ddy_dphi[p];
        }
    }

    Ok(total_wl)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndarray::Array1;

    fn tiny_data() -> PlaceData {
        let num_cells = 2usize;
        let num_pins = 2usize;
        PlaceData {
            num_cells,
            num_pins,
            num_nets: 1,
            cell_widths: Array1::from_vec(vec![10.0, 12.0]),
            cell_heights: Array1::from_vec(vec![8.0, 10.0]),
            cell_areas: Array1::from_vec(vec![80.0, 120.0]),
            cell_widths_keepout: Array1::from_vec(vec![10.0, 12.0]),
            cell_heights_keepout: Array1::from_vec(vec![8.0, 10.0]),
            cell_areas_keepout: Array1::from_vec(vec![80.0, 120.0]),
            is_pmos_mask: Array1::from_vec(vec![false, false]),
            pin_offset: Array1::from_vec(vec![0.0, 0.0, 0.0, 0.0]),
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
            sym_anchor_y: Array1::from_vec(vec![4.0; num_cells]),
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
        }
    }

    #[test]
    fn test_lsewl_ap_phi_grad_fd_sign() {
        let data = tiny_data();
        let pos = vec![0.0, 25.0, 0.0, 10.0];
        let mut phi = vec![0.2, -0.3];
        let mut phi_grad = vec![0.0; data.num_pins];
        let mut grad = vec![0.0; 2 * data.num_cells];
        let gamma = 1.0;
        let m = 4.0;
        let eps = 1e-6;

        let _ =
            compute_lsewl_ap(&pos, &data, &phi, &mut phi_grad, &mut grad, gamma, m, eps).unwrap();

        let p = 0usize;
        let g_analytic = phi_grad[p];
        let delta = 1e-5;

        phi[p] += delta;
        let mut phi_grad_p = vec![0.0; data.num_pins];
        let mut grad_p = vec![0.0; 2 * data.num_cells];
        let loss_p = compute_lsewl_ap(
            &pos,
            &data,
            &phi,
            &mut phi_grad_p,
            &mut grad_p,
            gamma,
            m,
            eps,
        )
        .unwrap();

        phi[p] -= 2.0 * delta;
        let mut phi_grad_m = vec![0.0; data.num_pins];
        let mut grad_m = vec![0.0; 2 * data.num_cells];
        let loss_m = compute_lsewl_ap(
            &pos,
            &data,
            &phi,
            &mut phi_grad_m,
            &mut grad_m,
            gamma,
            m,
            eps,
        )
        .unwrap();

        let g_fd = (loss_p - loss_m) / (2.0 * delta);

        assert!(g_analytic.is_finite());
        assert!(g_fd.is_finite());
        // Keep this as sign sanity (requested).
        if g_analytic.abs() > 1e-9 && g_fd.abs() > 1e-9 {
            assert_eq!(g_analytic.signum(), g_fd.signum());
        }
    }

    #[test]
    fn test_lsewl_ap_no_nan_inf() {
        let data = tiny_data();
        let pos = vec![0.0, 40.0, 3.0, 17.0];
        let phi = vec![0.7, -1.1];
        let mut phi_grad = vec![0.0; data.num_pins];
        let mut grad = vec![0.0; 2 * data.num_cells];

        let loss =
            compute_lsewl_ap(&pos, &data, &phi, &mut phi_grad, &mut grad, 2.0, 6.0, 1e-6).unwrap();

        assert!(loss.is_finite());
        assert!(grad.iter().all(|v| v.is_finite()));
        assert!(phi_grad.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn test_lsewl_ap_ignores_overlap_only_geometry() {
        let data = tiny_data();
        let pos = vec![0.0, 25.0, 0.0, 10.0];
        let phi = vec![0.2, -0.3];
        let gamma = 1.0;
        let m = 4.0;
        let eps = 1e-6;

        let mut phi_grad_a = vec![0.0; data.num_pins];
        let mut grad_a = vec![0.0; 2 * data.num_cells];
        let wl_a = compute_lsewl_ap(
            &pos,
            &data,
            &phi,
            &mut phi_grad_a,
            &mut grad_a,
            gamma,
            m,
            eps,
        )
        .unwrap();

        let mut data_ovl_changed = data.clone();
        data_ovl_changed.cell_widths_keepout = Array1::from_vec(vec![1310.0, 1412.0]);
        data_ovl_changed.cell_heights_keepout = Array1::from_vec(vec![1308.0, 1410.0]);
        data_ovl_changed.cell_areas_keepout =
            Array1::from_vec(vec![1310.0 * 1308.0, 1412.0 * 1410.0]);
        data_ovl_changed.is_pmos_mask = Array1::from_vec(vec![true, true]);

        let mut phi_grad_b = vec![0.0; data_ovl_changed.num_pins];
        let mut grad_b = vec![0.0; 2 * data_ovl_changed.num_cells];
        let wl_b = compute_lsewl_ap(
            &pos,
            &data_ovl_changed,
            &phi,
            &mut phi_grad_b,
            &mut grad_b,
            gamma,
            m,
            eps,
        )
        .unwrap();

        assert!((wl_a - wl_b).abs() < 1e-12, "wl_a={} wl_b={}", wl_a, wl_b);
        for i in 0..phi_grad_a.len() {
            assert!(
                (phi_grad_a[i] - phi_grad_b[i]).abs() < 1e-12,
                "phi_grad differs at {}: {} vs {}",
                i,
                phi_grad_a[i],
                phi_grad_b[i]
            );
        }
    }
}
