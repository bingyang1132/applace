//! Placement model - loss function and gradients
//!
//! Implements the objective function and the gradient computation

use crate::data::PlaceData;
use crate::error::{PlaceError, PlaceResult};
use crate::params::PlacementConfig;
use ndarray::Array1;
use std::cell::RefCell;

/// Placement model (objective function)
pub struct PlaceModel {
    /// Data
    pub data: PlaceData,

    /// Parameters
    pub params: PlacementConfig,

    /// Current wirelength gamma (it decays over the run)
    pub wl_gamma: f64,

    /// Current multipliers (Lagrange multipliers)
    pub lambdas: LagrangeMultipliers,

    /// AP-aware pin direction variables (one phi per pin)
    pub phi: Vec<f64>,

    /// AP-aware phi gradients (updated in forward when enabled)
    pub phi_grad: RefCell<Vec<f64>>,

    /// AP WL convex blending factor in [0, 1]
    pub lambda_ap: f64,

    /// Last raw WL_base component from forward()
    pub last_wl_base: RefCell<f64>,

    /// Last raw WL_ap component from forward()
    pub last_wl_ap: RefCell<f64>,

    /// Last per-term weighted loss breakdown from forward()
    pub last_breakdown: RefCell<LossBreakdown>,

    /// Per-cell inflation factors for overlap penalty (tail-stage only).
    pub overlap_inflation: Vec<f64>,

    /// Per-cell isotropic additive halo for overlap penalty (tail-v2).
    pub overlap_halo: Vec<f64>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct LossBreakdown {
    pub total: f64,
    pub wl: f64,
    pub wl_base_raw: f64,
    pub wl_ap_raw: f64,
    pub overlap: f64,
    pub overlap_raw: f64,
    pub symmetry: f64,
    pub symmetry_raw: f64,
    pub boundary: f64,
    pub boundary_raw: f64,
    pub well_group: f64,
    pub well_group_raw: f64,
    pub array: f64,
    pub array_raw: f64,
    pub compaction: f64,
    pub compaction_raw: f64,
}

/// Lagrange multipliers for the constrained objective.
#[derive(Debug, Clone)]
pub struct LagrangeMultipliers {
    pub overlap: f64,
    pub symmetry: f64,
    pub boundary: f64,
    pub well_group: f64,
    pub array: f64,
}

impl Default for LagrangeMultipliers {
    fn default() -> Self {
        Self::new()
    }
}

impl LagrangeMultipliers {
    pub fn new() -> Self {
        Self {
            overlap: 1.0,
            symmetry: 1.0,
            boundary: 1.0,
            well_group: 1.0,
            array: 1.0,
        }
    }
}

impl PlaceModel {
    /// Creates a new placement model
    pub fn new(data: PlaceData, params: PlacementConfig) -> PlaceResult<Self> {
        let num_cells = data.num_cells;
        let num_pins = data.num_pins;
        let wl_gamma = params.wl_smoothing_init;
        let lambdas = LagrangeMultipliers::new();

        Ok(Self {
            data,
            params,
            wl_gamma,
            lambdas,
            phi: vec![0.0; num_pins],
            phi_grad: RefCell::new(vec![0.0; num_pins]),
            lambda_ap: 0.0,
            last_wl_base: RefCell::new(0.0),
            last_wl_ap: RefCell::new(0.0),
            last_breakdown: RefCell::new(LossBreakdown::default()),
            overlap_inflation: vec![1.0; num_cells],
            overlap_halo: vec![0.0; num_cells],
        })
    }

    /// Computes the loss function and the gradients
    ///
    /// pos: [x0, x1, ..., x_{N-1}, y0, y1, ..., y_{N-1}]
    /// grad: the same layout
    pub fn forward(&self, pos: &Array1<f64>, grad: &mut Array1<f64>) -> PlaceResult<f64> {
        // Zero the gradients
        grad.fill(0.0);

        let mut total_loss = 0.0;
        let mut breakdown = LossBreakdown::default();

        // === 1. LSEWL wirelength term ===
        let mut wl_base_raw = 0.0;
        let mut wl_ap_raw = 0.0;

        if self.params.use_ap_wl {
            self.phi_grad.borrow_mut().fill(0.0);
        }

        if self.params.enable_wirelength {
            let mut base_grad = Array1::<f64>::zeros(grad.len());
            let (loss_base, _) = self.compute_lsewl(pos, &mut base_grad)?;
            wl_base_raw = loss_base;

            if self.params.use_ap_wl {
                let pos_slice = pos
                    .as_slice()
                    .ok_or_else(|| PlaceError::ShapeError("pos is not contiguous".to_string()))?;
                let mut ap_grad = vec![0.0; grad.len()];
                let mut phi_grad_ref = self.phi_grad.borrow_mut();
                let loss_ap = crate::constraints::compute_lsewl_ap(
                    pos_slice,
                    &self.data,
                    &self.phi,
                    phi_grad_ref.as_mut_slice(),
                    ap_grad.as_mut_slice(),
                    self.wl_gamma,
                    self.params.ap_exponent_m,
                    self.params.ap_eps,
                )?;
                wl_ap_raw = loss_ap;
                let lam = self.lambda_ap.clamp(0.0, 1.0);
                let one_minus = 1.0 - lam;
                for i in 0..grad.len() {
                    grad[i] += one_minus * base_grad[i] + lam * ap_grad[i];
                }
                let wl = one_minus * loss_base + lam * loss_ap;
                breakdown.wl = wl;
                total_loss += wl;
            } else {
                for i in 0..grad.len() {
                    grad[i] += base_grad[i];
                }
                breakdown.wl = loss_base;
                total_loss += loss_base;
            }
        }
        *self.last_wl_base.borrow_mut() = wl_base_raw;
        *self.last_wl_ap.borrow_mut() = wl_ap_raw;
        breakdown.wl_base_raw = wl_base_raw;
        breakdown.wl_ap_raw = wl_ap_raw;

        // === 2. Overlap penalty ===
        if self.params.enable_overlap {
            let (weighted, raw) = self.compute_overlap(pos, grad)?;
            breakdown.overlap = weighted;
            breakdown.overlap_raw = raw;
            total_loss += weighted;
        }

        // === 3. Symmetry constraint ===
        if self.params.enable_symmetry && self.data.num_sym_pairs > 0 {
            let (weighted, raw) = self.compute_symmetry(pos, grad)?;
            breakdown.symmetry = weighted;
            breakdown.symmetry_raw = raw;
            total_loss += weighted;
        }

        // === 4. Boundary constraint ===
        if self.params.enable_boundary {
            let (weighted, raw) = self.compute_boundary(pos, grad)?;
            breakdown.boundary = weighted;
            breakdown.boundary_raw = raw;
            total_loss += weighted;
        }

        // === 5. Well cluster constraint ===
        if self.params.use_well_cluster && self.data.num_clusters > 0 {
            let (weighted, raw) = self.compute_well_group(pos, grad)?;
            breakdown.well_group = weighted;
            breakdown.well_group_raw = raw;
            total_loss += weighted;
        }

        // === 6. Array constraint ===
        if self.params.enable_array && self.data.num_collinear_groups > 0 {
            let (weighted, raw) = self.compute_array(pos, grad)?;
            breakdown.array = weighted;
            breakdown.array_raw = raw;
            total_loss += weighted;
        }

        // === 7. Compaction (centroid) penalty ===
        if self.params.use_compaction && self.params.compaction_weight > 0.0 {
            let mut tmp_grad = Array1::<f64>::zeros(grad.len());
            let (weighted, raw) = self.compute_compaction(pos, &mut tmp_grad)?;
            let w = self.params.compaction_weight;
            for i in 0..grad.len() {
                grad[i] += tmp_grad[i] * w;
            }
            breakdown.compaction = weighted;
            breakdown.compaction_raw = raw;
            total_loss += weighted;
        }

        breakdown.total = total_loss;
        *self.last_breakdown.borrow_mut() = breakdown;

        Ok(total_loss)
    }

    /// Computes the LSEWL wirelength term
    fn compute_lsewl(&self, pos: &Array1<f64>, grad: &mut Array1<f64>) -> PlaceResult<(f64, f64)> {
        let loss = crate::constraints::compute_lsewl(pos, &self.data, self.wl_gamma, grad)?;
        Ok((loss, loss))
    }

    /// Computes the overlap penalty
    fn compute_overlap(
        &self,
        pos: &Array1<f64>,
        grad: &mut Array1<f64>,
    ) -> PlaceResult<(f64, f64)> {
        let gamma = 1.0; // gamma for overlap is normally set to 1
        let mut tmp_grad = Array1::<f64>::zeros(grad.len());
        let loss = crate::constraints::compute_overlap_with_inflation_and_halo(
            pos,
            &self.data,
            gamma,
            &mut tmp_grad,
            Some(&self.overlap_inflation),
            Some(&self.overlap_halo),
        )?;
        let w = self.lambdas.overlap;
        for i in 0..grad.len() {
            grad[i] += tmp_grad[i] * w;
        }
        let weighted_loss = loss * w;
        Ok((weighted_loss, loss))
    }

    /// Computes the symmetry constraint
    fn compute_symmetry(
        &self,
        pos: &Array1<f64>,
        grad: &mut Array1<f64>,
    ) -> PlaceResult<(f64, f64)> {
        // Symmetry axis: the boundary midpoint is used as the default
        let mut axes = Array1::<f64>::zeros(self.data.num_sym_groups);
        for i in 0..self.data.num_sym_groups {
            if self.data.sym_group_vertical[i] {
                // Vertical symmetry: the axis is an x coordinate
                axes[i] = (self.data.boundary_xl + self.data.boundary_xh) / 2.0;
            } else {
                // Horizontal symmetry: the axis is a y coordinate
                axes[i] = (self.data.boundary_yl + self.data.boundary_yh) / 2.0;
            }
        }

        let mut tmp_grad = Array1::<f64>::zeros(grad.len());
        let loss = crate::constraints::compute_symmetry(pos, &self.data, &axes, &mut tmp_grad)?;
        let w = self.lambdas.symmetry;
        for i in 0..grad.len() {
            grad[i] += tmp_grad[i] * w;
        }
        let weighted_loss = loss * w;
        Ok((weighted_loss, loss))
    }

    /// Computes the boundary constraint
    fn compute_boundary(
        &self,
        pos: &Array1<f64>,
        grad: &mut Array1<f64>,
    ) -> PlaceResult<(f64, f64)> {
        let mut tmp_grad = Array1::<f64>::zeros(grad.len());
        let loss = crate::constraints::compute_boundary(pos, &self.data, &mut tmp_grad)?;
        let w = self.lambdas.boundary * self.params.boundary_weight;
        for i in 0..grad.len() {
            grad[i] += tmp_grad[i] * w;
        }
        let weighted_loss = loss * w;
        Ok((weighted_loss, loss))
    }

    /// Computes the well cluster constraint (TODO)
    fn compute_well_group(
        &self,
        _pos: &Array1<f64>,
        _grad: &mut Array1<f64>,
    ) -> PlaceResult<(f64, f64)> {
        // TODO: implement the well cluster constraint
        Ok((0.0, 0.0))
    }

    /// Computes the array constraint (TODO)
    fn compute_array(
        &self,
        _pos: &Array1<f64>,
        _grad: &mut Array1<f64>,
    ) -> PlaceResult<(f64, f64)> {
        // TODO: implement the array constraint
        Ok((0.0, 0.0))
    }

    /// Computes the compaction constraint
    fn compute_compaction(
        &self,
        pos: &Array1<f64>,
        grad: &mut Array1<f64>,
    ) -> PlaceResult<(f64, f64)> {
        let loss = crate::constraints::compute_compaction(pos, &self.data, grad)?;
        let weighted_loss = loss * self.params.compaction_weight;
        Ok((weighted_loss, loss))
    }

    /// Updates gamma (wirelength gamma decay)
    pub fn update_gamma(&mut self) {
        self.wl_gamma *= self.params.wl_gamma_decay;
    }

    pub fn set_overlap_inflation(&mut self, inflation: &[f64]) {
        if inflation.len() == self.overlap_inflation.len() {
            self.overlap_inflation.copy_from_slice(inflation);
        }
    }

    pub fn set_overlap_halo(&mut self, halo: &[f64]) {
        if halo.len() == self.overlap_halo.len() {
            self.overlap_halo.copy_from_slice(halo);
        }
    }

    /// Return the latest raw WL components from forward().
    pub fn wl_components(&self) -> (f64, f64) {
        (*self.last_wl_base.borrow(), *self.last_wl_ap.borrow())
    }

    /// Return the latest weighted loss breakdown from forward().
    pub fn loss_breakdown(&self) -> LossBreakdown {
        *self.last_breakdown.borrow()
    }
}

#[cfg(test)]
mod tests {}
