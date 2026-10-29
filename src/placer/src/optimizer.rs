//! Optimizer implementations
//!
//! Adam + line-search utilities used by placer.

use ndarray::Array1;

/// Adam optimizer with explicit state, so it can be snapshotted/restored.
#[derive(Debug, Clone)]
pub struct AdamOptimizer {
    pub lr: f64,
    pub beta1: f64,
    pub beta2: f64,
    pub eps: f64,
    t: usize,
    m: Vec<f64>,
    v: Vec<f64>,
}

impl AdamOptimizer {
    pub fn new(size: usize, lr: f64, beta1: f64, beta2: f64) -> Self {
        Self {
            lr,
            beta1,
            beta2,
            eps: 1e-8,
            t: 0,
            m: vec![0.0; size],
            v: vec![0.0; size],
        }
    }

    pub fn step(&mut self, pos: &Array1<f64>, grad: &Array1<f64>) -> Array1<f64> {
        self.t += 1;
        let t = self.t as i32;
        let b1t = 1.0 - self.beta1.powi(t);
        let b2t = 1.0 - self.beta2.powi(t);
        let mut out = vec![0.0; pos.len()];

        for i in 0..pos.len() {
            let g = grad[i];
            self.m[i] = self.beta1 * self.m[i] + (1.0 - self.beta1) * g;
            self.v[i] = self.beta2 * self.v[i] + (1.0 - self.beta2) * g * g;
            let mhat = self.m[i] / b1t.max(1e-20);
            let vhat = self.v[i] / b2t.max(1e-20);
            out[i] = pos[i] - self.lr * mhat / (vhat.sqrt() + self.eps);
        }

        Array1::from_vec(out)
    }

    pub fn reset(&mut self) {
        self.m.fill(0.0);
        self.v.fill(0.0);
        self.t = 0;
    }

    pub fn iteration(&self) -> usize {
        self.t
    }

    pub fn state_norms(&self) -> (f64, f64) {
        let m = self.m.iter().map(|x| x * x).sum::<f64>().sqrt();
        let v = self.v.iter().map(|x| x * x).sum::<f64>().sqrt();
        (m, v)
    }

    pub fn state_dim(&self) -> usize {
        self.m.len()
    }
}

/// Armijo backtracking line-search.
pub struct LineSearch {
    pub ls_step_init: f64,
    pub ls_step_shrink: f64,
    pub ls_step_min: f64,
    pub c1: f64,
    pub max_iter: usize,
}

impl LineSearch {
    pub fn new(ls_step_init: f64, ls_step_shrink: f64, ls_step_min: f64, c1: f64) -> Self {
        Self {
            ls_step_init,
            ls_step_shrink,
            ls_step_min,
            c1,
            max_iter: 20,
        }
    }

    pub fn search<F>(
        &self,
        obj_fn: F,
        pos: &Array1<f64>,
        direction: &Array1<f64>,
        grad: &Array1<f64>,
        current_loss: f64,
    ) -> f64
    where
        F: Fn(&Array1<f64>) -> f64,
    {
        let directional_derivative = grad.dot(direction);
        if directional_derivative >= 0.0 {
            log::warn!(
                "Direction is not a descent direction (grad^T d = {:.4e})",
                directional_derivative
            );
            return self.ls_step_min;
        }

        let mut alpha = self.ls_step_init;
        for _ in 0..self.max_iter {
            let new_pos = pos + &(direction * alpha);
            let new_loss = obj_fn(&new_pos);
            let armijo_rhs = current_loss + self.c1 * alpha * directional_derivative;
            if new_loss <= armijo_rhs {
                return alpha;
            }
            alpha *= self.ls_step_shrink;
            if alpha < self.ls_step_min {
                break;
            }
        }
        self.ls_step_min
    }
}

pub fn compute_descent_direction(grad: &Array1<f64>) -> Array1<f64> {
    -grad
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_adam_basic() {
        let mut optimizer = AdamOptimizer::new(1, 0.5, 0.9, 0.999);
        let mut pos = Array1::from_vec(vec![10.0]);
        let grad = Array1::from_vec(vec![20.0]);
        let new_pos = optimizer.step(&pos, &grad);
        assert!(new_pos[0] < pos[0]);

        pos = new_pos;
        for _ in 0..500 {
            let grad = Array1::from_vec(vec![2.0 * pos[0]]);
            pos = optimizer.step(&pos, &grad);
        }
        assert_abs_diff_eq!(pos[0], 0.0, epsilon = 0.5);
    }

    #[test]
    fn test_adam_reset() {
        let mut optimizer = AdamOptimizer::new(2, 0.01, 0.9, 0.999);
        let pos = Array1::from_vec(vec![1.0, 2.0]);
        let grad = Array1::from_vec(vec![0.5, 0.3]);
        optimizer.step(&pos, &grad);
        optimizer.step(&pos, &grad);
        assert_eq!(optimizer.iteration(), 2);
        optimizer.reset();
        assert_eq!(optimizer.iteration(), 0);
    }

    #[test]
    fn test_line_search_basic() {
        let obj_fn = |pos: &Array1<f64>| pos[0] * pos[0];
        let ls = LineSearch::new(1.0, 0.5, 0.001, 0.0001);
        let pos = Array1::from_vec(vec![10.0]);
        let grad = Array1::from_vec(vec![20.0]);
        let direction = Array1::from_vec(vec![-20.0]);
        let current_loss = 100.0;
        let alpha = ls.search(obj_fn, &pos, &direction, &grad, current_loss);
        assert!(alpha > 0.0);
        let new_pos = &pos + &(&direction * alpha);
        let new_loss = obj_fn(&new_pos);
        assert!(new_loss < current_loss);
    }

    #[test]
    fn test_descent_direction() {
        let grad = Array1::from_vec(vec![1.0, 2.0, -3.0]);
        let dir = compute_descent_direction(&grad);
        assert_eq!(dir[0], -1.0);
        assert_eq!(dir[1], -2.0);
        assert_eq!(dir[2], 3.0);
    }
}
