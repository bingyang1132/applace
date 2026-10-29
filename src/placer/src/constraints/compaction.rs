//! Compaction (centroid) penalty
//!
//! Encourage cells to stay close to their centroid to reduce whitespace.

use crate::data::PlaceData;
use crate::error::PlaceResult;
use ndarray::Array1;

/// Compute centroid compaction loss.
///
/// Loss = sum_i ((x_i - mean_x)^2 + (y_i - mean_y)^2)
pub fn compute_compaction(
    pos: &Array1<f64>,
    data: &PlaceData,
    grad: &mut Array1<f64>,
) -> PlaceResult<f64> {
    let n = data.num_cells;
    if n == 0 {
        return Ok(0.0);
    }
    let nf = n as f64;
    let mut sx = 0.0;
    let mut sy = 0.0;
    for i in 0..n {
        sx += pos[i];
        sy += pos[i + n];
    }
    let mean_x = sx / nf;
    let mean_y = sy / nf;

    let mut loss = 0.0;
    for i in 0..n {
        let dx = pos[i] - mean_x;
        let dy = pos[i + n] - mean_y;
        loss += dx * dx + dy * dy;
        grad[i] += 2.0 * dx;
        grad[i + n] += 2.0 * dy;
    }

    Ok(loss)
}
