//! Optimizer performance tests
//!
//! Measures the performance and convergence speed of the Adam optimizer on different problems

use applace_placer::optimizer::AdamOptimizer;
use ndarray::Array1;
use std::time::Instant;

/// Test case: optimize the classic Rosenbrock function
/// f(x, y) = (a - x)^2 + b(y - x^2)^2
/// The global minimum is at (a, a^2); a=1, b=100 are the usual choices
///
/// Gradient:
/// df/dx = -2(a - x) - 4bx(y - x^2)
/// df/dy = 2b(y - x^2)
fn rosenbrock_function(pos: &Array1<f64>) -> f64 {
    let x = pos[0];
    let y = pos[1];
    let a = 1.0;
    let b = 100.0;

    (a - x).powi(2) + b * (y - x.powi(2)).powi(2)
}

fn rosenbrock_gradient(pos: &Array1<f64>) -> Array1<f64> {
    let x = pos[0];
    let y = pos[1];
    let a = 1.0;
    let b = 100.0;

    let dx = -2.0 * (a - x) - 4.0 * b * x * (y - x.powi(2));
    let dy = 2.0 * b * (y - x.powi(2));

    Array1::from_vec(vec![dx, dy])
}

/// Test case: optimize the Rastrigin function
/// f(x) = An + Σ[xi^2 - A*cos(2πxi)]
/// The global minimum is at the origin (0, 0, ..., 0)
fn rastrigin_function(pos: &Array1<f64>) -> f64 {
    let a = 10.0;
    let n = pos.len() as f64;
    let sum: f64 = pos
        .iter()
        .map(|&xi| xi.powi(2) - a * (2.0 * std::f64::consts::PI * xi).cos())
        .sum();

    a * n + sum
}

fn rastrigin_gradient(pos: &Array1<f64>) -> Array1<f64> {
    let a = 10.0;
    pos.mapv(|xi| {
        2.0 * xi + 2.0 * std::f64::consts::PI * a * (2.0 * std::f64::consts::PI * xi).sin()
    })
}

/// Test case: a simple quadratic function
/// f(x) = Σ(xi^2)
fn quadratic_function(pos: &Array1<f64>) -> f64 {
    pos.iter().map(|&xi| xi.powi(2)).sum()
}

fn quadratic_gradient(pos: &Array1<f64>) -> Array1<f64> {
    pos.mapv(|xi| 2.0 * xi)
}

/// Runs one optimization test
fn run_optimization_test(
    name: &str,
    initial_pos: Array1<f64>,
    obj_fn: fn(&Array1<f64>) -> f64,
    grad_fn: fn(&Array1<f64>) -> Array1<f64>,
    lr: f64,
    max_iter: usize,
    target_loss: f64,
) {
    println!("\n{:=<60}", "");
    println!("Test: {}", name);
    println!("{:=<60}", "");

    let mut optimizer = AdamOptimizer::new(initial_pos.len(), lr, 0.9, 0.999);
    let mut pos = initial_pos.clone();

    let start = Instant::now();
    let mut converged_at = None;

    println!("Initial position: {:?}", pos.as_slice().unwrap());
    println!("Initial loss: {:.6e}", obj_fn(&pos));
    println!();

    for iter in 0..max_iter {
        let grad = grad_fn(&pos);
        pos = optimizer.step(&pos, &grad);
        let loss = obj_fn(&pos);

        // Print once every so many iterations
        if iter % (max_iter / 10).max(1) == 0 || iter == max_iter - 1 {
            println!(
                "Iteration {:4}: loss = {:.6e}, position = {:?}",
                iter + 1,
                loss,
                pos.as_slice().unwrap()
            );
        }

        // Check for convergence
        if loss < target_loss && converged_at.is_none() {
            converged_at = Some(iter + 1);
        }
    }

    let elapsed = start.elapsed();

    println!();
    println!("Final position: {:?}", pos.as_slice().unwrap());
    println!("Final loss: {:.6e}", obj_fn(&pos));

    match converged_at {
        Some(iter) => println!(
            "Converged at iteration {} (target loss: {:.2e})",
            iter, target_loss
        ),
        None => println!("Did not reach the target loss {:.2e}", target_loss),
    }

    println!("Total time: {:.3}ms", elapsed.as_secs_f64() * 1000.0);
    println!(
        "Average per iteration: {:.3}μs",
        elapsed.as_micros() as f64 / max_iter as f64
    );
}

#[test]
fn test_quadratic_optimization() {
    let initial = Array1::from_vec(vec![10.0, -5.0, 8.0, -3.0]);
    run_optimization_test(
        "Quadratic function optimization (4D)",
        initial,
        quadratic_function,
        quadratic_gradient,
        0.1,
        1000,
        1e-6,
    );
}

#[test]
fn test_rosenbrock_optimization() {
    let initial = Array1::from_vec(vec![-1.0, 2.0]);
    run_optimization_test(
        "Rosenbrock function optimization",
        initial,
        rosenbrock_function,
        rosenbrock_gradient,
        0.001,
        10000,
        1e-4,
    );
}

#[test]
fn test_rastrigin_optimization() {
    let initial = Array1::from_vec(vec![2.0, -2.0, 1.5, -1.5]);
    run_optimization_test(
        "Rastrigin function optimization (4D)",
        initial,
        rastrigin_function,
        rastrigin_gradient,
        0.01,
        5000,
        1.0,
    );
}

#[test]
fn test_high_dimensional_quadratic() {
    // Measure optimization performance in high dimensions
    let dim = 100;
    let initial = Array1::from_vec(vec![5.0; dim]);
    run_optimization_test(
        "High-dimensional quadratic function optimization (100D)",
        initial,
        quadratic_function,
        quadratic_gradient,
        0.1,
        2000,
        1e-5,
    );
}

/// Performance comparison test: measures the speed of a single optimization step
#[test]
fn test_optimization_speed() {
    println!("\n{:=<60}", "");
    println!("Optimizer speed test");
    println!("{:=<60}", "");

    let dims = vec![10, 50, 100, 500, 1000];

    for dim in dims {
        let mut optimizer = AdamOptimizer::new(dim, 0.001, 0.9, 0.999);
        let pos = Array1::from_vec(vec![1.0; dim]);
        let grad = Array1::from_vec(vec![0.1; dim]);

        let iterations = 1000;
        let start = Instant::now();

        let mut current_pos = pos.clone();
        for _ in 0..iterations {
            current_pos = optimizer.step(&current_pos, &grad);
        }

        let elapsed = start.elapsed();

        println!(
            "Dimension {:4}: {} iterations took {:.3}ms, average per step {:.3}μs",
            dim,
            iterations,
            elapsed.as_secs_f64() * 1000.0,
            elapsed.as_micros() as f64 / iterations as f64
        );
    }
}

/// Tests the convergence properties of the optimizer
#[test]
fn test_convergence_quality() {
    println!("\n{:=<60}", "");
    println!("Convergence quality test");
    println!("{:=<60}", "");

    // Check convergence for different learning rates
    let learning_rates = vec![0.001, 0.01, 0.1];
    let initial = Array1::from_vec(vec![10.0; 10]);

    for &lr in &learning_rates {
        let mut optimizer = AdamOptimizer::new(10, lr, 0.9, 0.999);
        let mut pos = initial.clone();

        let start_loss = quadratic_function(&pos);

        for _ in 0..1000 {
            let grad = quadratic_gradient(&pos);
            pos = optimizer.step(&pos, &grad);
        }

        let final_loss = quadratic_function(&pos);
        let reduction = (start_loss - final_loss) / start_loss * 100.0;

        println!(
            "Learning rate {:.3}: initial loss {:.2e}, final loss {:.2e}, reduction {:.2}%",
            lr, start_loss, final_loss, reduction
        );
    }
}
