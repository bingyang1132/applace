//! Error types for the placer module

use thiserror::Error;

/// Placement error types
#[derive(Error, Debug)]
pub enum PlaceError {
    #[error("Data initialization error: {0}")]
    DataInitError(String),

    #[error("Optimization error: {0}")]
    OptimizationError(String),

    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("JSON error: {0}")]
    JsonError(#[from] serde_json::Error),

    #[error("Array shape error: {0}")]
    ShapeError(String),

    #[error("Convergence failed: {0}")]
    ConvergenceError(String),
}

/// Placement result type alias
pub type PlaceResult<T> = Result<T, PlaceError>;
