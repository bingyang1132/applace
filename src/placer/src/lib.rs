//! APPlace Placer - Global Placement Module
//!
//! Gradient-based global placer with multiple constraint support
//!
//! ## Main Components
//!
//! - `PlacementConfig`: Placement parameters (loaded from JSON)
//! - `PlaceData`: Data extracted from CircuitDb
//! - `PlaceModel`: Objective function and gradient computation
//! - `Placer`: Main placer
//! - `AdamOptimizer`: Adam optimizer
//!
//! ## Quick Start
//!
//! ```no_run
//! use applace_placer::{PlacementConfig, Placer};
//! use applace_core::CircuitDb;
//!
//! // Load parameters
//! let params = PlacementConfig::from_json("placement_config.json").unwrap();
//!
//! // Create placer
//! let mut db = CircuitDb::new();
//! let mut placer = Placer::new(&db, params).unwrap();
//!
//! // Run placement
//! let result = placer.run().unwrap();
//! println!("Converged: {}, Loss: {:.6e}", result.converged, result.best_loss);
//!
//! // Export visualization (loss curve + cell positions)
//! placer.export_visualization("output/").unwrap();
//!
//! // Write back positions
//! placer.write_positions(&mut db).unwrap();
//! ```
//!
//! ## Visualization
//!
//! The placer can export visualization data for analysis:
//!
//! ```no_run
//! # use applace_placer::{Placer, PlacementConfig};
//! # use applace_core::CircuitDb;
//! # let mut db = CircuitDb::new();
//! # let params = PlacementConfig::default();
//! # let mut placer = Placer::new(&db, params).unwrap();
//! # placer.run().unwrap();
//! // Export all visualization data
//! placer.export_visualization("results/").unwrap();
//!
//! // This creates:
//! // - results/loss_history.csv
//! // - results/cell_positions.csv
//! // - results/plot_loss.py (run to generate loss curve)
//! // - results/plot_positions.py (run to visualize positions)
//! ```

// Three clippy style lints are allowed for this crate as a whole. Each is a style
// lint whose advice would make this code harder to read, and the reason is written
// out next to it; everything clippy found that was worth fixing was fixed rather
// than allowed.
//
//   type_complexity      the data extractors return tuples of parallel arrays; a type
//                        alias per tuple would name nothing the reader cannot see.
//   too_many_arguments   the gradient entry points take flat parameter lists.
//   needless_range_loop  index loops in the numeric kernel, where the index is the
//                        thing being reasoned about.
#![allow(
    clippy::type_complexity,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]

pub mod constraints;
pub mod data;
pub mod error;
pub mod gp_controller;
pub mod model;
pub mod optimizer;
pub mod params;
pub mod placer;
pub mod visualization;

// Re-export commonly used types
pub use data::PlaceData;
pub use error::{PlaceError, PlaceResult};
pub use gp_controller::{Candidate, IterRow, RunSummary, OUTPUT_SCHEMA_VERSION};
pub use model::{LagrangeMultipliers, PlaceModel};
pub use optimizer::{AdamOptimizer, LineSearch};
pub use params::PlacementConfig;
pub use placer::{PlacementResult, Placer, TailDiagnosticsIterRow, TailDiagnosticsSnapshot};
pub use visualization::{export_all, export_loss_csv, export_positions_csv};
