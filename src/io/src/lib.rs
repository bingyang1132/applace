//! APPlace IO - input/output parser module
//!
//! Implements the netlist/tech/pcell/layout parsers and writers

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

pub mod parser;
pub mod writer;
// pub mod ffi;
pub mod pcell;
pub mod stage_trace;

pub use parser::*;
pub use writer::*;
// pub use ffi::*;
pub use pcell::*;
pub use stage_trace::*;
