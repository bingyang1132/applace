//! Main placer implementation
//!
//! The main placer, integrating the optimizer and the constraint model

use crate::error::{PlaceError, PlaceResult};
use crate::gp_controller::{
    better_base_composite, better_tail_composite, Candidate, CandidateSummaryEntry,
    ConflictMetrics, CoverMetrics, InflatedGeometrySummary, IterRow, LambdaSnapshot, Phase,
    RunSummary, CONTROLLER_VERSION, TAIL_MODE,
};
use crate::optimizer::{compute_descent_direction, AdamOptimizer, LineSearch};
use crate::{PlaceData, PlaceModel, PlacementConfig};
use applace_core::CircuitDb;
use ndarray::Array1;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use serde::Serialize;
use std::cmp::Ordering;
use std::collections::{HashSet, VecDeque};
use std::path::Path;

/// Which predicate decides that a de-overlap move flew over another device.
/// `lcrRetryCrossCriterion`, parsed once per pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CrossCriterion {
    /// The mover changed sides relative to a local neighbour.
    Order,
    /// The swept region covered more than the threshold of another device.
    Cover,
    /// Either of the two — they catch different failures.
    Either,
}

impl CrossCriterion {
    fn from_param(s: &str) -> Self {
        match s {
            "cover" => Self::Cover,
            "either" => Self::Either,
            // `validate()` rejects anything else, so this is the `"order"` arm.
            _ => Self::Order,
        }
    }

    fn wants_order(self) -> bool {
        matches!(self, Self::Order | Self::Either)
    }

    fn wants_cover(self) -> bool {
        matches!(self, Self::Cover | Self::Either)
    }
}

/// Tolerance for the "flew over a local neighbour" geometry, in DBU. Same spirit
/// as the overlap test's `1e-12`: exactly on the line is not inside it.
const CROSSING_EPS: f64 = 1e-12;

/// Cap on how many individual crossing events `lcr_retry_summary.json` spells out. The
/// per-round counts are always complete; only the itemized list is capped, because a
/// pathological case can produce thousands of them and this is an audit artifact, not
/// an input to anything.
const LCR_CROSSING_RECORD_CAP: usize = 1000;

fn stage_trace_enabled() -> bool {
    matches!(
        std::env::var("HIER_STAGE_TRACE").ok().as_deref(),
        Some("1" | "true" | "TRUE" | "yes" | "YES")
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Axis {
    X,
    Y,
}

#[derive(Debug, Clone, Copy)]
struct AffineTerm {
    var: Option<usize>,
    scale: f64,
    bias: f64,
}

#[derive(Debug, Clone)]
struct ReducedVarMeta {
    axis: Axis,
}

#[derive(Debug, Clone)]
struct SymReducedMap {
    full_to_reduced: Vec<AffineTerm>,
    var_terms: Vec<Vec<(usize, f64, f64)>>,
    vars: Vec<ReducedVarMeta>,
}

impl SymReducedMap {
    fn expand(&self, z: &Array1<f64>) -> Array1<f64> {
        let mut full = vec![0.0; self.full_to_reduced.len()];
        for (i, t) in self.full_to_reduced.iter().enumerate() {
            full[i] = if let Some(v) = t.var {
                t.scale * z[v] + t.bias
            } else {
                t.bias
            };
        }
        Array1::from_vec(full)
    }

    fn reduce_grad(&self, grad_full: &Array1<f64>) -> Array1<f64> {
        let mut gz = vec![0.0; self.vars.len()];
        for (i, t) in self.full_to_reduced.iter().enumerate() {
            if let Some(v) = t.var {
                gz[v] += t.scale * grad_full[i];
            }
        }
        Array1::from_vec(gz)
    }

    fn reduce_pos(&self, pos_full: &Array1<f64>) -> Array1<f64> {
        let mut z = vec![0.0; self.vars.len()];
        let mut cnt = vec![0usize; self.vars.len()];
        for v in 0..self.vars.len() {
            for (idx, scale, bias) in &self.var_terms[v] {
                if scale.abs() < 1e-12 {
                    continue;
                }
                z[v] += (pos_full[*idx] - *bias) / *scale;
                cnt[v] += 1;
            }
            if cnt[v] > 0 {
                z[v] /= cnt[v] as f64;
            }
        }
        Array1::from_vec(z)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TailState {
    Hold,
}

#[derive(Debug, Clone, Serialize)]
pub struct TailDiagnosticsIterRow {
    pub iter: usize,
    pub phase: String,
    pub wirelength_main: f64,
    pub overlap_loss: f64,
    pub symmetry_loss: f64,
    pub anchor_loss: f64,
    pub total_loss: f64,
    pub lambda_overlap: f64,
    pub lambda_symmetry: f64,
    pub lambda_boundary: f64,
    pub anchor_w_strong: f64,
    pub anchor_w_mid: f64,
    pub anchor_w_weak: f64,
    pub mobility_min: f64,
    pub mobility_max: f64,
    pub grad_norm_total: f64,
    pub grad_norm_wirelength: f64,
    pub grad_norm_overlap: f64,
    pub grad_norm_symmetry: f64,
    pub grad_norm_anchor: f64,
    pub update_norm_pre_damping: f64,
    pub update_norm_post_damping: f64,
    pub applied_step_norm: f64,
    pub disp_all_max: f64,
    pub disp_all_rms: f64,
    pub disp_all_median: f64,
    pub disp_conflict_max: f64,
    pub disp_conflict_rms: f64,
    pub disp_conflict_median: f64,
    pub disp_nonconflict_max: f64,
    pub disp_nonconflict_rms: f64,
    pub disp_nonconflict_median: f64,
    pub disp_top10_max: f64,
    pub disp_top10_rms: f64,
    pub disp_top10_median: f64,
    pub disp_mid40_max: f64,
    pub disp_mid40_rms: f64,
    pub disp_mid40_median: f64,
    pub disp_bot50_max: f64,
    pub disp_bot50_rms: f64,
    pub disp_bot50_median: f64,
    pub overlap_count: usize,
    pub overlap_total_area: f64,
    pub conflict_component_count: usize,
    pub largest_component_size: usize,
    pub full_variable_count: usize,
    pub reduced_variable_count: usize,
    pub pair_tie_count: usize,
    pub self_sym_tie_count: usize,
    pub movable_object_count: usize,
    pub adam_m_norm: f64,
    pub adam_v_norm: f64,
    pub reduced_state_dim: usize,
    pub expanded_dim: usize,
    pub shape_consistent: bool,
    pub inflation_scale_current: f64,
    pub inflated_component_ids: Vec<usize>,
    pub inflation_object_count: usize,
    pub dist_to_final_max: Option<f64>,
    pub dist_to_final_rms: Option<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct TailDiagnosticsSnapshot {
    pub interval: usize,
    pub rows: Vec<TailDiagnosticsIterRow>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DeoverlapObjectKind {
    Singleton,
    Pair,
    SelfSym,
}

impl DeoverlapObjectKind {
    fn as_str(self) -> &'static str {
        match self {
            Self::Singleton => "singleton",
            Self::Pair => "sym_pair",
            Self::SelfSym => "self_sym",
        }
    }
}

#[derive(Debug, Clone)]
struct DeoverlapObject {
    kind: DeoverlapObjectKind,
    members: Vec<usize>,
    area: f64,
    is_vert: Option<bool>,
    axis: Option<f64>,
    primary_lo: Option<usize>,
    primary_hi: Option<usize>,
}

/// What one accepted de-overlap move produced for the crossing trigger: the events
/// themselves plus the bookkeeping that keeps the two open design questions (which
/// predicate, which neighbour set) answerable from a log instead of from an argument.
#[derive(Debug, Clone, Default)]
struct CrossingScan {
    rows: Vec<DeoverlapCrossingRow>,
    /// (moved member, other device) pairs the sweep touched at all.
    covered_any: usize,
    /// … and how many cleared each of the three thresholds under measurement, so one
    /// default run reports all three bands.
    cover_over_25: usize,
    cover_over_50: usize,
    cover_over_75: usize,
    /// How many pairs the superseded order-inversion predicate would have flagged.
    order_reversed: usize,
}

/// One accepted de-overlap move that inverted the mover's relative position against a
/// local neighbour — "flew over its head" to reach a legal spot.
///
/// Recorded unconditionally (it is pure bookkeeping and costs one pass over the cells
/// per accepted move); only the *trigger* is gated by `lcrRetryOnCrossing`.
#[derive(Debug, Clone, Serialize, Default, PartialEq)]
struct DeoverlapCrossingRow {
    component_id: usize,
    object_id: usize,
    object_kind: String,
    moved_cell: usize,
    crossed_cell: usize,
    /// The trigger: how much of the crossed device the swept region covers, in [0, 1].
    cover_frac: f64,
    /// Superseded predicate — the mover changed sides along x / y. Reporting only.
    order_reversed_x: bool,
    order_reversed_y: bool,
    /// The N1 / N2 locality rules the order-based trigger needed. Reporting only.
    in_component: bool,
    adjacent_to_path: bool,
    /// The first candidate definition. Reporting only.
    swept_covers_centre: bool,
}

#[derive(Debug, Clone, Serialize, Default)]
struct DeoverlapMoveRow {
    component_id: usize,
    object_id: usize,
    object_kind: String,
    object_area: f64,
    attempt_idx: usize,
    tier: String,
    ring: usize,
    dx: f64,
    dy: f64,
    success: bool,
    reason: String,
    w_ref: f64,
    h_ref: f64,
    a_ref: f64,
    w_tail: f64,
    h_tail: f64,
    r_raw: f64,
    r_eff: f64,
    long_axis: String,
    candidates_tested: usize,
    candidates_feasible: usize,
    rejected_by_area_cap: usize,
    best_score: f64,
    d_ell: f64,
    e_w: f64,
    e_h: f64,
    e_a: f64,
    e_expand: f64,
    w_after: f64,
    h_after: f64,
    a_after: f64,
    area_increase_ratio: f64,
}

#[derive(Debug, Clone, Serialize, Default)]
struct DeoverlapSummary {
    enabled: bool,
    attempted: bool,
    force_sym_applied: bool,
    total_components_before: usize,
    total_components_after: usize,
    resolved_components: usize,
    unresolved_components: usize,
    overlap_pairs_before: usize,
    overlap_pairs_after: usize,
    move_count: usize,
    failed_attempt_count: usize,
    infl_tail_min: f64,
    infl_tail_mean: f64,
    infl_tail_max: f64,
    w_ref: f64,
    h_ref: f64,
    a_ref: f64,
    w_tail: f64,
    h_tail: f64,
    a_tail: f64,
    r_raw: f64,
    r_eff: f64,
    long_axis: String,
    total_candidates_tested: usize,
    total_candidates_feasible: usize,
    total_candidates_rejected_area_cap: usize,
    max_area_increase_ratio_observed: f64,
    final_width_ratio_to_ref: f64,
    final_height_ratio_to_ref: f64,
    final_area_ratio_to_ref: f64,
}

/// The slice of `Placer` state a multi-phase pass mutates and does not itself reset.
/// Captured before round 0 and restored before each retry pass.
struct EntryState {
    pos: Array1<f64>,
    z: Array1<f64>,
    best_pos: Array1<f64>,
    best_loss: f64,
    phi: Vec<f64>,
    phi_tilde: Vec<f64>,
    lambdas: crate::model::LagrangeMultipliers,
    lambda_ap: f64,
    checkpoint_pos: Option<Array1<f64>>,
    tail_state: TailState,
}

/// Everything a finished multi-phase pass produces that the caller and the stage
/// binary go on to read. Captured so the retry loop can hand back its *best* pass
/// rather than whichever one happened to run last -- the tail re-solves globally each
/// round, so a later round can trade one conflict for another and come out worse.
struct PassOutputs {
    result: PlacementResult,
    pos: Array1<f64>,
    z: Array1<f64>,
    phi: Vec<f64>,
    phi_tilde: Vec<f64>,
    base_end_pos: Option<Array1<f64>>,
    tail_end_pos: Option<Array1<f64>>,
    best_composite_pos: Option<Array1<f64>>,
    deoverlap_end_pos: Option<Array1<f64>>,
    deoverlap_summary: Option<DeoverlapSummary>,
    deoverlap_moves: Vec<DeoverlapMoveRow>,
    deoverlap_components_before: Vec<Vec<usize>>,
    deoverlap_components_after: Vec<Vec<usize>>,
    deoverlap_infl_tail_snapshot: Vec<f64>,
    deoverlap_unresolved_cells: Vec<usize>,
    deoverlap_crossings: Vec<DeoverlapCrossingRow>,
    deoverlap_crossing_cells: Vec<usize>,
    summary: Option<RunSummary>,
    selected_label: Option<String>,
    base_selected_label: Option<String>,
    tail_selected_label: Option<String>,
    base_end_ref_pos: Option<Array1<f64>>,
    rows: Vec<IterRow>,
    candidates: Vec<Candidate>,
    base_candidates: Vec<Candidate>,
    tail_candidates: Vec<Candidate>,
    tail_diag_rows: Vec<TailDiagnosticsIterRow>,
    tail_diag_positions: Vec<Vec<f64>>,
    tail_diag_snapshots: Vec<TailDiagnosticsIterRow>,
    conflict_component_count: usize,
    largest_component_size: usize,
    overlap_count: usize,
    overlap_total_area: f64,
    inflated_component_ids: Vec<usize>,
    inflation_object_count: usize,
    overlap_halo: Vec<f64>,
}

/// Audit trail for the LCR upgrade-retry loop.
///
/// `rounds_run == 0` means the loop never fired — the discrete de-overlap pass resolved
/// every conflict on its first try *and* did not have to move a device across a local
/// neighbour to do it. The summary is only emitted when the loop actually ran, so a
/// non-triggering case keeps its output byte-identical.
///
/// Whether a given case enters the loop is a property of that case, not of the
/// feature: several of the example cases do enter it even with the crossing
/// trigger switched off.
#[derive(Debug, Clone, Serialize, Default)]
struct LcrRetrySummary {
    enabled: bool,
    max_rounds: usize,
    infl_step: f64,
    /// Number of *extra* multi-phase passes executed (0 = the feature never fired).
    rounds_run: usize,
    /// True when a later pass reached `overlap_pairs_after == 0`.
    converged: bool,
    /// `overlap_pairs_after` observed at the end of each pass, round 0 first.
    overlap_pairs_after_by_round: Vec<usize>,
    /// `unresolved_components` observed at the end of each pass, round 0 first.
    unresolved_components_by_round: Vec<usize>,
    /// How many cells carried an inflated halo entering each retry pass.
    inflated_cells_by_round: Vec<usize>,
    /// Final per-cell factor for every cell that ever got inflated: `(cell, factor)`.
    inflated_cells: Vec<(usize, f64)>,
    /// Which pass supplied the returned placement -- the best one, not necessarily
    /// the last.
    accepted_round: usize,
    /// `overlap_pairs_after` of the accepted pass.
    best_overlap_pairs_after: usize,
    /// Set when the loop exhausted `max_rounds` with conflicts still present.
    exhausted: bool,

    // ---- second trigger (2026-09-22): "flew over a local neighbour" ----
    // Every field below is absent from the JSON unless the trigger is switched on and
    // actually had something to say, so `lcrRetryOnCrossing=false` — and any run
    // that never sees a crossing — serializes exactly the bytes it did before.
    /// True when the crossing trigger was armed for this run.
    #[serde(skip_serializing_if = "is_false")]
    crossing_trigger_enabled: bool,
    /// How many crossing events the de-overlap pass produced in each pass, round 0 first.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    crossing_events_by_round: Vec<usize>,
    /// How many distinct devices were inflated *because they crossed*, per round.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    crossing_cells_by_round: Vec<usize>,
    /// Which trigger(s) put each round on the board: `overlap`, `crossing`,
    /// `overlap+crossing`, or `none` for a pass that was already clean.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    trigger_by_round: Vec<String>,
    /// Every crossing event observed, in deterministic order.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    crossings: Vec<LcrCrossingRecord>,
    /// Crossing events left in the accepted pass.
    #[serde(skip_serializing_if = "is_zero_usize")]
    best_crossing_events: usize,
    /// Set when `crossings` hit `LCR_CROSSING_RECORD_CAP`; the per-round counts above
    /// stay complete.
    #[serde(skip_serializing_if = "is_false")]
    crossings_truncated: bool,
    /// The coverage threshold the trigger used.
    #[serde(skip_serializing_if = "is_zero_f64")]
    crossing_cover_frac: f64,
    /// Which predicate the trigger used: `order`, `cover` or `either`.
    #[serde(skip_serializing_if = "String::is_empty")]
    crossing_criterion: String,
}

fn is_zero_f64(v: &f64) -> bool {
    *v == 0.0
}

fn is_false(b: &bool) -> bool {
    !*b
}

fn is_zero_usize(v: &usize) -> bool {
    *v == 0
}

/// Crossing trigger: one "this device reached a legal position only by moving to the
/// other side of that local neighbour" event, tagged with the retry pass it happened in.
#[derive(Debug, Clone, Serialize, Default)]
struct LcrCrossingRecord {
    round: usize,
    component_id: usize,
    object_id: usize,
    object_kind: String,
    /// The member of the moved object that changed sides.
    moved_cell: usize,
    /// The device it flew over.
    crossed_cell: usize,
    /// The trigger value: fraction of that device the swept region covered.
    cover_frac: f64,
    /// Superseded predicate, kept for the record: which axis the order was inverted on.
    order_reversed_x: bool,
    order_reversed_y: bool,
    /// Diagnostic: would the rejected "swept region covers the centre" definition have
    /// reported this one too?
    swept_covers_centre: bool,
}

#[derive(Debug, Clone, Copy, Default)]
struct DeoverlapBBox {
    xl: f64,
    yl: f64,
    xh: f64,
    yh: f64,
    valid: bool,
}

impl DeoverlapBBox {
    fn width(self) -> f64 {
        if self.valid {
            (self.xh - self.xl).max(0.0)
        } else {
            0.0
        }
    }

    fn height(self) -> f64 {
        if self.valid {
            (self.yh - self.yl).max(0.0)
        } else {
            0.0
        }
    }

    fn area(self) -> f64 {
        self.width() * self.height()
    }

    fn extend_rect(&mut self, xl: f64, yl: f64, xh: f64, yh: f64) {
        if !self.valid {
            self.xl = xl;
            self.yl = yl;
            self.xh = xh;
            self.yh = yh;
            self.valid = true;
            return;
        }
        self.xl = self.xl.min(xl);
        self.yl = self.yl.min(yl);
        self.xh = self.xh.max(xh);
        self.yh = self.yh.max(yh);
    }

    fn union(self, other: Self) -> Self {
        if !self.valid {
            return other;
        }
        if !other.valid {
            return self;
        }
        Self {
            xl: self.xl.min(other.xl),
            yl: self.yl.min(other.yl),
            xh: self.xh.max(other.xh),
            yh: self.yh.max(other.yh),
            valid: true,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct DeoverlapSearchContext {
    w_ref: f64,
    h_ref: f64,
    a_ref: f64,
    w_tail: f64,
    h_tail: f64,
    r_raw: f64,
    r_eff: f64,
    long_axis: Axis,
    lambda_expand: f64,
    lambda_area: f64,
    lambda_topo: f64,
    rho_max: f64,
    use_elliptical: bool,
}

#[derive(Debug, Clone, Copy, Default)]
struct DeoverlapCandidateEval {
    score: f64,
    d_ell: f64,
    e_w: f64,
    e_h: f64,
    e_a: f64,
    e_expand: f64,
    dx: f64,
    dy: f64,
    ring_hit: usize,
    w_after: f64,
    h_after: f64,
    a_after: f64,
    area_increase_ratio: f64,
}

#[derive(Debug, Clone, Default)]
struct DeoverlapSearchAttempt {
    best: Option<DeoverlapCandidateEval>,
    best_member_xy: Vec<(f64, f64)>,
    candidates_tested: usize,
    candidates_feasible: usize,
    rejected_by_area_cap: usize,
}

/// Placement result
#[derive(Debug, Clone)]
pub struct PlacementResult {
    /// Whether it converged
    pub converged: bool,

    /// Number of iterations actually run
    pub num_iters: usize,

    /// Final loss
    pub final_loss: f64,

    /// Best loss
    pub best_loss: f64,

    /// Final positions [x0, x1, ..., x_{N-1}, y0, y1, ..., y_{N-1}]
    pub positions: Array1<f64>,

    /// Loss history
    pub loss_history: Vec<f64>,
}

/// Main placer
pub struct Placer {
    /// Placement model
    model: PlaceModel,

    /// Adam optimizer
    optimizer: AdamOptimizer,

    /// Separate Adam optimizer for phi (AP-WL)
    phi_optimizer: Option<AdamOptimizer>,

    /// Line search (optional)
    line_search: Option<LineSearch>,

    /// Current positions
    pos: Array1<f64>,

    /// Reduced-variable state when symmetry tying is enabled.
    reduced_map: Option<SymReducedMap>,
    z: Array1<f64>,

    /// Current gradient
    grad: Array1<f64>,

    /// Best positions
    best_pos: Array1<f64>,

    /// Best loss
    best_loss: f64,

    /// Loss history
    loss_history: Vec<f64>,

    /// Per-iteration displacement history (rms, max)
    move_history: Vec<(f64, f64)>,

    /// Externally supplied initial positions (when set, random initialization is skipped)
    has_external_init: bool,

    /// AP-WL phi in optimizer space: phi_tilde = phi / PI in [-1, 1).
    phi_tilde: Vec<f64>,

    /// Checkpoint from base GP.
    checkpoint_pos: Option<Array1<f64>>,
    /// Frozen semantic output: end of base stage.
    base_end_pos: Option<Array1<f64>>,
    /// Frozen semantic output: end of tail stage before candidate reselection.
    tail_end_pos: Option<Array1<f64>>,

    tail_state: TailState,

    /// Conflict-component data for tail stage.
    conflict_flags: Vec<bool>,
    small_conflict_flags: Vec<bool>,
    overlap_inflation: Vec<f64>,
    overlap_halo: Vec<f64>,
    /// Per-cell routability surcharge weight in `[0, 1]`; all-zero unless the caller
    /// supplied a congestion estimate.
    halo_weight: Vec<f64>,
    /// Scalar gain on `halo_weight`. `0.0` = feature off = byte-identical placement.
    halo_gain: f64,
    conflict_component_count: usize,
    largest_component_size: usize,
    overlap_count: usize,
    overlap_total_area: f64,
    inflated_component_ids: Vec<usize>,
    inflation_object_count: usize,
    tail_diag_rows: Vec<TailDiagnosticsIterRow>,
    tail_diag_positions: Vec<Vec<f64>>,
    tail_diag_snapshots: Vec<TailDiagnosticsIterRow>,
    /// Multi-phase per-iteration rows.
    iter_rows: Vec<IterRow>,
    /// Multi-phase collected candidate checkpoints (base + tail).
    candidates: Vec<Candidate>,
    base_candidates: Vec<Candidate>,
    tail_candidates: Vec<Candidate>,
    best_composite_pos: Option<Array1<f64>>,
    deoverlap_end_pos: Option<Array1<f64>>,
    deoverlap_summary: Option<DeoverlapSummary>,
    deoverlap_moves: Vec<DeoverlapMoveRow>,
    deoverlap_components_before: Vec<Vec<usize>>,
    deoverlap_components_after: Vec<Vec<usize>>,
    deoverlap_infl_tail_snapshot: Vec<f64>,
    /// Cells still overlapping after the discrete de-overlap pass (sorted, deterministic).
    /// Empty on every case whose LCR converges.
    deoverlap_unresolved_cells: Vec<usize>,
    /// Crossing trigger: accepted de-overlap moves that inverted the mover's
    /// relative position against a local neighbour, in the order the pass accepted them
    /// (moves are ordered, and within one move by `(moved_cell, crossed_cell)`).
    deoverlap_crossings: Vec<DeoverlapCrossingRow>,
    /// The *moved* cells of those events, sorted and deduplicated — the second half of
    /// the retry loop's offender set.
    deoverlap_crossing_cells: Vec<usize>,
    /// Last de-overlap pass's crossing bookkeeping: `(pairs the sweep touched, over
    /// 0.25, over 0.50, over 0.75, pairs the superseded order-inversion predicate would
    /// have flagged)`. Reporting only — it is what makes the threshold and the
    /// predicate choice answerable from a run instead of from an argument.
    deoverlap_crossing_stats: (usize, usize, usize, usize, usize),
    /// LCR retry: per-cell multiplicative factor on the **tail** halo. All-`1.0`
    /// during round 0, so the default path is bit-for-bit what it was before the
    /// feature existed; only cells that LCR failed to separate ever leave `1.0`.
    lcr_retry_infl: Vec<f64>,
    /// LCR retry: index of the multi-phase pass currently running (0 = the original).
    lcr_retry_round: usize,
    /// LCR retry: audit trail, `None` unless the loop actually ran.
    lcr_retry_summary: Option<LcrRetrySummary>,
    /// Multi-phase run summary and selected candidate label.
    run_summary: Option<RunSummary>,
    selected_label: Option<String>,
    base_selected_label: Option<String>,
    tail_selected_label: Option<String>,
    base_end_ref_pos: Option<Array1<f64>>,
    /// How many mirror pairs were sitting welded onto their symmetry axis at the
    /// Base→Tail transition and at LCR entry. Reporting only — both are routinely
    /// non-zero on a perfectly healthy run, because Base ends on that point and the
    /// tail (or the LCR retry loop) is what pulls the pairs off it.
    sym_weld_welded_at_tail_entry: usize,
    sym_weld_welded_at_lcr_entry: usize,
    /// `sym_pairs` indices the weld rescue separated in the finished placement, and how
    /// many it seeded a re-run with. Both `0`/empty unless the LCR retry loop exhausted
    /// with mirror pairs still welded — which is the degenerate placement this rescue
    /// exists for and which no committed case reaches.
    sym_weld_rescue_seeded: usize,
    sym_weld_rescue_repaired: usize,
    /// `sym_pairs` indices to place apart right after the initial placement. Non-empty
    /// only on the single seeded re-run the rescue allows itself.
    sym_weld_seed: Vec<usize>,
}

impl Placer {
    /// Creates a new placer (extracting the data from CircuitDb)
    ///
    /// # Arguments
    ///
    /// * `db` - the CircuitDb database
    /// * `params` - the placement parameters
    pub fn new(db: &CircuitDb, params: PlacementConfig) -> PlaceResult<Self> {
        let data = PlaceData::from_circuit_db(db, &params)?;
        Self::from_data(data, params)
    }

    /// Creates a placer from existing PlaceData (used by the bbox/pin/sym JSON
    /// tests, which do not depend on CircuitDb)
    ///
    /// # Arguments
    ///
    /// * `data` - the already-built placement data
    /// * `params` - the placement parameters
    pub fn from_data(data: PlaceData, params: PlacementConfig) -> PlaceResult<Self> {
        data.validate()?;

        let num_cells = data.num_cells;
        log::info!(
            "Placer initialized with {} cells, {} nets, {} pins",
            num_cells,
            data.num_nets,
            data.num_pins
        );

        let mut model = PlaceModel::new(data, params.clone())?;
        if stage_trace_enabled() {
            eprintln!(
                "[placer] data: num_cells={} num_pins={} num_nets={} num_sym_pairs={} num_sym_groups={} num_clusters={} num_collinear_groups={}",
                model.data.num_cells,
                model.data.num_pins,
                model.data.num_nets,
                model.data.num_sym_pairs,
                model.data.num_sym_groups,
                model.data.num_clusters,
                model.data.num_collinear_groups
            );
        }
        if params.use_ap_wl {
            let (phi, _phi0) = Self::init_phi_from_pin_offset(&model.data, params.random_seed);
            model.phi = phi;
        }
        let phi_tilde = model
            .phi
            .iter()
            .map(|p| (*p / std::f64::consts::PI).clamp(-1.0, 1.0))
            .collect::<Vec<_>>();

        let reduced_map = build_reduced_map(&model.data);
        let reduced_dim = reduced_map
            .as_ref()
            .map(|m| m.vars.len())
            .unwrap_or(2 * num_cells);
        let optimizer = AdamOptimizer::new(
            reduced_dim,
            params.learning_rate,
            params.adam_beta1,
            params.adam_beta2,
        );

        let phi_optimizer = if params.use_ap_wl {
            Some(AdamOptimizer::new(
                model.data.num_pins,
                params.learning_rate,
                params.adam_beta1,
                params.adam_beta2,
            ))
        } else {
            None
        };

        let line_search = if params.step_strategy == "linesearch" {
            Some(LineSearch::new(
                params.ls_step_init,
                params.ls_step_shrink,
                params.ls_step_min,
                params.armijo_c1,
            ))
        } else {
            None
        };

        let pos = Array1::zeros(2 * num_cells);
        let grad = Array1::zeros(2 * num_cells);
        let z = Array1::zeros(reduced_dim);
        let best_pos = Array1::zeros(2 * num_cells);

        Ok(Self {
            model,
            optimizer,
            phi_optimizer,
            line_search,
            pos,
            reduced_map,
            z,
            grad,
            best_pos,
            best_loss: f64::INFINITY,
            loss_history: Vec::new(),
            move_history: Vec::new(),
            has_external_init: false,
            phi_tilde,
            checkpoint_pos: None,
            base_end_pos: None,
            tail_end_pos: None,
            tail_state: TailState::Hold,
            conflict_flags: vec![false; num_cells],
            small_conflict_flags: vec![false; num_cells],
            overlap_inflation: vec![1.0; num_cells],
            overlap_halo: vec![0.0; num_cells],
            halo_weight: vec![0.0; num_cells],
            halo_gain: 0.0,
            conflict_component_count: 0,
            largest_component_size: 0,
            overlap_count: 0,
            overlap_total_area: 0.0,
            inflated_component_ids: Vec::new(),
            inflation_object_count: 0,
            tail_diag_rows: Vec::new(),
            tail_diag_positions: Vec::new(),
            tail_diag_snapshots: Vec::new(),
            iter_rows: Vec::new(),
            candidates: Vec::new(),
            base_candidates: Vec::new(),
            tail_candidates: Vec::new(),
            best_composite_pos: None,
            deoverlap_end_pos: None,
            deoverlap_summary: None,
            deoverlap_moves: Vec::new(),
            deoverlap_components_before: Vec::new(),
            deoverlap_components_after: Vec::new(),
            deoverlap_infl_tail_snapshot: Vec::new(),
            deoverlap_unresolved_cells: Vec::new(),
            deoverlap_crossings: Vec::new(),
            deoverlap_crossing_cells: Vec::new(),
            deoverlap_crossing_stats: (0, 0, 0, 0, 0),
            lcr_retry_infl: vec![1.0; num_cells],
            lcr_retry_round: 0,
            lcr_retry_summary: None,
            run_summary: None,
            selected_label: None,
            base_selected_label: None,
            tail_selected_label: None,
            base_end_ref_pos: None,
            sym_weld_welded_at_tail_entry: 0,
            sym_weld_welded_at_lcr_entry: 0,
            sym_weld_rescue_seeded: 0,
            sym_weld_rescue_repaired: 0,
            sym_weld_seed: Vec::new(),
        })
    }

    /// Runs global placement
    pub fn run(&mut self) -> PlaceResult<PlacementResult> {
        self.run_with_hooks(|_| Ok(()), |_, _| Ok(()))
    }

    /// Runs global placement, firing a callback after every iteration
    pub fn run_with_iter_hook<F>(&mut self, mut iter_hook: F) -> PlaceResult<PlacementResult>
    where
        F: FnMut(usize, &mut Placer) -> PlaceResult<()>,
    {
        self.run_with_hooks(|_| Ok(()), |iter, placer| iter_hook(iter, placer))
    }

    /// Runs global placement, firing a callback after initialization and after every iteration
    pub fn run_with_hooks<FInit, FIter>(
        &mut self,
        init_hook: FInit,
        iter_hook: FIter,
    ) -> PlaceResult<PlacementResult>
    where
        FInit: FnMut(&mut Placer) -> PlaceResult<()>,
        FIter: FnMut(usize, &mut Placer) -> PlaceResult<()>,
    {
        self.run_phases_with_sym_weld_rescue(init_hook, iter_hook)
    }

    /// Writes the positions back into CircuitDb
    pub fn write_positions(&self, db: &mut CircuitDb) -> PlaceResult<()> {
        let num_cells = self.model.data.num_cells;
        let mut written = 0usize;

        fn round_to_loc(v: f64) -> i32 {
            let rv = v.round();
            if rv > i32::MAX as f64 {
                i32::MAX
            } else if rv < i32::MIN as f64 {
                i32::MIN
            } else {
                rv as i32
            }
        }

        // Position layout: [x0, x1, ..., x_{N-1}, y0, y1, ..., y_{N-1}]
        for i in 0..num_cells {
            let x = self.pos[i];
            let y = self.pos[i + num_cells];

            if let Some(cell) = db.cell_mut(i as i32) {
                cell.set_coord_unsnapped(x, y);
                cell.set_coord(round_to_loc(x), round_to_loc(y));
                written += 1;
                log::debug!("Cell {}: ({:.2}, {:.2})", i, x, y);
            }
        }

        log::info!(
            "Positions written to CircuitDb ({} / {} cells)",
            written,
            num_cells
        );
        Ok(())
    }

    /// Initializes positions (all overlapping at the center by default; the old random initialization can be switched back on)
    fn init_positions(&mut self) {
        if !self.model.params.randomize_initial_placement {
            self.init_center_overlap_positions();
            return;
        }

        let mut rng = match self.model.params.random_seed {
            Some(seed) => StdRng::seed_from_u64(seed),
            None => {
                let mut trng = rand::thread_rng();
                StdRng::seed_from_u64(trng.gen::<u64>())
            }
        };
        let spread = self.model.params.init_spread_ratio.clamp(0.0, 1.0);
        let x_center = 0.5 * (self.model.data.boundary_xl + self.model.data.boundary_xh);
        let y_center = 0.5 * (self.model.data.boundary_yl + self.model.data.boundary_yh);
        let x_half_span =
            0.5 * (self.model.data.boundary_xh - self.model.data.boundary_xl) * spread;
        let y_half_span =
            0.5 * (self.model.data.boundary_yh - self.model.data.boundary_yl) * spread;
        let init_xl = x_center - x_half_span;
        let init_xh = x_center + x_half_span;
        let init_yl = y_center - y_half_span;
        let init_yh = y_center + y_half_span;
        let x_range = (init_xh - init_xl).max(0.0);
        let y_range = (init_yh - init_yl).max(0.0);

        if let Some(map) = &self.reduced_map {
            let mut z = self.z.clone();
            for (vidx, meta) in map.vars.iter().enumerate() {
                z[vidx] = match meta.axis {
                    Axis::X => init_xl + rng.gen::<f64>() * x_range,
                    Axis::Y => init_yl + rng.gen::<f64>() * y_range,
                };
            }
            self.z = z;
            self.pos = map.expand(&self.z);
        } else {
            let num_cells = self.model.data.num_cells;
            for i in 0..num_cells {
                self.pos[i] = init_xl + rng.gen::<f64>() * x_range;
            }
            for i in 0..num_cells {
                self.pos[i + num_cells] = init_yl + rng.gen::<f64>() * y_range;
            }
        }

        log::info!(
            "Initialized positions in boundary [{:.2}, {:.2}] x [{:.2}, {:.2}] with spread_ratio={:.3} (init window [{:.2}, {:.2}] x [{:.2}, {:.2}])",
            self.model.data.boundary_xl,
            self.model.data.boundary_xh,
            self.model.data.boundary_yl,
            self.model.data.boundary_yh,
            spread,
            init_xl,
            init_xh,
            init_yl,
            init_yh
        );
    }

    fn init_center_overlap_positions(&mut self) {
        let num_cells = self.model.data.num_cells;
        let x_center = 0.5 * (self.model.data.boundary_xl + self.model.data.boundary_xh);
        let y_center = 0.5 * (self.model.data.boundary_yl + self.model.data.boundary_yh);

        for i in 0..num_cells {
            self.pos[i] = x_center - 0.5 * self.model.data.cell_widths[i];
            self.pos[i + num_cells] = y_center - 0.5 * self.model.data.cell_heights[i];
        }
        if let Some(map) = &self.reduced_map {
            self.z = map.reduce_pos(&self.pos);
            self.pos = map.expand(&self.z);
        } else {
            self.z = self.pos.clone();
        }

        log::info!(
            "Initialized positions with center-overlap at symmetry center ({:.2}, {:.2}) in boundary [{:.2}, {:.2}] x [{:.2}, {:.2}]",
            x_center,
            y_center,
            self.model.data.boundary_xl,
            self.model.data.boundary_xh,
            self.model.data.boundary_yl,
            self.model.data.boundary_yh
        );
    }

    /// Initializes the AP direction variables: project pin_offset into phi0, then add a small perturbation to obtain phi
    fn init_phi_from_pin_offset(data: &PlaceData, seed: Option<u64>) -> (Vec<f64>, Vec<f64>) {
        let num_pins = data.num_pins;
        let mut phi0 = vec![0.0; num_pins];
        let mut phi = vec![0.0; num_pins];
        let mut rng = match seed {
            Some(s) => StdRng::seed_from_u64(s ^ 0xA5A5_5A5A_F0F0_0F0F),
            None => {
                let mut trng = rand::thread_rng();
                StdRng::seed_from_u64(trng.gen::<u64>())
            }
        };

        for p in 0..num_pins {
            let cidx_i32 = data.pin_owner_cell[p];
            if cidx_i32 >= 0 {
                let cidx = cidx_i32 as usize;
                if cidx < data.num_cells {
                    let a = (data.cell_widths[cidx] * 0.5).max(1e-12);
                    let b = (data.cell_heights[cidx] * 0.5).max(1e-12);
                    let ox = data.pin_offset[p];
                    let oy = data.pin_offset[p + num_pins];
                    if ox.abs() > 1e-12 || oy.abs() > 1e-12 {
                        phi0[p] = (oy / b).atan2(ox / a);
                    }
                }
            }

            let noise = (rng.gen::<f64>() - 0.5) * 2e-2;
            phi[p] = Self::wrap_angle(phi0[p] + noise);
        }

        (phi, phi0)
    }

    #[inline]
    fn wrap_angle(mut angle: f64) -> f64 {
        let two_pi = 2.0 * std::f64::consts::PI;
        while angle > std::f64::consts::PI {
            angle -= two_pi;
        }
        while angle < -std::f64::consts::PI {
            angle += two_pi;
        }
        angle
    }

    /// Sets externally supplied initial positions (the length must be 2 * num_cells)
    pub fn set_initial_positions(&mut self, positions: Array1<f64>) -> PlaceResult<()> {
        if positions.len() != self.pos.len() {
            return Err(crate::error::PlaceError::DataInitError(format!(
                "invalid initial positions length {}, expected {}",
                positions.len(),
                self.pos.len()
            )));
        }
        self.pos = positions;
        if let Some(map) = &self.reduced_map {
            self.z = map.reduce_pos(&self.pos);
        } else {
            self.z = self.pos.clone();
        }
        self.has_external_init = true;
        Ok(())
    }

    fn normalize_phi_tilde(v: f64) -> f64 {
        let mut x = v;
        while x >= 1.0 {
            x -= 2.0;
        }
        while x < -1.0 {
            x += 2.0;
        }
        x
    }

    fn sync_phi_from_tilde(&mut self) {
        if !self.model.params.use_ap_wl {
            return;
        }
        let pi = std::f64::consts::PI;
        for i in 0..self.model.phi.len() {
            let t = self
                .phi_tilde
                .get(i)
                .copied()
                .unwrap_or(0.0)
                .clamp(-1.0, 1.0);
            self.model.phi[i] = Self::wrap_angle(pi * t);
        }
    }

    fn sync_tilde_from_phi(&mut self) {
        if !self.model.params.use_ap_wl {
            self.phi_tilde.clear();
            return;
        }
        let pi = std::f64::consts::PI;
        self.phi_tilde = self
            .model
            .phi
            .iter()
            .map(|p| Self::normalize_phi_tilde(*p / pi))
            .collect();
    }

    fn expand_from_z(&self, z: &Array1<f64>) -> Array1<f64> {
        if let Some(map) = &self.reduced_map {
            map.expand(z)
        } else {
            z.clone()
        }
    }

    fn compute_symmetry_ref(&self) -> f64 {
        let mut s_ref = 0.0;
        let n = self.model.data.num_cells;
        for i in 0..self.model.data.num_sym_pairs {
            let a = self.model.data.sym_pairs[2 * i] as usize;
            let b = self.model.data.sym_pairs[2 * i + 1] as usize;
            if a >= n || b >= n {
                continue;
            }
            let w = 0.5 * (self.model.data.cell_widths[a] + self.model.data.cell_widths[b]);
            let h = 0.5 * (self.model.data.cell_heights[a] + self.model.data.cell_heights[b]);
            s_ref += w * w + h * h;
        }
        s_ref
    }

    fn compute_raw_grad_norms(&self) -> PlaceResult<(f64, f64, f64)> {
        let mut gw = Array1::<f64>::zeros(self.pos.len());
        let mut go = Array1::<f64>::zeros(self.pos.len());
        let mut gs = Array1::<f64>::zeros(self.pos.len());
        if self.model.params.enable_wirelength {
            let _ = crate::constraints::compute_lsewl(
                &self.pos,
                &self.model.data,
                self.model.wl_gamma,
                &mut gw,
            )?;
            if self.model.params.use_ap_wl {
                let mut ap_grad = vec![0.0; self.pos.len()];
                let mut phi_grad = vec![0.0; self.model.phi.len()];
                let pos_slice = self.pos.as_slice().ok_or_else(|| {
                    crate::error::PlaceError::ShapeError("pos is not contiguous".to_string())
                })?;
                let _ = crate::constraints::compute_lsewl_ap(
                    pos_slice,
                    &self.model.data,
                    &self.model.phi,
                    &mut phi_grad,
                    &mut ap_grad,
                    self.model.wl_gamma,
                    self.model.params.ap_exponent_m,
                    self.model.params.ap_eps,
                )?;
                let lam = self.model.lambda_ap.clamp(0.0, 1.0);
                for i in 0..gw.len() {
                    gw[i] = (1.0 - lam) * gw[i] + lam * ap_grad[i];
                }
            }
        }
        if self.model.params.enable_overlap {
            let _ = crate::constraints::compute_overlap_with_inflation_and_halo(
                &self.pos,
                &self.model.data,
                1.0,
                &mut go,
                Some(&self.overlap_inflation),
                Some(&self.overlap_halo),
            )?;
        }
        if self.model.params.enable_symmetry && self.model.data.num_sym_pairs > 0 {
            let mut axes = Array1::<f64>::zeros(self.model.data.num_sym_groups);
            for i in 0..self.model.data.num_sym_groups {
                axes[i] = if self.model.data.sym_group_vertical[i] {
                    (self.model.data.boundary_xl + self.model.data.boundary_xh) * 0.5
                } else {
                    (self.model.data.boundary_yl + self.model.data.boundary_yh) * 0.5
                };
            }
            let _ =
                crate::constraints::compute_symmetry(&self.pos, &self.model.data, &axes, &mut gs)?;
        }
        let nw = gw.dot(&gw).sqrt();
        let no = go.dot(&go).sqrt();
        let ns = gs.dot(&gs).sqrt();
        Ok((nw, no, ns))
    }

    fn compute_semantic_scales(&self, include_phi: bool) -> Vec<f64> {
        let n = self.model.data.num_cells;
        let mut scales = Vec::new();
        if let Some(map) = &self.reduced_map {
            for v in 0..map.vars.len() {
                let mut vals = Vec::new();
                for (idx, _, _) in &map.var_terms[v] {
                    let cid = if *idx >= n { *idx - n } else { *idx };
                    if cid >= n {
                        continue;
                    }
                    match map.vars[v].axis {
                        Axis::X => vals.push(self.model.data.cell_widths[cid]),
                        Axis::Y => vals.push(self.model.data.cell_heights[cid]),
                    }
                }
                if vals.is_empty() {
                    scales.push(1.0);
                } else {
                    vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
                    let m = vals[vals.len() / 2].max(1e-6);
                    scales.push(m);
                }
            }
        } else {
            for i in 0..n {
                scales.push(self.model.data.cell_widths[i].max(1e-6));
            }
            for i in 0..n {
                scales.push(self.model.data.cell_heights[i].max(1e-6));
            }
        }
        if include_phi {
            scales.extend(std::iter::repeat_n(1.0, self.phi_tilde.len()));
        }
        scales
    }

    fn clip_delta(delta: &[f64], scales: &[f64], c_delta: f64) -> Vec<f64> {
        delta
            .iter()
            .zip(scales.iter())
            .map(|(d, s)| {
                let lim = c_delta * s.max(1e-9);
                d.clamp(-lim, lim)
            })
            .collect()
    }

    fn clip_hit_ratio(raw: &[f64], clipped: &[f64]) -> f64 {
        if raw.is_empty() || clipped.is_empty() {
            return 0.0;
        }
        let n = raw.len().min(clipped.len());
        let hits = (0..n)
            .filter(|&i| (raw[i] - clipped[i]).abs() > 1e-12)
            .count();
        hits as f64 / n as f64
    }

    fn width_height_with_halo(&self, idx: usize, halo: Option<&[f64]>) -> (f64, f64) {
        let h = halo.and_then(|v| v.get(idx)).copied().unwrap_or(0.0);
        let w = (self.model.data.cell_widths_keepout[idx] + 2.0 * h).max(1e-12);
        let hh = (self.model.data.cell_heights_keepout[idx] + 2.0 * h).max(1e-12);
        (w, hh)
    }

    fn cell_rect_with_effective_dims(
        &self,
        idx: usize,
        w_eff: f64,
        h_eff: f64,
    ) -> (f64, f64, f64, f64) {
        let n = self.model.data.num_cells;
        let x = self.pos[idx];
        let y = self.pos[idx + n];
        self.cell_rect_at_with_effective_dims(idx, x, y, w_eff, h_eff)
    }

    /// The same rectangle as `cell_rect_with_effective_dims`, but for an arbitrary
    /// origin instead of the cell's current one — used to reconstruct where a device
    /// *was* before a de-overlap move.
    fn cell_rect_at_with_effective_dims(
        &self,
        idx: usize,
        x: f64,
        y: f64,
        w_eff: f64,
        h_eff: f64,
    ) -> (f64, f64, f64, f64) {
        let w_real = self.model.data.cell_widths.get(idx).copied().unwrap_or(0.0);
        let h_real = self
            .model
            .data
            .cell_heights
            .get(idx)
            .copied()
            .unwrap_or(0.0);
        let dx = 0.5 * (w_eff - w_real);
        let dy = 0.5 * (h_eff - h_real);
        (x - dx, y - dy, x + w_real + dx, y + h_real + dy)
    }

    /// Centre of a device's own footprint. Independent of any inflation: the effective
    /// dimensions grow symmetrically about this point.
    fn cell_centre(&self, idx: usize) -> (f64, f64) {
        let n = self.model.data.num_cells;
        let w_real = self.model.data.cell_widths.get(idx).copied().unwrap_or(0.0);
        let h_real = self
            .model
            .data
            .cell_heights
            .get(idx)
            .copied()
            .unwrap_or(0.0);
        (
            self.pos[idx] + 0.5 * w_real,
            self.pos[idx + n] + 0.5 * h_real,
        )
    }

    fn compute_conflict_metrics_with_halo(&self, halo: Option<&[f64]>) -> ConflictMetrics {
        let n = self.model.data.num_cells;
        if n == 0 {
            return ConflictMetrics::default();
        }
        let mut adj = vec![Vec::<usize>::new(); n];
        for i in 0..self.model.data.num_overlap_pairs {
            let a = self.model.data.overlap_pairs[2 * i] as usize;
            let b = self.model.data.overlap_pairs[2 * i + 1] as usize;
            if a >= n || b >= n || a == b {
                continue;
            }
            let (aw, ah) = self.width_height_with_halo(a, halo);
            let (bw, bh) = self.width_height_with_halo(b, halo);
            let (ax0, ay0, ax1, ay1) = self.cell_rect_with_effective_dims(a, aw, ah);
            let (bx0, by0, bx1, by1) = self.cell_rect_with_effective_dims(b, bw, bh);
            let ox = (ax1.min(bx1) - ax0.max(bx0)).max(0.0);
            let oy = (ay1.min(by1) - ay0.max(by0)).max(0.0);
            if ox > 0.0 && oy > 0.0 {
                adj[a].push(b);
                adj[b].push(a);
            }
        }

        let mut vis = vec![false; n];
        let mut metrics = ConflictMetrics::default();
        for s in 0..n {
            if vis[s] || adj[s].is_empty() {
                continue;
            }
            let mut q = VecDeque::new();
            let mut comp = Vec::new();
            q.push_back(s);
            vis[s] = true;
            while let Some(u) = q.pop_front() {
                comp.push(u);
                for &v in &adj[u] {
                    if !vis[v] {
                        vis[v] = true;
                        q.push_back(v);
                    }
                }
            }
            metrics.num_components += 1;
            metrics.largest_component_size = metrics.largest_component_size.max(comp.len());

            let mut xl = f64::INFINITY;
            let mut yl = f64::INFINITY;
            let mut xh = f64::NEG_INFINITY;
            let mut yh = f64::NEG_INFINITY;
            for &cid in &comp {
                let (w, h) = self.width_height_with_halo(cid, halo);
                let (cx0, cy0, cx1, cy1) = self.cell_rect_with_effective_dims(cid, w, h);
                xl = xl.min(cx0);
                yl = yl.min(cy0);
                xh = xh.max(cx1);
                yh = yh.max(cy1);
            }
            let area = (xh - xl).max(0.0) * (yh - yl).max(0.0);
            metrics.largest_component_area = metrics.largest_component_area.max(area);
        }
        metrics
    }

    fn cell_overlap_active(&self, idx: usize) -> bool {
        self.model.data.cell_widths_keepout[idx] > 1e-9
            && self.model.data.cell_heights_keepout[idx] > 1e-9
    }

    fn median_min_cell_dim(&self) -> f64 {
        let n = self.model.data.num_cells;
        if n == 0 {
            return 1.0;
        }
        let mut vals = Vec::with_capacity(n);
        for i in 0..n {
            if !self.cell_overlap_active(i) {
                continue;
            }
            vals.push(self.model.data.cell_widths[i].min(self.model.data.cell_heights[i]));
        }
        if vals.is_empty() {
            return 1.0;
        }
        vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        vals[vals.len() / 2].max(1e-12)
    }

    fn tail_delta0(&self) -> f64 {
        self.model.params.tail_infl_ratio * self.median_min_cell_dim()
    }

    fn tail_alpha_infl(&self, iter: usize, t_tail: usize) -> f64 {
        if iter < t_tail {
            return 0.0;
        }
        let ramp = self.model.params.tail_infl_ramp_iters.max(1) as f64;
        ((iter.saturating_sub(t_tail)) as f64 / ramp).clamp(0.0, 1.0)
    }

    /// Apply the tail overlap halo. `floor` is the DRC-aware lower bound (in placement
    /// DBU) every overlap-active cell's halo must reach; pass `0.0` outside the tail
    /// stage so base global placement keeps its zero halo.
    ///
    /// On top of that uniform halo each cell may carry a **routability surcharge**
    /// `alpha_infl * delta_0 * halo_gain * halo_weight[i]`, supplied by the caller.
    /// `halo_gain` defaults to
    /// `0.0`, in which case the surcharge is exactly `+0.0` and the resulting halo is
    /// bit-for-bit the value this function produced before the feature existed.
    fn set_overlap_geometry(&mut self, alpha_infl: f64, delta_0: f64, floor: f64) {
        self.overlap_inflation.fill(1.0);
        self.model.set_overlap_inflation(&self.overlap_inflation);
        let halo = (alpha_infl * delta_0).max(floor).max(0.0);
        let surcharge_scale = alpha_infl * delta_0 * self.halo_gain;
        // On a retry pass the still-conflicting devices get a strictly larger
        // halo so the tail itself opens the gap the local de-overlap search could not.
        // Round 0 skips the adjustment entirely, so the default path is unchanged.
        //
        // The factor multiplies the device's **inflated extent**, not the halo alone:
        // `d + 2*halo'` = `k * (d + 2*halo)`. That is deliberate -- the de-overlap pass
        // judges a conflict on `cell_dim * tail_deoverlap_infl`, so the retry has
        // to move the same quantity or the two are not commensurate. Measured on a
        // 49-device example case with `deoverlap_infl = 1.3`: scaling the halo alone
        // buys ~20 DBU of clearance per round against a ~1760 DBU deficit (the halo
        // floor is 100 DBU and devices are thousands of DBU wide) and the residual just
        // oscillates; scaling the extent closes it in a couple of rounds. `min(w, h)`
        // keeps the growth isotropic, since the halo is one scalar per cell.
        let retry_active = self.lcr_retry_round > 0;
        for i in 0..self.model.data.num_cells {
            self.overlap_halo[i] = if self.cell_overlap_active(i) {
                let base = halo + surcharge_scale * self.halo_weight[i];
                let k = self.lcr_retry_infl[i];
                if retry_active && k > 1.0 {
                    let d = self.model.data.cell_widths_keepout[i]
                        .min(self.model.data.cell_heights_keepout[i])
                        .max(0.0);
                    base + 0.5 * (k - 1.0) * (d + 2.0 * base)
                } else {
                    base
                }
            } else {
                0.0
            };
        }
        self.model.set_overlap_halo(&self.overlap_halo);
    }

    /// Install the per-cell routability surcharge weights (`weights[i]` in `[0, 1]`,
    /// indexed the same way as every other per-cell array) and the scalar gain.
    ///
    /// `gain == 0.0` (the default) leaves placement byte-identical; the caller is the
    /// placement stage binary, which resolves cell names against `CircuitDb`.
    pub fn set_halo_weights(&mut self, weights: &[f64], gain: f64) -> PlaceResult<()> {
        if weights.len() != self.halo_weight.len() {
            return Err(crate::error::PlaceError::DataInitError(format!(
                "invalid halo weight length {}, expected {}",
                weights.len(),
                self.halo_weight.len()
            )));
        }
        if !gain.is_finite() || gain < 0.0 {
            return Err(crate::error::PlaceError::DataInitError(format!(
                "halo gain must be finite and >= 0, got {gain}"
            )));
        }
        for (dst, src) in self.halo_weight.iter_mut().zip(weights) {
            if !src.is_finite() || *src < 0.0 {
                return Err(crate::error::PlaceError::DataInitError(format!(
                    "halo weight must be finite and >= 0, got {src}"
                )));
            }
            *dst = *src;
        }
        self.halo_gain = gain;
        Ok(())
    }

    fn compute_overlap_raw_with_halo(&self, halo: Option<&[f64]>) -> f64 {
        let mut g = Array1::zeros(self.pos.len());
        crate::constraints::compute_overlap_with_inflation_and_halo(
            &self.pos,
            &self.model.data,
            1.0,
            &mut g,
            None,
            halo,
        )
        .unwrap_or(f64::INFINITY)
    }

    fn apply_phase_mobility_weighting(&mut self, stage: Phase, gamma: f64) {
        let n = self.model.data.num_cells;
        if n == 0 {
            return;
        }
        let mean_area = self.model.data.cell_areas.sum() / (n as f64).max(1.0);
        let exp = self.model.params.mass_damping_exp;
        let mut sym_cells: HashSet<usize> = HashSet::new();
        for i in 0..self.model.data.num_sym_pairs {
            let a = self.model.data.sym_pairs[2 * i] as usize;
            let b = self.model.data.sym_pairs[2 * i + 1] as usize;
            if a < n {
                sym_cells.insert(a);
            }
            if b < n {
                sym_cells.insert(b);
            }
        }
        for i in 0..n {
            let area = self.model.data.cell_areas[i].max(1.0);
            let mut mu_target = (area / mean_area.max(1.0)).powf(exp);
            mu_target = mu_target.clamp(
                self.model.params.mobility_min,
                self.model.params.mobility_max,
            );
            if sym_cells.contains(&i) {
                mu_target *= self.model.params.mobility_sym_anchor_mult;
            }
            if area > 2.0 * mean_area {
                mu_target *= self.model.params.mobility_large_mult;
            }
            if area < 0.5 * mean_area {
                mu_target *= self.model.params.mobility_small_mult;
            }
            if stage == Phase::Tail && self.small_conflict_flags.get(i).copied().unwrap_or(false) {
                mu_target *= 0.8;
            }
            let mu_eff = (1.0 + gamma * (mu_target - 1.0)).max(1e-6);
            self.grad[i] /= mu_eff;
            self.grad[i + n] /= mu_eff;
        }
    }

    fn displacement_to_checkpoint(&self, ckpt_pos: &Array1<f64>) -> (f64, f64) {
        let n = self.model.data.num_cells;
        if n == 0 {
            return (0.0, 0.0);
        }
        let mut sum = 0.0;
        let mut maxv: f64 = 0.0;
        for i in 0..n {
            let dx = self.pos[i] - ckpt_pos[i];
            let dy = self.pos[i + n] - ckpt_pos[i + n];
            let d = (dx * dx + dy * dy).sqrt();
            sum += d * d;
            maxv = maxv.max(d);
        }
        ((sum / n as f64).sqrt(), maxv)
    }

    fn dilated_reference_positions(&self, alpha_infl: f64) -> (Array1<f64>, f64) {
        let n = self.model.data.num_cells;
        if n == 0 {
            return (self.pos.clone(), 1.0);
        }
        let Some(base_ref) = &self.base_end_ref_pos else {
            return (self.pos.clone(), 1.0);
        };

        let beta = self.model.params.tail_ref_dilate_beta.max(0.0);
        let scale = 1.0 + beta * alpha_infl.clamp(0.0, 1.0);
        let mut cx = 0.0;
        let mut cy = 0.0;
        let mut wsum = 0.0;
        for i in 0..n {
            let area = self.model.data.cell_areas[i].max(1e-12);
            let cxi = base_ref[i] + 0.5 * self.model.data.cell_widths[i];
            let cyi = base_ref[i + n] + 0.5 * self.model.data.cell_heights[i];
            cx += area * cxi;
            cy += area * cyi;
            wsum += area;
        }
        if wsum <= 0.0 {
            return (base_ref.clone(), scale);
        }
        cx /= wsum;
        cy /= wsum;

        let mut out = base_ref.clone();
        for i in 0..n {
            let wi = self.model.data.cell_widths[i];
            let hi = self.model.data.cell_heights[i];
            let cxi = base_ref[i] + 0.5 * wi;
            let cyi = base_ref[i + n] + 0.5 * hi;
            let cxo = cx + scale * (cxi - cx);
            let cyo = cy + scale * (cyi - cy);
            out[i] = cxo - 0.5 * wi;
            out[i + n] = cyo - 0.5 * hi;
        }
        (out, scale)
    }

    fn softplus(x: f64) -> f64 {
        if x > 40.0 {
            x
        } else if x < -40.0 {
            0.0
        } else {
            (1.0 + x.exp()).ln()
        }
    }

    fn sigmoid(x: f64) -> f64 {
        if x >= 0.0 {
            let z = (-x).exp();
            1.0 / (1.0 + z)
        } else {
            let z = x.exp();
            z / (1.0 + z)
        }
    }

    fn compute_cover_metrics_and_grad_real(
        &self,
        tau_cover: f64,
        s_cover: f64,
    ) -> (CoverMetrics, f64, Array1<f64>) {
        let n = self.model.data.num_cells;
        let mut grad = Array1::<f64>::zeros(self.pos.len());
        if n == 0 {
            return (CoverMetrics::default(), 0.0, grad);
        }

        let eps = 1e-12;
        let tau = tau_cover.clamp(0.0, 1.0);
        let soft = s_cover.max(1e-9);
        let mut metrics = CoverMetrics::default();
        let mut cover_loss = 0.0;
        let mut overlap_pair_count = 0usize;
        let mut cover_ratio_sum = 0.0;

        for p in 0..self.model.data.num_overlap_pairs {
            let i = self.model.data.overlap_pairs[2 * p] as usize;
            let j = self.model.data.overlap_pairs[2 * p + 1] as usize;
            if i >= n || j >= n || i == j {
                continue;
            }

            let wi = self.model.data.cell_widths_keepout[i];
            let hi = self.model.data.cell_heights_keepout[i];
            let (xi0, yi0, xi1, yi1) = self.cell_rect_with_effective_dims(i, wi, hi);

            let wj = self.model.data.cell_widths_keepout[j];
            let hj = self.model.data.cell_heights_keepout[j];
            let (xj0, yj0, xj1, yj1) = self.cell_rect_with_effective_dims(j, wj, hj);

            let left = xi0.max(xj0);
            let right = xi1.min(xj1);
            let bottom = yi0.max(yj0);
            let top = yi1.min(yj1);
            let ox = (right - left).max(0.0);
            let oy = (top - bottom).max(0.0);
            if ox <= 0.0 || oy <= 0.0 {
                continue;
            }

            let inter_area = ox * oy;
            let denom = self.model.data.cell_areas_keepout[i]
                .min(self.model.data.cell_areas_keepout[j])
                .max(eps);
            let r_cover = inter_area / (denom + eps);

            overlap_pair_count += 1;
            cover_ratio_sum += r_cover;
            metrics.max_ratio = metrics.max_ratio.max(r_cover);
            if r_cover >= tau {
                metrics.num_pairs += 1;
            }

            let x = (r_cover - tau) / soft;
            let sp = Self::softplus(x);
            let sig = Self::sigmoid(x);
            let dpsi_dx = 2.0 * sp * sig;
            let dpsi_dr = dpsi_dx / soft;
            let dpsi_dinter = dpsi_dr / (denom + eps);
            cover_loss += sp * sp;

            let d_right_dxi = if xi1 < xj1 {
                1.0
            } else if (xi1 - xj1).abs() <= 1e-12 {
                0.5
            } else {
                0.0
            };
            let d_left_dxi = if xi0 > xj0 {
                1.0
            } else if (xi0 - xj0).abs() <= 1e-12 {
                0.5
            } else {
                0.0
            };
            let d_ox_dxi = d_right_dxi - d_left_dxi;

            let d_right_dxj = if xj1 < xi1 {
                1.0
            } else if (xj1 - xi1).abs() <= 1e-12 {
                0.5
            } else {
                0.0
            };
            let d_left_dxj = if xj0 > xi0 {
                1.0
            } else if (xj0 - xi0).abs() <= 1e-12 {
                0.5
            } else {
                0.0
            };
            let d_ox_dxj = d_right_dxj - d_left_dxj;

            let d_top_dyi = if yi1 < yj1 {
                1.0
            } else if (yi1 - yj1).abs() <= 1e-12 {
                0.5
            } else {
                0.0
            };
            let d_bottom_dyi = if yi0 > yj0 {
                1.0
            } else if (yi0 - yj0).abs() <= 1e-12 {
                0.5
            } else {
                0.0
            };
            let d_oy_dyi = d_top_dyi - d_bottom_dyi;

            let d_top_dyj = if yj1 < yi1 {
                1.0
            } else if (yj1 - yi1).abs() <= 1e-12 {
                0.5
            } else {
                0.0
            };
            let d_bottom_dyj = if yj0 > yi0 {
                1.0
            } else if (yj0 - yi0).abs() <= 1e-12 {
                0.5
            } else {
                0.0
            };
            let d_oy_dyj = d_top_dyj - d_bottom_dyj;

            grad[i] += dpsi_dinter * oy * d_ox_dxi;
            grad[j] += dpsi_dinter * oy * d_ox_dxj;
            grad[i + n] += dpsi_dinter * ox * d_oy_dyi;
            grad[j + n] += dpsi_dinter * ox * d_oy_dyj;
        }

        if overlap_pair_count > 0 {
            metrics.mean_ratio = cover_ratio_sum / overlap_pair_count as f64;
        }
        (metrics, cover_loss, grad)
    }

    fn inflated_geometry_summary(&self, alpha_infl: f64, delta_0: f64) -> InflatedGeometrySummary {
        let n = self.model.data.num_cells;
        if n == 0 {
            return InflatedGeometrySummary {
                alpha_infl,
                delta_0,
                min_width_infl: 0.0,
                max_width_infl: 0.0,
                min_height_infl: 0.0,
                max_height_infl: 0.0,
            };
        }
        let halo = alpha_infl * delta_0;
        let mut min_w = f64::INFINITY;
        let mut max_w = f64::NEG_INFINITY;
        let mut min_h = f64::INFINITY;
        let mut max_h = f64::NEG_INFINITY;
        for i in 0..n {
            let w = self.model.data.cell_widths_keepout[i] + 2.0 * halo;
            let h = self.model.data.cell_heights_keepout[i] + 2.0 * halo;
            min_w = min_w.min(w);
            max_w = max_w.max(w);
            min_h = min_h.min(h);
            max_h = max_h.max(h);
        }
        InflatedGeometrySummary {
            alpha_infl,
            delta_0,
            min_width_infl: min_w,
            max_width_infl: max_w,
            min_height_infl: min_h,
            max_height_infl: max_h,
        }
    }

    fn make_candidate(
        &self,
        stage: Phase,
        iter: usize,
        q_o_real: f64,
        q_o_infl: f64,
        q_s: f64,
        wl_mix_real: f64,
        ovl_raw_real: f64,
        ovl_raw_infl: f64,
        conflict_real: ConflictMetrics,
        conflict_infl: ConflictMetrics,
        cover_real: CoverMetrics,
        alpha_infl: f64,
        ref_scale_s: f64,
        disp_to_ref_rms: f64,
        disp_to_ref_max: f64,
        delta_0: f64,
        lambda_anchor: f64,
        lambda_cover_tail: f64,
    ) -> Candidate {
        Candidate {
            label: format!("iter_{:05}", iter),
            stage: stage.as_str().to_string(),
            iter,
            q_o_real,
            q_o_infl,
            q_s,
            wl_mix_real,
            ovl_raw_real,
            ovl_raw_infl,
            conflict_real,
            conflict_infl,
            cover_real,
            ap_mix: self.model.lambda_ap,
            alpha_infl,
            ref_scale_s,
            disp_to_ref_rms,
            disp_to_ref_max,
            lambdas: LambdaSnapshot {
                lambda_ovl: self.model.lambdas.overlap,
                lambda_sym: self.model.lambdas.symmetry,
                ap_mix: self.model.lambda_ap,
                lambda_anchor,
                lambda_cover_tail,
            },
            reduced_vars: self.z.to_vec(),
            full_positions: self.pos.to_vec(),
            phi_tilde: self.phi_tilde.clone(),
            inflated_geometry: self.inflated_geometry_summary(alpha_infl, delta_0),
        }
    }

    fn overlap_dims_for_deoverlap(&self) -> Vec<f64> {
        let n = self.model.data.num_cells;
        let infl_deoverlap = self.model.params.tail_deoverlap_infl.max(1.0);
        let mut infl = vec![1.0; n];
        for i in 0..n {
            if !self.cell_overlap_active(i) {
                infl[i] = 1.0;
                continue;
            }
            infl[i] = infl_deoverlap;
        }
        infl
    }

    fn deoverlap_feasible_dims(&self, infl_tail: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = self.model.data.num_cells;
        let mut w_feas = vec![0.0; n];
        let mut h_feas = vec![0.0; n];
        // The tail keeps every device pair at least `tail_hard_inflation_floor`
        // apart -- that floor is the placer's one DRC-derived clearance, computed from
        // `max_pairwise_clean_spacing`. The de-overlap pass historically judged
        // feasibility on the multiplicative `tail_deoverlap_infl` alone, so it
        // could legally park a pair closer than the floor and undo the tail's
        // compliance: `1.05 * w` is below `w + 2 * floor` for any device narrower than
        // 40x the floor. Adding the floor here makes the two stages agree.
        let floor = if self.model.params.lcr_honor_drc_floor {
            self.model.params.tail_hard_inflation_floor.max(0.0)
        } else {
            0.0
        };
        for i in 0..n {
            if !self.cell_overlap_active(i) {
                w_feas[i] = 0.0;
                h_feas[i] = 0.0;
                continue;
            }
            let inf = infl_tail.get(i).copied().unwrap_or(1.0).max(1.0);
            let w = self.model.data.cell_widths_keepout[i];
            let h = self.model.data.cell_heights_keepout[i];
            w_feas[i] = w * inf + 2.0 * floor;
            h_feas[i] = h * inf + 2.0 * floor;
        }
        (w_feas, h_feas)
    }

    fn axis_for_group(&self, is_vert: bool) -> f64 {
        if is_vert {
            0.5 * (self.model.data.boundary_xl + self.model.data.boundary_xh)
        } else {
            0.5 * (self.model.data.boundary_yl + self.model.data.boundary_yh)
        }
    }

    /// Clearance two devices need between their centres along one axis before the
    /// legalizer will call them separated: the de-overlap pass's own feasibility extent,
    /// `dim * tail_deoverlap_infl + 2 * tail_hard_inflation_floor`, halved and
    /// summed. Using the legalizer's number rather than the raw outline means a pair
    /// separated by `separate_self_overlapping_sym_pairs` is already clear by the
    /// standard the next stage judges it against.
    ///
    /// The separated pair lands exactly on that clearance, not beyond it: touching is
    /// not overlapping (the same closed-boundary convention the DRC checks use), and the
    /// two placements are mirror images of one expression, so the touch comes out exact
    /// rather than one ULP either way. Should a future geometry make it land one ULP
    /// short, the pair is simply still reported as self-overlapping by the detector and
    /// by the de-overlap pass that runs straight afterwards — a missed repair, never a
    /// silent one.
    fn deoverlap_clearance(&self, dim_a: f64, dim_b: f64) -> f64 {
        let infl = self.model.params.tail_deoverlap_infl.max(1.0);
        let floor = if self.model.params.lcr_honor_drc_floor {
            self.model.params.tail_hard_inflation_floor.max(0.0)
        } else {
            0.0
        };
        0.5 * ((dim_a.max(0.0) * infl + 2.0 * floor) + (dim_b.max(0.0) * infl + 2.0 * floor))
    }

    /// Everything the weld repair needs about symmetry pair `i`, or `None` when the
    /// pair is not a repairable mirror pair (self-symmetric, out of range, or a device
    /// with no overlap geometry).
    ///
    /// `owner` is `Some(v)` only when this pair is the one that owns the reduced mirror
    /// variable `v` — `a` bound to it with scale `+1` and `b` with scale `-1`. A pair
    /// whose devices were already bound by an earlier pair gets `None` and is repaired
    /// in `pos` alone.
    fn sym_pair_mirror_geometry(
        &self,
        i: usize,
    ) -> Option<(usize, usize, bool, f64, Option<usize>)> {
        let n = self.model.data.num_cells;
        if i >= self.model.data.num_sym_pairs {
            return None;
        }
        let a = self.model.data.sym_pairs[2 * i] as usize;
        let b = self.model.data.sym_pairs[2 * i + 1] as usize;
        if a == b || a >= n || b >= n {
            return None;
        }
        if !self.cell_overlap_active(a) || !self.cell_overlap_active(b) {
            return None;
        }
        let g = self.model.data.sym_pair_group[i] as usize;
        let is_vert = self
            .model
            .data
            .sym_group_vertical
            .get(g)
            .copied()
            .unwrap_or(true);
        let (slot_a, slot_b) = if is_vert { (a, b) } else { (a + n, b + n) };
        let owner = self.reduced_map.as_ref().and_then(|m| {
            let ta = m.full_to_reduced.get(slot_a)?;
            let tb = m.full_to_reduced.get(slot_b)?;
            let (va, vb) = (ta.var?, tb.var?);
            (va == vb && (ta.scale - 1.0).abs() < 1e-12 && (tb.scale + 1.0).abs() < 1e-12)
                .then_some(va)
        });
        Some((a, b, is_vert, self.axis_for_group(is_vert), owner))
    }

    /// Place one mirror pair symmetrically about its axis, just clear of each other.
    ///
    /// The two devices go to `axis -/+ clearance/2` along the mirror direction and both
    /// keep the coordinate they share across it, so the pair comes out exactly mirrored:
    /// this separates a pair without weakening the symmetry it was declared with. The
    /// direction is fixed by the pair's stored order (`a` low, `b` high), so there is
    /// nothing random about the repair.
    fn place_sym_pair_clear_of_axis(
        &mut self,
        a: usize,
        b: usize,
        is_vert: bool,
        axis: f64,
        owner: Option<usize>,
    ) -> bool {
        let n = self.model.data.num_cells;
        let (dim_a, dim_b) = if is_vert {
            (
                self.model.data.cell_widths_keepout[a],
                self.model.data.cell_widths_keepout[b],
            )
        } else {
            (
                self.model.data.cell_heights_keepout[a],
                self.model.data.cell_heights_keepout[b],
            )
        };
        let half = 0.5 * self.deoverlap_clearance(dim_a, dim_b);
        if !half.is_finite() || half <= 0.0 {
            return false;
        }
        let (slot_a, slot_b, anchor_a, anchor_b) = if is_vert {
            (
                a,
                b,
                self.model.data.sym_anchor_x[a],
                self.model.data.sym_anchor_x[b],
            )
        } else {
            (
                a + n,
                b + n,
                self.model.data.sym_anchor_y[a],
                self.model.data.sym_anchor_y[b],
            )
        };
        self.pos[slot_a] = (axis - half) - anchor_a;
        self.pos[slot_b] = (axis + half) - anchor_b;
        if let Some(v) = owner {
            if v < self.z.len() {
                // `pos_a = z_v - anchor_a`, so the reduced variable *is* the centre of
                // `a`. Writing it keeps `z` and `pos` in step for the phases that go on
                // optimizing `z`.
                self.z[v] = axis - half;
            }
        }
        true
    }

    /// The de-overlap pass's feasibility extents, the same ones
    /// `discrete_deoverlap_after_tail` judges a conflict on.
    fn deoverlap_feasibility_extents(&self) -> (Vec<f64>, Vec<f64>) {
        let infl_tail = self.overlap_dims_for_deoverlap();
        self.deoverlap_feasible_dims(&infl_tail)
    }

    /// Symmetry pairs welded to themselves: the two mirrored devices overlap each other.
    ///
    /// A mirrored pair is **one** optimizer variable, not two: `build_reduced_map` binds
    /// `centre_b = 2 * axis - centre_a`, so the pair's entire freedom along the mirror
    /// direction is the single reduced variable `z_v = centre_a`, and `z_v` sits on the
    /// axis exactly when the two devices coincide. The initial placement puts every
    /// device at the boundary centre (`init_center_overlap_positions`), and for a group
    /// whose axis is the boundary centre that *is* the axis — so every pair starts fully
    /// welded, and the gradient there is `grad[a] - grad[b]` over two geometrically
    /// identical devices at the same point, zero to the last bit whatever the overlap
    /// multiplier has grown to. A pair leaves that point only on whatever asymmetry its
    /// own nets supply, which is why the same case can converge in one build and stay
    /// welded in another.
    ///
    /// The test is the de-overlap pass's own `self_conflict`: do the pair's two members
    /// overlap under the extents that pass judges feasibility on. Exact coincidence is
    /// the extreme of that, not a separate case — a pair 232 DBU apart with 1,848 DBU of
    /// overlap is just as stuck, because de-overlap treats a pair as one rigid object and
    /// the single-object component it forms has no other object to move against.
    ///
    /// `pos` is the placement to judge, so the caller can ask about a finished placement
    /// rather than whatever `self.pos` currently holds.
    fn self_overlapping_sym_pairs(&self, pos: &Array1<f64>) -> Vec<usize> {
        let n = self.model.data.num_cells;
        let (w_feas, h_feas) = self.deoverlap_feasibility_extents();
        let rect = |idx: usize| -> (f64, f64, f64, f64) {
            let w_eff = w_feas.get(idx).copied().unwrap_or(0.0).max(0.0);
            let h_eff = h_feas.get(idx).copied().unwrap_or(0.0).max(0.0);
            let w_real = self.model.data.cell_widths.get(idx).copied().unwrap_or(0.0);
            let h_real = self
                .model
                .data
                .cell_heights
                .get(idx)
                .copied()
                .unwrap_or(0.0);
            let dx = 0.5 * (w_eff - w_real);
            let dy = 0.5 * (h_eff - h_real);
            (
                pos[idx] - dx,
                pos[idx + n] - dy,
                pos[idx] + w_real + dx,
                pos[idx + n] + h_real + dy,
            )
        };
        let mut out = Vec::new();
        for i in 0..self.model.data.num_sym_pairs {
            let Some((a, b, _is_vert, _axis, _owner)) = self.sym_pair_mirror_geometry(i) else {
                continue;
            };
            if w_feas[a] <= 0.0 || h_feas[a] <= 0.0 || w_feas[b] <= 0.0 || h_feas[b] <= 0.0 {
                continue;
            }
            let (ax0, ay0, ax1, ay1) = rect(a);
            let (bx0, by0, bx1, by1) = rect(b);
            let ox = ax1.min(bx1) - ax0.max(bx0);
            let oy = ay1.min(by1) - ay0.max(by0);
            if ox > 1e-12 && oy > 1e-12 {
                out.push(i);
            }
        }
        out
    }

    /// Push every self-overlapping symmetry pair apart, symmetrically about its axis.
    ///
    /// Returns the `sym_pairs` indices that were separated.
    fn separate_self_overlapping_sym_pairs(&mut self, where_: &str) -> Vec<usize> {
        let welded = self.self_overlapping_sym_pairs(&self.pos.clone());
        if welded.is_empty() {
            return welded;
        }
        let mut separated = Vec::with_capacity(welded.len());
        for &i in &welded {
            let Some((a, b, is_vert, axis, owner)) = self.sym_pair_mirror_geometry(i) else {
                continue;
            };
            if self.place_sym_pair_clear_of_axis(a, b, is_vert, axis, owner) {
                separated.push(i);
            }
        }
        if let Some(map) = &self.reduced_map {
            // Re-expanding is exact: every coordinate written above satisfies the map's
            // mirror relation, so only the separated pairs change value.
            self.pos = map.expand(&self.z);
        }
        log::info!(
            "[sym-weld] {}: separated {} self-overlapping symmetry pair(s)",
            where_,
            separated.len()
        );
        eprintln!(
            "[sym-weld] {}: separated {} mirror pair(s) that were overlapping themselves \
             (the two mirrored devices on top of each other)",
            where_,
            separated.len()
        );
        separated
    }

    /// Re-apply a separation recorded by an earlier pass, right after the fresh initial
    /// placement.
    ///
    /// Used by the one re-run the rescue allows itself when the first pass finished with
    /// mirror pairs still overlapping themselves: that pass spent both of its phases on
    /// a placement in which those pairs were effectively one device, so it is worth
    /// starting over with them already apart. No recorded pair, nothing happens.
    fn apply_sym_weld_seed(&mut self) {
        if self.sym_weld_seed.is_empty() {
            return;
        }
        let seed = self.sym_weld_seed.clone();
        let mut applied = 0usize;
        for i in seed {
            let Some((a, b, is_vert, axis, owner)) = self.sym_pair_mirror_geometry(i) else {
                continue;
            };
            if self.place_sym_pair_clear_of_axis(a, b, is_vert, axis, owner) {
                applied += 1;
            }
        }
        if let Some(map) = &self.reduced_map {
            self.pos = map.expand(&self.z);
        }
        eprintln!(
            "[sym-weld] seeding the re-run with {} mirror pair(s) already separated",
            applied
        );
    }

    fn force_symmetry_before_deoverlap(&mut self) -> bool {
        let n = self.model.data.num_cells;
        let mut changed = false;
        for i in 0..self.model.data.num_sym_pairs {
            let a = self.model.data.sym_pairs[2 * i] as usize;
            let b = self.model.data.sym_pairs[2 * i + 1] as usize;
            if a >= n || b >= n {
                continue;
            }
            let g = self.model.data.sym_pair_group[i] as usize;
            let is_vert = self
                .model
                .data
                .sym_group_vertical
                .get(g)
                .copied()
                .unwrap_or(true);
            let axis = self.axis_for_group(is_vert);

            if a == b {
                if is_vert {
                    let x_new = axis - self.model.data.sym_anchor_x[a];
                    if (self.pos[a] - x_new).abs() > 1e-12 {
                        self.pos[a] = x_new;
                        changed = true;
                    }
                } else {
                    let y_new = axis - self.model.data.sym_anchor_y[a];
                    if (self.pos[a + n] - y_new).abs() > 1e-12 {
                        self.pos[a + n] = y_new;
                        changed = true;
                    }
                }
                continue;
            }

            if is_vert {
                let ax = self.pos[a] + self.model.data.sym_anchor_x[a];
                let bx = self.pos[b] + self.model.data.sym_anchor_x[b];
                let ay = self.pos[a + n] + self.model.data.sym_anchor_y[a];
                let by = self.pos[b + n] + self.model.data.sym_anchor_y[b];
                let d = 0.5 * ((ax - axis).abs() + (bx - axis).abs());
                let y_mid = 0.5 * (ay + by);
                let (left, right) = if ax <= bx { (a, b) } else { (b, a) };
                let left_anchor_x = axis - d;
                let right_anchor_x = axis + d;
                let left_x = left_anchor_x - self.model.data.sym_anchor_x[left];
                let right_x = right_anchor_x - self.model.data.sym_anchor_x[right];
                let left_y = y_mid - self.model.data.sym_anchor_y[left];
                let right_y = y_mid - self.model.data.sym_anchor_y[right];
                if (self.pos[left] - left_x).abs() > 1e-12
                    || (self.pos[right] - right_x).abs() > 1e-12
                    || (self.pos[left + n] - left_y).abs() > 1e-12
                    || (self.pos[right + n] - right_y).abs() > 1e-12
                {
                    self.pos[left] = left_x;
                    self.pos[right] = right_x;
                    self.pos[left + n] = left_y;
                    self.pos[right + n] = right_y;
                    changed = true;
                }
            } else {
                let ay = self.pos[a + n] + self.model.data.sym_anchor_y[a];
                let by = self.pos[b + n] + self.model.data.sym_anchor_y[b];
                let ax = self.pos[a] + self.model.data.sym_anchor_x[a];
                let bx = self.pos[b] + self.model.data.sym_anchor_x[b];
                let d = 0.5 * ((ay - axis).abs() + (by - axis).abs());
                let x_mid = 0.5 * (ax + bx);
                let (bottom, top) = if ay <= by { (a, b) } else { (b, a) };
                let bottom_anchor_y = axis - d;
                let top_anchor_y = axis + d;
                let bottom_y = bottom_anchor_y - self.model.data.sym_anchor_y[bottom];
                let top_y = top_anchor_y - self.model.data.sym_anchor_y[top];
                let bottom_x = x_mid - self.model.data.sym_anchor_x[bottom];
                let top_x = x_mid - self.model.data.sym_anchor_x[top];
                if (self.pos[bottom + n] - bottom_y).abs() > 1e-12
                    || (self.pos[top + n] - top_y).abs() > 1e-12
                    || (self.pos[bottom] - bottom_x).abs() > 1e-12
                    || (self.pos[top] - top_x).abs() > 1e-12
                {
                    self.pos[bottom + n] = bottom_y;
                    self.pos[top + n] = top_y;
                    self.pos[bottom] = bottom_x;
                    self.pos[top] = top_x;
                    changed = true;
                }
            }
        }
        changed
    }

    fn build_deoverlap_objects(
        &self,
        w_feas: &[f64],
        h_feas: &[f64],
    ) -> (Vec<DeoverlapObject>, Vec<usize>) {
        let n = self.model.data.num_cells;
        let mut objects = Vec::<DeoverlapObject>::new();
        let mut cell_to_obj = vec![usize::MAX; n];
        let mut used = vec![false; n];

        for i in 0..self.model.data.num_sym_pairs {
            let a = self.model.data.sym_pairs[2 * i] as usize;
            let b = self.model.data.sym_pairs[2 * i + 1] as usize;
            if a >= n || b >= n {
                continue;
            }
            if used[a] || used[b] {
                continue;
            }
            let g = self.model.data.sym_pair_group[i] as usize;
            let is_vert = self
                .model
                .data
                .sym_group_vertical
                .get(g)
                .copied()
                .unwrap_or(true);
            let axis = self.axis_for_group(is_vert);
            if a == b {
                let obj_id = objects.len();
                let area = w_feas[a] * h_feas[a];
                objects.push(DeoverlapObject {
                    kind: DeoverlapObjectKind::SelfSym,
                    members: vec![a],
                    area,
                    is_vert: Some(is_vert),
                    axis: Some(axis),
                    primary_lo: None,
                    primary_hi: None,
                });
                used[a] = true;
                cell_to_obj[a] = obj_id;
            } else {
                let obj_id = objects.len();
                let (lo, hi) = if is_vert {
                    let aa = self.pos[a] + self.model.data.sym_anchor_x[a];
                    let bb = self.pos[b] + self.model.data.sym_anchor_x[b];
                    if aa <= bb {
                        (a, b)
                    } else {
                        (b, a)
                    }
                } else {
                    let aa = self.pos[a + n] + self.model.data.sym_anchor_y[a];
                    let bb = self.pos[b + n] + self.model.data.sym_anchor_y[b];
                    if aa <= bb {
                        (a, b)
                    } else {
                        (b, a)
                    }
                };
                let area = w_feas[a] * h_feas[a] + w_feas[b] * h_feas[b];
                objects.push(DeoverlapObject {
                    kind: DeoverlapObjectKind::Pair,
                    members: vec![a, b],
                    area,
                    is_vert: Some(is_vert),
                    axis: Some(axis),
                    primary_lo: Some(lo),
                    primary_hi: Some(hi),
                });
                used[a] = true;
                used[b] = true;
                cell_to_obj[a] = obj_id;
                cell_to_obj[b] = obj_id;
            }
        }

        for i in 0..n {
            if used[i] {
                continue;
            }
            let obj_id = objects.len();
            objects.push(DeoverlapObject {
                kind: DeoverlapObjectKind::Singleton,
                members: vec![i],
                area: w_feas[i] * h_feas[i],
                is_vert: None,
                axis: None,
                primary_lo: None,
                primary_hi: None,
            });
            used[i] = true;
            cell_to_obj[i] = obj_id;
        }
        (objects, cell_to_obj)
    }

    fn cells_overlap_with_dims(&self, i: usize, j: usize, w_dims: &[f64], h_dims: &[f64]) -> bool {
        let n = self.model.data.num_cells;
        if i >= n || j >= n || i == j {
            return false;
        }
        if w_dims[i] <= 0.0 || h_dims[i] <= 0.0 || w_dims[j] <= 0.0 || h_dims[j] <= 0.0 {
            return false;
        }
        let (ax0, ay0, ax1, ay1) = self.cell_rect_with_dims(i, w_dims, h_dims);
        let (bx0, by0, bx1, by1) = self.cell_rect_with_dims(j, w_dims, h_dims);
        let ox = (ax1.min(bx1) - ax0.max(bx0)).max(0.0);
        let oy = (ay1.min(by1) - ay0.max(by0)).max(0.0);
        ox > 1e-12 && oy > 1e-12
    }

    fn cell_rect_with_dims(
        &self,
        idx: usize,
        w_dims: &[f64],
        h_dims: &[f64],
    ) -> (f64, f64, f64, f64) {
        let w_eff = w_dims.get(idx).copied().unwrap_or(0.0).max(0.0);
        let h_eff = h_dims.get(idx).copied().unwrap_or(0.0).max(0.0);
        self.cell_rect_with_effective_dims(idx, w_eff, h_eff)
    }

    fn build_object_conflict_components(
        &self,
        objects: &[DeoverlapObject],
        cell_to_object: &[usize],
        w_dims: &[f64],
        h_dims: &[f64],
    ) -> (Vec<Vec<usize>>, usize) {
        let nobj = objects.len();
        if nobj == 0 {
            return (Vec::new(), 0);
        }
        let mut adj = vec![HashSet::<usize>::new(); nobj];
        let mut self_conflict = vec![false; nobj];
        let mut overlap_pairs = 0usize;

        for p in 0..self.model.data.num_overlap_pairs {
            let i = self.model.data.overlap_pairs[2 * p] as usize;
            let j = self.model.data.overlap_pairs[2 * p + 1] as usize;
            if i >= cell_to_object.len() || j >= cell_to_object.len() || i == j {
                continue;
            }
            if !self.cells_overlap_with_dims(i, j, w_dims, h_dims) {
                continue;
            }
            overlap_pairs += 1;
            let oi = cell_to_object[i];
            let oj = cell_to_object[j];
            if oi == usize::MAX || oj == usize::MAX {
                continue;
            }
            if oi == oj {
                self_conflict[oi] = true;
            } else {
                adj[oi].insert(oj);
                adj[oj].insert(oi);
            }
        }

        let mut comps = Vec::<Vec<usize>>::new();
        let mut vis = vec![false; nobj];
        for s in 0..nobj {
            if vis[s] {
                continue;
            }
            if adj[s].is_empty() && !self_conflict[s] {
                continue;
            }
            let mut q = VecDeque::new();
            let mut comp = Vec::<usize>::new();
            q.push_back(s);
            vis[s] = true;
            while let Some(u) = q.pop_front() {
                comp.push(u);
                let mut nbrs: Vec<usize> = adj[u].iter().copied().collect();
                nbrs.sort_unstable();
                for v in nbrs {
                    if !vis[v] {
                        vis[v] = true;
                        q.push_back(v);
                    }
                }
            }
            if !comp.is_empty() {
                comp.sort_unstable();
                comps.push(comp);
            }
        }
        comps.sort_by_key(|c| c.first().copied().unwrap_or(usize::MAX));
        (comps, overlap_pairs)
    }

    /// Does the bounding box of `start ∪ end` contain `(px, py)`?
    ///
    /// **Diagnostic only** (`swept_covers_centre`): this was the first candidate
    /// definition of "crossed over another device". Boundary equality does not count.
    fn swept_box_contains_point(
        start: (f64, f64, f64, f64),
        end: (f64, f64, f64, f64),
        px: f64,
        py: f64,
    ) -> bool {
        let xl = start.0.min(end.0);
        let yl = start.1.min(end.1);
        let xh = start.2.max(end.2);
        let yh = start.3.max(end.3);
        px - xl > CROSSING_EPS
            && xh - px > CROSSING_EPS
            && py - yl > CROSSING_EPS
            && yh - py > CROSSING_EPS
    }

    /// The exact region an axis-aligned rectangle sweeps when it is translated from
    /// `start` to `end`: the Minkowski sum of the rectangle with the displacement
    /// segment, which for two congruent axis-aligned boxes is the convex hull of their
    /// eight corners. An axis-aligned displacement degenerates to the bounding box of
    /// the two, a diagonal one to a hexagon.
    ///
    /// Why the hull and not the bounding box of `start ∪ end`: for a diagonal move the
    /// bounding box adds two corner triangles the body never passes through, and those
    /// corners are where the over-reporting of the earlier candidate definitions came
    /// from. Returned counter-clockwise; may be fewer than 3 points if the rectangle is
    /// degenerate.
    fn swept_hull(start: (f64, f64, f64, f64), end: (f64, f64, f64, f64)) -> Vec<(f64, f64)> {
        let mut pts = vec![
            (start.0, start.1),
            (start.2, start.1),
            (start.2, start.3),
            (start.0, start.3),
            (end.0, end.1),
            (end.2, end.1),
            (end.2, end.3),
            (end.0, end.3),
        ];
        pts.sort_by(|a, b| {
            a.0.partial_cmp(&b.0)
                .unwrap_or(Ordering::Equal)
                .then_with(|| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal))
        });
        pts.dedup_by(|a, b| (a.0 - b.0).abs() <= CROSSING_EPS && (a.1 - b.1).abs() <= CROSSING_EPS);
        if pts.len() < 3 {
            return pts;
        }
        let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| {
            (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
        };
        // Andrew's monotone chain. Input is sorted, so the result is deterministic.
        let mut hull: Vec<(f64, f64)> = Vec::with_capacity(pts.len() * 2);
        for &pt in pts.iter() {
            while hull.len() >= 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], pt) <= 0.0 {
                hull.pop();
            }
            hull.push(pt);
        }
        let lower = hull.len() + 1;
        for &pt in pts.iter().rev() {
            while hull.len() >= lower
                && cross(hull[hull.len() - 2], hull[hull.len() - 1], pt) <= 0.0
            {
                hull.pop();
            }
            hull.push(pt);
        }
        hull.pop();
        hull
    }

    /// Sutherland–Hodgman clip of a convex polygon by one axis-aligned half-plane.
    /// `axis_x` picks the coordinate, `keep_upper` whether to keep `coord >= bound`.
    fn clip_convex_by_axis(
        poly: &[(f64, f64)],
        axis_x: bool,
        bound: f64,
        keep_upper: bool,
    ) -> Vec<(f64, f64)> {
        if poly.is_empty() {
            return Vec::new();
        }
        let coord = |p: (f64, f64)| if axis_x { p.0 } else { p.1 };
        let inside = |p: (f64, f64)| {
            if keep_upper {
                coord(p) >= bound
            } else {
                coord(p) <= bound
            }
        };
        let mut out = Vec::with_capacity(poly.len() + 1);
        for i in 0..poly.len() {
            let cur = poly[i];
            let prev = poly[(i + poly.len() - 1) % poly.len()];
            let cur_in = inside(cur);
            let prev_in = inside(prev);
            if cur_in != prev_in {
                let d = coord(cur) - coord(prev);
                let t = if d.abs() <= CROSSING_EPS {
                    0.0
                } else {
                    (bound - coord(prev)) / d
                };
                out.push((prev.0 + t * (cur.0 - prev.0), prev.1 + t * (cur.1 - prev.1)));
            }
            if cur_in {
                out.push(cur);
            }
        }
        out
    }

    fn polygon_area(poly: &[(f64, f64)]) -> f64 {
        if poly.len() < 3 {
            return 0.0;
        }
        let mut acc = 0.0;
        for i in 0..poly.len() {
            let a = poly[i];
            let b = poly[(i + 1) % poly.len()];
            acc += a.0 * b.1 - b.0 * a.1;
        }
        0.5 * acc.abs()
    }

    /// Crossing trigger, the predicate: what fraction of `other` does the region
    /// swept from `start` to `end` cover?
    ///
    /// `0.0` when the two do not meet, so a device far from the path costs nothing and
    /// never triggers — which is why this predicate needs no "who counts as a
    /// neighbour" rule at all.
    fn swept_coverage_fraction(
        start: (f64, f64, f64, f64),
        end: (f64, f64, f64, f64),
        other: (f64, f64, f64, f64),
    ) -> f64 {
        let area = (other.2 - other.0).max(0.0) * (other.3 - other.1).max(0.0);
        if area <= CROSSING_EPS {
            return 0.0;
        }
        // Cheap reject on the bounding boxes before any clipping.
        if start.0.min(end.0) >= other.2
            || start.2.max(end.2) <= other.0
            || start.1.min(end.1) >= other.3
            || start.3.max(end.3) <= other.1
        {
            return 0.0;
        }
        let mut poly = Self::swept_hull(start, end);
        for (axis_x, bound, keep_upper) in [
            (true, other.0, true),
            (true, other.2, false),
            (false, other.1, true),
            (false, other.3, false),
        ] {
            poly = Self::clip_convex_by_axis(&poly, axis_x, bound, keep_upper);
            if poly.is_empty() {
                return 0.0;
            }
        }
        (Self::polygon_area(&poly) / area).clamp(0.0, 1.0)
    }

    /// Did `mover` change sides relative to `other` along one axis?
    ///
    /// **Diagnostic only**: it was briefly the trigger itself. `before`/`after`
    /// are the mover's centre
    /// coordinate on that axis, `other` the neighbour's. The product form makes
    /// "exactly level" a non-event at both ends.
    fn order_reversed_on_axis(before: f64, after: f64, other: f64) -> bool {
        (before - other) * (after - other) < 0.0
    }

    /// Do two axis-aligned boxes intersect or touch? Reporting only: it labels an event
    /// as "the mover could have touched this device along the way".
    fn boxes_touch_or_intersect(a: (f64, f64, f64, f64), b: (f64, f64, f64, f64)) -> bool {
        a.0 <= b.2 + CROSSING_EPS
            && b.0 <= a.2 + CROSSING_EPS
            && a.1 <= b.3 + CROSSING_EPS
            && b.1 <= a.3 + CROSSING_EPS
    }

    /// Crossing trigger: after the de-overlap pass accepted a move, list the devices
    /// the move flew over.
    ///
    /// ## The predicate
    ///
    /// The maintainer's rule is that legalization must not destroy the local
    /// arrangement — "the point is not to fly over somebody's head to land on a legal
    /// position" — and his operational form of it is **area coverage**: *"the area swept
    /// from the start point to the end point must not cover another device by more than
    /// a threshold."* So for each member of the moved object and each other device,
    /// `covered area / that device's area` is computed against the exact swept region
    /// (`swept_hull`), and anything strictly above `cover_frac` is an event.
    ///
    /// Coverage needs **no neighbour rule**: a device the body never passed near has
    /// coverage 0 and cannot trigger, so every other device is simply tested.
    ///
    /// A symmetry pair is one rigid object but two bodies, so each member sweeps and is
    /// judged on its own.
    ///
    /// ## What is recorded but does not trigger
    ///
    /// `order_reversed_x/y` (the mover changed sides), `swept_covers_centre` (the first
    /// candidate definition), and `in_component` / `adjacent_to_path` (the N1 / N2
    /// locality rules that the earlier, order-based trigger needed). All four were
    /// triggers or trigger inputs at some point; keeping them per event is what lets
    /// every one of those decisions be re-litigated from a log. `§10` of the design doc
    /// has the numbers.
    fn collect_move_crossings(
        &self,
        component_id: usize,
        object_id: usize,
        object: &DeoverlapObject,
        comp_cells: &[usize],
        pre_xy: &[(f64, f64)],
        w_dims: &[f64],
        h_dims: &[f64],
        cover_frac: f64,
        criterion: CrossCriterion,
    ) -> CrossingScan {
        let n = self.model.data.num_cells;
        let mut scan = CrossingScan::default();
        for (k, &m) in object.members.iter().enumerate() {
            if m >= n || k >= pre_xy.len() {
                continue;
            }
            let w = w_dims.get(m).copied().unwrap_or(0.0);
            let h = h_dims.get(m).copied().unwrap_or(0.0);
            if w <= 0.0 || h <= 0.0 {
                continue;
            }
            let start = self.cell_rect_at_with_effective_dims(m, pre_xy[k].0, pre_xy[k].1, w, h);
            let end = self.cell_rect_with_dims(m, w_dims, h_dims);
            let swept = (
                start.0.min(end.0),
                start.1.min(end.1),
                start.2.max(end.2),
                start.3.max(end.3),
            );
            let w_real = self.model.data.cell_widths.get(m).copied().unwrap_or(0.0);
            let h_real = self.model.data.cell_heights.get(m).copied().unwrap_or(0.0);
            let mx0 = pre_xy[k].0 + 0.5 * w_real;
            let my0 = pre_xy[k].1 + 0.5 * h_real;
            let (mx1, my1) = self.cell_centre(m);
            for c in 0..n {
                if object.members.contains(&c) || !self.cell_overlap_active(c) {
                    continue;
                }
                let other = self.cell_rect_with_dims(c, w_dims, h_dims);
                let cover = Self::swept_coverage_fraction(start, end, other);
                if cover > 0.0 {
                    scan.covered_any += 1;
                    if cover > 0.25 {
                        scan.cover_over_25 += 1;
                    }
                    if cover > 0.5 {
                        scan.cover_over_50 += 1;
                    }
                    if cover > 0.75 {
                        scan.cover_over_75 += 1;
                    }
                }
                let (cx, cy) = self.cell_centre(c);
                let order_reversed_x = Self::order_reversed_on_axis(mx0, mx1, cx);
                let order_reversed_y = Self::order_reversed_on_axis(my0, my1, cy);
                let in_component = comp_cells.binary_search(&c).is_ok();
                let adjacent_to_path = Self::boxes_touch_or_intersect(swept, other);
                // The order predicate is the one that needs a locality rule: changing
                // sides with a device on the far side of the die is none of its
                // business. Coverage needs none -- a device the body never passed near
                // has coverage 0.
                let order_hit =
                    (order_reversed_x || order_reversed_y) && (in_component || adjacent_to_path);
                if order_hit {
                    scan.order_reversed += 1;
                }
                let fired = (criterion.wants_order() && order_hit)
                    || (criterion.wants_cover() && cover > cover_frac);
                if !fired {
                    continue;
                }
                scan.rows.push(DeoverlapCrossingRow {
                    component_id,
                    object_id,
                    object_kind: object.kind.as_str().to_string(),
                    moved_cell: m,
                    crossed_cell: c,
                    cover_frac: cover,
                    order_reversed_x,
                    order_reversed_y,
                    in_component,
                    adjacent_to_path,
                    swept_covers_centre: Self::swept_box_contains_point(start, end, cx, cy),
                });
            }
        }
        scan
    }

    /// The set of cells that are still in an overlapping pair under the given
    /// effective dims. Sorted and de-duplicated, so the retry loop is deterministic.
    fn collect_overlapping_cells(&self, w_dims: &[f64], h_dims: &[f64]) -> Vec<usize> {
        let mut hit = Vec::<usize>::new();
        for p in 0..self.model.data.num_overlap_pairs {
            let i = self.model.data.overlap_pairs[2 * p] as usize;
            let j = self.model.data.overlap_pairs[2 * p + 1] as usize;
            if i == j || i >= self.model.data.num_cells || j >= self.model.data.num_cells {
                continue;
            }
            if self.cells_overlap_with_dims(i, j, w_dims, h_dims) {
                hit.push(i);
                hit.push(j);
            }
        }
        hit.sort_unstable();
        hit.dedup();
        hit
    }

    fn object_feasible_against_layout(
        &self,
        object: &DeoverlapObject,
        w_dims: &[f64],
        h_dims: &[f64],
    ) -> bool {
        let n = self.model.data.num_cells;
        let mut moving = vec![false; n];
        for &m in &object.members {
            if m < n {
                moving[m] = true;
            }
        }
        for p in 0..self.model.data.num_overlap_pairs {
            let i = self.model.data.overlap_pairs[2 * p] as usize;
            let j = self.model.data.overlap_pairs[2 * p + 1] as usize;
            if i >= n || j >= n || i == j {
                continue;
            }
            if !moving[i] && !moving[j] {
                continue;
            }
            if self.cells_overlap_with_dims(i, j, w_dims, h_dims) {
                return false;
            }
        }
        true
    }

    fn ring_offsets_2d(ring: i32) -> Vec<(i32, i32)> {
        if ring <= 0 {
            return vec![(0, 0)];
        }
        let mut pts = Vec::new();
        for ix in -ring..=ring {
            let rem = ring - ix.abs();
            if rem == 0 {
                pts.push((ix, 0));
            } else {
                pts.push((ix, rem));
                pts.push((ix, -rem));
            }
        }
        pts
    }

    fn ring_offsets_1d(ring: i32) -> Vec<i32> {
        if ring <= 0 {
            vec![0]
        } else {
            vec![-ring, ring]
        }
    }

    fn global_bbox_with_dims(&self, w_dims: &[f64], h_dims: &[f64]) -> DeoverlapBBox {
        let n = self
            .model
            .data
            .num_cells
            .min(w_dims.len())
            .min(h_dims.len());
        let mut bbox = DeoverlapBBox::default();
        for i in 0..n {
            if w_dims[i] <= 0.0 || h_dims[i] <= 0.0 {
                continue;
            }
            let (xl, yl, xh, yh) = self.cell_rect_with_dims(i, w_dims, h_dims);
            bbox.extend_rect(xl, yl, xh, yh);
        }
        bbox
    }

    fn static_bbox_excluding_members(
        &self,
        object: &DeoverlapObject,
        w_dims: &[f64],
        h_dims: &[f64],
    ) -> DeoverlapBBox {
        let n = self
            .model
            .data
            .num_cells
            .min(w_dims.len())
            .min(h_dims.len());
        let mut is_member = vec![false; n];
        for &m in &object.members {
            if m < n {
                is_member[m] = true;
            }
        }
        let mut bbox = DeoverlapBBox::default();
        for i in 0..n {
            if is_member[i] || w_dims[i] <= 0.0 || h_dims[i] <= 0.0 {
                continue;
            }
            let (xl, yl, xh, yh) = self.cell_rect_with_dims(i, w_dims, h_dims);
            bbox.extend_rect(xl, yl, xh, yh);
        }
        bbox
    }

    fn object_bbox_with_dims(
        &self,
        object: &DeoverlapObject,
        w_dims: &[f64],
        h_dims: &[f64],
    ) -> DeoverlapBBox {
        let n = self
            .model
            .data
            .num_cells
            .min(w_dims.len())
            .min(h_dims.len());
        let mut bbox = DeoverlapBBox::default();
        for &m in &object.members {
            if m >= n || w_dims[m] <= 0.0 || h_dims[m] <= 0.0 {
                continue;
            }
            let (xl, yl, xh, yh) = self.cell_rect_with_dims(m, w_dims, h_dims);
            bbox.extend_rect(xl, yl, xh, yh);
        }
        bbox
    }

    fn expand_axis_weights(ctx: &DeoverlapSearchContext) -> (f64, f64) {
        if ctx.r_eff >= 1.0 {
            (1.0 / ctx.r_eff.max(1e-12), ctx.r_eff)
        } else {
            let q = (1.0 / ctx.r_eff.max(1e-12)).max(1.0);
            (q, 1.0 / q)
        }
    }

    fn compute_expand_terms(
        ctx: &DeoverlapSearchContext,
        w_after: f64,
        h_after: f64,
        a_after: f64,
    ) -> (f64, f64, f64, f64, f64) {
        let e_w = if ctx.w_ref > 0.0 {
            ((w_after - ctx.w_ref) / ctx.w_ref).max(0.0).powi(2)
        } else {
            0.0
        };
        let e_h = if ctx.h_ref > 0.0 {
            ((h_after - ctx.h_ref) / ctx.h_ref).max(0.0).powi(2)
        } else {
            0.0
        };
        let e_a = if ctx.a_ref > 0.0 {
            ((a_after - ctx.a_ref) / ctx.a_ref).max(0.0).powi(2)
        } else {
            0.0
        };
        let (lambda_w, lambda_h) = Self::expand_axis_weights(ctx);
        let e_expand = lambda_w * e_w + lambda_h * e_h + ctx.lambda_area * e_a;
        let area_increase_ratio = if ctx.a_ref > 0.0 {
            ((a_after - ctx.a_ref) / ctx.a_ref).max(0.0)
        } else {
            0.0
        };
        (e_w, e_h, e_a, e_expand, area_increase_ratio)
    }

    fn elliptical_distance(dx: f64, dy: f64, ctx: &DeoverlapSearchContext) -> f64 {
        if !ctx.use_elliptical {
            return (dx * dx + dy * dy).sqrt();
        }
        let d2 = if ctx.r_eff >= 1.0 {
            dx * dx / ctx.r_eff.max(1e-12) + ctx.r_eff * dy * dy
        } else {
            let q = (1.0 / ctx.r_eff.max(1e-12)).max(1.0);
            dy * dy / q + q * dx * dx
        };
        d2.max(0.0).sqrt()
    }

    fn better_deoverlap_candidate(
        a: &DeoverlapCandidateEval,
        b: &DeoverlapCandidateEval,
    ) -> Ordering {
        a.score
            .partial_cmp(&b.score)
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                let a_disp2 = a.dx * a.dx + a.dy * a.dy;
                let b_disp2 = b.dx * b.dx + b.dy * b.dy;
                a_disp2.partial_cmp(&b_disp2).unwrap_or(Ordering::Equal)
            })
            .then_with(|| a.ring_hit.cmp(&b.ring_hit))
            .then_with(|| {
                a.dx.partial_cmp(&b.dx)
                    .unwrap_or(Ordering::Equal)
                    .then_with(|| a.dy.partial_cmp(&b.dy).unwrap_or(Ordering::Equal))
            })
    }

    fn search_object_best_feasible(
        &mut self,
        object: &DeoverlapObject,
        w_dims: &[f64],
        h_dims: &[f64],
        grid: f64,
        max_ring: f64,
        ctx: &DeoverlapSearchContext,
    ) -> DeoverlapSearchAttempt {
        let mut attempt = DeoverlapSearchAttempt::default();
        if object.members.is_empty() || grid <= 0.0 || max_ring < 0.0 {
            return attempt;
        }
        let n = self.model.data.num_cells;
        let members = object.members.clone();
        let old_xy: Vec<(f64, f64)> = members
            .iter()
            .map(|&m| (self.pos[m], self.pos[m + n]))
            .collect();

        let member_centroid = |placer: &Placer| -> (f64, f64) {
            if members.is_empty() {
                return (0.0, 0.0);
            }
            let mut sx = 0.0;
            let mut sy = 0.0;
            for &m in &members {
                sx += placer.pos[m];
                sy += placer.pos[m + n];
            }
            (sx / members.len() as f64, sy / members.len() as f64)
        };
        let (seed_cx, seed_cy) = member_centroid(self);
        let max_ring_idx = (max_ring / grid).ceil().max(0.0) as i32;
        let static_bbox = self.static_bbox_excluding_members(object, w_dims, h_dims);

        let restore_old = |placer: &mut Placer| {
            for (k, &m) in members.iter().enumerate() {
                placer.pos[m] = old_xy[k].0;
                placer.pos[m + n] = old_xy[k].1;
            }
        };

        let mut evaluate_candidate = |placer: &mut Placer, ring_hit: usize| {
            attempt.candidates_tested += 1;
            if !placer.object_feasible_against_layout(object, w_dims, h_dims) {
                return;
            }
            attempt.candidates_feasible += 1;
            let (cx, cy) = member_centroid(placer);
            let dx = cx - seed_cx;
            let dy = cy - seed_cy;
            let moved_bbox = placer.object_bbox_with_dims(object, w_dims, h_dims);
            let after_bbox = static_bbox.union(moved_bbox);
            let w_after = after_bbox.width();
            let h_after = after_bbox.height();
            let a_after = after_bbox.area();
            let (e_w, e_h, e_a, e_expand, area_increase_ratio) =
                Self::compute_expand_terms(ctx, w_after, h_after, a_after);
            if area_increase_ratio > ctx.rho_max {
                attempt.rejected_by_area_cap += 1;
                return;
            }

            let d_ell = Self::elliptical_distance(dx, dy, ctx);
            let lambda_d = 1.0;
            let e_topo = 0.0;
            let score = lambda_d * d_ell + ctx.lambda_expand * e_expand + ctx.lambda_topo * e_topo;
            let cand = DeoverlapCandidateEval {
                score,
                d_ell,
                e_w,
                e_h,
                e_a,
                e_expand,
                dx,
                dy,
                ring_hit,
                w_after,
                h_after,
                a_after,
                area_increase_ratio,
            };
            let member_xy: Vec<(f64, f64)> = members
                .iter()
                .map(|&m| (placer.pos[m], placer.pos[m + n]))
                .collect();
            let is_better = match attempt.best {
                None => true,
                Some(best) => Self::better_deoverlap_candidate(&cand, &best) == Ordering::Less,
            };
            if is_better {
                attempt.best = Some(cand);
                attempt.best_member_xy = member_xy;
            }
        };

        match object.kind {
            DeoverlapObjectKind::Singleton => {
                let m = object.members[0];
                let sx = old_xy[0].0;
                let sy = old_xy[0].1;
                for ring in 0..=max_ring_idx {
                    for (ix, iy) in Self::ring_offsets_2d(ring) {
                        self.pos[m] = sx + ix as f64 * grid;
                        self.pos[m + n] = sy + iy as f64 * grid;
                        evaluate_candidate(self, ring as usize);
                    }
                }
            }
            DeoverlapObjectKind::SelfSym => {
                let m = object.members[0];
                let is_vert = object.is_vert.unwrap_or(true);
                let axis = object.axis.unwrap_or_else(|| self.axis_for_group(is_vert));
                let sx = old_xy[0].0;
                let sy = old_xy[0].1;
                if is_vert {
                    let fixed_x = axis - self.model.data.sym_anchor_x[m];
                    for ring in 0..=max_ring_idx {
                        for iy in Self::ring_offsets_1d(ring) {
                            self.pos[m] = fixed_x;
                            self.pos[m + n] = sy + iy as f64 * grid;
                            evaluate_candidate(self, ring as usize);
                        }
                    }
                } else {
                    let fixed_y = axis - self.model.data.sym_anchor_y[m];
                    for ring in 0..=max_ring_idx {
                        for ix in Self::ring_offsets_1d(ring) {
                            self.pos[m] = sx + ix as f64 * grid;
                            self.pos[m + n] = fixed_y;
                            evaluate_candidate(self, ring as usize);
                        }
                    }
                }
            }
            DeoverlapObjectKind::Pair => {
                if object.members.len() != 2 {
                    restore_old(self);
                    return attempt;
                }
                let is_vert = object.is_vert.unwrap_or(true);
                let axis = object.axis.unwrap_or_else(|| self.axis_for_group(is_vert));
                let lo = object.primary_lo.unwrap_or(object.members[0]);
                let hi = object.primary_hi.unwrap_or(object.members[1]);
                let (a, b) = if lo == hi {
                    (object.members[0], object.members[1])
                } else {
                    (lo, hi)
                };

                if is_vert {
                    let ax = self.pos[a] + self.model.data.sym_anchor_x[a];
                    let bx = self.pos[b] + self.model.data.sym_anchor_x[b];
                    let ay = self.pos[a + n] + self.model.data.sym_anchor_y[a];
                    let by = self.pos[b + n] + self.model.data.sym_anchor_y[b];
                    let d0 = 0.5 * ((ax - axis).abs() + (bx - axis).abs());
                    let y0 = 0.5 * (ay + by);
                    for ring in 0..=max_ring_idx {
                        for (id, io) in Self::ring_offsets_2d(ring) {
                            let d = (d0 + id as f64 * grid).max(0.0);
                            let y = y0 + io as f64 * grid;
                            let ax_new = axis - d;
                            let bx_new = axis + d;
                            self.pos[a] = ax_new - self.model.data.sym_anchor_x[a];
                            self.pos[b] = bx_new - self.model.data.sym_anchor_x[b];
                            self.pos[a + n] = y - self.model.data.sym_anchor_y[a];
                            self.pos[b + n] = y - self.model.data.sym_anchor_y[b];
                            evaluate_candidate(self, ring as usize);
                        }
                    }
                } else {
                    let ay = self.pos[a + n] + self.model.data.sym_anchor_y[a];
                    let by = self.pos[b + n] + self.model.data.sym_anchor_y[b];
                    let ax = self.pos[a] + self.model.data.sym_anchor_x[a];
                    let bx = self.pos[b] + self.model.data.sym_anchor_x[b];
                    let d0 = 0.5 * ((ay - axis).abs() + (by - axis).abs());
                    let x0 = 0.5 * (ax + bx);
                    for ring in 0..=max_ring_idx {
                        for (io, id) in Self::ring_offsets_2d(ring) {
                            let d = (d0 + id as f64 * grid).max(0.0);
                            let x = x0 + io as f64 * grid;
                            let ay_new = axis - d;
                            let by_new = axis + d;
                            self.pos[a + n] = ay_new - self.model.data.sym_anchor_y[a];
                            self.pos[b + n] = by_new - self.model.data.sym_anchor_y[b];
                            self.pos[a] = x - self.model.data.sym_anchor_x[a];
                            self.pos[b] = x - self.model.data.sym_anchor_x[b];
                            evaluate_candidate(self, ring as usize);
                        }
                    }
                }
            }
        }

        if let Some(best_xy) =
            (!attempt.best_member_xy.is_empty()).then_some(attempt.best_member_xy.clone())
        {
            for (k, &m) in members.iter().enumerate() {
                self.pos[m] = best_xy[k].0;
                self.pos[m + n] = best_xy[k].1;
            }
        } else {
            restore_old(self);
        }
        attempt
    }

    fn cmp_deoverlap_area_then_id_for_move(
        objects: &[DeoverlapObject],
        a: usize,
        b: usize,
    ) -> Ordering {
        objects[a]
            .area
            .partial_cmp(&objects[b].area)
            .unwrap_or(Ordering::Equal)
            .then_with(|| a.cmp(&b))
    }

    fn cmp_deoverlap_area_then_id_for_fixed(
        objects: &[DeoverlapObject],
        a: usize,
        b: usize,
    ) -> Ordering {
        objects[a]
            .area
            .partial_cmp(&objects[b].area)
            .unwrap_or(Ordering::Equal)
            .then_with(|| b.cmp(&a))
    }

    fn select_deoverlap_fixed_obj(objects: &[DeoverlapObject], comp: &[usize]) -> Option<usize> {
        comp.iter()
            .copied()
            .max_by(|&a, &b| Self::cmp_deoverlap_area_then_id_for_fixed(objects, a, b))
    }

    fn select_deoverlap_move_target(
        objects: &[DeoverlapObject],
        conflicted: &HashSet<usize>,
        fixed_obj: usize,
        blocked: &HashSet<usize>,
    ) -> Option<usize> {
        conflicted
            .iter()
            .copied()
            .filter(|oid| *oid != fixed_obj && !blocked.contains(oid))
            .min_by(|&a, &b| Self::cmp_deoverlap_area_then_id_for_move(objects, a, b))
    }

    fn discrete_deoverlap_after_tail(&mut self) {
        let debug_deoverlap = matches!(
            std::env::var("HIER_DEBUG_DEOVERLAP").ok().as_deref(),
            Some("1" | "true" | "TRUE" | "yes" | "YES")
        );
        // Crossing trigger: crossings are reported per de-overlap pass, so start each
        // pass with an empty ledger.
        self.deoverlap_crossings.clear();
        self.deoverlap_crossing_cells.clear();
        self.deoverlap_crossing_stats = (0, 0, 0, 0, 0);
        let mut summary = DeoverlapSummary {
            enabled: self.model.params.tail_deoverlap_enable,
            attempted: false,
            ..DeoverlapSummary::default()
        };
        if !self.model.params.tail_deoverlap_enable {
            self.deoverlap_summary = Some(summary);
            self.deoverlap_end_pos = Some(self.pos.clone());
            return;
        }

        summary.attempted = true;
        let infl_tail = self.overlap_dims_for_deoverlap();
        self.deoverlap_infl_tail_snapshot = infl_tail.clone();
        if !infl_tail.is_empty() {
            summary.infl_tail_min = infl_tail.iter().copied().fold(f64::INFINITY, f64::min);
            summary.infl_tail_max = infl_tail.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            summary.infl_tail_mean =
                infl_tail.iter().copied().sum::<f64>() / infl_tail.len() as f64;
        }

        summary.force_sym_applied = self.force_symmetry_before_deoverlap();
        // Diagnostics only. Do NOT repair self-overlapping mirror pairs here: round 0
        // of every committed case arrives with all of its pairs welded onto the axis,
        // and the LCR retry loop's halo inflation is what normally breaks them apart
        // over the following rounds. The welding worth repairing is the one still
        // standing when that loop has exhausted, and that is where the rescue lives
        // (`run_phases_with_sym_weld_rescue`).
        self.sym_weld_welded_at_lcr_entry = self.self_overlapping_sym_pairs(&self.pos).len();
        let (w_feas, h_feas) = self.deoverlap_feasible_dims(&infl_tail);
        let (objects, cell_to_obj) = self.build_deoverlap_objects(&w_feas, &h_feas);
        if objects.is_empty() {
            self.deoverlap_components_before.clear();
            self.deoverlap_components_after.clear();
            self.deoverlap_summary = Some(summary);
            self.deoverlap_end_pos = Some(self.pos.clone());
            return;
        }

        let tail_bbox = self.global_bbox_with_dims(&w_feas, &h_feas);
        let w_tail = tail_bbox.width();
        let h_tail = tail_bbox.height();
        let a_tail = tail_bbox.area();
        let kappa = self.model.params.tail_deoverlap_search_kappa.max(1.0);
        let r_raw = if h_tail > 1e-12 && w_tail > 1e-12 {
            w_tail / h_tail
        } else {
            1.0
        };
        let r_eff = r_raw.clamp(1.0 / kappa, kappa);
        let long_axis = if r_eff >= 1.0 { Axis::X } else { Axis::Y };
        let use_elliptical = self.model.params.tail_deoverlap_use_elliptical_search;
        let score_ctx = DeoverlapSearchContext {
            w_ref: w_tail,
            h_ref: h_tail,
            a_ref: a_tail.max(1e-12),
            w_tail,
            h_tail,
            r_raw,
            r_eff,
            long_axis,
            lambda_expand: if use_elliptical {
                self.model.params.tail_deoverlap_lambda_expand
            } else {
                0.0
            },
            lambda_area: self.model.params.tail_deoverlap_lambda_area,
            lambda_topo: self.model.params.tail_deoverlap_lambda_topo,
            rho_max: if use_elliptical {
                self.model.params.tail_deoverlap_rho_max
            } else {
                f64::INFINITY
            },
            use_elliptical,
        };
        let long_axis_str = match score_ctx.long_axis {
            Axis::X => "x",
            Axis::Y => "y",
        };
        summary.w_ref = score_ctx.w_ref;
        summary.h_ref = score_ctx.h_ref;
        summary.a_ref = score_ctx.a_ref;
        summary.w_tail = score_ctx.w_tail;
        summary.h_tail = score_ctx.h_tail;
        summary.a_tail = a_tail;
        summary.r_raw = score_ctx.r_raw;
        summary.r_eff = score_ctx.r_eff;
        summary.long_axis = long_axis_str.to_string();
        eprintln!(
            "[deoverlap] ref_bbox(w={:.6e},h={:.6e},a={:.6e}) tail_bbox(w={:.6e},h={:.6e}) r_raw={:.6e} r_eff={:.6e} long_axis={} elliptical={} lambda_expand={:.6e} lambda_area={:.6e} rho_max={:.6e}",
            summary.w_ref,
            summary.h_ref,
            summary.a_ref,
            summary.w_tail,
            summary.h_tail,
            summary.r_raw,
            summary.r_eff,
            summary.long_axis,
            score_ctx.use_elliptical,
            score_ctx.lambda_expand,
            score_ctx.lambda_area,
            score_ctx.rho_max
        );

        let (components_before, overlap_pairs_before) =
            self.build_object_conflict_components(&objects, &cell_to_obj, &w_feas, &h_feas);
        self.deoverlap_components_before = components_before.clone();
        summary.total_components_before = components_before.len();
        summary.overlap_pairs_before = overlap_pairs_before;

        if debug_deoverlap {
            eprintln!(
                "[deoverlap-debug] components_before={} overlap_pairs_before={}",
                components_before.len(),
                overlap_pairs_before
            );
            for (cid, comp) in components_before.iter().enumerate() {
                let mut lines = Vec::new();
                for &oid in comp {
                    if oid >= objects.len() {
                        continue;
                    }
                    let obj = &objects[oid];
                    let area_real = obj
                        .members
                        .iter()
                        .copied()
                        .filter(|&m| {
                            m < self.model.data.num_cells
                                && m < self.model.data.cell_widths_keepout.len()
                                && m < self.model.data.cell_heights_keepout.len()
                        })
                        .map(|m| {
                            self.model.data.cell_widths_keepout[m]
                                * self.model.data.cell_heights_keepout[m]
                        })
                        .sum::<f64>();
                    lines.push(format!(
                        "oid={} kind={} area_feas={:.6e} area_real={:.6e} members={:?}",
                        oid,
                        obj.kind.as_str(),
                        obj.area,
                        area_real,
                        obj.members
                    ));
                }
                eprintln!(
                    "[deoverlap-debug] component={} objects: {}",
                    cid,
                    lines.join(" | ")
                );
            }
        }

        let mut object_to_component = vec![usize::MAX; objects.len()];
        for (cid, comp) in components_before.iter().enumerate() {
            for &oid in comp {
                if oid < object_to_component.len() {
                    object_to_component[oid] = cid;
                }
            }
        }
        let mut comp_overlap_pairs = vec![0usize; components_before.len()];
        for p in 0..self.model.data.num_overlap_pairs {
            let i = self.model.data.overlap_pairs[2 * p] as usize;
            let j = self.model.data.overlap_pairs[2 * p + 1] as usize;
            if i >= cell_to_obj.len() || j >= cell_to_obj.len() || i == j {
                continue;
            }
            if !self.cells_overlap_with_dims(i, j, &w_feas, &h_feas) {
                continue;
            }
            let oi = cell_to_obj[i];
            let oj = cell_to_obj[j];
            if oi >= object_to_component.len()
                || oj >= object_to_component.len()
                || oi == oj
                || oi == usize::MAX
                || oj == usize::MAX
            {
                continue;
            }
            let ci = object_to_component[oi];
            let cj = object_to_component[oj];
            if ci == cj && ci != usize::MAX {
                comp_overlap_pairs[ci] += 1;
            }
        }
        let mut comp_bbox_area = vec![0.0f64; components_before.len()];
        for (cid, comp) in components_before.iter().enumerate() {
            let mut bbox = DeoverlapBBox::default();
            for &oid in comp {
                if oid >= objects.len() {
                    continue;
                }
                for &m in &objects[oid].members {
                    if m >= self.model.data.num_cells || m >= w_feas.len() || m >= h_feas.len() {
                        continue;
                    }
                    if w_feas[m] <= 0.0 || h_feas[m] <= 0.0 {
                        continue;
                    }
                    let (xl, yl, xh, yh) = self.cell_rect_with_dims(m, &w_feas, &h_feas);
                    bbox.extend_rect(xl, yl, xh, yh);
                }
            }
            comp_bbox_area[cid] = bbox.area();
        }
        let mut component_order: Vec<usize> = (0..components_before.len()).collect();
        component_order.sort_by(|&a, &b| {
            comp_overlap_pairs[b]
                .cmp(&comp_overlap_pairs[a])
                .then_with(|| {
                    comp_bbox_area[b]
                        .partial_cmp(&comp_bbox_area[a])
                        .unwrap_or(Ordering::Equal)
                })
                .then_with(|| {
                    let amin = components_before[a].first().copied().unwrap_or(usize::MAX);
                    let bmin = components_before[b].first().copied().unwrap_or(usize::MAX);
                    amin.cmp(&bmin)
                })
        });

        let mut attempt_idx = 0usize;
        // Crossing trigger, reporting only: the coverage histogram at the three
        // thresholds under measurement, plus what the superseded order-inversion
        // predicate would have flagged. One default run answers all of them.
        let mut cover_touched = 0usize;
        let mut cover_over_25 = 0usize;
        let mut cover_over_50 = 0usize;
        let mut cover_over_75 = 0usize;
        let mut order_reversed_pairs = 0usize;
        let crossing_cover_frac = self.model.params.lcr_retry_cross_cover_frac;
        let crossing_criterion =
            CrossCriterion::from_param(&self.model.params.lcr_retry_cross_criterion);
        for &cid in &component_order {
            let comp = &components_before[cid];
            if comp.is_empty() {
                continue;
            }
            let comp_set: HashSet<usize> = comp.iter().copied().collect();
            // Crossing trigger: the cells of this conflict component, sorted, so the
            // locality test ("is this device one of the ones the move is about?") is a
            // binary search and never depends on set iteration order.
            let comp_cells: Vec<usize> = {
                let mut v = Vec::new();
                for &oid in comp {
                    if oid < objects.len() {
                        v.extend_from_slice(&objects[oid].members);
                    }
                }
                v.sort_unstable();
                v.dedup();
                v
            };
            let fixed_obj = Self::select_deoverlap_fixed_obj(&objects, comp).unwrap_or(comp[0]);
            let fixed = &objects[fixed_obj];
            log::info!(
                "[deoverlap] component={} fixed_obj={} kind={} area={:.6e}",
                cid,
                fixed_obj,
                fixed.kind.as_str(),
                fixed.area
            );
            if debug_deoverlap {
                let area_real = fixed
                    .members
                    .iter()
                    .copied()
                    .filter(|&m| {
                        m < self.model.data.num_cells
                            && m < self.model.data.cell_widths_keepout.len()
                            && m < self.model.data.cell_heights_keepout.len()
                    })
                    .map(|m| {
                        self.model.data.cell_widths_keepout[m]
                            * self.model.data.cell_heights_keepout[m]
                    })
                    .sum::<f64>();
                eprintln!(
                    "[deoverlap-debug] component={} fixed_obj={} kind={} area_feas={:.6e} area_real={:.6e} members={:?}",
                    cid,
                    fixed_obj,
                    fixed.kind.as_str(),
                    fixed.area,
                    area_real,
                    fixed.members
                );
            }
            let mut blocked = HashSet::<usize>::new();
            let max_rounds = comp.len().saturating_mul(8).saturating_add(8);

            for _ in 0..max_rounds {
                let mut conflicted = HashSet::<usize>::new();
                for p in 0..self.model.data.num_overlap_pairs {
                    let i = self.model.data.overlap_pairs[2 * p] as usize;
                    let j = self.model.data.overlap_pairs[2 * p + 1] as usize;
                    if i >= cell_to_obj.len() || j >= cell_to_obj.len() || i == j {
                        continue;
                    }
                    let oi = cell_to_obj[i];
                    let oj = cell_to_obj[j];
                    if !comp_set.contains(&oi) || !comp_set.contains(&oj) {
                        continue;
                    }
                    if self.cells_overlap_with_dims(i, j, &w_feas, &h_feas) {
                        conflicted.insert(oi);
                        conflicted.insert(oj);
                    }
                }
                if conflicted.is_empty() {
                    break;
                }
                let move_target =
                    Self::select_deoverlap_move_target(&objects, &conflicted, fixed_obj, &blocked);
                // NOTE (measured, do not "improve" this): letting a self-overlapping
                // object move itself here -- a symmetry pair collapsed towards its axis
                // is one rigid object, so the component it forms alone has no other
                // object to move against and the loop stops -- looks like the obvious
                // fix and makes every example design worse. It resolves round 0
                // locally, which starves the LCR retry loop of the halo escalation that
                // is what actually opens these layouts up: measured, wirelength rises by
                // up to 62% and two of the four examples end up outside their declared
                // region. The welded case is handled where it belongs, after the retry
                // loop has genuinely exhausted (`run_phases_with_sym_weld_rescue`).
                let Some(oid) = move_target else {
                    break;
                };
                let obj = &objects[oid];
                log::info!(
                    "[deoverlap] component={} move_obj={} kind={} area={:.6e} conflicted={} blocked={}",
                    cid,
                    oid,
                    obj.kind.as_str(),
                    obj.area,
                    conflicted.len(),
                    blocked.len()
                );
                if debug_deoverlap {
                    let area_real = obj
                        .members
                        .iter()
                        .copied()
                        .filter(|&m| {
                            m < self.model.data.num_cells
                                && m < self.model.data.cell_widths_keepout.len()
                                && m < self.model.data.cell_heights_keepout.len()
                        })
                        .map(|m| {
                            self.model.data.cell_widths_keepout[m]
                                * self.model.data.cell_heights_keepout[m]
                        })
                        .sum::<f64>();
                    eprintln!(
                        "[deoverlap-debug] component={} move_obj={} kind={} area_feas={:.6e} area_real={:.6e} conflicted={} blocked={}",
                        cid,
                        oid,
                        obj.kind.as_str(),
                        obj.area,
                        area_real,
                        conflicted.len(),
                        blocked.len()
                    );
                }
                let mut moved = false;
                // Crossing trigger: remember where the body was, so an accepted move
                // can be tested for a side change against its local neighbours. A failed
                // tier restores the object, so one snapshot before the tier loop is enough.
                let pre_xy: Vec<(f64, f64)> = obj
                    .members
                    .iter()
                    .map(|&m| {
                        if m < self.model.data.num_cells {
                            (self.pos[m], self.pos[m + self.model.data.num_cells])
                        } else {
                            (0.0, 0.0)
                        }
                    })
                    .collect();
                for (tier, grid, ring) in [
                    (
                        "fine",
                        self.model.params.tail_deoverlap_grid_fine,
                        self.model.params.tail_deoverlap_ring_fine,
                    ),
                    (
                        "coarse",
                        self.model.params.tail_deoverlap_grid_coarse,
                        self.model.params.tail_deoverlap_ring_coarse,
                    ),
                ] {
                    attempt_idx += 1;
                    let attempt = self
                        .search_object_best_feasible(obj, &w_feas, &h_feas, grid, ring, &score_ctx);
                    summary.total_candidates_tested += attempt.candidates_tested;
                    summary.total_candidates_feasible += attempt.candidates_feasible;
                    summary.total_candidates_rejected_area_cap += attempt.rejected_by_area_cap;
                    if let Some(best) = attempt.best {
                        summary.max_area_increase_ratio_observed = summary
                            .max_area_increase_ratio_observed
                            .max(best.area_increase_ratio);
                        self.deoverlap_moves.push(DeoverlapMoveRow {
                            component_id: cid,
                            object_id: oid,
                            object_kind: obj.kind.as_str().to_string(),
                            object_area: obj.area,
                            attempt_idx,
                            tier: tier.to_string(),
                            ring: best.ring_hit,
                            dx: best.dx,
                            dy: best.dy,
                            success: true,
                            reason: "found_feasible".to_string(),
                            w_ref: score_ctx.w_ref,
                            h_ref: score_ctx.h_ref,
                            a_ref: score_ctx.a_ref,
                            w_tail: score_ctx.w_tail,
                            h_tail: score_ctx.h_tail,
                            r_raw: score_ctx.r_raw,
                            r_eff: score_ctx.r_eff,
                            long_axis: long_axis_str.to_string(),
                            candidates_tested: attempt.candidates_tested,
                            candidates_feasible: attempt.candidates_feasible,
                            rejected_by_area_cap: attempt.rejected_by_area_cap,
                            best_score: best.score,
                            d_ell: best.d_ell,
                            e_w: best.e_w,
                            e_h: best.e_h,
                            e_a: best.e_a,
                            e_expand: best.e_expand,
                            w_after: best.w_after,
                            h_after: best.h_after,
                            a_after: best.a_after,
                            area_increase_ratio: best.area_increase_ratio,
                        });
                        eprintln!(
                            "[deoverlap] component={} move_obj={} tier={} best(dx={:.6e},dy={:.6e}) d_ell={:.6e} e_expand={:.6e} score={:.6e} cap_reject={}/{} bbox_after(w={:.6e},h={:.6e},a={:.6e})",
                            cid,
                            oid,
                            tier,
                            best.dx,
                            best.dy,
                            best.d_ell,
                            best.e_expand,
                            best.score,
                            attempt.rejected_by_area_cap,
                            attempt.candidates_tested,
                            best.w_after,
                            best.h_after,
                            best.a_after
                        );
                        let scan = self.collect_move_crossings(
                            cid,
                            oid,
                            obj,
                            &comp_cells,
                            &pre_xy,
                            &w_feas,
                            &h_feas,
                            crossing_cover_frac,
                            crossing_criterion,
                        );
                        cover_touched += scan.covered_any;
                        cover_over_25 += scan.cover_over_25;
                        cover_over_50 += scan.cover_over_50;
                        cover_over_75 += scan.cover_over_75;
                        order_reversed_pairs += scan.order_reversed;
                        let crossings = scan.rows;
                        for row in &crossings {
                            eprintln!(
                                "[lcr-cross] component={} object={} kind={} moved_cell={} \
                                 crossed_cell={} cover={:.4} rev_x={} rev_y={} \
                                 in_component={} adjacent_to_path={} \
                                 swept_covers_centre={}",
                                row.component_id,
                                row.object_id,
                                row.object_kind,
                                row.moved_cell,
                                row.crossed_cell,
                                row.cover_frac,
                                row.order_reversed_x,
                                row.order_reversed_y,
                                row.in_component,
                                row.adjacent_to_path,
                                row.swept_covers_centre
                            );
                        }
                        self.deoverlap_crossings.extend(crossings);
                        moved = true;
                        break;
                    }
                    self.deoverlap_moves.push(DeoverlapMoveRow {
                        component_id: cid,
                        object_id: oid,
                        object_kind: obj.kind.as_str().to_string(),
                        object_area: obj.area,
                        attempt_idx,
                        tier: tier.to_string(),
                        ring: (ring / grid.max(1e-9)).ceil().max(0.0) as usize,
                        dx: 0.0,
                        dy: 0.0,
                        success: false,
                        reason: if attempt.rejected_by_area_cap > 0
                            && attempt.candidates_feasible > 0
                        {
                            "rejected_by_area_cap".to_string()
                        } else {
                            "no_feasible_in_tier".to_string()
                        },
                        w_ref: score_ctx.w_ref,
                        h_ref: score_ctx.h_ref,
                        a_ref: score_ctx.a_ref,
                        w_tail: score_ctx.w_tail,
                        h_tail: score_ctx.h_tail,
                        r_raw: score_ctx.r_raw,
                        r_eff: score_ctx.r_eff,
                        long_axis: long_axis_str.to_string(),
                        candidates_tested: attempt.candidates_tested,
                        candidates_feasible: attempt.candidates_feasible,
                        rejected_by_area_cap: attempt.rejected_by_area_cap,
                        best_score: 0.0,
                        d_ell: 0.0,
                        e_w: 0.0,
                        e_h: 0.0,
                        e_a: 0.0,
                        e_expand: 0.0,
                        w_after: 0.0,
                        h_after: 0.0,
                        a_after: 0.0,
                        area_increase_ratio: 0.0,
                    });
                }
                if !moved {
                    blocked.insert(oid);
                }
            }
        }

        let (components_after, overlap_pairs_after) =
            self.build_object_conflict_components(&objects, &cell_to_obj, &w_feas, &h_feas);
        // Record exactly which cells LCR failed to separate. This is the input to
        // the retry loop's "only inflate the conflicting devices" rule.
        self.deoverlap_unresolved_cells = self.collect_overlapping_cells(&w_feas, &h_feas);
        // Crossing trigger: the offenders are the devices that *did the crossing* --
        // they are the ones the maintainer's rule says had too little local space.
        self.deoverlap_crossing_cells = {
            let mut v: Vec<usize> = self
                .deoverlap_crossings
                .iter()
                .map(|r| r.moved_cell)
                .collect();
            v.sort_unstable();
            v.dedup();
            v
        };
        self.deoverlap_crossing_stats = (
            cover_touched,
            cover_over_25,
            cover_over_50,
            cover_over_75,
            order_reversed_pairs,
        );
        if !self.deoverlap_crossings.is_empty() || cover_touched > 0 {
            eprintln!(
                "[lcr-cross] pass produced {} crossing event(s) over {} device(s) \
                 (criterion={}, cover_frac={}); coverage histogram: touched={} \
                 >0.25={} >0.50={} >0.75={}; the order predicate alone would flag {}; \
                 of the events, {} are against a conflict partner (N1)",
                self.deoverlap_crossings.len(),
                self.deoverlap_crossing_cells.len(),
                self.model.params.lcr_retry_cross_criterion,
                crossing_cover_frac,
                cover_touched,
                cover_over_25,
                cover_over_50,
                cover_over_75,
                order_reversed_pairs,
                self.deoverlap_crossings
                    .iter()
                    .filter(|r| r.in_component)
                    .count(),
            );
        }
        self.deoverlap_components_after = components_after.clone();
        summary.total_components_after = components_after.len();
        summary.overlap_pairs_after = overlap_pairs_after;
        summary.unresolved_components = components_after.len();
        summary.resolved_components = summary
            .total_components_before
            .saturating_sub(summary.unresolved_components);
        summary.move_count = self.deoverlap_moves.iter().filter(|m| m.success).count();
        summary.failed_attempt_count = self.deoverlap_moves.iter().filter(|m| !m.success).count();
        let final_bbox = self.global_bbox_with_dims(&w_feas, &h_feas);
        let w_final = final_bbox.width();
        let h_final = final_bbox.height();
        let a_final = final_bbox.area();
        summary.final_width_ratio_to_ref = if summary.w_ref > 0.0 {
            w_final / summary.w_ref
        } else {
            1.0
        };
        summary.final_height_ratio_to_ref = if summary.h_ref > 0.0 {
            h_final / summary.h_ref
        } else {
            1.0
        };
        summary.final_area_ratio_to_ref = if summary.a_ref > 0.0 {
            a_final / summary.a_ref
        } else {
            1.0
        };
        eprintln!(
            "[deoverlap] summary moves={} failed={} overlap_pairs_before={} overlap_pairs_after={} cap_reject_total={} max_area_increase={:.6e} final_ratio(w={:.6e},h={:.6e},a={:.6e})",
            summary.move_count,
            summary.failed_attempt_count,
            summary.overlap_pairs_before,
            summary.overlap_pairs_after,
            summary.total_candidates_rejected_area_cap,
            summary.max_area_increase_ratio_observed,
            summary.final_width_ratio_to_ref,
            summary.final_height_ratio_to_ref,
            summary.final_area_ratio_to_ref
        );

        self.deoverlap_summary = Some(summary);
        self.deoverlap_end_pos = Some(self.pos.clone());
    }

    /// Snapshot of everything a multi-phase pass mutates that its own prologue does *not*
    /// reset. Restoring it makes a second pass start from exactly the same state as
    /// the first, so the only thing that differs between passes is `lcr_retry_infl`.
    fn snapshot_entry_state(&self) -> EntryState {
        EntryState {
            pos: self.pos.clone(),
            z: self.z.clone(),
            best_pos: self.best_pos.clone(),
            best_loss: self.best_loss,
            phi: self.model.phi.clone(),
            phi_tilde: self.phi_tilde.clone(),
            lambdas: self.model.lambdas.clone(),
            lambda_ap: self.model.lambda_ap,
            checkpoint_pos: self.checkpoint_pos.clone(),
            tail_state: self.tail_state,
        }
    }

    fn restore_entry_state(&mut self, s: &EntryState) {
        self.pos = s.pos.clone();
        self.z = s.z.clone();
        self.best_pos = s.best_pos.clone();
        self.best_loss = s.best_loss;
        self.model.phi = s.phi.clone();
        self.phi_tilde = s.phi_tilde.clone();
        self.model.lambdas = s.lambdas.clone();
        self.model.lambda_ap = s.lambda_ap;
        self.checkpoint_pos = s.checkpoint_pos.clone();
        self.tail_state = s.tail_state;
        self.loss_history.clear();
        self.move_history.clear();
        self.tail_diag_rows.clear();
        self.tail_diag_positions.clear();
        self.tail_diag_snapshots.clear();
        self.conflict_flags.fill(false);
        self.small_conflict_flags.fill(false);
    }

    fn capture_pass(&self, result: &PlacementResult) -> PassOutputs {
        PassOutputs {
            result: result.clone(),
            pos: self.pos.clone(),
            z: self.z.clone(),
            phi: self.model.phi.clone(),
            phi_tilde: self.phi_tilde.clone(),
            base_end_pos: self.base_end_pos.clone(),
            tail_end_pos: self.tail_end_pos.clone(),
            best_composite_pos: self.best_composite_pos.clone(),
            deoverlap_end_pos: self.deoverlap_end_pos.clone(),
            deoverlap_summary: self.deoverlap_summary.clone(),
            deoverlap_moves: self.deoverlap_moves.clone(),
            deoverlap_components_before: self.deoverlap_components_before.clone(),
            deoverlap_components_after: self.deoverlap_components_after.clone(),
            deoverlap_infl_tail_snapshot: self.deoverlap_infl_tail_snapshot.clone(),
            deoverlap_unresolved_cells: self.deoverlap_unresolved_cells.clone(),
            deoverlap_crossings: self.deoverlap_crossings.clone(),
            deoverlap_crossing_cells: self.deoverlap_crossing_cells.clone(),
            summary: self.run_summary.clone(),
            selected_label: self.selected_label.clone(),
            base_selected_label: self.base_selected_label.clone(),
            tail_selected_label: self.tail_selected_label.clone(),
            base_end_ref_pos: self.base_end_ref_pos.clone(),
            rows: self.iter_rows.clone(),
            candidates: self.candidates.clone(),
            base_candidates: self.base_candidates.clone(),
            tail_candidates: self.tail_candidates.clone(),
            tail_diag_rows: self.tail_diag_rows.clone(),
            tail_diag_positions: self.tail_diag_positions.clone(),
            tail_diag_snapshots: self.tail_diag_snapshots.clone(),
            conflict_component_count: self.conflict_component_count,
            largest_component_size: self.largest_component_size,
            overlap_count: self.overlap_count,
            overlap_total_area: self.overlap_total_area,
            inflated_component_ids: self.inflated_component_ids.clone(),
            inflation_object_count: self.inflation_object_count,
            overlap_halo: self.overlap_halo.clone(),
        }
    }

    fn restore_pass(&mut self, p: PassOutputs) -> PlacementResult {
        self.pos = p.pos;
        self.z = p.z;
        self.model.phi = p.phi;
        self.phi_tilde = p.phi_tilde;
        self.base_end_pos = p.base_end_pos;
        self.tail_end_pos = p.tail_end_pos;
        self.best_composite_pos = p.best_composite_pos;
        self.deoverlap_end_pos = p.deoverlap_end_pos;
        self.deoverlap_summary = p.deoverlap_summary;
        self.deoverlap_moves = p.deoverlap_moves;
        self.deoverlap_components_before = p.deoverlap_components_before;
        self.deoverlap_components_after = p.deoverlap_components_after;
        self.deoverlap_infl_tail_snapshot = p.deoverlap_infl_tail_snapshot;
        self.deoverlap_unresolved_cells = p.deoverlap_unresolved_cells;
        self.deoverlap_crossings = p.deoverlap_crossings;
        self.deoverlap_crossing_cells = p.deoverlap_crossing_cells;
        self.run_summary = p.summary;
        self.selected_label = p.selected_label;
        self.base_selected_label = p.base_selected_label;
        self.tail_selected_label = p.tail_selected_label;
        self.base_end_ref_pos = p.base_end_ref_pos;
        self.iter_rows = p.rows;
        self.candidates = p.candidates;
        self.base_candidates = p.base_candidates;
        self.tail_candidates = p.tail_candidates;
        self.tail_diag_rows = p.tail_diag_rows;
        self.tail_diag_positions = p.tail_diag_positions;
        self.tail_diag_snapshots = p.tail_diag_snapshots;
        self.conflict_component_count = p.conflict_component_count;
        self.largest_component_size = p.largest_component_size;
        self.overlap_count = p.overlap_count;
        self.overlap_total_area = p.overlap_total_area;
        self.inflated_component_ids = p.inflated_component_ids;
        self.inflation_object_count = p.inflation_object_count;
        self.overlap_halo = p.overlap_halo;
        self.model.set_overlap_halo(&self.overlap_halo);
        p.result
    }

    /// Last-resort rescue for a placement in which mirrored devices ended up welded
    /// on top of each other.
    ///
    /// ## The failure
    ///
    /// A mirrored pair is **one** optimizer variable, not two: `build_reduced_map`
    /// binds `centre_b = 2 * axis - centre_a`, so the pair's whole freedom along the
    /// mirror direction is the single reduced variable `z_v = centre_a`, and `z_v` sits
    /// on the axis exactly when the two devices coincide. The initial placement puts
    /// every device at the boundary centre, which for these groups *is* the axis, so
    /// every pair starts fully welded; the overlap penalty cannot separate it there (the
    /// smoothed overlap is at a maximum, so its derivative along the one direction the
    /// pair can move is zero) and the symmetry penalty is perfectly happy either way
    /// (`x_a + x_b - 2*axis` is zero whether the pair is apart or on top of itself). A
    /// pair leaves the axis only on whatever asymmetry its own nets supply.
    ///
    /// Every committed case comes out of the Base phase welded; the tail and the LCR
    /// retry loop's halo inflation are what normally break the pairs apart over the
    /// following rounds. What this rescue catches is the case where they do not: the
    /// retry loop exhausts and the residual is a mirrored pair overlapping *itself*.
    /// Nothing downstream can fix that — de-overlap treats a pair as one rigid object,
    /// so the single-object component such a pair forms has no other object to move
    /// against and the loop stops without attempting a move, and the retry loop's answer
    /// (inflate the offenders and re-run) is near enough the identity on a pair welded
    /// to itself. The criterion is the pair's own `self_conflict`, not bit-exact
    /// coincidence: a pair 232 DBU apart with 1,848 DBU of overlap is stuck for exactly
    /// the same structural reason as one at zero.
    ///
    /// ## The rescue
    ///
    /// Seed the separation and run the whole thing once more: place every such pair at
    /// `axis -/+ clearance/2` right after the initial placement, so both phases optimize
    /// a layout in which those devices exist as two devices. Repair both arms the same
    /// way — separate in the finished placement, then hand the result back to de-overlap
    /// — and keep whichever ends up with fewer residual overlap pairs.
    ///
    /// ## Why this does not move any existing result
    ///
    /// It runs only after the retry loop has failed *and* the failure is a
    /// self-overlapping mirror pair. Every committed case converges, so none of them
    /// reaches this code and all of them are byte-for-byte what they were before it
    /// existed.
    fn run_phases_with_sym_weld_rescue<FInit, FIter>(
        &mut self,
        mut init_hook: FInit,
        mut iter_hook: FIter,
    ) -> PlaceResult<PlacementResult>
    where
        FInit: FnMut(&mut Placer) -> PlaceResult<()>,
        FIter: FnMut(usize, &mut Placer) -> PlaceResult<()>,
    {
        self.sym_weld_seed.clear();
        self.sym_weld_rescue_seeded = 0;
        self.sym_weld_rescue_repaired = 0;

        let entry = self.snapshot_entry_state();
        let mut result = self.run_phases_with_lcr_retry(&mut init_hook, &mut iter_hook)?;

        let residual = |p: &Placer| -> usize {
            p.deoverlap_summary
                .as_ref()
                .map(|s| s.overlap_pairs_after)
                .unwrap_or(0)
        };
        let first_residual = residual(self);
        if first_residual == 0 {
            return Ok(result);
        }
        let Some(final_pos) = self.deoverlap_end_pos.clone() else {
            return Ok(result);
        };
        let stuck = self.self_overlapping_sym_pairs(&final_pos);
        if stuck.is_empty() {
            // A residual the retry loop could not clear, but not this failure. Leave it
            // exactly as it was, and as honestly reported as it was.
            return Ok(result);
        }

        eprintln!(
            "[sym-weld] the LCR retry loop exhausted with {} residual overlap pair(s), \
             {} of which are mirror pairs overlapping themselves; re-running the whole of \
             multi-phase once with those pairs seeded apart",
            first_residual,
            stuck.len()
        );
        self.sym_weld_rescue_seeded = stuck.len();

        // Attempt 1, repaired: separate the stuck pairs in the finished placement and
        // hand the result back to the de-overlap pass. Pushing a pair apart inside a
        // tight layout can collide it with its neighbours, and resolving collisions
        // between *distinct* objects is exactly what that pass is for -- what it could
        // not do is anything about a pair colliding with itself, which is why we are
        // here.
        let first_repaired = self.repair_self_overlapping_sym_pairs_in_finished_placement();
        let first_residual = residual(self);
        let first_pass = self.capture_pass(&result);

        // Attempt 2: run the whole of multi-phase again with those pairs placed apart from
        // the initial placement, so both phases optimize a layout in which the devices
        // exist as two devices rather than one point.
        self.sym_weld_seed = stuck;
        self.restore_entry_state(&entry);
        result = self.run_phases_with_lcr_retry(&mut init_hook, &mut iter_hook)?;
        let second_repaired = self.repair_self_overlapping_sym_pairs_in_finished_placement();
        let second_residual = residual(self);

        // Both arms have had the same repair applied, so the comparison is like for like:
        // residual overlap decides it, and on a tie the arm that needed no patching
        // wins, because a layout the optimizer produced end to end is worth more than
        // the same count reached by pushing devices apart afterwards. A full tie keeps
        // attempt 1 -- the seeded re-run has to earn the change.
        if (second_residual, second_repaired) < (first_residual, first_repaired) {
            self.sym_weld_rescue_repaired = second_repaired;
            eprintln!(
                "[sym-weld] keeping the seeded re-run: {second_residual} residual overlap \
                 pair(s) against {first_residual}"
            );
        } else {
            self.sym_weld_rescue_repaired = first_repaired;
            eprintln!(
                "[sym-weld] keeping the first attempt: {first_residual} residual overlap \
                 pair(s) against the seeded re-run's {second_residual}"
            );
            result = self.restore_pass(first_pass);
        }
        Ok(result)
    }

    /// Separate any mirror pair that overlaps itself in the finished placement, then
    /// re-run the de-overlap pass over the result. Returns how many pairs moved; `0`
    /// leaves the placement untouched.
    fn repair_self_overlapping_sym_pairs_in_finished_placement(&mut self) -> usize {
        let Some(pos) = self.deoverlap_end_pos.clone() else {
            return 0;
        };
        if self.self_overlapping_sym_pairs(&pos).is_empty() {
            return 0;
        }
        self.pos = pos;
        let repaired = self
            .separate_self_overlapping_sym_pairs("finished placement")
            .len();
        self.discrete_deoverlap_after_tail();
        repaired
    }

    /// Run the multi-phase controller; if the discrete de-overlap pass (the public LCR stage) could
    /// not resolve every conflict with local moves, inflate the halo of exactly the
    /// still-conflicting devices and run the whole pass again.
    ///
    /// The maintainer's spec: *when local conflict resolution cannot resolve a
    /// conflict without crossing over another device, re-run the stage before
    /// legalization with the conflicting devices' inflation increased (×1.1) and try
    /// once more; growth is one-directional so it converges.*
    ///
    /// That sentence carries **two** triggers, and both are implemented here
    /// (the second one is off by default):
    ///
    /// 1. **Unresolved conflict.** The de-overlap search is local (a bounded
    ///    fine→coarse ring around the object, `search_object_best_feasible`);
    ///    when every candidate ring position is blocked, the object is blacklisted and
    ///    the conflict survives.
    /// 2. **Resolved, but only by flying over a neighbour.** An accepted move that
    ///    inverted the mover's relative position against a *local* neighbour is the
    ///    same verdict — the local space was too tight to settle the conflict without
    ///    destroying the arrangement around it. Gated by
    ///    `lcrRetryOnCrossing` (default on); with it off this function is
    ///    bit-for-bit what it was before the trigger existed.
    ///
    /// Either way the remedy is the same: the stage before de-overlap — the Tail / RGE
    /// gradient phase — is what actually creates the spacing, and its per-device extent
    /// is the halo. Enlarging the halo of the offenders makes the tail push them apart
    /// itself, which is strictly monotone: a larger halo can only increase the
    /// separation the tail settles at. Offenders for trigger 2 are the devices that
    /// *did* the crossing.
    ///
    /// Fires on failure only. Round 0 does not touch `overlap_halo` at all, so a case
    /// whose LCR converges with no crossing is byte-identical.
    fn run_phases_with_lcr_retry<FInit, FIter>(
        &mut self,
        mut init_hook: FInit,
        mut iter_hook: FIter,
    ) -> PlaceResult<PlacementResult>
    where
        FInit: FnMut(&mut Placer) -> PlaceResult<()>,
        FIter: FnMut(usize, &mut Placer) -> PlaceResult<()>,
    {
        let enabled = self.model.params.lcr_retry_enable
            && self.model.params.lcr_retry_max_rounds > 0
            && self.model.params.tail_deoverlap_enable;
        let max_rounds = self.model.params.lcr_retry_max_rounds;
        let step = self.model.params.lcr_retry_infl_step;
        let on_crossing = self.model.params.lcr_retry_on_crossing;

        self.lcr_retry_round = 0;
        self.lcr_retry_infl.iter_mut().for_each(|v| *v = 1.0);
        self.lcr_retry_summary = None;

        // Round 0 is the ordinary pass. Only snapshot when a retry is even possible,
        // so the common path pays nothing.
        let entry = if enabled {
            Some(self.snapshot_entry_state())
        } else {
            None
        };

        let mut result = self.run_phases_with_hooks(&mut init_hook, &mut iter_hook)?;
        if !enabled {
            return Ok(result);
        }
        let entry = entry.expect("snapshot taken when retry is enabled");

        let residual = |p: &Placer| -> (usize, usize) {
            p.deoverlap_summary
                .as_ref()
                .map(|s| (s.overlap_pairs_after, s.unresolved_components))
                .unwrap_or((0, 0))
        };
        // Second trigger. Detection always runs (it is bookkeeping in the de-overlap
        // pass); the *count that steers this loop* is zero unless the trigger is armed,
        // so `lcrRetryOnCrossing=false` restores the previous control flow exactly.
        let crossings = |p: &Placer| -> usize {
            if on_crossing {
                p.deoverlap_crossings.len()
            } else {
                0
            }
        };
        let trigger_label = |pairs: usize, cross: usize| -> String {
            match (pairs > 0, cross > 0) {
                (true, true) => "overlap+crossing".to_string(),
                (true, false) => "overlap".to_string(),
                (false, true) => "crossing".to_string(),
                (false, false) => "none".to_string(),
            }
        };

        let (pairs0, comps0) = residual(self);
        let cross0 = crossings(self);
        if pairs0 == 0 && cross0 == 0 {
            // The overwhelmingly common case: LCR converged without crossing anything,
            // nothing to do, and the summary is left as `None` so no new artifact
            // appears.
            return Ok(result);
        }

        let mut summary = LcrRetrySummary {
            enabled: true,
            max_rounds,
            infl_step: step,
            crossing_trigger_enabled: on_crossing,
            crossing_cover_frac: if on_crossing {
                self.model.params.lcr_retry_cross_cover_frac
            } else {
                0.0
            },
            crossing_criterion: if on_crossing {
                self.model.params.lcr_retry_cross_criterion.clone()
            } else {
                String::new()
            },
            ..LcrRetrySummary::default()
        };
        summary.overlap_pairs_after_by_round.push(pairs0);
        summary.unresolved_components_by_round.push(comps0);
        summary.inflated_cells_by_round.push(0);
        let record_crossings = |summary: &mut LcrRetrySummary, p: &Placer, round: usize| {
            if !on_crossing {
                return;
            }
            summary
                .crossing_events_by_round
                .push(p.deoverlap_crossings.len());
            summary
                .crossing_cells_by_round
                .push(p.deoverlap_crossing_cells.len());
            for r in &p.deoverlap_crossings {
                if summary.crossings.len() >= LCR_CROSSING_RECORD_CAP {
                    summary.crossings_truncated = true;
                    break;
                }
                summary.crossings.push(LcrCrossingRecord {
                    round,
                    component_id: r.component_id,
                    object_id: r.object_id,
                    object_kind: r.object_kind.clone(),
                    moved_cell: r.moved_cell,
                    crossed_cell: r.crossed_cell,
                    cover_frac: r.cover_frac,
                    order_reversed_x: r.order_reversed_x,
                    order_reversed_y: r.order_reversed_y,
                    swept_covers_centre: r.swept_covers_centre,
                });
            }
        };
        record_crossings(&mut summary, self, 0);
        if on_crossing {
            summary.trigger_by_round.push(trigger_label(pairs0, cross0));
        }

        // The tail re-solves the whole placement each round, so a later round can trade
        // one conflict for another and land worse than an earlier one. Keep the best
        // pass and hand that back; the loop is then monotone in what it returns, not
        // just in how much halo it has handed out.
        let mut best_pairs = pairs0;
        let mut best_cross = cross0;
        let mut best_round = 0usize;
        let mut best = Some(self.capture_pass(&result));

        for round in 1..=max_rounds {
            // Offenders = the devices LCR could not separate, plus the devices that got
            // out only by moving to the other side of a local neighbour. Sorted and
            // deduplicated, so the inflation order never depends on iteration order.
            let mut offenders = self.deoverlap_unresolved_cells.clone();
            if on_crossing {
                offenders.extend_from_slice(&self.deoverlap_crossing_cells);
                offenders.sort_unstable();
                offenders.dedup();
            }
            if offenders.is_empty() {
                // Conflicts (or crossings) exist but none maps to a movable cell —
                // inflating nothing would spin the loop for no reason.
                log::warn!(
                    "[lcr-retry] round {round}: {} overlap pairs remain but no cell was \
                     attributable; stopping",
                    summary
                        .overlap_pairs_after_by_round
                        .last()
                        .copied()
                        .unwrap_or(0)
                );
                break;
            }
            for &c in &offenders {
                if let Some(v) = self.lcr_retry_infl.get_mut(c) {
                    *v *= step;
                }
            }
            summary.inflated_cells_by_round.push(offenders.len());
            eprintln!(
                "[lcr-retry] round {round}/{max_rounds}: LCR left {} overlap pairs / {} \
                 components and {} crossing event(s); inflating {} device(s) by x{step} \
                 and re-running the tail",
                summary
                    .overlap_pairs_after_by_round
                    .last()
                    .copied()
                    .unwrap_or(0),
                summary
                    .unresolved_components_by_round
                    .last()
                    .copied()
                    .unwrap_or(0),
                summary
                    .crossing_events_by_round
                    .last()
                    .copied()
                    .unwrap_or(0),
                offenders.len(),
            );

            self.lcr_retry_round = round;
            self.restore_entry_state(&entry);
            result = self.run_phases_with_hooks(&mut init_hook, &mut iter_hook)?;
            summary.rounds_run = round;

            let (pairs, comps) = residual(self);
            let cross = crossings(self);
            summary.overlap_pairs_after_by_round.push(pairs);
            summary.unresolved_components_by_round.push(comps);
            record_crossings(&mut summary, self, round);
            if on_crossing {
                summary.trigger_by_round.push(trigger_label(pairs, cross));
            }
            // Residual overlap outranks crossings: a pass that still leaves devices on
            // top of each other is worse than one that merely had to squeeze past.
            if (pairs, cross) < (best_pairs, best_cross) {
                best_pairs = pairs;
                best_cross = cross;
                best_round = round;
                best = Some(self.capture_pass(&result));
            }
            if pairs == 0 && cross == 0 {
                summary.converged = true;
                break;
            }
        }

        if best_round != summary.rounds_run {
            if let Some(b) = best.take() {
                eprintln!(
                    "[lcr-retry] last round left {} overlap pair(s); reverting to round {}                      ({} pair(s))",
                    summary.overlap_pairs_after_by_round.last().copied().unwrap_or(0),
                    best_round,
                    best_pairs
                );
                result = self.restore_pass(b);
            }
        }
        summary.accepted_round = best_round;
        summary.best_overlap_pairs_after = best_pairs;
        summary.best_crossing_events = best_cross;
        summary.converged = best_pairs == 0 && best_cross == 0;
        summary.exhausted = !summary.converged;
        summary.inflated_cells = self
            .lcr_retry_infl
            .iter()
            .enumerate()
            .filter(|(_, v)| **v != 1.0)
            .map(|(i, v)| (i, *v))
            .collect();
        if summary.exhausted {
            log::warn!(
                "[lcr-retry] exhausted {max_rounds} round(s); {} overlap pair(s) remain \
                 unresolved",
                summary
                    .overlap_pairs_after_by_round
                    .last()
                    .copied()
                    .unwrap_or(0)
            );
        }
        eprintln!(
            "[lcr-retry] summary rounds_run={} converged={} pairs_by_round={:?} \
             crossings_by_round={:?} inflated_cells={}",
            summary.rounds_run,
            summary.converged,
            summary.overlap_pairs_after_by_round,
            summary.crossing_events_by_round,
            summary.inflated_cells.len()
        );
        self.lcr_retry_summary = Some(summary);
        Ok(result)
    }

    fn run_phases_with_hooks<FInit, FIter>(
        &mut self,
        mut init_hook: FInit,
        mut iter_hook: FIter,
    ) -> PlaceResult<PlacementResult>
    where
        FInit: FnMut(&mut Placer) -> PlaceResult<()>,
        FIter: FnMut(usize, &mut Placer) -> PlaceResult<()>,
    {
        self.iter_rows.clear();
        self.candidates.clear();
        self.base_candidates.clear();
        self.tail_candidates.clear();
        self.run_summary = None;
        self.selected_label = None;
        self.base_selected_label = None;
        self.tail_selected_label = None;
        self.base_end_ref_pos = None;
        self.base_end_pos = None;
        self.tail_end_pos = None;
        self.best_composite_pos = None;
        self.deoverlap_end_pos = None;
        self.deoverlap_summary = None;
        self.deoverlap_moves.clear();
        self.deoverlap_crossings.clear();
        self.deoverlap_crossing_cells.clear();
        self.deoverlap_components_before.clear();
        self.deoverlap_components_after.clear();
        self.deoverlap_infl_tail_snapshot.clear();
        self.sym_weld_welded_at_tail_entry = 0;
        self.sym_weld_welded_at_lcr_entry = 0;
        self.set_overlap_geometry(0.0, 0.0, 0.0);

        if !self.has_external_init {
            self.init_positions();
        }
        if let Some(map) = &self.reduced_map {
            self.z = map.reduce_pos(&self.pos);
        } else {
            self.z = self.pos.clone();
        }
        // Empty unless an earlier pass came out of Base with mirror pairs still welded
        // onto their axis; then this is what makes the re-run worth running.
        self.apply_sym_weld_seed();
        self.optimizer.reset();
        if let Some(phi_opt) = self.phi_optimizer.as_mut() {
            phi_opt.reset();
        }
        self.sync_tilde_from_phi();
        self.sync_phi_from_tilde();

        let total_iters = self.model.params.max_iterations
            + if self.model.params.enable_gp_tail {
                self.model.params.tail_iters
            } else {
                0
            };
        let t_tail = if self.model.params.enable_gp_tail {
            self.model.params.max_iterations
        } else {
            total_iters
        };
        let candidate_start_base = t_tail.min(((t_tail as f64) * 0.7).ceil() as usize);
        let candidate_stride_base = self.model.params.candidate_stride.max(1);
        let candidate_stride_tail = self.model.params.tail_candidate_stride.max(1);
        let tail_first_stable_alpha = 0.5f64;
        let tail_collect_start = t_tail
            + std::cmp::max(
                10usize,
                (0.5 * self.model.params.tail_infl_ramp_iters as f64).ceil() as usize,
            );
        let eps = 1e-12;
        let total_area = self.model.data.cell_areas_keepout.sum().max(1.0);
        let s_ref = self.compute_symmetry_ref();
        let sym_denom = (s_ref + eps).max(eps);
        let delta_0 = self.tail_delta0();
        let beta = self.model.params.tail_ref_dilate_beta;
        let tau_cover = self.model.params.tail_cover_tau;
        let s_cover = self.model.params.tail_cover_softness;
        let phi_hard_frozen = true;

        let mut init_grad = Array1::<f64>::zeros(self.pos.len());
        let init_loss = self.model.forward(&self.pos, &mut init_grad)?;
        self.loss_history.clear();
        self.loss_history.push(init_loss);
        self.best_loss = init_loss;
        self.best_pos = self.pos.clone();
        init_hook(self)?;

        let ckpt_base_z = self.z.clone();
        let ckpt_base_phi = self.phi_tilde.clone();
        let mut ckpt_tail_z: Option<Array1<f64>> = None;
        let mut ckpt_tail_pos: Option<Array1<f64>> = None;
        let mut ap_mix_tail_ckpt: Option<f64> = None;
        let mut lambda_cover_tail_fixed = 0.0f64;

        let b0 = self.model.loss_breakdown();
        let q_o0_real = b0.overlap_raw / total_area;
        let q_s0 = b0.symmetry_raw / sym_denom;
        let mut qbar_o_base = q_o0_real;
        // Overwritten before it is read; the initializer is here so the
        // declaration reads next to the value it shadows. Upstream item.
        #[allow(unused_assignments)]
        let mut qbar_o_base_prev = qbar_o_base;
        let mut qbar_o_tail: Option<f64> = None;
        let mut qbar_o_tail_prev: Option<f64> = None;
        let mut tau_o_tail: Option<f64> = None;
        let mut qbar_s = q_s0;
        // Overwritten before it is read; the initializer is here so the
        // declaration reads next to the value it shadows. Upstream item.
        #[allow(unused_assignments)]
        let mut qbar_s_prev = qbar_s;

        let tau_o_base = self.model.params.tau_o_ratio * q_o0_real;
        let tau_s = self.model.params.tau_s_ratio * q_s0;
        let tau_gate_hi = self.model.params.tau_gate_hi_ratio * q_o0_real;
        let tau_gate_lo = self.model.params.tau_gate_lo_ratio * q_o0_real;

        self.model.lambda_ap = 0.0;
        let (g_wl_0, g_ovl_0, g_sym_0) = self.compute_raw_grad_norms()?;
        self.model.lambdas.overlap = (g_wl_0 / (g_ovl_0 + eps)).clamp(
            self.model.params.lambda_ovl_min,
            self.model.params.lambda_ovl_max,
        );
        self.model.lambdas.symmetry = (g_wl_0 / (g_sym_0 + eps)).clamp(
            self.model.params.lambda_sym_min,
            self.model.params.lambda_sym_max,
        );

        let g_anchor_base = 0.0;
        let lambda_anchor_base_target =
            (self.model.params.anchor_base_ratio * g_wl_0 / (g_anchor_base + eps)).clamp(
                self.model.params.anchor_min,
                self.model.params.anchor_max_base,
            );
        let mut lambda_anchor = lambda_anchor_base_target;
        let mut lambda_anchor_tail_target = lambda_anchor_base_target;
        let mut anchor_ramp_start = lambda_anchor_base_target;
        let mut anchor_ramp_left: usize = 0;

        let mut mobility_triggered = false;
        let mut mobility_trigger_iter: Option<usize> = None;
        let mut ap_activation_iter: Option<usize> = None;
        let mut entered_tail = false;

        let mut base_idx_last: Option<usize> = None;
        let mut base_idx_min_q_o_real: Option<usize> = None;
        let mut base_idx_min_conflict: Option<usize> = None;
        let mut base_idx_best_composite: Option<usize> = None;

        let mut tail_idx_last: Option<usize> = None;
        let mut tail_idx_first_stable: Option<usize> = None;
        let mut tail_idx_min_q_o_infl: Option<usize> = None;
        let mut tail_idx_min_conflict_components_infl: Option<usize> = None;
        let mut tail_idx_min_cover_pairs_real: Option<usize> = None;
        let mut tail_idx_best_composite: Option<usize> = None;
        let mut last_base_snapshot: Option<Candidate> = None;
        let mut last_tail_snapshot: Option<Candidate> = None;
        let mut converged = false;

        for iter in 0..total_iters {
            let stage = if iter < t_tail {
                Phase::Base
            } else {
                Phase::Tail
            };

            if stage == Phase::Tail && !entered_tail {
                entered_tail = true;
                // Note for anyone tempted to repair self-overlapping mirror pairs
                // here: do not. At the end of the Base phase *every* committed case has
                // all of its mirror pairs welded onto the axis — Base ends on that point
                // routinely, and the tail and the LCR retry loop are what pull them off
                // it. The welding that matters is the one still standing once that loop
                // has exhausted (`run_phases_with_sym_weld_rescue`).
                self.sym_weld_welded_at_tail_entry =
                    self.self_overlapping_sym_pairs(&self.pos).len();
                ckpt_tail_z = Some(self.z.clone());
                ckpt_tail_pos = Some(self.pos.clone());
                ap_mix_tail_ckpt = Some(self.model.lambda_ap);
                self.base_end_ref_pos = Some(self.pos.clone());
                self.checkpoint_pos = Some(self.pos.clone());
                let g_anchor_tail = 0.0;
                lambda_anchor_tail_target =
                    (self.model.params.anchor_tail_ratio * g_wl_0 / (g_anchor_tail + eps)).clamp(
                        self.model.params.anchor_min,
                        self.model.params.anchor_max_tail,
                    );
                anchor_ramp_start = lambda_anchor;
                anchor_ramp_left = self.model.params.anchor_ramp_iters;
                let q_o_tail_entry =
                    self.compute_overlap_raw_with_halo(Some(&self.overlap_halo)) / total_area;
                qbar_o_tail = Some(q_o_tail_entry);
                qbar_o_tail_prev = Some(q_o_tail_entry);
                tau_o_tail = Some(self.model.params.tail_tau_o_ratio * q_o_tail_entry);
                lambda_cover_tail_fixed =
                    self.model.params.tail_cover_lambda_ratio * self.model.lambdas.overlap;
            }

            if stage == Phase::Base {
                lambda_anchor = lambda_anchor_base_target;
            } else if anchor_ramp_left > 0 {
                let done = self.model.params.anchor_ramp_iters - anchor_ramp_left + 1;
                let t = (done as f64) / (self.model.params.anchor_ramp_iters as f64);
                lambda_anchor =
                    anchor_ramp_start + t * (lambda_anchor_tail_target - anchor_ramp_start);
                anchor_ramp_left -= 1;
            } else {
                lambda_anchor = lambda_anchor_tail_target;
            }

            if stage == Phase::Base {
                let g = if tau_gate_hi > tau_gate_lo {
                    ((tau_gate_hi - qbar_o_base) / (tau_gate_hi - tau_gate_lo)).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let old_lambda_ap = self.model.lambda_ap;
                self.model.lambda_ap = (self.model.lambda_ap + self.model.params.eta_ap * g)
                    .min(self.model.params.lambda_ap_max)
                    .clamp(0.0, 1.0);
                if ap_activation_iter.is_none() && self.model.lambda_ap > old_lambda_ap + 1e-12 {
                    ap_activation_iter = Some(iter);
                }
            } else if let Some(ap_ckpt) = ap_mix_tail_ckpt {
                self.model.lambda_ap = ap_ckpt;
            }

            self.sync_phi_from_tilde();
            let lambda_cover_tail_iter = if stage == Phase::Tail {
                lambda_cover_tail_fixed
            } else {
                0.0
            };
            let alpha_infl = if stage == Phase::Tail {
                self.tail_alpha_infl(iter, t_tail)
            } else {
                0.0
            };
            // DRC-aware inflation floor only applies in the tail (legalization) stage;
            // base global placement keeps a zero halo.
            let overlap_floor = if stage == Phase::Tail {
                self.model.params.tail_hard_inflation_floor
            } else {
                0.0
            };
            self.set_overlap_geometry(alpha_infl, delta_0, overlap_floor);
            let (ref_pos, ref_scale_s) = if stage == Phase::Tail {
                self.dilated_reference_positions(alpha_infl)
            } else {
                (self.expand_from_z(&ckpt_base_z), 1.0)
            };
            let ref_z = if let Some(map) = &self.reduced_map {
                map.reduce_pos(&ref_pos)
            } else {
                ref_pos.clone()
            };

            let old_pos = self.pos.clone();
            let old_z = self.z.clone();
            let old_phi_tilde = self.phi_tilde.clone();

            let mut pre_grad = Array1::<f64>::zeros(self.pos.len());
            let _ = self.model.forward(&self.pos, &mut pre_grad)?;
            self.grad = pre_grad;
            let pre_breakdown = self.model.loss_breakdown();
            let q_o_pre_real = self.compute_overlap_raw_with_halo(None) / total_area;
            let q_s_pre = pre_breakdown.symmetry_raw / sym_denom;
            let mut grad_norm_cover = 0.0;
            if stage == Phase::Tail {
                let (_, _, cover_grad_raw) =
                    self.compute_cover_metrics_and_grad_real(tau_cover, s_cover);
                grad_norm_cover =
                    cover_grad_raw.dot(&cover_grad_raw).sqrt() * lambda_cover_tail_iter.abs();
                for i in 0..self.grad.len() {
                    self.grad[i] += lambda_cover_tail_iter * cover_grad_raw[i];
                }
            }

            if !mobility_triggered
                && q_o_pre_real <= self.model.params.mobility_trigger_ratio * q_o0_real
            {
                mobility_triggered = true;
                mobility_trigger_iter = Some(iter);
            }
            let gamma_mob = if !mobility_triggered {
                0.0
            } else {
                let t0 = mobility_trigger_iter.unwrap_or(iter);
                ((iter.saturating_sub(t0)) as f64 / self.model.params.mobility_ramp_iters as f64)
                    .clamp(0.0, 1.0)
            };
            self.apply_phase_mobility_weighting(stage, gamma_mob);

            let mut grad_z = if let Some(map) = &self.reduced_map {
                map.reduce_grad(&self.grad)
            } else {
                self.grad.clone()
            };

            let ckpt_z_ref: &Array1<f64> = if stage == Phase::Base {
                &ckpt_base_z
            } else {
                ckpt_tail_z.as_ref().unwrap_or(&ckpt_base_z)
            };

            let mut anchor_loss = 0.0;
            for i in 0..grad_z.len() {
                let target = if stage == Phase::Base {
                    ckpt_base_z[i]
                } else {
                    ref_z[i]
                };
                let d = self.z[i] - target;
                grad_z[i] += lambda_anchor * d;
                anchor_loss += 0.5 * lambda_anchor * d * d;
            }

            let mut phi_grad_tilde = vec![0.0; self.phi_tilde.len()];
            if self.model.params.use_ap_wl && stage == Phase::Base {
                let pi = std::f64::consts::PI;
                let raw_phi_grad = self.model.phi_grad.borrow();
                for i in 0..self.phi_tilde.len() {
                    let mut gphi = raw_phi_grad.get(i).copied().unwrap_or(0.0) * pi;
                    let d = self.phi_tilde[i] - ckpt_base_phi.get(i).copied().unwrap_or(0.0);
                    gphi += lambda_anchor * d;
                    phi_grad_tilde[i] = gphi;
                    anchor_loss += 0.5 * lambda_anchor * d * d;
                }
            }

            let (grad_norm_wl, grad_norm_ovl, grad_norm_sym) = self.compute_raw_grad_norms()?;
            let mut grad_norm_total = grad_z.dot(&grad_z).sqrt();
            if !phi_grad_tilde.is_empty() {
                grad_norm_total = (grad_norm_total * grad_norm_total
                    + phi_grad_tilde.iter().map(|v| v * v).sum::<f64>())
                .sqrt();
            }

            // Overwritten before it is read on every path. Upstream item.
            #[allow(unused_assignments)]
            let mut proposed_z = self.z.clone();
            let line_search =
                if self.reduced_map.is_none() && self.model.params.step_strategy == "linesearch" {
                    self.line_search.as_ref()
                } else {
                    None
                };
            if let Some(ls) = line_search {
                let direction = compute_descent_direction(&grad_z);
                let obj_fn = |pos: &Array1<f64>| {
                    let mut dummy = Array1::zeros(pos.len());
                    self.model.forward(pos, &mut dummy).unwrap_or(f64::INFINITY)
                };
                let loss_pre = self
                    .model
                    .forward(&self.pos, &mut Array1::<f64>::zeros(self.pos.len()))?;
                let alpha = ls.search(
                    obj_fn,
                    &self.pos,
                    &direction,
                    &grad_z,
                    loss_pre + anchor_loss,
                );
                proposed_z = &self.z + &(&direction * alpha);
            } else {
                proposed_z = self.optimizer.step(&self.z, &grad_z);
            }
            let delta_raw_z: Vec<f64> = proposed_z
                .iter()
                .zip(self.z.iter())
                .map(|(a, b)| a - b)
                .collect();

            let c_delta = if stage == Phase::Base {
                self.model.params.c_delta_base
            } else {
                self.model.params.c_delta_tail
            };
            let semantic_scales = self.compute_semantic_scales(false);
            let delta_clip_z = Self::clip_delta(&delta_raw_z, &semantic_scales, c_delta);
            let clip_ratio_xy = Self::clip_hit_ratio(&delta_raw_z, &delta_clip_z);
            for i in 0..self.z.len() {
                self.z[i] += delta_clip_z[i];
            }
            self.pos = self.expand_from_z(&self.z);

            let mut phi_step_rms = 0.0;
            let mut phi_step_max: f64 = 0.0;
            let mut clip_ratio_phi = 0.0;
            let mut phi_clip_len = 0usize;
            if self.model.params.use_ap_wl {
                if stage == Phase::Base {
                    if let Some(phi_opt) = self.phi_optimizer.as_mut() {
                        let phi_vec = Array1::from_vec(self.phi_tilde.clone());
                        let phi_grad_vec = Array1::from_vec(phi_grad_tilde.clone());
                        let proposed_phi = phi_opt.step(&phi_vec, &phi_grad_vec);
                        let delta_raw_phi: Vec<f64> = proposed_phi
                            .iter()
                            .zip(self.phi_tilde.iter())
                            .map(|(a, b)| a - b)
                            .collect();
                        let delta_clip_phi = Self::clip_delta(
                            &delta_raw_phi,
                            &vec![1.0; self.phi_tilde.len()],
                            c_delta,
                        );
                        clip_ratio_phi = Self::clip_hit_ratio(&delta_raw_phi, &delta_clip_phi);
                        phi_clip_len = delta_clip_phi.len();
                        let mut sq = 0.0;
                        for (i, d) in delta_clip_phi.iter().enumerate() {
                            self.phi_tilde[i] = Self::normalize_phi_tilde(self.phi_tilde[i] + d);
                            sq += d * d;
                            phi_step_max = phi_step_max.max(d.abs());
                        }
                        if !delta_clip_phi.is_empty() {
                            phi_step_rms = (sq / delta_clip_phi.len() as f64).sqrt();
                        }
                    }
                } else {
                    // Tail-v2 hard freeze: do not update phi.
                    phi_step_rms = 0.0;
                    phi_step_max = 0.0;
                    clip_ratio_phi = 0.0;
                    phi_clip_len = 0;
                }
            }
            self.sync_phi_from_tilde();

            let mut post_grad = Array1::<f64>::zeros(self.pos.len());
            let mut loss_total = self.model.forward(&self.pos, &mut post_grad)?;
            self.grad = post_grad;
            let b = self.model.loss_breakdown();
            let wl_base = b.wl_base_raw;
            let wl_ap = b.wl_ap_raw;
            let ap_mix = self.model.lambda_ap;
            let wl_mix_real = (1.0 - ap_mix) * wl_base + ap_mix * wl_ap;
            let ovl_raw_real = self.compute_overlap_raw_with_halo(None);
            let ovl_raw_infl = self.compute_overlap_raw_with_halo(Some(&self.overlap_halo));
            let q_o_real = ovl_raw_real / total_area;
            let q_o_infl = ovl_raw_infl / total_area;
            let q_s = b.symmetry_raw / sym_denom;
            let (cover_real, cover_loss_raw, _) =
                self.compute_cover_metrics_and_grad_real(tau_cover, s_cover);
            let cover_loss = lambda_cover_tail_iter * cover_loss_raw;
            loss_total += anchor_loss + cover_loss;

            let mut move_sq_sum = 0.0;
            let mut move_max: f64 = 0.0;
            let n = self.model.data.num_cells;
            for i in 0..n {
                let dx = self.pos[i] - old_pos[i];
                let dy = self.pos[i + n] - old_pos[i + n];
                let d = (dx * dx + dy * dy).sqrt();
                move_sq_sum += d * d;
                move_max = move_max.max(d);
            }
            let move_rms = if n > 0 {
                (move_sq_sum / n as f64).sqrt()
            } else {
                0.0
            };
            self.move_history.push((move_rms, move_max));

            let ckpt_pos = self.expand_from_z(ckpt_z_ref);
            let (disp_ckpt_rms, disp_ckpt_max) = self.displacement_to_checkpoint(&ckpt_pos);
            let (disp_tail_ckpt_rms, disp_tail_ckpt_max) = if let Some(p) = &ckpt_tail_pos {
                self.displacement_to_checkpoint(p)
            } else {
                (0.0, 0.0)
            };
            let (disp_to_ref_rms, disp_to_ref_max) = self.displacement_to_checkpoint(&ref_pos);
            let conflict_real = self.compute_conflict_metrics_with_halo(None);
            let conflict_infl = self.compute_conflict_metrics_with_halo(Some(&self.overlap_halo));
            let z_step_norm = delta_clip_z.iter().map(|v| v * v).sum::<f64>().sqrt();
            let phi_step_norm = self
                .phi_tilde
                .iter()
                .zip(old_phi_tilde.iter())
                .map(|(a, b)| (a - b) * (a - b))
                .sum::<f64>()
                .sqrt();
            let step_norm = (z_step_norm * z_step_norm + phi_step_norm * phi_step_norm).sqrt();

            qbar_s_prev = qbar_s;
            qbar_s = self.model.params.rho * q_s + (1.0 - self.model.params.rho) * qbar_s;

            let h_o = if stage == Phase::Base {
                qbar_o_base_prev = qbar_o_base;
                qbar_o_base =
                    self.model.params.rho * q_o_real + (1.0 - self.model.params.rho) * qbar_o_base;
                self.model.params.a_o * (qbar_o_base / (tau_o_base + eps) - 1.0)
                    + self.model.params.b_o * ((qbar_o_base + eps) / (qbar_o_base_prev + eps)).ln()
            } else {
                let prev = qbar_o_tail_prev.unwrap_or(q_o_infl);
                let curr = qbar_o_tail.unwrap_or(q_o_infl);
                qbar_o_tail_prev = Some(curr);
                let updated =
                    self.model.params.rho * q_o_infl + (1.0 - self.model.params.rho) * curr;
                qbar_o_tail = Some(updated);
                let tau_tail = tau_o_tail.unwrap_or(self.model.params.tail_tau_o_ratio * q_o_infl);
                self.model.params.a_o * (updated / (tau_tail + eps) - 1.0)
                    + self.model.params.b_o * ((updated + eps) / (prev + eps)).ln()
            };
            let h_s = self.model.params.a_s * (qbar_s / (tau_s + eps) - 1.0)
                + self.model.params.b_s * ((qbar_s + eps) / (qbar_s_prev + eps)).ln();
            let norm_h = (h_o * h_o + h_s * h_s).sqrt() + eps;
            let d_o = h_o / norm_h;
            let d_s = h_s / norm_h;
            let lambda_ovl_before = self.model.lambdas.overlap;
            let lambda_sym_before = self.model.lambdas.symmetry;
            let s_lambda = if stage == Phase::Base {
                self.model.params.s_lambda_base
            } else {
                self.model.params.s_lambda_tail
            };
            self.model.lambdas.overlap = (self.model.lambdas.overlap.ln() + s_lambda * d_o)
                .clamp(
                    self.model.params.lambda_ovl_min.ln(),
                    self.model.params.lambda_ovl_max.ln(),
                )
                .exp();
            self.model.lambdas.symmetry = (self.model.lambdas.symmetry.ln() + s_lambda * d_s)
                .clamp(
                    self.model.params.lambda_sym_min.ln(),
                    self.model.params.lambda_sym_max.ln(),
                )
                .exp();

            let total_clip_len = delta_clip_z.len() + phi_clip_len;
            let total_clip_hits = (clip_ratio_xy * delta_clip_z.len() as f64)
                + (clip_ratio_phi * phi_clip_len as f64);
            let clip_ratio_all = if total_clip_len > 0 {
                total_clip_hits / total_clip_len as f64
            } else {
                0.0
            };

            self.iter_rows.push(IterRow {
                iter,
                stage: stage.as_str().to_string(),
                loss_total,
                wl_base,
                wl_ap,
                wl_mix_real,
                ovl_raw_real,
                ovl_raw_infl,
                q_o_real,
                q_o_infl,
                ap_mix,
                alpha_infl,
                ref_scale_s,
                disp_to_tail_ckpt_rms: disp_tail_ckpt_rms,
                disp_to_tail_ckpt_max: disp_tail_ckpt_max,
                disp_to_ref_rms,
                disp_to_ref_max,
                num_conflict_components_real: conflict_real.num_components,
                largest_conflict_component_size_real: conflict_real.largest_component_size,
                largest_conflict_component_area_real: conflict_real.largest_component_area,
                num_conflict_components_infl: conflict_infl.num_components,
                largest_conflict_component_area_infl: conflict_infl.largest_component_area,
                num_cover_pairs_real: cover_real.num_pairs,
                max_cover_ratio_real: cover_real.max_ratio,
                mean_cover_ratio_real: cover_real.mean_ratio,
                sym_raw: b.symmetry_raw,
                q_s,
                lambda_ovl: lambda_ovl_before,
                lambda_sym: lambda_sym_before,
                lambda_anchor,
                lambda_cover_tail: lambda_cover_tail_iter,
                grad_norm_total,
                grad_norm_wl,
                grad_norm_ovl,
                grad_norm_sym,
                grad_norm_cover,
                h_o,
                h_s,
                d_o,
                d_s,
                step_norm,
                disp_rms: move_rms,
                disp_max: move_max,
                disp_to_ckpt_rms: disp_ckpt_rms,
                disp_to_ckpt_max: disp_ckpt_max,
                phi_step_rms,
                phi_step_max,
                clip_ratio_all,
                clip_ratio_xy,
                clip_ratio_phi,
                mobility_enabled: mobility_triggered,
            });

            self.loss_history.push(loss_total);
            self.best_loss = loss_total;
            self.best_pos = self.pos.clone();

            let current_candidate = self.make_candidate(
                stage,
                iter,
                q_o_real,
                q_o_infl,
                q_s,
                wl_mix_real,
                ovl_raw_real,
                ovl_raw_infl,
                conflict_real,
                conflict_infl,
                cover_real,
                alpha_infl,
                ref_scale_s,
                disp_to_ref_rms,
                disp_to_ref_max,
                delta_0,
                lambda_anchor,
                lambda_cover_tail_iter,
            );

            if stage == Phase::Base {
                last_base_snapshot = Some(current_candidate.clone());
                if iter >= candidate_start_base && iter % candidate_stride_base == 0 {
                    self.base_candidates.push(current_candidate.clone());
                    let idx = self.base_candidates.len() - 1;
                    base_idx_last = Some(idx);
                    if base_idx_min_q_o_real
                        .map(|j| {
                            self.base_candidates[idx].q_o_real < self.base_candidates[j].q_o_real
                        })
                        .unwrap_or(true)
                    {
                        base_idx_min_q_o_real = Some(idx);
                    }
                    if base_idx_min_conflict
                        .map(|j| {
                            let a = &self.base_candidates[idx];
                            let b = &self.base_candidates[j];
                            (
                                a.conflict_real.num_components,
                                a.conflict_real.largest_component_area,
                            ) < (
                                b.conflict_real.num_components,
                                b.conflict_real.largest_component_area,
                            )
                        })
                        .unwrap_or(true)
                    {
                        base_idx_min_conflict = Some(idx);
                    }
                    if base_idx_best_composite
                        .map(|j| {
                            better_base_composite(
                                &self.base_candidates[idx],
                                &self.base_candidates[j],
                            )
                        })
                        .unwrap_or(true)
                    {
                        base_idx_best_composite = Some(idx);
                    }
                }
            } else {
                last_tail_snapshot = Some(current_candidate.clone());
                let mut collected_idx: Option<usize> = None;
                if iter >= tail_collect_start && iter % candidate_stride_tail == 0 {
                    self.tail_candidates.push(current_candidate.clone());
                    collected_idx = Some(self.tail_candidates.len() - 1);
                }
                if let Some(idx) = collected_idx {
                    tail_idx_last = Some(idx);
                    if tail_idx_first_stable.is_none() && alpha_infl >= tail_first_stable_alpha {
                        tail_idx_first_stable = Some(idx);
                    }
                    if tail_idx_min_q_o_infl
                        .map(|j| {
                            self.tail_candidates[idx].q_o_infl < self.tail_candidates[j].q_o_infl
                        })
                        .unwrap_or(true)
                    {
                        tail_idx_min_q_o_infl = Some(idx);
                    }
                    if tail_idx_min_conflict_components_infl
                        .map(|j| {
                            let a = &self.tail_candidates[idx];
                            let b = &self.tail_candidates[j];
                            a.conflict_infl.num_components < b.conflict_infl.num_components
                        })
                        .unwrap_or(true)
                    {
                        tail_idx_min_conflict_components_infl = Some(idx);
                    }
                    if tail_idx_min_cover_pairs_real
                        .map(|j| {
                            let a = &self.tail_candidates[idx];
                            let b = &self.tail_candidates[j];
                            (a.cover_real.num_pairs, a.cover_real.max_ratio)
                                < (b.cover_real.num_pairs, b.cover_real.max_ratio)
                        })
                        .unwrap_or(true)
                    {
                        tail_idx_min_cover_pairs_real = Some(idx);
                    }
                    if tail_idx_best_composite
                        .map(|j| {
                            better_tail_composite(
                                &self.tail_candidates[idx],
                                &self.tail_candidates[j],
                            )
                        })
                        .unwrap_or(true)
                    {
                        tail_idx_best_composite = Some(idx);
                    }
                }
            }

            iter_hook(iter, self)?;
            if self.check_convergence(iter) {
                converged = true;
                break;
            }
            let _ = q_s_pre; // retained for strict staged visibility/debug symmetry.
            let _ = old_z;
        }

        if self.base_candidates.is_empty() {
            if let Some(c) = last_base_snapshot {
                self.base_candidates.push(c);
                let idx = self.base_candidates.len() - 1;
                base_idx_last = Some(idx);
                base_idx_min_q_o_real = Some(idx);
                base_idx_min_conflict = Some(idx);
                base_idx_best_composite = Some(idx);
            }
        }
        if self.tail_candidates.is_empty() {
            if let Some(c) = last_tail_snapshot {
                self.tail_candidates.push(c.clone());
                let idx = self.tail_candidates.len() - 1;
                tail_idx_last = Some(idx);
                tail_idx_min_q_o_infl = Some(idx);
                tail_idx_min_conflict_components_infl = Some(idx);
                tail_idx_min_cover_pairs_real = Some(idx);
                tail_idx_best_composite = Some(idx);
                if tail_idx_first_stable.is_none() && c.alpha_infl >= tail_first_stable_alpha {
                    tail_idx_first_stable = Some(idx);
                }
            }
        }

        // For de-overlap seed selection, prefer tail min_q_o_infl.
        // If multiple candidates share the same q_o_infl, choose the one with smaller
        // loss_total from the corresponding tail iteration row.
        let tail_idx_min_q_o_infl_loss_tie = if self.tail_candidates.is_empty() {
            None
        } else {
            let tail_loss_for_iter = |it: usize| -> f64 {
                self.iter_rows
                    .iter()
                    .rev()
                    .find(|r| r.stage == "tail" && r.iter == it)
                    .map(|r| r.loss_total)
                    .unwrap_or(f64::INFINITY)
            };
            let mut best_idx: Option<usize> = None;
            for (idx, cand) in self.tail_candidates.iter().enumerate() {
                let loss = tail_loss_for_iter(cand.iter);
                best_idx = match best_idx {
                    None => Some(idx),
                    Some(bi) => {
                        let b = &self.tail_candidates[bi];
                        let b_loss = tail_loss_for_iter(b.iter);
                        let ord = cand
                            .q_o_infl
                            .partial_cmp(&b.q_o_infl)
                            .unwrap_or(Ordering::Equal)
                            .then_with(|| loss.partial_cmp(&b_loss).unwrap_or(Ordering::Equal))
                            .then_with(|| cand.iter.cmp(&b.iter))
                            .then_with(|| idx.cmp(&bi));
                        if ord == Ordering::Less {
                            Some(idx)
                        } else {
                            Some(bi)
                        }
                    }
                };
            }
            best_idx
        };
        if tail_idx_min_q_o_infl_loss_tie.is_some() {
            tail_idx_min_q_o_infl = tail_idx_min_q_o_infl_loss_tie;
        }

        let base_selected_idx = base_idx_min_q_o_real.or(base_idx_last);
        if let Some(i) = base_selected_idx {
            let c = &self.base_candidates[i];
            self.base_end_pos = Some(Array1::from_vec(c.full_positions.clone()));
            self.checkpoint_pos = Some(Array1::from_vec(c.full_positions.clone()));
            self.base_selected_label = Some(if Some(i) == base_idx_min_q_o_real {
                "base_min_q_o_real".to_string()
            } else {
                "base_last".to_string()
            });
        } else {
            self.base_end_pos = Some(self.pos.clone());
            self.checkpoint_pos = Some(self.pos.clone());
            self.base_selected_label = Some("base_last".to_string());
        }

        let tail_selected_idx = tail_idx_min_q_o_infl_loss_tie
            .or(tail_idx_min_q_o_infl)
            .or(tail_idx_last);
        if let Some(i) = tail_selected_idx {
            let c = self.tail_candidates[i].clone();
            self.tail_end_pos = Some(Array1::from_vec(c.full_positions.clone()));
            self.z = Array1::from_vec(c.reduced_vars.clone());
            self.pos = Array1::from_vec(c.full_positions.clone());
            self.phi_tilde = c.phi_tilde.clone();
            self.sync_phi_from_tilde();
            self.model.lambdas.overlap = c.lambdas.lambda_ovl;
            self.model.lambdas.symmetry = c.lambdas.lambda_sym;
            self.model.lambda_ap = c.lambdas.ap_mix;
            self.tail_selected_label = Some(if Some(i) == tail_idx_min_q_o_infl {
                "min_qo_infl".to_string()
            } else if Some(i) == tail_idx_best_composite {
                "best_composite_infl".to_string()
            } else {
                "tail_last".to_string()
            });
            self.selected_label = self.tail_selected_label.clone();
        } else if let Some(i) = base_selected_idx {
            let c = self.base_candidates[i].clone();
            self.tail_end_pos = Some(Array1::from_vec(c.full_positions.clone()));
            self.z = Array1::from_vec(c.reduced_vars.clone());
            self.pos = Array1::from_vec(c.full_positions.clone());
            self.phi_tilde = c.phi_tilde.clone();
            self.sync_phi_from_tilde();
            self.model.lambdas.overlap = c.lambdas.lambda_ovl;
            self.model.lambdas.symmetry = c.lambdas.lambda_sym;
            self.model.lambda_ap = c.lambdas.ap_mix;
            self.selected_label = self.base_selected_label.clone();
        } else {
            self.tail_end_pos = Some(self.pos.clone());
        }

        self.best_composite_pos = Some(self.pos.clone());
        self.discrete_deoverlap_after_tail();
        if let Some(p) = &self.deoverlap_end_pos {
            self.pos = p.clone();
        }
        if let Some(map) = &self.reduced_map {
            self.z = map.reduce_pos(&self.pos);
        } else {
            self.z = self.pos.clone();
        }

        if self.selected_label.is_none() {
            self.selected_label = self
                .tail_selected_label
                .clone()
                .or_else(|| self.base_selected_label.clone())
                .or_else(|| Some("last".to_string()));
        }

        self.candidates = {
            let mut v = Vec::with_capacity(self.base_candidates.len() + self.tail_candidates.len());
            v.extend(self.base_candidates.clone());
            v.extend(self.tail_candidates.clone());
            v
        };

        let mut summary_rows = Vec::new();
        let mut push_summary = |label: &str, c: &Candidate| {
            summary_rows.push(CandidateSummaryEntry {
                label: label.to_string(),
                stage: c.stage.clone(),
                iter: c.iter,
                q_o_real: c.q_o_real,
                q_o_infl: c.q_o_infl,
                q_s: c.q_s,
                wl_mix_real: c.wl_mix_real,
                num_conflict_components_real: c.conflict_real.num_components,
                largest_conflict_component_area_real: c.conflict_real.largest_component_area,
                num_conflict_components_infl: c.conflict_infl.num_components,
                largest_conflict_component_area_infl: c.conflict_infl.largest_component_area,
                num_cover_pairs_real: c.cover_real.num_pairs,
                max_cover_ratio_real: c.cover_real.max_ratio,
                mean_cover_ratio_real: c.cover_real.mean_ratio,
                ref_scale_s: c.ref_scale_s,
                disp_to_ref_rms: c.disp_to_ref_rms,
                alpha_infl: c.alpha_infl,
            });
        };

        if let Some(i) = base_idx_last {
            push_summary("base_last", &self.base_candidates[i]);
        }
        if let Some(i) = base_idx_min_q_o_real {
            push_summary("base_min_q_o_real", &self.base_candidates[i]);
        }
        if let Some(i) = base_idx_min_conflict {
            push_summary("base_min_conflict", &self.base_candidates[i]);
        }
        if let Some(i) = base_idx_best_composite {
            push_summary("base_best_composite", &self.base_candidates[i]);
        }
        if let Some(i) = tail_idx_first_stable {
            push_summary("tail_first_stable", &self.tail_candidates[i]);
        }
        if let Some(i) = tail_idx_min_q_o_infl {
            push_summary("min_qo_infl", &self.tail_candidates[i]);
        }
        if let Some(i) = tail_idx_min_conflict_components_infl {
            push_summary("min_conflict_components_infl", &self.tail_candidates[i]);
        }
        if let Some(i) = tail_idx_min_cover_pairs_real {
            push_summary("min_cover_pairs_real", &self.tail_candidates[i]);
        }
        if let Some(i) = tail_idx_best_composite {
            push_summary("best_composite_infl", &self.tail_candidates[i]);
        }
        if let Some(i) = tail_idx_last {
            push_summary("tail_last", &self.tail_candidates[i]);
        }

        self.run_summary = Some(RunSummary {
            schema_version: crate::gp_controller::OUTPUT_SCHEMA_VERSION,
            initialization_mode: if self.has_external_init {
                "external".to_string()
            } else {
                "random".to_string()
            },
            prior_init_used: false,
            symmetry_binding_used: self.reduced_map.is_some(),
            base_end_iter: base_selected_idx
                .map(|i| self.base_candidates[i].iter)
                .or(Some(t_tail.saturating_sub(1))),
            tail_end_iter: tail_selected_idx
                .map(|i| self.tail_candidates[i].iter)
                .or_else(|| base_selected_idx.map(|i| self.base_candidates[i].iter))
                .or(Some(total_iters.saturating_sub(1))),
            gp_end_iter: base_selected_idx
                .map(|i| self.base_candidates[i].iter)
                .or(Some(t_tail.saturating_sub(1))),
            mobility_activation_iteration: mobility_trigger_iter,
            ap_activation_iteration: ap_activation_iter,
            selected_final_candidate_label: self.selected_label.clone(),
            selected_base_end_label: self.base_selected_label.clone(),
            selected_tail_seed_label: self.tail_selected_label.clone(),
            controller_version: CONTROLLER_VERSION.to_string(),
            tail_mode: TAIL_MODE.to_string(),
            r_infl: self.model.params.tail_infl_ratio,
            delta_0,
            beta,
            n_infl_ramp: self.model.params.tail_infl_ramp_iters,
            tau_cover,
            s_cover,
            lambda_cover_tail: lambda_cover_tail_fixed,
            ap_mix_tail_ckpt: ap_mix_tail_ckpt.unwrap_or(self.model.lambda_ap),
            phi_hard_frozen,
            candidate_summary: summary_rows,
        });

        Ok(PlacementResult {
            converged,
            num_iters: self.loss_history.len().saturating_sub(1),
            final_loss: self.loss_history.last().copied().unwrap_or(f64::INFINITY),
            best_loss: self.best_loss,
            positions: self.pos.clone(),
            loss_history: self.loss_history.clone(),
        })
    }

    fn current_overlap_ratio(&self) -> f64 {
        let mut g = Array1::zeros(self.pos.len());
        let ovl_abs = crate::constraints::compute_overlap_with_inflation_and_halo(
            &self.pos,
            &self.model.data,
            1.0,
            &mut g,
            Some(&self.overlap_inflation),
            Some(&self.overlap_halo),
        )
        .unwrap_or(f64::INFINITY);
        ovl_abs / self.model.data.cell_areas_keepout.sum().max(1.0)
    }

    fn current_symmetry_ratio(&self) -> f64 {
        let b = self.model.loss_breakdown();
        let denom = self
            .model
            .data
            .cell_widths
            .iter()
            .map(|w| w * w)
            .sum::<f64>()
            .max(1.0);
        b.symmetry_raw / denom
    }

    fn current_drift_ratio(&self) -> f64 {
        let Some(ckpt) = &self.checkpoint_pos else {
            return 0.0;
        };
        let n = self.model.data.num_cells;
        if n == 0 {
            return 0.0;
        }
        let mut sum = 0.0;
        for i in 0..n {
            let dx = self.pos[i] - ckpt[i];
            let dy = self.pos[i + n] - ckpt[i + n];
            sum += dx * dx + dy * dy;
        }
        let rms = (sum / n as f64).sqrt();
        let bx = self.model.data.boundary_xh - self.model.data.boundary_xl;
        let by = self.model.data.boundary_yh - self.model.data.boundary_yl;
        let diag = (bx * bx + by * by).sqrt().max(1.0);
        rms / diag
    }

    fn current_conflict_count(&self) -> usize {
        self.conflict_flags.iter().filter(|x| **x).count()
    }

    /// Checks the convergence criterion
    fn check_convergence(&self, iter: usize) -> bool {
        if iter < self.model.params.convergence_window {
            return false;
        }

        // Overlap criterion uses normalized ratio: overlap_area / total_cell_area.
        if self.model.params.enable_overlap && self.model.params.overlap_target > 0.0 {
            let mut g = Array1::zeros(self.pos.len());
            let ovl_abs =
                crate::constraints::compute_overlap(&self.pos, &self.model.data, 1.0, &mut g)
                    .unwrap_or(f64::INFINITY);
            let total_area = self.model.data.cell_areas_keepout.sum().max(1.0);
            let ovl_ratio = ovl_abs / total_area;
            if ovl_ratio > self.model.params.overlap_target {
                return false;
            }
        }

        // Move-based convergence: recent displacement must be small.
        let window = self.model.params.convergence_window;
        if self.move_history.len() < window {
            false
        } else {
            let recent_moves = &self.move_history[self.move_history.len() - window..];
            let max_rms = recent_moves
                .iter()
                .map(|(r, _)| *r)
                .fold(f64::NEG_INFINITY, f64::max);
            let max_move = recent_moves
                .iter()
                .map(|(_, m)| *m)
                .fold(f64::NEG_INFINITY, f64::max);
            let dx = self.model.data.boundary_xh - self.model.data.boundary_xl;
            let dy = self.model.data.boundary_yh - self.model.data.boundary_yl;
            let diag = (dx * dx + dy * dy).sqrt().max(1.0);
            max_rms <= self.model.params.move_rtol * diag
                && max_move <= self.model.params.move_max_rtol * diag
        }
    }

    /// Returns a copy of the current positions
    pub fn get_positions(&self) -> Array1<f64> {
        self.pos.clone()
    }

    pub fn get_checkpoint_positions(&self) -> Option<Array1<f64>> {
        self.checkpoint_pos.clone()
    }

    pub fn base_end_positions(&self) -> Option<Array1<f64>> {
        self.base_end_pos.clone()
    }

    pub fn tail_end_positions(&self) -> Option<Array1<f64>> {
        self.tail_end_pos.clone()
    }

    pub fn best_composite_positions(&self) -> Option<Array1<f64>> {
        self.best_composite_pos.clone()
    }

    pub fn deoverlap_end_positions(&self) -> Option<Array1<f64>> {
        self.deoverlap_end_pos.clone()
    }

    pub fn deoverlap_summary_json(&self) -> serde_json::Value {
        match &self.deoverlap_summary {
            Some(s) => serde_json::to_value(s).unwrap_or_else(|_| serde_json::json!({})),
            None => serde_json::json!({}),
        }
    }

    pub fn deoverlap_moves_csv(&self) -> String {
        let mut w = String::new();
        w.push_str("component_id,object_id,object_kind,object_area,attempt_idx,tier,ring,dx,dy,success,reason,w_ref,h_ref,a_ref,w_tail,h_tail,r_raw,r_eff,long_axis,candidates_tested,candidates_feasible,rejected_by_area_cap,best_score,d_ell,e_w,e_h,e_a,e_expand,w_after,h_after,a_after,area_increase_ratio\n");
        for r in &self.deoverlap_moves {
            let cols = [
                r.component_id.to_string(),
                r.object_id.to_string(),
                r.object_kind.clone(),
                format!("{:.6e}", r.object_area),
                r.attempt_idx.to_string(),
                r.tier.clone(),
                r.ring.to_string(),
                format!("{:.6e}", r.dx),
                format!("{:.6e}", r.dy),
                r.success.to_string(),
                r.reason.clone(),
                format!("{:.6e}", r.w_ref),
                format!("{:.6e}", r.h_ref),
                format!("{:.6e}", r.a_ref),
                format!("{:.6e}", r.w_tail),
                format!("{:.6e}", r.h_tail),
                format!("{:.6e}", r.r_raw),
                format!("{:.6e}", r.r_eff),
                r.long_axis.clone(),
                r.candidates_tested.to_string(),
                r.candidates_feasible.to_string(),
                r.rejected_by_area_cap.to_string(),
                format!("{:.6e}", r.best_score),
                format!("{:.6e}", r.d_ell),
                format!("{:.6e}", r.e_w),
                format!("{:.6e}", r.e_h),
                format!("{:.6e}", r.e_a),
                format!("{:.6e}", r.e_expand),
                format!("{:.6e}", r.w_after),
                format!("{:.6e}", r.h_after),
                format!("{:.6e}", r.a_after),
                format!("{:.6e}", r.area_increase_ratio),
            ];
            w.push_str(&cols.join(","));
            w.push('\n');
        }
        w
    }

    pub fn deoverlap_components_before(&self) -> Vec<Vec<usize>> {
        self.deoverlap_components_before.clone()
    }

    pub fn deoverlap_components_after(&self) -> Vec<Vec<usize>> {
        self.deoverlap_components_after.clone()
    }

    /// `None` unless the LCR retry loop actually fired. The stage binary writes
    /// it as `multiphase_controller/lcr_retry_summary.json`, so a non-triggering run emits
    /// no new artifact and its output directory is unchanged.
    pub fn lcr_retry_summary_json(&self) -> Option<serde_json::Value> {
        self.lcr_retry_summary
            .as_ref()
            .and_then(|s| serde_json::to_value(s).ok())
    }

    /// Symmetry-weld diagnostics for the finished run.
    /// `(welded at Base→Tail entry, welded at LCR entry, pairs the rescue seeded a
    /// re-run with, pairs the rescue separated in the finished placement)`.
    ///
    /// The first two are reporting figures and are routinely non-zero on a healthy run.
    /// The last two are `0` unless the LCR retry loop exhausted with mirror pairs still
    /// welded onto their axis, which is the degenerate placement the rescue exists for.
    pub fn sym_weld_report(&self) -> (usize, usize, usize, usize) {
        (
            self.sym_weld_welded_at_tail_entry,
            self.sym_weld_welded_at_lcr_entry,
            self.sym_weld_rescue_seeded,
            self.sym_weld_rescue_repaired,
        )
    }

    pub fn deoverlap_unresolved_cells(&self) -> &[usize] {
        &self.deoverlap_unresolved_cells
    }

    pub fn deoverlap_infl_tail_snapshot(&self) -> Vec<f64> {
        self.deoverlap_infl_tail_snapshot.clone()
    }

    pub fn iter_rows(&self) -> Vec<IterRow> {
        self.iter_rows.clone()
    }

    pub fn candidates(&self) -> Vec<Candidate> {
        self.candidates.clone()
    }

    pub fn run_summary(&self) -> Option<RunSummary> {
        self.run_summary.clone()
    }

    fn optimize_phi_apwl_fixed_placement_internal(
        &self,
        oracle_iters: usize,
    ) -> PlaceResult<(Vec<f64>, serde_json::Value)> {
        let n_pins = self.model.data.num_pins;
        let n_cells = self.model.data.num_cells;
        if n_pins == 0 || n_cells == 0 {
            return Ok((
                self.model.phi.clone(),
                serde_json::json!({
                    "applied": true,
                    "reason": "empty_design",
                    "oracle_iters_budget": oracle_iters,
                    "iters_used": 0,
                    "wl_ap_initial": 0.0,
                    "wl_ap_optimal": 0.0,
                    "improve_abs": 0.0,
                    "improve_pct": 0.0
                }),
            ));
        }
        let pos_slice = self.pos.as_slice().ok_or_else(|| {
            PlaceError::ShapeError("pos is not contiguous for oracle apwl eval".to_string())
        })?;
        let mut phi = self.model.phi.clone();
        if phi.len() != n_pins {
            phi = vec![0.0; n_pins];
        }
        let mut grad_xy = vec![0.0; 2 * n_cells];
        let mut phi_grad = vec![0.0; n_pins];
        let mut wl_ap = crate::constraints::compute_lsewl_ap(
            pos_slice,
            &self.model.data,
            &phi,
            &mut phi_grad,
            &mut grad_xy,
            self.model.wl_gamma,
            self.model.params.ap_exponent_m,
            self.model.params.ap_eps,
        )?;
        let wl_ap_initial = wl_ap;
        let mut best = wl_ap_initial;
        let mut best_phi = phi.clone();
        let mut opt = AdamOptimizer::new(
            n_pins,
            self.model.params.learning_rate,
            self.model.params.adam_beta1,
            self.model.params.adam_beta2,
        );
        let mut used = 0usize;
        for _ in 0..oracle_iters {
            grad_xy.fill(0.0);
            phi_grad.fill(0.0);
            wl_ap = crate::constraints::compute_lsewl_ap(
                pos_slice,
                &self.model.data,
                &phi,
                &mut phi_grad,
                &mut grad_xy,
                self.model.wl_gamma,
                self.model.params.ap_exponent_m,
                self.model.params.ap_eps,
            )?;
            if !wl_ap.is_finite() {
                break;
            }
            if wl_ap < best {
                best = wl_ap;
                best_phi.clone_from(&phi);
            }
            let phi_arr = Array1::from_vec(phi.clone());
            let g_arr = Array1::from_vec(phi_grad.clone());
            let next = opt.step(&phi_arr, &g_arr);
            phi = next.iter().map(|v| Self::wrap_angle(*v)).collect();
            used += 1;
        }

        let improve_abs = wl_ap_initial - best;
        let improve_pct = if wl_ap_initial.abs() > 1e-20 {
            improve_abs / wl_ap_initial.abs()
        } else {
            0.0
        };
        Ok((
            best_phi.clone(),
            serde_json::json!({
                "applied": true,
                "reason": "ok",
                "oracle_iters_budget": oracle_iters,
                "iters_used": used,
                "wl_ap_initial": wl_ap_initial,
                "wl_ap_optimal": best,
                "improve_abs": improve_abs,
                "improve_pct": improve_pct,
                "best_phi_sample": best_phi.into_iter().take(8).collect::<Vec<_>>()
            }),
        ))
    }

    pub fn batch_oracle_apwl_fixed_placement(
        &mut self,
        oracle_iters: usize,
    ) -> PlaceResult<serde_json::Value> {
        let (_, summary) = self.optimize_phi_apwl_fixed_placement_internal(oracle_iters)?;
        Ok(summary)
    }

    pub fn deoverlap_finalize_phi_apwl_fixed_placement(
        &mut self,
        oracle_iters: usize,
    ) -> PlaceResult<serde_json::Value> {
        let (best_phi, summary) = self.optimize_phi_apwl_fixed_placement_internal(oracle_iters)?;
        self.model.phi = best_phi;
        self.sync_tilde_from_phi();
        Ok(summary)
    }

    pub fn current_phi_radians(&self) -> Vec<f64> {
        self.model.phi.clone()
    }

    pub fn export_multiphase_artifacts<P: AsRef<Path>>(&self, output_dir: P) -> PlaceResult<()> {
        if self.iter_rows.is_empty() {
            return Ok(());
        }
        let summary = self.run_summary.as_ref().ok_or_else(|| {
            PlaceError::InvalidParameter("multiphase run summary missing".to_string())
        })?;
        if summary.schema_version != crate::gp_controller::OUTPUT_SCHEMA_VERSION {
            return Err(PlaceError::InvalidParameter(format!(
                "multiphase run summary schema mismatch: expected {}, got {}",
                crate::gp_controller::OUTPUT_SCHEMA_VERSION,
                summary.schema_version
            )));
        }
        if summary.base_end_iter.is_none() || summary.tail_end_iter.is_none() {
            return Err(PlaceError::InvalidParameter(
                "multiphase run summary missing required fields: base_end_iter/tail_end_iter"
                    .to_string(),
            ));
        }
        if summary.selected_final_candidate_label.is_none() {
            return Err(PlaceError::InvalidParameter(
                "multiphase run summary missing required field: selected_final_candidate_label"
                    .to_string(),
            ));
        }
        if summary.candidate_summary.is_empty() {
            return Err(PlaceError::InvalidParameter(
                "multiphase candidate summary must be non-empty".to_string(),
            ));
        }

        std::fs::create_dir_all(output_dir.as_ref())?;
        let checkpoints_dir = output_dir.as_ref().join("checkpoints");
        if checkpoints_dir.exists() {
            std::fs::remove_dir_all(&checkpoints_dir)?;
        }
        std::fs::create_dir_all(&checkpoints_dir)?;

        let csv_header = "iter,stage,loss_total,wl_base,wl_ap,wl_mix_real,ovl_raw_real,ovl_raw_infl,q_o_real,q_o_infl,sym_raw,q_s,lambda_ovl,lambda_sym,ap_mix,lambda_anchor,lambda_cover_tail,alpha_infl,ref_scale_s,grad_norm_total,grad_norm_wl,grad_norm_ovl,grad_norm_sym,grad_norm_cover,h_o,h_s,d_o,d_s,step_norm,disp_rms,disp_max,disp_to_ckpt_rms,disp_to_ckpt_max,disp_to_tail_ckpt_rms,disp_to_tail_ckpt_max,disp_to_ref_rms,disp_to_ref_max,phi_step_rms,phi_step_max,clip_ratio_all,clip_ratio_xy,clip_ratio_phi,mobility_enabled,num_conflict_components_real,largest_conflict_component_size_real,largest_conflict_component_area_real,num_conflict_components_infl,largest_conflict_component_area_infl,num_cover_pairs_real,max_cover_ratio_real,mean_cover_ratio_real";
        let csv_path = output_dir.as_ref().join("iter.csv");
        let mut w = String::new();
        w.push_str(csv_header);
        w.push('\n');
        for r in &self.iter_rows {
            w.push_str(&format!(
                "{},{},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{},{},{},{:.6e},{},{:.6e},{},{:.6e},{:.6e}\n",
                r.iter,
                r.stage,
                r.loss_total,
                r.wl_base,
                r.wl_ap,
                r.wl_mix_real,
                r.ovl_raw_real,
                r.ovl_raw_infl,
                r.q_o_real,
                r.q_o_infl,
                r.sym_raw,
                r.q_s,
                r.lambda_ovl,
                r.lambda_sym,
                r.ap_mix,
                r.lambda_anchor,
                r.lambda_cover_tail,
                r.alpha_infl,
                r.ref_scale_s,
                r.grad_norm_total,
                r.grad_norm_wl,
                r.grad_norm_ovl,
                r.grad_norm_sym,
                r.grad_norm_cover,
                r.h_o,
                r.h_s,
                r.d_o,
                r.d_s,
                r.step_norm,
                r.disp_rms,
                r.disp_max,
                r.disp_to_ckpt_rms,
                r.disp_to_ckpt_max,
                r.disp_to_tail_ckpt_rms,
                r.disp_to_tail_ckpt_max,
                r.disp_to_ref_rms,
                r.disp_to_ref_max,
                r.phi_step_rms,
                r.phi_step_max,
                r.clip_ratio_all,
                r.clip_ratio_xy,
                r.clip_ratio_phi,
                r.mobility_enabled,
                r.num_conflict_components_real,
                r.largest_conflict_component_size_real,
                r.largest_conflict_component_area_real,
                r.num_conflict_components_infl,
                r.largest_conflict_component_area_infl,
                r.num_cover_pairs_real,
                r.max_cover_ratio_real,
                r.mean_cover_ratio_real
            ));
        }
        std::fs::write(csv_path, w)?;

        let iter_meta_path = output_dir.as_ref().join("iter_meta.json");
        let required_columns: Vec<serde_json::Value> = csv_header
            .split(',')
            .map(|c| serde_json::Value::String(c.to_string()))
            .collect();
        let mut column_units = serde_json::Map::new();
        for (k, v) in [
            ("iter", "index"),
            ("stage", "base|tail"),
            ("loss_total", "objective"),
            ("wl_base", "objective"),
            ("wl_ap", "objective"),
            ("wl_mix_real", "objective"),
            ("ovl_raw_real", "objective"),
            ("ovl_raw_infl", "objective"),
            ("q_o_real", "normalized_overlap"),
            ("q_o_infl", "normalized_overlap"),
            ("sym_raw", "objective"),
            ("q_s", "normalized_symmetry"),
            ("lambda_ovl", "weight"),
            ("lambda_sym", "weight"),
            ("ap_mix", "mix"),
            ("lambda_anchor", "weight"),
            ("lambda_cover_tail", "weight"),
            ("alpha_infl", "ratio"),
            ("ref_scale_s", "ratio"),
            ("grad_norm_total", "l2"),
            ("grad_norm_wl", "l2"),
            ("grad_norm_ovl", "l2"),
            ("grad_norm_sym", "l2"),
            ("grad_norm_cover", "l2"),
            ("h_o", "ema"),
            ("h_s", "ema"),
            ("d_o", "controller_signal"),
            ("d_s", "controller_signal"),
            ("step_norm", "l2"),
            ("disp_rms", "distance"),
            ("disp_max", "distance"),
            ("disp_to_ckpt_rms", "distance"),
            ("disp_to_ckpt_max", "distance"),
            ("disp_to_tail_ckpt_rms", "distance"),
            ("disp_to_tail_ckpt_max", "distance"),
            ("disp_to_ref_rms", "distance"),
            ("disp_to_ref_max", "distance"),
            ("phi_step_rms", "angle_norm"),
            ("phi_step_max", "angle_norm"),
            ("clip_ratio_all", "ratio"),
            ("clip_ratio_xy", "ratio"),
            ("clip_ratio_phi", "ratio"),
            ("mobility_enabled", "boolean"),
            ("num_conflict_components_real", "count"),
            ("largest_conflict_component_size_real", "count"),
            ("largest_conflict_component_area_real", "area"),
            ("num_conflict_components_infl", "count"),
            ("largest_conflict_component_area_infl", "area"),
            ("num_cover_pairs_real", "count"),
            ("max_cover_ratio_real", "ratio"),
            ("mean_cover_ratio_real", "ratio"),
        ] {
            column_units.insert(k.to_string(), serde_json::Value::String(v.to_string()));
        }
        let mut iter_meta_obj = serde_json::Map::new();
        iter_meta_obj.insert(
            "schemaVersion".to_string(),
            serde_json::Value::from(crate::gp_controller::OUTPUT_SCHEMA_VERSION),
        );
        iter_meta_obj.insert(
            "csv".to_string(),
            serde_json::Value::String("iter.csv".to_string()),
        );
        iter_meta_obj.insert(
            "requiredColumns".to_string(),
            serde_json::Value::Array(required_columns),
        );
        iter_meta_obj.insert(
            "columnUnits".to_string(),
            serde_json::Value::Object(column_units),
        );
        let iter_meta = serde_json::Value::Object(iter_meta_obj);
        std::fs::write(iter_meta_path, serde_json::to_string_pretty(&iter_meta)?)?;

        let candidates_path = output_dir.as_ref().join("candidate_summary.json");
        let candidate_summary = serde_json::json!({
            "schemaVersion": crate::gp_controller::OUTPUT_SCHEMA_VERSION,
            "candidates": &summary.candidate_summary,
        });
        std::fs::write(
            candidates_path,
            serde_json::to_string_pretty(&candidate_summary)?,
        )?;

        let summary_path = output_dir.as_ref().join("run_summary.json");
        std::fs::write(summary_path, serde_json::to_string_pretty(summary)?)?;

        for c in &self.candidates {
            let path = checkpoints_dir.join(format!("checkpoint_iter_{:05}.json", c.iter));
            std::fs::write(path, serde_json::to_string_pretty(c)?)?;
        }
        Ok(())
    }

    pub fn export_iter_csv_only<P: AsRef<Path>>(&self, output_dir: P) -> PlaceResult<()> {
        if self.iter_rows.is_empty() {
            return Ok(());
        }
        std::fs::create_dir_all(output_dir.as_ref())?;
        let csv_header = "iter,stage,loss_total,wl_base,wl_ap,wl_mix_real,ovl_raw_real,ovl_raw_infl,q_o_real,q_o_infl,sym_raw,q_s,lambda_ovl,lambda_sym,ap_mix,lambda_anchor,lambda_cover_tail,alpha_infl,ref_scale_s,grad_norm_total,grad_norm_wl,grad_norm_ovl,grad_norm_sym,grad_norm_cover,h_o,h_s,d_o,d_s,step_norm,disp_rms,disp_max,disp_to_ckpt_rms,disp_to_ckpt_max,disp_to_tail_ckpt_rms,disp_to_tail_ckpt_max,disp_to_ref_rms,disp_to_ref_max,phi_step_rms,phi_step_max,clip_ratio_all,clip_ratio_xy,clip_ratio_phi,mobility_enabled,num_conflict_components_real,largest_conflict_component_size_real,largest_conflict_component_area_real,num_conflict_components_infl,largest_conflict_component_area_infl,num_cover_pairs_real,max_cover_ratio_real,mean_cover_ratio_real";
        let csv_path = output_dir.as_ref().join("iter.csv");
        let mut w = String::new();
        w.push_str(csv_header);
        w.push('\n');
        for r in &self.iter_rows {
            w.push_str(&format!(
                "{},{},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{},{},{},{:.6e},{},{:.6e},{},{:.6e},{:.6e}\n",
                r.iter,
                r.stage,
                r.loss_total,
                r.wl_base,
                r.wl_ap,
                r.wl_mix_real,
                r.ovl_raw_real,
                r.ovl_raw_infl,
                r.q_o_real,
                r.q_o_infl,
                r.sym_raw,
                r.q_s,
                r.lambda_ovl,
                r.lambda_sym,
                r.ap_mix,
                r.lambda_anchor,
                r.lambda_cover_tail,
                r.alpha_infl,
                r.ref_scale_s,
                r.grad_norm_total,
                r.grad_norm_wl,
                r.grad_norm_ovl,
                r.grad_norm_sym,
                r.grad_norm_cover,
                r.h_o,
                r.h_s,
                r.d_o,
                r.d_s,
                r.step_norm,
                r.disp_rms,
                r.disp_max,
                r.disp_to_ckpt_rms,
                r.disp_to_ckpt_max,
                r.disp_to_tail_ckpt_rms,
                r.disp_to_tail_ckpt_max,
                r.disp_to_ref_rms,
                r.disp_to_ref_max,
                r.phi_step_rms,
                r.phi_step_max,
                r.clip_ratio_all,
                r.clip_ratio_xy,
                r.clip_ratio_phi,
                r.mobility_enabled,
                r.num_conflict_components_real,
                r.largest_conflict_component_size_real,
                r.largest_conflict_component_area_real,
                r.num_conflict_components_infl,
                r.largest_conflict_component_area_infl,
                r.num_cover_pairs_real,
                r.max_cover_ratio_real,
                r.mean_cover_ratio_real
            ));
        }
        std::fs::write(csv_path, w)?;
        Ok(())
    }

    pub fn tail_metrics_json(&self) -> serde_json::Value {
        serde_json::json!({
            "tail_state": match self.tail_state {
                TailState::Hold => "hold",
            },
            "overlap_ratio": self.current_overlap_ratio(),
            "drift_ratio": self.current_drift_ratio(),
            "symmetry_ratio": self.current_symmetry_ratio(),
            "conflict_component_cells": self.current_conflict_count(),
            "lambda": {
                "overlap": self.model.lambdas.overlap,
                "symmetry": self.model.lambdas.symmetry,
                "boundary": self.model.lambdas.boundary
            }
        })
    }

    pub fn tail_diagnostics_iter_row(&self) -> Vec<TailDiagnosticsIterRow> {
        self.tail_diag_rows.clone()
    }

    pub fn tail_diagnostics_snapshot(&self) -> TailDiagnosticsSnapshot {
        TailDiagnosticsSnapshot {
            interval: self.model.params.tail_diag_interval.max(1),
            rows: self.tail_diag_snapshots.clone(),
        }
    }

    pub fn conflict_components_json(&self) -> serde_json::Value {
        let idxs: Vec<usize> = self
            .conflict_flags
            .iter()
            .enumerate()
            .filter_map(|(i, f)| if *f { Some(i) } else { None })
            .collect();
        serde_json::json!({
            "conflict_cells": idxs,
            "inflation": self.overlap_inflation,
            "component_count": self.conflict_component_count,
            "largest_component_size": self.largest_component_size,
            "overlap_count": self.overlap_count,
            "overlap_total_area": self.overlap_total_area,
            "inflated_component_ids": self.inflated_component_ids,
            "inflated_object_count": self.inflation_object_count,
        })
    }

    pub fn mobility_stats_json(&self) -> serde_json::Value {
        let n = self.model.data.num_cells.max(1);
        let mean_area = self.model.data.cell_areas.sum() / (n as f64);
        let mut large = 0usize;
        let mut small = 0usize;
        for i in 0..self.model.data.num_cells {
            let a = self.model.data.cell_areas[i];
            if a > 2.0 * mean_area {
                large += 1;
            }
            if a < 0.5 * mean_area {
                small += 1;
            }
        }
        serde_json::json!({
            "mean_area": mean_area,
            "large_cells": large,
            "small_cells": small,
            "mobility_params": {
                "min": self.model.params.mobility_min,
                "max": self.model.params.mobility_max,
                "sym_anchor_mult": self.model.params.mobility_sym_anchor_mult,
                "large_mult": self.model.params.mobility_large_mult,
                "small_mult": self.model.params.mobility_small_mult
            }
        })
    }

    pub fn object_displacement_json(&self) -> serde_json::Value {
        let Some(ckpt) = &self.checkpoint_pos else {
            return serde_json::json!([]);
        };
        let n = self.model.data.num_cells;
        let mut objs = Vec::with_capacity(n);
        for i in 0..n {
            let dx = self.pos[i] - ckpt[i];
            let dy = self.pos[i + n] - ckpt[i + n];
            let d = (dx * dx + dy * dy).sqrt();
            objs.push(serde_json::json!({
                "idx": i,
                "x": self.pos[i],
                "y": self.pos[i+n],
                "disp": d,
                "conflict": self.conflict_flags.get(i).copied().unwrap_or(false),
            }));
        }
        serde_json::json!(objs)
    }

    /// Get loss history
    pub fn get_loss_history(&self) -> &[f64] {
        &self.loss_history
    }

    /// Export visualization data (loss curve + cell positions)
    ///
    /// This exports:
    /// - loss_history.csv
    /// - cell_positions.csv
    /// - plot_loss.py (Python script to plot loss curve)
    /// - plot_positions.py (Python script to visualize positions)
    ///
    /// # Arguments
    ///
    /// * `output_dir` - Directory to save outputs
    pub fn export_visualization<P: AsRef<Path>>(&self, output_dir: P) -> PlaceResult<()> {
        crate::visualization::export_all(
            &self.pos,
            &self.loss_history,
            &self.model.data,
            output_dir,
        )
    }
}

fn build_reduced_map(data: &PlaceData) -> Option<SymReducedMap> {
    let n = data.num_cells;
    if n == 0 {
        return None;
    }
    if data.num_sym_pairs == 0 {
        return None;
    }

    let mut full = vec![
        AffineTerm {
            var: None,
            scale: 0.0,
            bias: 0.0
        };
        2 * n
    ];
    let mut vars: Vec<ReducedVarMeta> = Vec::new();
    let mut var_terms: Vec<Vec<(usize, f64, f64)>> = Vec::new();

    let alloc_var = |axis: Axis,
                     vars: &mut Vec<ReducedVarMeta>,
                     var_terms: &mut Vec<Vec<(usize, f64, f64)>>| {
        let v = vars.len();
        vars.push(ReducedVarMeta { axis });
        var_terms.push(Vec::new());
        v
    };

    let mut x_assigned = vec![false; n];
    let mut y_assigned = vec![false; n];

    // pair-mirror + pair-alignment (center-based)
    for i in 0..data.num_sym_pairs {
        let a = data.sym_pairs[2 * i] as usize;
        let b = data.sym_pairs[2 * i + 1] as usize;
        if a >= n || b >= n {
            continue;
        }
        let g = data.sym_pair_group[i] as usize;
        let is_vert = data.sym_group_vertical.get(g).copied().unwrap_or(true);
        let axis = if is_vert {
            (data.boundary_xl + data.boundary_xh) * 0.5
        } else {
            (data.boundary_yl + data.boundary_yh) * 0.5
        };

        if is_vert {
            if !x_assigned[a] && !x_assigned[b] {
                let v = alloc_var(Axis::X, &mut vars, &mut var_terms);
                let ax = data.sym_anchor_x[a];
                let bx = data.sym_anchor_x[b];
                full[a] = AffineTerm {
                    var: Some(v),
                    scale: 1.0,
                    bias: -ax,
                };
                full[b] = AffineTerm {
                    var: Some(v),
                    scale: -1.0,
                    bias: 2.0 * axis - bx,
                };
                var_terms[v].push((a, 1.0, -ax));
                var_terms[v].push((b, -1.0, 2.0 * axis - bx));
                x_assigned[a] = true;
                x_assigned[b] = true;
            }
            if !y_assigned[a] && !y_assigned[b] {
                let v = alloc_var(Axis::Y, &mut vars, &mut var_terms);
                let ay = data.sym_anchor_y[a];
                let by = data.sym_anchor_y[b];
                full[a + n] = AffineTerm {
                    var: Some(v),
                    scale: 1.0,
                    bias: -ay,
                };
                full[b + n] = AffineTerm {
                    var: Some(v),
                    scale: 1.0,
                    bias: -by,
                };
                var_terms[v].push((a + n, 1.0, -ay));
                var_terms[v].push((b + n, 1.0, -by));
                y_assigned[a] = true;
                y_assigned[b] = true;
            }
        } else {
            if !y_assigned[a] && !y_assigned[b] {
                let v = alloc_var(Axis::Y, &mut vars, &mut var_terms);
                let ay = data.sym_anchor_y[a];
                let by = data.sym_anchor_y[b];
                full[a + n] = AffineTerm {
                    var: Some(v),
                    scale: 1.0,
                    bias: -ay,
                };
                full[b + n] = AffineTerm {
                    var: Some(v),
                    scale: -1.0,
                    bias: 2.0 * axis - by,
                };
                var_terms[v].push((a + n, 1.0, -ay));
                var_terms[v].push((b + n, -1.0, 2.0 * axis - by));
                y_assigned[a] = true;
                y_assigned[b] = true;
            }
            if !x_assigned[a] && !x_assigned[b] {
                let v = alloc_var(Axis::X, &mut vars, &mut var_terms);
                let ax = data.sym_anchor_x[a];
                let bx = data.sym_anchor_x[b];
                full[a] = AffineTerm {
                    var: Some(v),
                    scale: 1.0,
                    bias: -ax,
                };
                full[b] = AffineTerm {
                    var: Some(v),
                    scale: 1.0,
                    bias: -bx,
                };
                var_terms[v].push((a, 1.0, -ax));
                var_terms[v].push((b, 1.0, -bx));
                x_assigned[a] = true;
                x_assigned[b] = true;
            }
        }
    }

    // self-sym: mirror axis fixed, orthogonal free.
    // v1: self-sym is inferred from pairs where a==b (if provided in data path).
    for i in 0..data.num_sym_pairs {
        let a = data.sym_pairs[2 * i] as usize;
        let b = data.sym_pairs[2 * i + 1] as usize;
        if a != b || a >= n {
            continue;
        }
        let g = data.sym_pair_group[i] as usize;
        let is_vert = data.sym_group_vertical.get(g).copied().unwrap_or(true);
        let axis = if is_vert {
            (data.boundary_xl + data.boundary_xh) * 0.5
        } else {
            (data.boundary_yl + data.boundary_yh) * 0.5
        };
        if is_vert {
            if !x_assigned[a] {
                full[a] = AffineTerm {
                    var: None,
                    scale: 0.0,
                    bias: axis - data.sym_anchor_x[a],
                };
                x_assigned[a] = true;
            }
            if !y_assigned[a] {
                let v = alloc_var(Axis::Y, &mut vars, &mut var_terms);
                full[a + n] = AffineTerm {
                    var: Some(v),
                    scale: 1.0,
                    bias: -data.sym_anchor_y[a],
                };
                var_terms[v].push((a + n, 1.0, -data.sym_anchor_y[a]));
                y_assigned[a] = true;
            }
        } else {
            if !y_assigned[a] {
                full[a + n] = AffineTerm {
                    var: None,
                    scale: 0.0,
                    bias: axis - data.sym_anchor_y[a],
                };
                y_assigned[a] = true;
            }
            if !x_assigned[a] {
                let v = alloc_var(Axis::X, &mut vars, &mut var_terms);
                full[a] = AffineTerm {
                    var: Some(v),
                    scale: 1.0,
                    bias: -data.sym_anchor_x[a],
                };
                var_terms[v].push((a, 1.0, -data.sym_anchor_x[a]));
                x_assigned[a] = true;
            }
        }
    }

    for i in 0..n {
        if !x_assigned[i] && full[i].var.is_none() {
            let v = alloc_var(Axis::X, &mut vars, &mut var_terms);
            full[i] = AffineTerm {
                var: Some(v),
                scale: 1.0,
                bias: 0.0,
            };
            var_terms[v].push((i, 1.0, 0.0));
        }
        if !y_assigned[i] && full[i + n].var.is_none() {
            let v = alloc_var(Axis::Y, &mut vars, &mut var_terms);
            full[i + n] = AffineTerm {
                var: Some(v),
                scale: 1.0,
                bias: 0.0,
            };
            var_terms[v].push((i + n, 1.0, 0.0));
        }
    }

    Some(SymReducedMap {
        full_to_reduced: full,
        var_terms,
        vars,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndarray::Array1;
    use std::collections::HashSet;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn testcase_data(no_sym: bool) -> PlaceData {
        let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let bbox = manifest_dir.join("tests/testcase/bbox.json");
        let pin = manifest_dir.join("tests/testcase/pin.json");
        let sym = manifest_dir.join("tests/testcase/sym.json");
        let mut data = PlaceData::from_json_files(bbox, pin, sym).expect("testcase data");
        if no_sym {
            data.num_sym_pairs = 0;
            data.num_sym_groups = 0;
            data.sym_pairs = Array1::zeros(0);
            data.sym_pair_group = Array1::zeros(0);
            data.sym_group_vertical = Array1::from_vec(vec![]);
        }
        data
    }

    /// Two identical mirrored devices and nothing else, so the only thing the fixture
    /// can express is the weld: one symmetry pair whose shared reduced variable sits on
    /// the axis puts both devices at the same point.
    fn welded_mirror_pair_data() -> PlaceData {
        let num_cells = 2;
        PlaceData {
            num_cells,
            num_pins: 0,
            num_nets: 0,
            cell_widths: Array1::from_vec(vec![400.0, 400.0]),
            cell_heights: Array1::from_vec(vec![200.0, 200.0]),
            cell_areas: Array1::from_vec(vec![80000.0, 80000.0]),
            cell_widths_keepout: Array1::from_vec(vec![400.0, 400.0]),
            cell_heights_keepout: Array1::from_vec(vec![200.0, 200.0]),
            cell_areas_keepout: Array1::from_vec(vec![80000.0, 80000.0]),
            is_pmos_mask: Array1::from_vec(vec![false, false]),
            pin_offset: Array1::zeros(0),
            pin_owner_cell: Array1::zeros(0),
            io_pin_mask: Array1::from_vec(vec![]),
            net_weights: Array1::zeros(0),
            net_pin_offsets: Array1::from_vec(vec![0]),
            net_pin_index: Array1::zeros(0),
            cell_pin_offsets: Array1::from_vec(vec![0, 0, 0]),
            cell_pin_index: Array1::zeros(0),
            overlap_pairs: Array1::from_vec(vec![0, 1]),
            num_overlap_pairs: 1,
            sym_pairs: Array1::from_vec(vec![0, 1]),
            sym_pair_group: Array1::from_vec(vec![0]),
            sym_group_vertical: Array1::from_vec(vec![true]),
            sym_group_names: vec!["G".to_string()],
            num_sym_pairs: 1,
            num_sym_groups: 1,
            sym_self_weight: 1.0,
            sym_anchor_x: Array1::from_vec(vec![200.0; num_cells]),
            sym_anchor_y: Array1::from_vec(vec![100.0; num_cells]),
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
            boundary_xh: 10000.0,
            boundary_yh: 10000.0,
        }
    }

    fn welded_mirror_pair_placer() -> Placer {
        let mut params = PlacementConfig::default();
        // The separation the repair aims for is the de-overlap pass's own feasibility
        // extent, so pin both knobs that define it rather than inherit a default.
        params.tail_deoverlap_infl = 1.0;
        params.lcr_honor_drc_floor = true;
        params.tail_hard_inflation_floor = 50.0;
        let mut placer = Placer::from_data(welded_mirror_pair_data(), params).expect("placer");
        // Exactly what `init_center_overlap_positions` does on a real case, and exactly
        // the degenerate state: both devices centred on the group's axis.
        placer.init_positions();
        placer
    }

    #[test]
    fn a_mirror_pair_on_its_axis_is_reported_stuck() {
        let placer = welded_mirror_pair_placer();
        let n = placer.model.data.num_cells;
        let axis = placer.axis_for_group(true);
        let c0 = placer.pos[0] + placer.model.data.sym_anchor_x[0];
        let c1 = placer.pos[1] + placer.model.data.sym_anchor_x[1];
        assert_eq!(c0, axis, "the initial placement puts device 0 on the axis");
        assert_eq!(c1, axis, "and device 1 on top of it");
        assert_eq!(placer.pos[n], placer.pos[n + 1]);

        let stuck = placer.self_overlapping_sym_pairs(&placer.pos);
        assert_eq!(stuck, vec![0usize]);
    }

    #[test]
    fn a_mirror_pair_that_overlaps_itself_without_coinciding_is_reported_too() {
        // The failure is structural, not numerical: de-overlap treats a pair as one
        // rigid object, so a pair that overlaps *itself* has no other object in its
        // component to move against however far from bit-exact coincidence it is. The
        // detector therefore asks the de-overlap pass's own question -- do the two
        // members collide under its feasibility extents -- and 1 DBU off the axis is
        // still a collision between two 400 DBU devices.
        let mut placer = welded_mirror_pair_placer();
        placer.pos[0] -= 1.0;
        placer.pos[1] += 1.0;
        let n = placer.model.data.num_cells;
        let c0 = placer.pos[0] + placer.model.data.sym_anchor_x[0];
        let c1 = placer.pos[1] + placer.model.data.sym_anchor_x[1];
        assert_ne!(c0, c1, "the fixture must not be bit-exactly coincident");
        assert_eq!(placer.pos[n], placer.pos[n + 1]);
        assert_eq!(placer.self_overlapping_sym_pairs(&placer.pos), vec![0usize]);

        // And the repair clears it the same way it clears a fully welded pair.
        let separated = placer.separate_self_overlapping_sym_pairs("unit test");
        assert_eq!(separated, vec![0usize]);
        assert!(placer.self_overlapping_sym_pairs(&placer.pos).is_empty());
        let d0 = placer.pos[0] + placer.model.data.sym_anchor_x[0];
        let d1 = placer.pos[1] + placer.model.data.sym_anchor_x[1];
        let axis = placer.axis_for_group(true);
        assert_eq!(d0 + d1, 2.0 * axis, "centres stay mirrored about the axis");
    }

    #[test]
    fn a_mirror_pair_clear_of_itself_is_left_alone() {
        // The other side of the criterion: once the two devices no longer collide under
        // the de-overlap pass's extents, the pair is nothing special and the rescue must
        // not touch it -- that is what keeps every converging run byte-identical.
        let mut placer = welded_mirror_pair_placer();
        let axis = placer.axis_for_group(true);
        // Extent per device is 400 * 1.0 + 2 * 50 = 500, so 600 DBU apart is clear.
        placer.pos[0] = (axis - 300.0) - placer.model.data.sym_anchor_x[0];
        placer.pos[1] = (axis + 300.0) - placer.model.data.sym_anchor_x[1];
        assert!(placer.self_overlapping_sym_pairs(&placer.pos).is_empty());
        let before = placer.pos.clone();
        assert!(placer
            .separate_self_overlapping_sym_pairs("unit test")
            .is_empty());
        assert_eq!(placer.pos, before);
    }

    #[test]
    fn separating_a_stuck_mirror_pair_keeps_it_mirrored() {
        let mut placer = welded_mirror_pair_placer();
        let n = placer.model.data.num_cells;
        let axis = placer.axis_for_group(true);

        let separated = placer.separate_self_overlapping_sym_pairs("unit test");
        assert_eq!(separated, vec![0usize]);

        // Nothing is welded any more.
        assert!(placer.self_overlapping_sym_pairs(&placer.pos).is_empty());

        let c0 = placer.pos[0] + placer.model.data.sym_anchor_x[0];
        let c1 = placer.pos[1] + placer.model.data.sym_anchor_x[1];
        // Still exactly mirrored: the repair separates the pair without weakening the
        // symmetry it was declared with.
        assert_eq!(c0 + c1, 2.0 * axis, "centres stay mirrored about the axis");
        assert!(c0 < axis && c1 > axis, "and land on opposite sides");
        assert_eq!(
            placer.pos[n],
            placer.pos[n + 1],
            "the coordinate the pair shares across the axis is untouched"
        );

        // And they are clear of each other by the standard the de-overlap pass judges
        // them against: width * infl + 2 * floor = 400 + 100 = 500 per device, so the
        // centres must be at least 500 apart.
        assert!(
            (c1 - c0) >= 500.0 - 1e-9,
            "centres {c0} and {c1} are not clear by the legalizer's own extent"
        );
        let infl = vec![1.0f64; n];
        let (w_feas, h_feas) = placer.deoverlap_feasible_dims(&infl);
        assert!(!placer.cells_overlap_with_dims(0, 1, &w_feas, &h_feas));
    }

    #[test]
    fn the_weld_seed_is_a_no_op_when_nothing_was_recorded() {
        let mut placer = welded_mirror_pair_placer();
        let before = placer.pos.clone();
        placer.apply_sym_weld_seed();
        assert_eq!(placer.pos, before);
    }

    #[test]
    fn the_weld_seed_separates_the_recorded_pair_after_the_initial_placement() {
        let mut placer = welded_mirror_pair_placer();
        placer.sym_weld_seed = vec![0usize];
        placer.init_positions();
        assert_eq!(placer.self_overlapping_sym_pairs(&placer.pos), vec![0usize]);
        placer.apply_sym_weld_seed();
        assert!(placer.self_overlapping_sym_pairs(&placer.pos).is_empty());
    }

    #[test]
    fn test_select_deoverlap_move_target_prefers_smallest_area() {
        let objects = vec![
            DeoverlapObject {
                kind: DeoverlapObjectKind::Singleton,
                members: vec![0],
                area: 10.0,
                is_vert: None,
                axis: None,
                primary_lo: None,
                primary_hi: None,
            },
            DeoverlapObject {
                kind: DeoverlapObjectKind::Singleton,
                members: vec![1],
                area: 3.0,
                is_vert: None,
                axis: None,
                primary_lo: None,
                primary_hi: None,
            },
            DeoverlapObject {
                kind: DeoverlapObjectKind::Pair,
                members: vec![2, 3],
                area: 20.0,
                is_vert: Some(true),
                axis: Some(0.0),
                primary_lo: Some(2),
                primary_hi: Some(3),
            },
            DeoverlapObject {
                kind: DeoverlapObjectKind::Singleton,
                members: vec![4],
                area: 3.0,
                is_vert: None,
                axis: None,
                primary_lo: None,
                primary_hi: None,
            },
        ];
        let conflicted = HashSet::from([0usize, 1usize, 2usize, 3usize]);
        let fixed_obj = 2usize;
        let mut blocked = HashSet::new();

        let first =
            Placer::select_deoverlap_move_target(&objects, &conflicted, fixed_obj, &blocked);
        // Tie on area(oid=1 and oid=3): smaller object id should win deterministically.
        assert_eq!(first, Some(1usize));

        blocked.insert(1usize);
        let second =
            Placer::select_deoverlap_move_target(&objects, &conflicted, fixed_obj, &blocked);
        assert_eq!(second, Some(3usize));

        blocked.insert(3usize);
        let third =
            Placer::select_deoverlap_move_target(&objects, &conflicted, fixed_obj, &blocked);
        assert_eq!(third, Some(0usize));
    }

    #[test]
    fn test_select_deoverlap_fixed_obj_tie_prefers_smaller_id() {
        let objects = vec![
            DeoverlapObject {
                kind: DeoverlapObjectKind::Singleton,
                members: vec![0],
                area: 10.0,
                is_vert: None,
                axis: None,
                primary_lo: None,
                primary_hi: None,
            },
            DeoverlapObject {
                kind: DeoverlapObjectKind::Singleton,
                members: vec![1],
                area: 10.0,
                is_vert: None,
                axis: None,
                primary_lo: None,
                primary_hi: None,
            },
            DeoverlapObject {
                kind: DeoverlapObjectKind::Singleton,
                members: vec![2],
                area: 9.0,
                is_vert: None,
                axis: None,
                primary_lo: None,
                primary_hi: None,
            },
        ];
        let comp = vec![1usize, 0usize, 2usize];
        let fixed = Placer::select_deoverlap_fixed_obj(&objects, &comp);
        assert_eq!(fixed, Some(0usize));
    }

    #[test]
    fn default_initial_positions_overlap_cell_centers_at_symmetry_center() {
        let data = testcase_data(true);
        let params = PlacementConfig::default();
        let mut placer = Placer::from_data(data, params).expect("placer init");

        placer.init_positions();

        let n = placer.model.data.num_cells;
        let cx = 0.5 * (placer.model.data.boundary_xl + placer.model.data.boundary_xh);
        let cy = 0.5 * (placer.model.data.boundary_yl + placer.model.data.boundary_yh);
        for i in 0..n {
            let cell_cx = placer.pos[i] + 0.5 * placer.model.data.cell_widths[i];
            let cell_cy = placer.pos[i + n] + 0.5 * placer.model.data.cell_heights[i];
            assert!((cell_cx - cx).abs() < 1e-9);
            assert!((cell_cy - cy).abs() < 1e-9);
        }
    }

    #[test]
    fn randomize_initial_placement_keeps_seeded_random_spread() {
        let data = testcase_data(true);
        let mut params = PlacementConfig::default();
        params.randomize_initial_placement = true;
        params.random_seed = Some(123);
        let mut placer = Placer::from_data(data, params).expect("placer init");

        placer.init_positions();

        let n = placer.model.data.num_cells;
        let first_cx = placer.pos[0] + 0.5 * placer.model.data.cell_widths[0];
        let first_cy = placer.pos[n] + 0.5 * placer.model.data.cell_heights[0];
        let any_different_center = (1..n).any(|i| {
            let cell_cx = placer.pos[i] + 0.5 * placer.model.data.cell_widths[i];
            let cell_cy = placer.pos[i + n] + 0.5 * placer.model.data.cell_heights[i];
            (cell_cx - first_cx).abs() > 1e-9 || (cell_cy - first_cy).abs() > 1e-9
        });
        assert!(any_different_center);
    }

    #[test]
    fn test_deoverlap_sym_pair_area_is_sum() {
        let data = testcase_data(false);
        let params = PlacementConfig::default();
        let placer = Placer::from_data(data, params).expect("placer init");

        let n = placer.model.data.num_cells;
        let w_feas: Vec<f64> = (0..n)
            .map(|i| placer.model.data.cell_widths_keepout[i])
            .collect();
        let h_feas: Vec<f64> = (0..n)
            .map(|i| placer.model.data.cell_heights_keepout[i])
            .collect();
        let (objects, cell_to_obj) = placer.build_deoverlap_objects(&w_feas, &h_feas);

        let mut validated = false;
        for i in 0..placer.model.data.num_sym_pairs {
            let a = placer.model.data.sym_pairs[2 * i] as usize;
            let b = placer.model.data.sym_pairs[2 * i + 1] as usize;
            if a >= n || b >= n || a == b {
                continue;
            }
            let oid_a = cell_to_obj[a];
            let oid_b = cell_to_obj[b];
            if oid_a == usize::MAX || oid_a != oid_b {
                continue;
            }
            let obj = &objects[oid_a];
            if obj.kind != DeoverlapObjectKind::Pair {
                continue;
            }
            let expected = w_feas[a] * h_feas[a] + w_feas[b] * h_feas[b];
            assert!((obj.area - expected).abs() < 1e-12);
            validated = true;
            break;
        }
        assert!(
            validated,
            "no non-self symmetry pair object found for validation"
        );
    }

    #[test]
    fn test_cell_rect_with_dims_is_centered_on_real_bbox() {
        let data = testcase_data(false);
        let params = PlacementConfig::default();
        let mut placer = Placer::from_data(data, params).expect("placer init");
        let n = placer.model.data.num_cells;
        assert!(n > 0);
        let i = 0usize;
        placer.pos[i] = 100.0;
        placer.pos[i + n] = 200.0;
        placer.model.data.cell_widths[i] = 10.0;
        placer.model.data.cell_heights[i] = 20.0;
        let mut w = vec![10.0; n];
        let mut h = vec![20.0; n];
        w[i] = 1310.0;
        h[i] = 1320.0;
        let (x0, y0, x1, y1) = placer.cell_rect_with_dims(i, &w, &h);
        assert!((x0 - (100.0 - 650.0)).abs() < 1e-9);
        assert!((y0 - (200.0 - 650.0)).abs() < 1e-9);
        assert!((x1 - (100.0 + 10.0 + 650.0)).abs() < 1e-9);
        assert!((y1 - (200.0 + 20.0 + 650.0)).abs() < 1e-9);
    }

    #[test]
    fn test_deoverlap_elliptical_distance_prefers_long_axis() {
        let ctx_wide = DeoverlapSearchContext {
            w_ref: 100.0,
            h_ref: 50.0,
            a_ref: 5000.0,
            w_tail: 100.0,
            h_tail: 50.0,
            r_raw: 2.0,
            r_eff: 2.0,
            long_axis: Axis::X,
            lambda_expand: 1.0,
            lambda_area: 1.0,
            lambda_topo: 0.0,
            rho_max: 0.15,
            use_elliptical: true,
        };
        let dx_long = Placer::elliptical_distance(10.0, 0.0, &ctx_wide);
        let dy_short = Placer::elliptical_distance(0.0, 10.0, &ctx_wide);
        assert!(dx_long < dy_short);

        let ctx_tall = DeoverlapSearchContext {
            r_raw: 0.5,
            r_eff: 0.5,
            long_axis: Axis::Y,
            ..ctx_wide
        };
        let dy_long = Placer::elliptical_distance(0.0, 10.0, &ctx_tall);
        let dx_short = Placer::elliptical_distance(10.0, 0.0, &ctx_tall);
        assert!(dy_long < dx_short);
    }

    #[test]
    fn test_deoverlap_expand_terms_and_area_cap() {
        let ctx = DeoverlapSearchContext {
            w_ref: 100.0,
            h_ref: 50.0,
            a_ref: 5000.0,
            w_tail: 100.0,
            h_tail: 50.0,
            r_raw: 2.0,
            r_eff: 2.0,
            long_axis: Axis::X,
            lambda_expand: 1.0,
            lambda_area: 1.0,
            lambda_topo: 0.0,
            rho_max: 0.15,
            use_elliptical: true,
        };
        let (e_w, e_h, e_a, e_expand, area_inc) =
            Placer::compute_expand_terms(&ctx, 100.0, 60.0, 6000.0);
        assert!((e_w - 0.0).abs() < 1e-12);
        assert!(e_h > 0.0);
        assert!(e_a > 0.0);
        assert!(e_expand > 0.0);
        assert!((area_inc - 0.2).abs() < 1e-12);
        assert!(area_inc > ctx.rho_max);
    }

    #[test]
    fn test_deoverlap_candidate_tie_break_stable() {
        let a = DeoverlapCandidateEval {
            score: 1.0,
            dx: 3.0,
            dy: 4.0,
            ring_hit: 2,
            ..DeoverlapCandidateEval::default()
        };
        let b = DeoverlapCandidateEval {
            score: 1.0,
            dx: 6.0,
            dy: 8.0,
            ring_hit: 1,
            ..DeoverlapCandidateEval::default()
        };
        assert_eq!(Placer::better_deoverlap_candidate(&a, &b), Ordering::Less);
    }

    #[test]
    fn test_placer_creation() {
        // This test would need a real CircuitDb
        // For now it only verifies that it compiles
        let db = CircuitDb::new();
        let params = PlacementConfig::default();

        // Create the placer (an empty DB may fail, or succeed with empty data)
        let result = Placer::new(&db, params);
        // If it succeeds there should be no cells
        if let Ok(placer) = result {
            assert_eq!(placer.model.data.num_cells, 0);
        }
        // Failing is acceptable too
    }

    // ---- LCR upgrade-retry -----------------------------------------------------
    //
    // Whether a real case leaves an unresolved conflict after the discrete
    // de-overlap pass depends on the case, and the fixtures here are far too small
    // to reproduce one, so the triggered path is exercised with a synthetic fixture
    // instead: shrink the LCR
    // ring search to ~nothing, which is the fixture equivalent of "every candidate
    // position is blocked by another device". The tail must then open the gap by
    // itself, which is exactly the mechanism the retry loop leans on.

    /// Clearance requirement that makes the shared fixture's LCR fail.
    const LCR_TEST_INFL: f64 = 1.6;

    /// Small but *ramped* multi-phase settings: the tail halo ramps over
    /// `tail_infl_ramp_iters`, so a fixture with only a handful of tail
    /// iterations never inflates at all and cannot exercise the retry lever.
    fn lcr_retry_params(deoverlap_infl: f64) -> PlacementConfig {
        let mut params = PlacementConfig::default();
        // Small, but the tail halo still ramps to full inside the tail -- a fixture
        // with fewer tail iterations than `tail_infl_ramp_iters` never inflates
        // and cannot exercise the retry lever at all. Kept cheap on purpose: these run
        // in the default debug profile.
        params.max_iterations = 40;
        params.tail_iters = 45;
        params.tail_infl_ramp_iters = 15;
        // Shrink the de-overlap ring search as well; at the stock 50/5000 it probes
        // ~40k candidates per attempt, which dominates this fixture's debug runtime.
        params.tail_deoverlap_grid_fine = 200.0;
        params.tail_deoverlap_ring_fine = 2000.0;
        params.tail_deoverlap_grid_coarse = 800.0;
        params.tail_deoverlap_ring_coarse = 8000.0;
        params.enable_gp_tail = true;
        params.gp_controller_mode = "multiphase".to_string();
        params.use_ap_wl = false;
        params.tail_deoverlap_enable = true;
        // Raise the clearance the de-overlap pass demands until its bounded local
        // search can no longer satisfy it -- the fixture stand-in for "resolving this
        // would mean crossing over another device".
        params.tail_deoverlap_infl = deoverlap_infl;
        params
    }

    #[test]
    fn test_lcr_retry_fires_exactly_when_lcr_failed() {
        // The contract: the loop leaves no trace unless round-0 LCR actually failed.
        // Asserted as an iff so it cannot go stale with the fixture's density.
        //
        // The "converged -> untouched" half is what the byte gate proves for real:
        // every example case reports `overlap_pairs_after == 0`, and their
        // placement JSON plus GDS geometry are unchanged by this feature.
        let data = testcase_data(false);
        let params = lcr_retry_params(1.05);
        let mut placer = Placer::from_data(data, params).expect("placer init");
        placer.run().expect("multiphase run");

        let round0_failed = placer.lcr_retry_summary.is_some();
        if !round0_failed {
            // LCR converged on its first try *and* crossed nothing: nothing may have
            // been touched.
            assert_eq!(placer.lcr_retry_round, 0);
            assert!(placer.lcr_retry_infl.iter().all(|v| *v == 1.0));
            assert!(placer.deoverlap_unresolved_cells().is_empty());
            assert!(placer.deoverlap_crossings.is_empty());
            assert_eq!(
                placer
                    .deoverlap_summary
                    .as_ref()
                    .expect("summary")
                    .overlap_pairs_after,
                0
            );
        } else {
            // The loop ran, so round 0 must have hit one of the two triggers: an
            // unresolved conflict, or a move that only got out by landing on the other
            // side of a local neighbour.
            let s = placer.lcr_retry_summary.as_ref().expect("summary");
            assert!(
                s.overlap_pairs_after_by_round[0] > 0
                    || s.crossing_events_by_round.first().copied().unwrap_or(0) > 0
            );
            assert!(s.rounds_run >= 1);
        }
    }

    #[test]
    fn test_lcr_retry_fires_and_inflates_only_conflicting_devices() {
        let data = testcase_data(false);
        let mut placer =
            Placer::from_data(data, lcr_retry_params(LCR_TEST_INFL)).expect("placer init");
        placer.run().expect("multiphase run");

        let summary = placer
            .lcr_retry_summary
            .as_ref()
            .expect("crippled LCR search must leave a conflict and fire the retry");
        assert!(summary.enabled);
        assert!(summary.rounds_run >= 1, "at least one retry pass must run");
        assert!(
            summary.rounds_run <= summary.max_rounds,
            "round cap honoured"
        );
        assert_eq!(
            summary.overlap_pairs_after_by_round.len(),
            summary.rounds_run + 1,
            "one residual reading per pass, round 0 included"
        );
        assert!(
            summary.overlap_pairs_after_by_round[0] > 0,
            "round 0 must have been a failure, else the loop should not have fired"
        );

        // Only conflicting devices are inflated, and every factor is a power of the
        // configured step -- growth is one-directional, never a decrease.
        assert!(!summary.inflated_cells.is_empty());
        assert!(
            summary.inflated_cells.len() <= placer.model.data.num_cells,
            "cannot inflate more cells than exist"
        );
        // Every inflated cell must be one the de-overlap pass actually flagged; the
        // rule is "only the conflicting devices", never a blanket bump.
        for (cell, _) in &summary.inflated_cells {
            assert!(
                placer.model.data.cell_widths_keepout[*cell] > 0.0,
                "cell {cell} is not overlap-active and must never be inflated"
            );
        }
        let step = summary.infl_step;
        for (cell, factor) in &summary.inflated_cells {
            assert!(*factor > 1.0, "cell {cell} factor must have grown");
            assert!(
                *factor <= step.powi(summary.max_rounds as i32) + 1e-9,
                "cell {cell} factor {factor} exceeds the round cap"
            );
        }
        for (i, v) in placer.lcr_retry_infl.iter().enumerate() {
            assert!(*v >= 1.0, "cell {i} inflation went backwards");
        }
    }

    #[test]
    fn test_lcr_retry_is_deterministic() {
        let run = || {
            let data = testcase_data(false);
            let mut placer =
                Placer::from_data(data, lcr_retry_params(LCR_TEST_INFL)).expect("placer init");
            placer.run().expect("multiphase run");
            let s = placer.lcr_retry_summary.clone().expect("summary");
            (
                s.rounds_run,
                s.converged,
                s.overlap_pairs_after_by_round.clone(),
                s.inflated_cells.clone(),
                placer
                    .deoverlap_end_positions()
                    .expect("deoverlap positions")
                    .to_vec(),
            )
        };
        let a = run();
        let b = run();
        assert_eq!(a.0, b.0);
        assert_eq!(a.1, b.1);
        assert_eq!(a.2, b.2);
        assert_eq!(a.3, b.3);
        assert_eq!(
            a.4, b.4,
            "retried placement must be reproducible bit-for-bit"
        );
    }

    #[test]
    fn test_lcr_retry_respects_disable_switch() {
        let data = testcase_data(false);
        let mut params = lcr_retry_params(LCR_TEST_INFL);
        params.lcr_retry_enable = false;
        let mut placer = Placer::from_data(data, params).expect("placer init");
        placer.run().expect("multiphase run");
        assert!(
            placer
                .deoverlap_summary
                .as_ref()
                .expect("summary")
                .overlap_pairs_after
                > 0,
            "fixture precondition: LCR still fails"
        );
        assert!(
            placer.lcr_retry_summary.is_none(),
            "disabled retry must not run"
        );
        assert_eq!(placer.lcr_retry_round, 0);
    }

    #[test]
    fn test_lcr_retry_round_cap_terminates() {
        // A cap of 1 must produce exactly one extra pass and then give up rather than
        // spinning -- the guard against a pathological case.
        let data = testcase_data(false);
        let mut params = lcr_retry_params(LCR_TEST_INFL);
        params.lcr_retry_max_rounds = 1;
        let mut placer = Placer::from_data(data, params).expect("placer init");
        placer.run().expect("multiphase run");
        let s = placer.lcr_retry_summary.as_ref().expect("summary");
        assert_eq!(s.max_rounds, 1);
        assert!(s.rounds_run <= 1);
        assert_eq!(s.overlap_pairs_after_by_round.len(), s.rounds_run + 1);
    }

    #[test]
    fn test_lcr_retry_infl_step_must_be_greater_than_one() {
        let mut params = PlacementConfig::default();
        params.lcr_retry_infl_step = 1.0;
        let err = params.validate().expect_err("step of 1 must be rejected");
        assert!(format!("{err}").contains("lcr_retry_infl_step"));
    }

    // ---- Crossing trigger: "how much of another device did it sweep over" ------

    #[test]
    fn an_axis_aligned_sweep_is_the_bounding_box_of_the_two_boxes() {
        // 100x100 body sliding right by 300: the swept region is 400x100 and it covers
        // exactly the part of a neighbour that lies in the corridor.
        let start = (0.0, 0.0, 100.0, 100.0);
        let end = (300.0, 0.0, 400.0, 100.0);
        let hull = Placer::swept_hull(start, end);
        assert!((Placer::polygon_area(&hull) - 400.0 * 100.0).abs() < 1e-6);
        // A 100x100 device sitting squarely in the corridor is swept completely.
        let f = Placer::swept_coverage_fraction(start, end, (150.0, 0.0, 250.0, 100.0));
        assert!((f - 1.0).abs() < 1e-9, "{f}");
        // Half of it out of the corridor in y -> half covered.
        let f = Placer::swept_coverage_fraction(start, end, (150.0, 50.0, 250.0, 150.0));
        assert!((f - 0.5).abs() < 1e-9, "{f}");
        // Clear of the path -> nothing, which is why the predicate needs no locality
        // rule of its own.
        let f = Placer::swept_coverage_fraction(start, end, (150.0, 500.0, 250.0, 600.0));
        assert_eq!(f, 0.0);
        // Touching the corridor edge only -> zero area.
        let f = Placer::swept_coverage_fraction(start, end, (150.0, 100.0, 250.0, 200.0));
        assert_eq!(f, 0.0);
    }

    #[test]
    fn a_diagonal_sweep_is_the_hull_not_the_bounding_box() {
        // The corner triangles of the start-union-end bounding box are NOT swept: this
        // is the over-reporting the earlier candidate definitions had, and the hull
        // removes it.
        let start = (0.0, 0.0, 100.0, 100.0);
        let end = (300.0, 300.0, 400.0, 400.0);
        let hull = Placer::swept_hull(start, end);
        let bbox = 400.0 * 400.0;
        let area = Placer::polygon_area(&hull);
        assert!(
            area < bbox,
            "hull {area} must be smaller than the bbox {bbox}"
        );
        // Minkowski sum of a 100x100 square with a (300,300) segment:
        // 100*100 + 300*100 + 300*100 + 300*300/... -- check against the exact value,
        // which for a square and a 45-degree segment is w*h + |dx|*h + |dy|*w - |dx*dy|
        // ... simplest exact check: the hexagon area = square + 2 * (parallelogram
        // halves) = w*h + |dx|*h ... verify numerically against a fine grid instead.
        let mut hits = 0u32;
        let n = 400;
        for i in 0..n {
            for j in 0..n {
                let x = 400.0 * (i as f64 + 0.5) / n as f64;
                let y = 400.0 * (j as f64 + 0.5) / n as f64;
                // point is swept iff (x,y) - t*(300,300) lies in the start box for some
                // t in [0,1]
                let t_lo = ((x - 100.0) / 300.0).max((y - 100.0) / 300.0).max(0.0);
                let t_hi = (x / 300.0).min(y / 300.0).min(1.0);
                if t_lo <= t_hi {
                    hits += 1;
                }
            }
        }
        let grid_area = hits as f64 * (400.0 / n as f64).powi(2);
        assert!(
            (area - grid_area).abs() / grid_area < 0.01,
            "hull {area} vs sampled {grid_area}"
        );
        // A device in the bbox corner the body never passes through: zero.
        let f = Placer::swept_coverage_fraction(start, end, (330.0, 10.0, 390.0, 70.0));
        assert_eq!(f, 0.0, "the bbox corner is not swept");
    }

    #[test]
    fn coverage_is_a_fraction_of_the_crossed_device_not_of_the_sweep() {
        // Same sweep, two devices of different size sitting fully inside it: both are
        // fully covered, so the measure does not favour small devices.
        let start = (0.0, 0.0, 400.0, 400.0);
        let end = (0.0, 2000.0, 400.0, 2400.0);
        for b in [(100.0, 900.0, 200.0, 1000.0), (10.0, 900.0, 390.0, 1500.0)] {
            let f = Placer::swept_coverage_fraction(start, end, b);
            assert!((f - 1.0).abs() < 1e-9, "{b:?} -> {f}");
        }
        // A device the sweep only clips at its edge stays well under any threshold.
        let f = Placer::swept_coverage_fraction(start, end, (380.0, 900.0, 780.0, 1000.0));
        assert!((f - 0.05).abs() < 1e-9, "{f}");
    }

    /// Four devices: a mirror pair (0, 1) about a vertical axis plus two standalone
    /// devices, all 400 x 200, with room to move them anywhere in the fixture.
    fn crossing_fixture_data() -> PlaceData {
        let num_cells = 4;
        PlaceData {
            num_cells,
            num_pins: 0,
            num_nets: 0,
            cell_widths: Array1::from_vec(vec![400.0; num_cells]),
            cell_heights: Array1::from_vec(vec![200.0; num_cells]),
            cell_areas: Array1::from_vec(vec![80000.0; num_cells]),
            cell_widths_keepout: Array1::from_vec(vec![400.0; num_cells]),
            cell_heights_keepout: Array1::from_vec(vec![200.0; num_cells]),
            cell_areas_keepout: Array1::from_vec(vec![80000.0; num_cells]),
            is_pmos_mask: Array1::from_vec(vec![false; num_cells]),
            pin_offset: Array1::zeros(0),
            pin_owner_cell: Array1::zeros(0),
            io_pin_mask: Array1::from_vec(vec![]),
            net_weights: Array1::zeros(0),
            net_pin_offsets: Array1::from_vec(vec![0]),
            net_pin_index: Array1::zeros(0),
            cell_pin_offsets: Array1::from_vec(vec![0, 0, 0, 0, 0]),
            cell_pin_index: Array1::zeros(0),
            overlap_pairs: Array1::from_vec(vec![0, 1]),
            num_overlap_pairs: 1,
            sym_pairs: Array1::from_vec(vec![0, 1]),
            sym_pair_group: Array1::from_vec(vec![0]),
            sym_group_vertical: Array1::from_vec(vec![true]),
            sym_group_names: vec!["G".to_string()],
            num_sym_pairs: 1,
            num_sym_groups: 1,
            sym_self_weight: 1.0,
            sym_anchor_x: Array1::from_vec(vec![200.0; num_cells]),
            sym_anchor_y: Array1::from_vec(vec![100.0; num_cells]),
            well_group_offsets: Array1::zeros(1),
            well_group_index: Array1::zeros(0),
            cell_well_group: Array1::from_vec(vec![-1; num_cells]),
            num_clusters: 0,
            collinear_member_index: Array1::zeros(0),
            collinear_member_offsets: Array1::zeros(1),
            collinear_vertical: Array1::from_vec(vec![]),
            num_collinear_groups: 0,
            boundary_xl: 0.0,
            boundary_yl: 0.0,
            boundary_xh: 100000.0,
            boundary_yh: 100000.0,
        }
    }

    fn crossing_fixture_placer() -> Placer {
        let mut params = PlacementConfig::default();
        params.tail_deoverlap_infl = 1.0;
        params.lcr_honor_drc_floor = false;
        Placer::from_data(crossing_fixture_data(), params).expect("placer")
    }

    /// Put cell `i`'s *centre* at `(cx, cy)`; the fixture's devices are 400 x 200 and
    /// `pos` holds the lower-left corner.
    fn put_centre(p: &mut Placer, i: usize, cx: f64, cy: f64) {
        let n = p.model.data.num_cells;
        p.pos[i] = cx - 200.0;
        p.pos[i + n] = cy - 100.0;
    }

    fn singleton_object(cell: usize) -> DeoverlapObject {
        DeoverlapObject {
            kind: DeoverlapObjectKind::Singleton,
            members: vec![cell],
            area: 80000.0,
            is_vert: None,
            axis: None,
            primary_lo: None,
            primary_hi: None,
        }
    }

    #[test]
    fn a_singleton_that_swept_over_a_device_is_reported_and_the_threshold_decides() {
        let mut placer = crossing_fixture_placer();
        let n = placer.model.data.num_cells;
        put_centre(&mut placer, 0, 1000.0, 50000.0);
        put_centre(&mut placer, 1, 3000.0, 50000.0);
        put_centre(&mut placer, 3, 2000.0, 1000.0); // squarely on the corridor
        put_centre(&mut placer, 2, 3000.0, 1000.0); // the mover, after its move
        let w = vec![400.0; n];
        let h = vec![200.0; n];
        let pre = vec![(1000.0 - 200.0, 1000.0 - 100.0)];
        let scan = placer.collect_move_crossings(
            7,
            2,
            &singleton_object(2),
            &[2],
            &pre,
            &w,
            &h,
            0.5,
            CrossCriterion::Cover,
        );
        assert_eq!(scan.rows.len(), 1, "{:?}", scan.rows);
        assert_eq!(scan.rows[0].crossed_cell, 3);
        assert!(
            (scan.rows[0].cover_frac - 1.0).abs() < 1e-9,
            "device 3 is swept whole: {}",
            scan.rows[0].cover_frac
        );
        assert_eq!(scan.cover_over_25, 1);
        assert_eq!(scan.cover_over_75, 1);
        // A threshold of 1.0 means "only a device buried completely", and strict
        // greater-than keeps exactly-equal out.
        let scan = placer.collect_move_crossings(
            7,
            2,
            &singleton_object(2),
            &[2],
            &pre,
            &w,
            &h,
            1.0,
            CrossCriterion::Cover,
        );
        assert!(
            scan.rows.is_empty(),
            "equal to the threshold is not over it"
        );
    }

    #[test]
    fn a_device_the_sweep_only_grazes_stays_under_the_threshold() {
        // The mover slides past device 3 with only a sliver of overlap in y. It is a
        // neighbour, it is on the path, its centre is even inside the swept box -- and
        // it is still not "flown over", which is the whole point of measuring area.
        let mut placer = crossing_fixture_placer();
        let n = placer.model.data.num_cells;
        put_centre(&mut placer, 0, 1000.0, 50000.0);
        put_centre(&mut placer, 1, 3000.0, 50000.0);
        put_centre(&mut placer, 3, 2000.0, 1180.0); // 20 of its 200 DBU in the corridor
        put_centre(&mut placer, 2, 3000.0, 1000.0);
        let w = vec![400.0; n];
        let h = vec![200.0; n];
        let pre = vec![(1000.0 - 200.0, 1000.0 - 100.0)];
        let scan = placer.collect_move_crossings(
            0,
            2,
            &singleton_object(2),
            &[2],
            &pre,
            &w,
            &h,
            0.5,
            CrossCriterion::Cover,
        );
        assert!(scan.rows.is_empty(), "{:?}", scan.rows);
        assert_eq!(scan.covered_any, 1, "it was touched");
        assert_eq!(scan.cover_over_25, 0);
        assert_eq!(
            scan.order_reversed, 1,
            "the superseded predicate would have fired here"
        );
    }

    #[test]
    fn a_symmetry_pair_is_judged_body_by_body() {
        // The pair is one rigid object but two devices; each member sweeps on its own,
        // and the pair's own members are never "another device".
        let mut placer = crossing_fixture_placer();
        let n = placer.model.data.num_cells;
        put_centre(&mut placer, 0, 2500.0, 1000.0);
        put_centre(&mut placer, 1, 7500.0, 1000.0);
        put_centre(&mut placer, 2, 3500.0, 1000.0);
        put_centre(&mut placer, 3, 6500.0, 1000.0);
        let w = vec![400.0; n];
        let h = vec![200.0; n];
        let object = DeoverlapObject {
            kind: DeoverlapObjectKind::Pair,
            members: vec![0, 1],
            area: 160000.0,
            is_vert: Some(true),
            axis: Some(5000.0),
            primary_lo: Some(0),
            primary_hi: Some(1),
        };
        let pre = vec![
            (4500.0 - 200.0, 1000.0 - 100.0),
            (5500.0 - 200.0, 1000.0 - 100.0),
        ];
        let rows = placer
            .collect_move_crossings(
                0,
                3,
                &object,
                &[0, 1],
                &pre,
                &w,
                &h,
                0.5,
                CrossCriterion::Cover,
            )
            .rows;
        assert_eq!(rows.len(), 2, "one event per member: {rows:?}");
        assert_eq!((rows[0].moved_cell, rows[0].crossed_cell), (0, 2));
        assert_eq!((rows[1].moved_cell, rows[1].crossed_cell), (1, 3));
        assert!(rows.iter().all(|r| r.object_kind == "sym_pair"));
        assert!(rows
            .iter()
            .all(|r| r.crossed_cell != 0 && r.crossed_cell != 1));
    }

    // ---- the "ring around a hole" mechanism probe --------------------------------
    //
    // Maintainer's question (2026-09-22): put 4-8 devices in a ring, drop one more in
    // the middle where the hole is slightly too small for it, and see whether local
    // conflict resolution *flies that device out of the ring* to a legal spot outside,
    // or fails and leaves the retry loop to enlarge the inflation. Then take one ring
    // device away on a side, and one on a corner, and watch it change.
    //
    // The abstract case format has no initial-placement input, so an end-to-end
    // case cannot stage that picture — the
    // optimizer picks the layout. Here the picture is staged exactly: positions are set
    // by hand and the de-overlap pass is run on them, which is the stage the question is
    // about.

    /// `n` ring devices of `ring` x `ring` on a circle of radius `r` about (0,0), plus
    /// one `core` x `core` device at the centre; `drop` removes ring slots by index.
    /// Cell 0 is CORE, cells 1.. are the ring in slot order.
    fn ring_fixture(n: usize, drop: &[usize], ring: f64, core: f64, r: f64) -> PlaceData {
        assert!(core > 0.0 && ring > 0.0 && r > 0.0);
        let kept: Vec<usize> = (0..n).filter(|k| !drop.contains(k)).collect();
        let num_cells = kept.len() + 1;
        let mut w = vec![core];
        let mut h = vec![core];
        for _ in &kept {
            w.push(ring);
            h.push(ring);
        }
        let mut pairs = Vec::new();
        for i in 0..num_cells {
            for j in (i + 1)..num_cells {
                pairs.push(i as i32);
                pairs.push(j as i32);
            }
        }
        let npairs = pairs.len() / 2;
        let areas: Vec<f64> = w.iter().zip(&h).map(|(a, b)| a * b).collect();
        PlaceData {
            num_cells,
            num_pins: 0,
            num_nets: 0,
            cell_widths: Array1::from_vec(w.clone()),
            cell_heights: Array1::from_vec(h.clone()),
            cell_areas: Array1::from_vec(areas.clone()),
            cell_widths_keepout: Array1::from_vec(w),
            cell_heights_keepout: Array1::from_vec(h),
            cell_areas_keepout: Array1::from_vec(areas),
            is_pmos_mask: Array1::from_vec(vec![false; num_cells]),
            pin_offset: Array1::zeros(0),
            pin_owner_cell: Array1::zeros(0),
            io_pin_mask: Array1::from_vec(vec![]),
            net_weights: Array1::zeros(0),
            net_pin_offsets: Array1::from_vec(vec![0]),
            net_pin_index: Array1::zeros(0),
            cell_pin_offsets: Array1::from_vec(vec![0; num_cells + 1]),
            cell_pin_index: Array1::zeros(0),
            overlap_pairs: Array1::from_vec(pairs),
            num_overlap_pairs: npairs,
            sym_pairs: Array1::zeros(0),
            sym_pair_group: Array1::zeros(0),
            sym_group_vertical: Array1::from_vec(vec![]),
            sym_group_names: vec![],
            num_sym_pairs: 0,
            num_sym_groups: 0,
            sym_self_weight: 1.0,
            sym_anchor_x: Array1::zeros(num_cells),
            sym_anchor_y: Array1::zeros(num_cells),
            well_group_offsets: Array1::zeros(1),
            well_group_index: Array1::zeros(0),
            cell_well_group: Array1::from_vec(vec![-1; num_cells]),
            num_clusters: 0,
            collinear_member_index: Array1::zeros(0),
            collinear_member_offsets: Array1::zeros(1),
            collinear_vertical: Array1::from_vec(vec![]),
            num_collinear_groups: 0,
            boundary_xl: -10.0 * r,
            boundary_yl: -10.0 * r,
            boundary_xh: 10.0 * r,
            boundary_yh: 10.0 * r,
        }
    }

    /// Stage the ring, run the de-overlap pass, and report what happened.
    /// `(|core| before, |core| after, mean ring radius before, after, moves,
    /// residual pairs, crossing events, (touched, >0.25, >0.50, >0.75, order-inverted))`.
    #[allow(clippy::type_complexity)]
    fn run_ring_probe(
        n: usize,
        drop: &[usize],
        cover_frac: f64,
        criterion: &str,
    ) -> (
        f64,
        f64,
        f64,
        f64,
        usize,
        usize,
        usize,
        (usize, usize, usize, usize, usize),
    ) {
        // Size the picture so it is the maintainer's: the ring devices sit shoulder to
        // shoulder without overlapping each other (chord >= side), and the hole they
        // enclose is 100 DBU per side too small for CORE. Both follow from `n`.
        let ring = 2000.0;
        let r = 1.02 * ring / (2.0 * (std::f64::consts::PI / n as f64).sin());
        let hole_half = r - 0.5 * ring;
        let core = 2.0 * (hole_half + 100.0);
        let data = ring_fixture(n, drop, ring, core, r);
        let mut params = PlacementConfig::default();
        params.tail_deoverlap_enable = true;
        params.tail_deoverlap_infl = 1.0;
        params.lcr_honor_drc_floor = false;
        params.lcr_retry_cross_cover_frac = cover_frac;
        params.lcr_retry_cross_criterion = criterion.to_string();
        let mut placer = Placer::from_data(data, params).expect("placer");
        let nc = placer.model.data.num_cells;
        // CORE at the origin; the ring devices on their slots. `pos` is lower-left.
        placer.pos[0] = -0.5 * core;
        placer.pos[nc] = -0.5 * core;
        let kept: Vec<usize> = (0..n).filter(|k| !drop.contains(k)).collect();
        for (i, &k) in kept.iter().enumerate() {
            let a = 2.0 * std::f64::consts::PI * k as f64 / n as f64;
            placer.pos[i + 1] = r * a.cos() - 0.5 * ring;
            placer.pos[i + 1 + nc] = r * a.sin() - 0.5 * ring;
        }
        let mean_ring = |p: &Placer| -> f64 {
            let mut acc = 0.0;
            for i in 1..nc {
                let c = p.cell_centre(i);
                acc += c.0.hypot(c.1);
            }
            acc / (nc - 1) as f64
        };
        let before = placer.cell_centre(0);
        let ring_before = mean_ring(&placer);
        placer.discrete_deoverlap_after_tail();
        if let Some(p) = placer.deoverlap_end_pos.clone() {
            placer.pos = p;
        }
        let after = placer.cell_centre(0);
        let s = placer.deoverlap_summary.as_ref().expect("summary");
        (
            before.0.hypot(before.1),
            after.0.hypot(after.1),
            ring_before,
            mean_ring(&placer),
            s.move_count,
            s.overlap_pairs_after,
            placer.deoverlap_crossings.len(),
            placer.deoverlap_crossing_stats,
        )
    }

    #[test]
    fn a_device_boxed_in_by_a_ring_is_not_flown_out_the_ring_is_pushed_open() {
        // The maintainer's worry was that the boxed-in device would be relocated out
        // through the ring. Measured: it is not. The de-overlap pass keeps the largest
        // object of the component fixed and moves the smaller ones, and here that means
        // the *ring* devices travel outwards while CORE does not move at all.
        //
        // That is the reassuring half of the answer. The unreassuring half is that the
        // ring devices themselves sweep across each other on the way out, which is what
        // the crossing events count.
        for n in [4usize, 5, 6, 8] {
            let (cb, ca, rb, ra, moves, residual, events, st) =
                run_ring_probe(n, &[], 0.5, "either");
            eprintln!(
                "[ring] n={n} closed: |core| {cb:.0} -> {ca:.0}, mean ring r {rb:.0} -> \
                 {ra:.0}, moves={moves}, residual={residual}, crossings@0.5={events}, \
                 touched={} c25={} c50={} c75={} order_inverted={}",
                st.0, st.1, st.2, st.3, st.4
            );
            assert!(
                (ca - cb).abs() < 1.0 || ra > rb + 1.0,
                "n={n}: something has to give -- either CORE leaves or the ring opens"
            );
        }
    }

    #[test]
    fn opening_the_ring_changes_what_local_conflict_resolution_can_do() {
        // One ring device removed: slot 0 is on the +x axis (a "side" of the ring),
        // slot 1 is 45 degrees round from it (a "corner" of an 8-ring). Reported at the
        // three thresholds under measurement. Only determinism is asserted -- which way
        // the pass goes is the behaviour to read off, not something to freeze.
        for (label, drop) in [("side", vec![0usize]), ("corner", vec![1usize])] {
            for cover in [0.25f64, 0.5, 0.75] {
                let a = run_ring_probe(8, &drop, cover, "either");
                let b = run_ring_probe(8, &drop, cover, "either");
                eprintln!(
                    "[ring] n=8 {label}-open cover={cover}: |core| {:.0} -> {:.0}, mean \
                     ring r {:.0} -> {:.0}, moves={}, residual={}, crossings={}, \
                     touched={} c25={} c50={} c75={} order_inverted={}",
                    a.0, a.1, a.2, a.3, a.4, a.5, a.6, a.7 .0, a.7 .1, a.7 .2, a.7 .3, a.7 .4
                );
                assert_eq!(a, b, "the probe must be reproducible");
            }
        }
    }

    #[test]
    fn the_threshold_decides_whether_flying_out_of_the_ring_is_reported() {
        // Same geometry, three thresholds, coverage predicate: the report must be
        // monotone in the threshold
        // (a stricter threshold can only report fewer events), and the placement itself
        // must not depend on it at all -- the trigger observes the pass, it does not
        // steer it.
        let mut last = usize::MAX;
        let mut geom = None;
        for cover in [0.25f64, 0.5, 0.75] {
            let (cb, ca, rb, ra, moves, residual, events, st) =
                run_ring_probe(8, &[], cover, "cover");
            eprintln!(
                "[ring] n=8 closed cover={cover}: |core| {cb:.0} -> {ca:.0}, mean ring r \
                 {rb:.0} -> {ra:.0}, moves={moves}, residual={residual}, \
                 crossings={events}, order_inverted={}",
                st.4
            );
            assert!(events <= last, "a stricter threshold reported more events");
            last = events;
            let g = (cb, ca, rb, ra, moves, residual);
            assert!(
                geom.is_none() || geom == Some(g),
                "the threshold changed the placement, which it must never do"
            );
            geom = Some(g);
        }
    }

    #[test]
    fn the_two_predicates_catch_different_things_and_either_is_their_union() {
        // Device 3 is ploughed over (coverage 1.0, order also inverted); device 1 is
        // slipped past inside the conflict component without being covered at all.
        // `order` and `cover` therefore report different sets, and `either` their union.
        let mut placer = crossing_fixture_placer();
        let n = placer.model.data.num_cells;
        put_centre(&mut placer, 0, 1000.0, 50000.0);
        put_centre(&mut placer, 1, 2000.0, 1400.0); // slipped past: 200 DBU clear in y
        put_centre(&mut placer, 3, 2000.0, 1000.0); // ploughed over
        put_centre(&mut placer, 2, 3000.0, 1000.0);
        let w = vec![400.0; n];
        let h = vec![200.0; n];
        let pre = vec![(1000.0 - 200.0, 1000.0 - 100.0)];
        let ask = |c: CrossCriterion| -> Vec<usize> {
            placer
                .collect_move_crossings(0, 2, &singleton_object(2), &[1, 2], &pre, &w, &h, 0.5, c)
                .rows
                .iter()
                .map(|r| r.crossed_cell)
                .collect()
        };
        assert_eq!(
            ask(CrossCriterion::Cover),
            vec![3],
            "coverage sees the plough"
        );
        assert_eq!(
            ask(CrossCriterion::Order),
            vec![1, 3],
            "order sees the slip-past as well"
        );
        assert_eq!(
            ask(CrossCriterion::Either),
            vec![1, 3],
            "either is the union"
        );
    }

    #[test]
    fn the_order_predicate_still_needs_a_locality_rule_but_coverage_does_not() {
        // A device on the far side of the die whose x order the mover inverts is none of
        // the order predicate's business; coverage rejects it for free.
        let mut placer = crossing_fixture_placer();
        let n = placer.model.data.num_cells;
        put_centre(&mut placer, 0, 1000.0, 50000.0);
        put_centre(&mut placer, 1, 9000.0, 90000.0);
        put_centre(&mut placer, 3, 2000.0, 90000.0); // same x band, 89,000 DBU away in y
        put_centre(&mut placer, 2, 3000.0, 1000.0);
        let w = vec![400.0; n];
        let h = vec![200.0; n];
        let pre = vec![(1000.0 - 200.0, 1000.0 - 100.0)];
        for c in [
            CrossCriterion::Order,
            CrossCriterion::Cover,
            CrossCriterion::Either,
        ] {
            let rows = placer
                .collect_move_crossings(0, 2, &singleton_object(2), &[2], &pre, &w, &h, 0.5, c)
                .rows;
            assert!(rows.is_empty(), "{c:?} reported a distant device: {rows:?}");
        }
    }

    #[test]
    fn the_criterion_name_is_validated() {
        let params = PlacementConfig::default();
        assert_eq!(
            params.lcr_retry_cross_criterion, "either",
            "maintainer ruling 2026-09-23: either (order OR cover) is what an enable gets"
        );
        for bad in ["", "Order", "area", "both"] {
            let mut p = PlacementConfig::default();
            p.lcr_retry_cross_criterion = bad.to_string();
            let err = p.validate().expect_err("bad criterion must be rejected");
            assert!(format!("{err}").contains("lcr_retry_cross_criterion"));
        }
        for good in ["order", "cover", "either"] {
            let mut p = PlacementConfig::default();
            p.lcr_retry_cross_criterion = good.to_string();
            p.validate().expect("valid criterion");
        }
    }

    #[test]
    fn crossing_trigger_can_be_switched_off_and_the_threshold_is_validated() {
        let mut params = PlacementConfig::default();
        assert!(
            !params.lcr_retry_on_crossing,
            "off pending a ruling on the predicate -- the coverage one leaves residual \
             overlap on abstract comp, and zero residual overlap is the released \
             version's claim"
        );
        assert_eq!(params.lcr_retry_cross_cover_frac, 0.5);
        params.lcr_retry_on_crossing = true;
        params.validate().expect("switching it on stays valid");

        for bad in [0.0, -0.1, 1.5, f64::NAN] {
            let mut p = PlacementConfig::default();
            p.lcr_retry_cross_cover_frac = bad;
            let err = p.validate().expect_err("threshold {bad} must be rejected");
            assert!(format!("{err}").contains("lcr_retry_cross_cover_frac"));
        }
    }

    #[test]
    fn test_runs_with_adam_mainline() {
        let data = testcase_data(false);
        let mut params = PlacementConfig::default();
        params.max_iterations = 3;
        params.enable_gp_tail = false;
        params.tail_iters = 0;
        params.gp_controller_mode = "multiphase".to_string();
        params.step_strategy = "none".to_string();
        params.learning_rate = 0.01;
        params.use_ap_wl = false;
        // These fixtures run 3-5 iterations on purpose -- they assert that multi-phase runs
        // and emits its artifacts, not that the placement is any good. Such a
        // placement leaves conflicts LCR cannot fix, which now makes the retry
        // loop fire and re-run multi-phase up to `lcrRetryMaxRounds` more times. That
        // is correct behaviour but irrelevant here and dominates the debug runtime, so
        // opt out and keep testing the one thing the fixture is for.
        params.lcr_retry_enable = false;
        let mut placer = Placer::from_data(data, params).expect("placer init");
        let res = placer.run().expect("multiphase adam run");
        assert!(res.num_iters >= 1);
        assert!(!placer.iter_rows().is_empty());
        assert!(placer.run_summary().is_some());
    }

    #[test]
    fn test_runs_with_linesearch_compat() {
        let data = testcase_data(true);
        let mut params = PlacementConfig::default();
        params.max_iterations = 3;
        params.enable_gp_tail = false;
        params.tail_iters = 0;
        params.gp_controller_mode = "multiphase".to_string();
        params.step_strategy = "linesearch".to_string();
        params.ls_step_init = 1.0;
        params.ls_step_shrink = 0.5;
        params.ls_step_min = 1e-4;
        params.use_ap_wl = false;
        // These fixtures run 3-5 iterations on purpose -- they assert that multi-phase runs
        // and emits its artifacts, not that the placement is any good. Such a
        // placement leaves conflicts LCR cannot fix, which now makes the retry
        // loop fire and re-run multi-phase up to `lcrRetryMaxRounds` more times. That
        // is correct behaviour but irrelevant here and dominates the debug runtime, so
        // opt out and keep testing the one thing the fixture is for.
        params.lcr_retry_enable = false;
        let mut placer = Placer::from_data(data, params).expect("placer init");
        let res = placer.run().expect("multiphase linesearch run");
        assert!(res.num_iters >= 1);
        assert!(!placer.iter_rows().is_empty());
        assert!(placer.run_summary().is_some());
    }

    #[test]
    fn test_export_contract_files() {
        let data = testcase_data(false);
        let mut params = PlacementConfig::default();
        params.max_iterations = 5;
        params.tail_iters = 2;
        params.enable_gp_tail = true;
        params.gp_controller_mode = "multiphase".to_string();
        params.use_ap_wl = false;
        // These fixtures run 3-5 iterations on purpose -- they assert that multi-phase runs
        // and emits its artifacts, not that the placement is any good. Such a
        // placement leaves conflicts LCR cannot fix, which now makes the retry
        // loop fire and re-run multi-phase up to `lcrRetryMaxRounds` more times. That
        // is correct behaviour but irrelevant here and dominates the debug runtime, so
        // opt out and keep testing the one thing the fixture is for.
        params.lcr_retry_enable = false;
        let mut placer = Placer::from_data(data, params).expect("placer init");
        placer.run().expect("multiphase run");

        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("time")
            .as_nanos();
        let out_dir = std::env::temp_dir().join(format!("applace_export_contract_{stamp}"));
        placer
            .export_multiphase_artifacts(&out_dir)
            .expect("export artifacts");

        let iter_meta: serde_json::Value = serde_json::from_slice(
            &std::fs::read(out_dir.join("iter_meta.json")).expect("read iter meta"),
        )
        .expect("parse iter meta");
        assert_eq!(
            iter_meta["schemaVersion"].as_u64(),
            Some(crate::gp_controller::OUTPUT_SCHEMA_VERSION as u64)
        );

        let run_summary: serde_json::Value = serde_json::from_slice(
            &std::fs::read(out_dir.join("run_summary.json")).expect("read summary"),
        )
        .expect("parse summary");
        assert_eq!(
            run_summary["schemaVersion"].as_u64(),
            Some(crate::gp_controller::OUTPUT_SCHEMA_VERSION as u64)
        );
        assert!(run_summary["base_end_iter"].is_number());
        assert!(run_summary["tail_end_iter"].is_number());
        assert!(run_summary["selected_final_candidate_label"].is_string());

        let candidate_summary: serde_json::Value = serde_json::from_slice(
            &std::fs::read(out_dir.join("candidate_summary.json")).expect("read candidate summary"),
        )
        .expect("parse candidate summary");
        assert_eq!(
            candidate_summary["schemaVersion"].as_u64(),
            Some(crate::gp_controller::OUTPUT_SCHEMA_VERSION as u64)
        );
        assert!(candidate_summary["candidates"].is_array());

        std::fs::remove_dir_all(out_dir).ok();
    }

    #[test]
    fn test_batch_oracle_apwl_fixed_placement_improves_and_freezes_pos() {
        let data = testcase_data(false);
        let mut params = PlacementConfig::default();
        params.max_iterations = 5;
        params.tail_iters = 0;
        params.enable_gp_tail = false;
        params.gp_controller_mode = "multiphase".to_string();
        params.use_ap_wl = false;
        let mut placer = Placer::from_data(data, params).expect("placer init");
        placer.run().expect("run");
        let pos_before = placer.get_positions();
        let out = placer
            .batch_oracle_apwl_fixed_placement(50)
            .expect("oracle run");
        let init = out["wl_ap_initial"].as_f64().expect("initial");
        let best = out["wl_ap_optimal"].as_f64().expect("optimal");
        assert!(best <= init + 1e-9);
        let pos_after = placer.get_positions();
        assert_eq!(pos_before.len(), pos_after.len());
        for i in 0..pos_before.len() {
            assert!((pos_before[i] - pos_after[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn test_deoverlap_finalize_phi_apwl_fixed_placement_applies_and_freezes_pos() {
        let data = testcase_data(false);
        let mut params = PlacementConfig::default();
        params.max_iterations = 5;
        params.tail_iters = 0;
        params.enable_gp_tail = false;
        params.gp_controller_mode = "multiphase".to_string();
        params.use_ap_wl = false;
        let mut placer = Placer::from_data(data, params).expect("placer init");
        placer.run().expect("run");
        let pos_before = placer.get_positions();
        let out = placer
            .deoverlap_finalize_phi_apwl_fixed_placement(50)
            .expect("deoverlap final phi");
        let init = out["wl_ap_initial"].as_f64().expect("initial");
        let best = out["wl_ap_optimal"].as_f64().expect("optimal");
        assert!(best <= init + 1e-9);
        let phi = placer.current_phi_radians();
        assert_eq!(phi.len(), placer.model.data.num_pins);
        let pos_after = placer.get_positions();
        assert_eq!(pos_before.len(), pos_after.len());
        for i in 0..pos_before.len() {
            assert!((pos_before[i] - pos_after[i]).abs() < 1e-12);
        }
    }
}
