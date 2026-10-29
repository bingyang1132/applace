//! Placement parameters

use crate::error::{PlaceError, PlaceResult};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Path;

/// Every knob the placer reads, in one struct.
///
/// Serialized as camelCase, which is the spelling the parameter files use.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PlacementConfig {
    // === Basic parameters ===
    /// Maximum number of iterations
    pub max_iterations: usize,

    /// Optimization algorithm ("Adam" or "SGD")
    #[serde(default = "default_optimizer")]
    pub optimizer: String,

    // === Adam parameters ===
    /// Adam learning rate
    pub learning_rate: f64,

    /// Adam beta1 parameter
    pub adam_beta1: f64,

    /// Adam beta2 parameter
    pub adam_beta2: f64,

    // === Constraint switches ===
    /// Whether to use LSEWL wirelength constraint
    pub enable_wirelength: bool,

    /// Whether to blend in AP-aware LSEWL wirelength term
    #[serde(default)]
    pub use_ap_wl: bool,

    /// Whether to use overlap penalty
    pub enable_overlap: bool,

    /// Whether to use symmetry constraint
    pub enable_symmetry: bool,

    /// Whether to use boundary constraint
    #[serde(default)]
    pub enable_boundary: bool,

    /// Whether to use Well Cluster constraint
    #[serde(default)]
    pub use_well_cluster: bool,

    /// Whether to use Array constraint
    #[serde(default)]
    pub enable_array: bool,

    /// Whether to use centroid compaction penalty
    #[serde(default)]
    pub use_compaction: bool,

    /// Whether to use area-based movement damping (larger blocks move less)
    #[serde(default)]
    pub use_mass_damping: bool,

    // === LSEWL parameters ===
    /// Initial wirelength gamma
    pub wl_smoothing_init: f64,

    /// Wirelength gamma decay rate
    #[serde(default = "default_wl_gamma_decay")]
    pub wl_gamma_decay: f64,

    /// AP superellipse exponent m (m >= 2)
    #[serde(default = "default_ap_exponent_m")]
    pub ap_exponent_m: f64,

    /// AP superellipse numerical epsilon (> 0)
    #[serde(default = "default_ap_eps")]
    pub ap_eps: f64,

    /// AP stable regularization weight
    #[serde(default = "default_ap_stable_weight")]
    pub ap_stable_weight: f64,

    // === Line Search parameters ===
    /// Step size strategy ("linesearch" or "none")
    #[serde(default = "default_step_strategy")]
    pub step_strategy: String,

    /// Initial step size
    #[serde(default = "default_ls_step_init")]
    pub ls_step_init: f64,

    /// Step size decay factor
    #[serde(default = "default_ls_step_shrink")]
    pub ls_step_shrink: f64,

    /// Minimum step size
    #[serde(default = "default_ls_step_min")]
    pub ls_step_min: f64,

    /// Armijo condition constant
    #[serde(default = "default_armijo_c1")]
    pub armijo_c1: f64,

    // === Bias parameters ===
    /// Bias position unit (multiplier)
    #[serde(default = "default_bias_unit")]
    pub bias_position_unit: f64,

    /// Aspect-ratio boundary inflation (area multiplier)
    #[serde(default = "default_aspect_inflation")]
    pub aspect_inflation: f64,

    /// Boundary penalty weight (multiplier)
    #[serde(default = "default_boundary_weight")]
    pub boundary_weight: f64,

    /// Symmetry self-pair weight multiplier
    #[serde(default = "default_sym_self_weight")]
    pub sym_self_weight: f64,

    /// Compaction penalty weight
    #[serde(default = "default_compaction_weight")]
    pub compaction_weight: f64,

    /// Optional fixed random seed for reproducible initialization
    #[serde(default)]
    pub random_seed: Option<u64>,

    /// Whether to scatter devices at random initially instead of center overlap
    #[serde(default)]
    pub randomize_initial_placement: bool,

    /// Whether to restore the minimum-loss position at the end of optimization
    #[serde(default)]
    pub use_best_solution: bool,

    /// Initial-position jitter ratio applied to cell width/height
    #[serde(default = "default_init_jitter_ratio")]
    pub init_jitter_ratio: f64,

    /// Initial random spread ratio inside placement boundary (0=center, 1=full range)
    #[serde(default = "default_init_spread_ratio")]
    pub init_spread_ratio: f64,

    /// Exponent for area-based movement damping
    #[serde(default = "default_mass_damping_exp")]
    pub mass_damping_exp: f64,

    // === Convergence parameters ===
    /// Gradient norm convergence threshold
    #[serde(default = "default_grad_tol")]
    pub grad_tol: f64,

    /// Loss relative change convergence threshold
    #[serde(default = "default_loss_rtol")]
    pub loss_rtol: f64,

    /// Move RMS threshold relative to boundary diagonal
    #[serde(default = "default_move_rtol")]
    pub move_rtol: f64,

    /// Max move threshold relative to boundary diagonal
    #[serde(default = "default_move_max_rtol")]
    pub move_max_rtol: f64,

    /// Overlap threshold as ratio to total cell area
    #[serde(default = "default_overlap_target")]
    pub overlap_target: f64,

    /// Symmetry threshold (normalized by total width square)
    #[serde(default = "default_symmetry_target")]
    pub symmetry_target: f64,

    /// Boundary threshold (normalized by total area)
    #[serde(default = "default_boundary_target")]
    pub boundary_target: f64,

    /// Convergence check window size
    #[serde(default = "default_convergence_window")]
    pub convergence_window: usize,

    /// Enable tail-stage refinement after base GP.
    #[serde(default = "default_enable_gp_tail")]
    pub enable_gp_tail: bool,

    /// Tail iterations.
    #[serde(default = "default_tail_iters")]
    pub tail_iters: usize,

    /// EMA alpha for adaptive lambda controller.
    #[serde(default = "default_tail_ema_alpha")]
    pub tail_ema_alpha: f64,

    /// Relative overlap improvement threshold for Push state.
    #[serde(default = "default_tail_tau_ovl_good")]
    pub tail_tau_ovl_good: f64,

    /// Drift hysteresis soft threshold.
    #[serde(default = "default_tail_d_soft")]
    pub tail_d_soft: f64,

    /// Drift hysteresis hard threshold.
    #[serde(default = "default_tail_d_hard")]
    pub tail_d_hard: f64,

    /// Symmetry hysteresis soft threshold.
    #[serde(default = "default_tail_s_soft")]
    pub tail_s_soft: f64,

    /// Symmetry hysteresis hard threshold.
    #[serde(default = "default_tail_s_hard")]
    pub tail_s_hard: f64,

    /// Max lambda growth per update.
    #[serde(default = "default_tail_lambda_growth_cap")]
    pub tail_lambda_growth_cap: f64,

    /// Max lambda decay per update.
    #[serde(default = "default_tail_lambda_decay_cap")]
    pub tail_lambda_decay_cap: f64,

    /// Acceptance overlap improvement threshold.
    #[serde(default = "default_tail_tau_accept_ovl")]
    pub tail_tau_accept_ovl: f64,

    /// Rollback min overlap improvement threshold.
    #[serde(default = "default_tail_tau_min")]
    pub tail_tau_min: f64,

    /// Accept drift threshold.
    #[serde(default = "default_tail_d_accept")]
    pub tail_d_accept: f64,

    /// Rollback drift threshold.
    #[serde(default = "default_tail_d_rollback")]
    pub tail_d_rollback: f64,

    /// Accept symmetry threshold.
    #[serde(default = "default_tail_s_accept")]
    pub tail_s_accept: f64,

    /// Rollback symmetry threshold.
    #[serde(default = "default_tail_s_rollback")]
    pub tail_s_rollback: f64,

    /// Tail acceptance window length.
    #[serde(default = "default_tail_window")]
    pub tail_window: usize,

    /// Strong anchor weight for non-conflict cells.
    #[serde(default = "default_tail_anchor_strong")]
    pub tail_anchor_strong: f64,

    /// Mid anchor weight for conflict cells.
    #[serde(default = "default_tail_anchor_mid")]
    pub tail_anchor_mid: f64,

    /// Weak anchor weight for small conflict cells.
    #[serde(default = "default_tail_anchor_weak")]
    pub tail_anchor_weak: f64,

    /// Mobility min clamp.
    #[serde(default = "default_mobility_min")]
    pub mobility_min: f64,

    /// Mobility max clamp.
    #[serde(default = "default_mobility_max")]
    pub mobility_max: f64,

    /// Symmetry-anchor mobility multiplier.
    #[serde(default = "default_mobility_sym_anchor_mult")]
    pub mobility_sym_anchor_mult: f64,

    /// Large-device mobility multiplier.
    #[serde(default = "default_mobility_large_mult")]
    pub mobility_large_mult: f64,

    /// Small-device mobility multiplier.
    #[serde(default = "default_mobility_small_mult")]
    pub mobility_small_mult: f64,

    /// Enable selective inflation in tail stage.
    #[serde(default = "default_tail_selective_inflation")]
    pub tail_selective_inflation: bool,

    /// Inflation increment per step.
    #[serde(default = "default_tail_inflation_step")]
    pub tail_inflation_step: f64,

    /// Inflation max cap.
    #[serde(default = "default_tail_inflation_max")]
    pub tail_inflation_max: f64,

    /// PMOS hard expansion half-width for guard-ring overlap geometry.
    #[serde(default = "default_pmos_hard_expand_for_gr")]
    pub pmos_hard_expand_for_gr: f64,

    /// Safety margin (µm) added to half of `max_pairwise_clean_spacing` when the stage
    /// binary derives the DRC-aware tail inflation floor.
    #[serde(default = "default_tail_inflation_safety_margin")]
    pub tail_inflation_safety_margin: f64,

    /// DRC-aware tail inflation floor, in placement DBU. Resolved by the stage binary
    /// from PDK spacing (`ceil_to_grid(0.5 * max_pairwise_clean_spacing + safety)`);
    /// `0.0` disables the floor. Every overlap-active cell's tail halo is lower-bounded
    /// by this value, so PMOS devices keep their static guard-ring expansion plus this floor.
    #[serde(default = "default_tail_hard_inflation_floor")]
    pub tail_hard_inflation_floor: f64,

    /// Tail diagnostics snapshot interval (JSON every K iter).
    #[serde(default = "default_tail_diag_interval")]
    pub tail_diag_interval: usize,

    /// Whether to generate tail plots automatically.
    #[serde(default = "default_tail_diag_enable_plots")]
    pub tail_diag_enable_plots: bool,

    /// Controller mode. One accepted value, "multiphase".
    #[serde(default = "default_gp_controller_mode")]
    pub gp_controller_mode: String,

    /// Multi-phase: EMA factor for q_o / q_s.
    #[serde(default = "default_rho")]
    pub rho: f64,

    /// Multi-phase: overlap target ratio.
    #[serde(default = "default_tau_o_ratio")]
    pub tau_o_ratio: f64,

    /// Multi-phase: symmetry target ratio.
    #[serde(default = "default_tau_s_ratio")]
    pub tau_s_ratio: f64,

    /// Multi-phase: overlap controller coefficients.
    #[serde(default = "default_a_o")]
    pub a_o: f64,
    #[serde(default = "default_b_o")]
    pub b_o: f64,

    /// Multi-phase: symmetry controller coefficients.
    #[serde(default = "default_a_s")]
    pub a_s: f64,
    #[serde(default = "default_b_s")]
    pub b_s: f64,

    /// Multi-phase: lambda bounds.
    #[serde(default = "default_lambda_ovl_min")]
    pub lambda_ovl_min: f64,
    #[serde(default = "default_lambda_ovl_max")]
    pub lambda_ovl_max: f64,
    #[serde(default = "default_lambda_sym_min")]
    pub lambda_sym_min: f64,
    #[serde(default = "default_lambda_sym_max")]
    pub lambda_sym_max: f64,

    /// Multi-phase: log-domain lambda update magnitudes.
    #[serde(default = "default_s_lambda_base")]
    pub s_lambda_base: f64,
    #[serde(default = "default_s_lambda_tail")]
    pub s_lambda_tail: f64,

    /// Multi-phase: AP gate ratios and lambda growth.
    #[serde(default = "default_tau_gate_hi_ratio")]
    pub tau_gate_hi_ratio: f64,
    #[serde(default = "default_tau_gate_lo_ratio")]
    pub tau_gate_lo_ratio: f64,
    #[serde(default = "default_eta_ap")]
    pub eta_ap: f64,
    #[serde(default = "default_lambda_ap_max")]
    pub lambda_ap_max: f64,

    /// Multi-phase: anchor ratio and bounds.
    #[serde(default = "default_anchor_base_ratio")]
    pub anchor_base_ratio: f64,
    #[serde(default = "default_anchor_tail_ratio")]
    pub anchor_tail_ratio: f64,
    #[serde(default = "default_anchor_min")]
    pub anchor_min: f64,
    #[serde(default = "default_anchor_max_base")]
    pub anchor_max_base: f64,
    #[serde(default = "default_anchor_max_tail")]
    pub anchor_max_tail: f64,
    #[serde(default = "default_anchor_ramp_iters")]
    pub anchor_ramp_iters: usize,

    /// Multi-phase: delayed mobility trigger and ramp.
    #[serde(default = "default_mobility_trigger_ratio")]
    pub mobility_trigger_ratio: f64,
    #[serde(default = "default_mobility_ramp_iters")]
    pub mobility_ramp_iters: usize,

    /// Multi-phase: semantic delta clipping scales.
    #[serde(default = "default_c_delta_base")]
    pub c_delta_base: f64,
    #[serde(default = "default_c_delta_tail")]
    pub c_delta_tail: f64,

    /// Multi-phase: candidate checkpoint stride.
    #[serde(default = "default_candidate_stride")]
    pub candidate_stride: usize,

    /// Multi-phase tail: isotropic inflation ratio relative to median(min(w,h)).
    #[serde(default = "default_tail_infl_ratio")]
    pub tail_infl_ratio: f64,

    /// Multi-phase tail: inflation ramp iters from alpha=0 to alpha=1.
    #[serde(default = "default_tail_infl_ramp_iters")]
    pub tail_infl_ramp_iters: usize,

    /// Multi-phase tail: overlap target ratio under inflated metric.
    #[serde(default = "default_tail_tau_o_ratio")]
    pub tail_tau_o_ratio: f64,

    /// Multi-phase tail: collect tail candidates once alpha >= this threshold.
    #[serde(default = "default_tail_candidate_alpha_start")]
    pub tail_candidate_alpha_start: f64,

    /// Multi-phase tail: tail candidate checkpoint stride.
    #[serde(default = "default_tail_candidate_stride")]
    pub tail_candidate_stride: usize,

    /// Multi-phase tail: dilated reference scale coefficient.
    #[serde(default = "default_tail_ref_dilate_beta")]
    pub tail_ref_dilate_beta: f64,

    /// Multi-phase tail: cover-ratio threshold for severe cover penalty.
    #[serde(default = "default_tail_cover_tau")]
    pub tail_cover_tau: f64,

    /// Multi-phase tail: softplus softness for cover penalty.
    #[serde(default = "default_tail_cover_softness")]
    pub tail_cover_softness: f64,

    /// Multi-phase tail: tail-entry lambda_cover init ratio w.r.t lambda_ovl.
    #[serde(default = "default_tail_cover_lambda_ratio")]
    pub tail_cover_lambda_ratio: f64,

    /// Multi-phase tail de-overlap: enable discrete de-overlap pass after tail selection.
    #[serde(default = "default_tail_deoverlap_enable")]
    pub tail_deoverlap_enable: bool,

    /// Multi-phase tail de-overlap: fine search grid step in placer units.
    #[serde(default = "default_tail_deoverlap_grid_fine")]
    pub tail_deoverlap_grid_fine: f64,

    /// Multi-phase tail de-overlap: fine search max ring in placer units.
    #[serde(default = "default_tail_deoverlap_ring_fine")]
    pub tail_deoverlap_ring_fine: f64,

    /// Multi-phase tail de-overlap: coarse fallback search grid step in placer units.
    #[serde(default = "default_tail_deoverlap_grid_coarse")]
    pub tail_deoverlap_grid_coarse: f64,

    /// Multi-phase tail de-overlap: coarse fallback search max ring in placer units.
    #[serde(default = "default_tail_deoverlap_ring_coarse")]
    pub tail_deoverlap_ring_coarse: f64,

    /// Multi-phase tail de-overlap: fixed inflation ratio used in de-overlap overlap geometry.
    #[serde(default = "default_tail_deoverlap_infl")]
    pub tail_deoverlap_infl: f64,

    /// Multi-phase tail de-overlap: anisotropy clamp kappa for R_eff = clip(R, 1/kappa, kappa).
    #[serde(default = "default_tail_deoverlap_search_kappa")]
    pub tail_deoverlap_search_kappa: f64,

    /// Multi-phase tail de-overlap: global expansion penalty weight.
    #[serde(default = "default_tail_deoverlap_lambda_expand")]
    pub tail_deoverlap_lambda_expand: f64,

    /// Multi-phase tail de-overlap: area term weight inside expansion penalty.
    #[serde(default = "default_tail_deoverlap_lambda_area")]
    pub tail_deoverlap_lambda_area: f64,

    /// Multi-phase tail de-overlap: hard cap for relative global area growth.
    #[serde(default = "default_tail_deoverlap_rho_max")]
    pub tail_deoverlap_rho_max: f64,

    /// Multi-phase tail de-overlap: enable elliptical anisotropic search.
    #[serde(default = "default_tail_deoverlap_use_elliptical_search")]
    pub tail_deoverlap_use_elliptical_search: bool,

    /// Multi-phase tail de-overlap: optional topology term weight (reserved, default 0).
    #[serde(default = "default_tail_deoverlap_lambda_topo")]
    pub tail_deoverlap_lambda_topo: f64,

    /// Multi-phase tail de-overlap: fixed-layout AP-WL phi optimization iters after de-overlap.
    #[serde(default = "default_tail_deoverlap_final_phi_iters")]
    pub tail_deoverlap_final_phi_iters: usize,

    /// Make the discrete de-overlap pass honour the same DRC-derived clearance
    /// floor (`tail_hard_inflation_floor`, from `max_pairwise_clean_spacing`) that the
    /// tail enforces. Without it the de-overlap pass judges feasibility on the
    /// multiplicative `tail_deoverlap_infl` alone and can place a pair closer
    /// than the floor, silently undoing the tail's compliance.
    #[serde(default = "default_lcr_honor_drc_floor")]
    pub lcr_honor_drc_floor: bool,

    /// LCR retry: when the discrete de-overlap pass cannot resolve every
    /// conflict with local moves, re-run the whole multi-phase pass with the halo of the
    /// still-conflicting devices inflated, so the tail itself opens the gap.
    ///
    /// Only fires on failure: a case whose LCR already converges never enters the
    /// loop and its output stays byte-identical.
    #[serde(default = "default_lcr_retry_enable")]
    pub lcr_retry_enable: bool,

    /// LCR retry: maximum number of extra multi-phase passes (0 disables the loop).
    #[serde(default = "default_lcr_retry_max_rounds")]
    pub lcr_retry_max_rounds: usize,

    /// LCR retry: per-round multiplicative step applied to the tail halo of each
    /// still-conflicting device. Must be > 1 (monotone, one-directional growth).
    #[serde(default = "default_lcr_retry_infl_step")]
    pub lcr_retry_infl_step: f64,

    /// LCR retry, second trigger: also re-run when the de-overlap pass could only
    /// reach a legal position by flying a device over another one — legalization is not
    /// allowed to land somewhere legal that way, because it says the local space is too
    /// tight. The first trigger (a conflict left unresolved) is only the extreme case of
    /// the same thing.
    ///
    /// **Default `false` since 2026-09-22, pending a ruling on which predicate.** The
    /// mechanism is implemented and measured three ways
    /// (`lcr_retry_cross_criterion`), but the one that was briefly on by default
    /// -- area coverage at 0.5 -- ships the abstract `comp` case with two residual
    /// overlaps, and zero residual overlap is what the released version claims. Off
    /// until the predicate is decided; off is bit-for-bit the pre-2026-09-22 behaviour.
    #[serde(default = "default_lcr_retry_on_crossing")]
    pub lcr_retry_on_crossing: bool,

    /// LCR retry, second trigger: how much of another device a de-overlap move is
    /// allowed to sweep across before it counts as having flown over it.
    ///
    /// The move's swept region (the start box translated to the end box) is intersected
    /// with each other device's box; `covered area / that device's area` above this
    /// fraction is a crossing. Must be in `(0, 1]`; `1.0` means only a move that
    /// completely buries another device counts.
    #[serde(default = "default_lcr_retry_cross_cover_frac")]
    pub lcr_retry_cross_cover_frac: f64,

    /// LCR retry, second trigger: which predicate decides that a de-overlap move "flew
    /// over" another device.
    ///
    /// * `"order"` — the mover changed sides relative to a *local* neighbour (same
    ///   conflict component, or touching the swept region). Catches a device slipping
    ///   *past* another one, including out through a gap in a ring of them.
    /// * `"cover"` — the swept region covers more than `cross_cover_frac` of another
    ///   device. Catches a device ploughing *over* another one.
    /// * `"either"` — their union. The two catch different failures, so this is the
    ///   candidate the measurements point at.
    ///
    /// Inert while `lcr_retry_on_crossing` is off, which is the default.
    #[serde(default = "default_lcr_retry_cross_criterion")]
    pub lcr_retry_cross_criterion: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PlacementConfigV2 {
    pub schema_version: u32,
    pub mode: String,
    #[serde(default)]
    pub common: serde_json::Map<String, serde_json::Value>,
    #[serde(default)]
    pub multiphase: serde_json::Map<String, serde_json::Value>,
}

#[derive(Debug, Clone)]
struct CasePlacementParamsV1 {
    top_schema_version: u32,
    placement_schema_version: u32,
    mode: String,
    global: serde_json::Map<String, serde_json::Value>,
    base: serde_json::Map<String, serde_json::Value>,
    tail: serde_json::Map<String, serde_json::Value>,
    deoverlap: serde_json::Map<String, serde_json::Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlacementConfigResolveMeta {
    pub schema_version: u32,
    pub mode: String,
    pub sources: BTreeMap<String, String>,
    pub cli_overrides: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ParamSection {
    Common,
    MultiPhase,
}

// Default value functions
fn default_wl_gamma_decay() -> f64 {
    0.95
}
fn default_optimizer() -> String {
    "Adam".to_string()
}
fn default_ap_exponent_m() -> f64 {
    6.0
}
fn default_ap_eps() -> f64 {
    1e-6
}
fn default_ap_stable_weight() -> f64 {
    0.0
}
fn default_step_strategy() -> String {
    "none".to_string()
}
fn default_ls_step_init() -> f64 {
    1.0
}
fn default_ls_step_shrink() -> f64 {
    0.5
}
fn default_ls_step_min() -> f64 {
    1e-6
}
fn default_armijo_c1() -> f64 {
    1e-4
}
fn default_bias_unit() -> f64 {
    1.0
}
fn default_aspect_inflation() -> f64 {
    1.0
}
fn default_boundary_weight() -> f64 {
    1.0
}
fn default_sym_self_weight() -> f64 {
    2.0
}
fn default_compaction_weight() -> f64 {
    0.0
}
fn default_init_jitter_ratio() -> f64 {
    0.1
}
fn default_init_spread_ratio() -> f64 {
    1.0
}
fn default_mass_damping_exp() -> f64 {
    0.5
}
fn default_grad_tol() -> f64 {
    1e-4
}
fn default_loss_rtol() -> f64 {
    1e-5
}
fn default_move_rtol() -> f64 {
    1e-4
}
fn default_move_max_rtol() -> f64 {
    5e-4
}
fn default_overlap_target() -> f64 {
    5e-4
}
fn default_symmetry_target() -> f64 {
    1e-2
}
fn default_boundary_target() -> f64 {
    0.0
}
fn default_convergence_window() -> usize {
    10
}
fn default_enable_gp_tail() -> bool {
    true
}
fn default_tail_iters() -> usize {
    500
}
fn default_tail_ema_alpha() -> f64 {
    0.2
}
fn default_tail_tau_ovl_good() -> f64 {
    0.01
}
fn default_tail_d_soft() -> f64 {
    0.06
}
fn default_tail_d_hard() -> f64 {
    0.12
}
fn default_tail_s_soft() -> f64 {
    0.02
}
fn default_tail_s_hard() -> f64 {
    0.06
}
fn default_tail_lambda_growth_cap() -> f64 {
    0.08
}
fn default_tail_lambda_decay_cap() -> f64 {
    0.03
}
fn default_tail_tau_accept_ovl() -> f64 {
    0.005
}
fn default_tail_tau_min() -> f64 {
    0.001
}
fn default_tail_d_accept() -> f64 {
    0.10
}
fn default_tail_d_rollback() -> f64 {
    0.16
}
fn default_tail_s_accept() -> f64 {
    0.05
}
fn default_tail_s_rollback() -> f64 {
    0.10
}
fn default_tail_window() -> usize {
    20
}
fn default_tail_anchor_strong() -> f64 {
    1.0
}
fn default_tail_anchor_mid() -> f64 {
    0.35
}
fn default_tail_anchor_weak() -> f64 {
    0.15
}
fn default_mobility_min() -> f64 {
    0.6
}
fn default_mobility_max() -> f64 {
    3.0
}
fn default_mobility_sym_anchor_mult() -> f64 {
    1.3
}
fn default_mobility_large_mult() -> f64 {
    1.3
}
fn default_mobility_small_mult() -> f64 {
    0.8
}
fn default_tail_selective_inflation() -> bool {
    true
}
fn default_tail_inflation_step() -> f64 {
    0.05
}
fn default_tail_inflation_max() -> f64 {
    1.10
}
fn default_pmos_hard_expand_for_gr() -> f64 {
    650.0
}
fn default_tail_inflation_safety_margin() -> f64 {
    0.01
}
fn default_tail_hard_inflation_floor() -> f64 {
    0.0
}
fn default_tail_diag_interval() -> usize {
    5
}
fn default_tail_diag_enable_plots() -> bool {
    true
}
fn default_gp_controller_mode() -> String {
    "multiphase".to_string()
}
fn default_rho() -> f64 {
    0.15
}
fn default_tau_o_ratio() -> f64 {
    0.20
}
fn default_tau_s_ratio() -> f64 {
    0.50
}
fn default_a_o() -> f64 {
    1.0
}
fn default_b_o() -> f64 {
    0.25
}
fn default_a_s() -> f64 {
    0.35
}
fn default_b_s() -> f64 {
    0.10
}
fn default_lambda_ovl_min() -> f64 {
    1e-4
}
fn default_lambda_ovl_max() -> f64 {
    30.0
}
fn default_lambda_sym_min() -> f64 {
    1e-5
}
fn default_lambda_sym_max() -> f64 {
    10.0
}
fn default_s_lambda_base() -> f64 {
    (1.01f64).ln()
}
fn default_s_lambda_tail() -> f64 {
    (1.015f64).ln()
}
fn default_tau_gate_hi_ratio() -> f64 {
    0.25
}
fn default_tau_gate_lo_ratio() -> f64 {
    0.10
}
fn default_eta_ap() -> f64 {
    0.002
}
fn default_lambda_ap_max() -> f64 {
    1.0
}
fn default_anchor_base_ratio() -> f64 {
    1e-3
}
fn default_anchor_tail_ratio() -> f64 {
    5e-3
}
fn default_anchor_min() -> f64 {
    1e-6
}
fn default_anchor_max_base() -> f64 {
    2.0
}
fn default_anchor_max_tail() -> f64 {
    10.0
}
fn default_anchor_ramp_iters() -> usize {
    20
}
fn default_mobility_trigger_ratio() -> f64 {
    0.3
}
fn default_mobility_ramp_iters() -> usize {
    50
}
fn default_c_delta_base() -> f64 {
    0.05
}
fn default_c_delta_tail() -> f64 {
    0.08
}
fn default_candidate_stride() -> usize {
    5
}
fn default_tail_infl_ratio() -> f64 {
    0.15
}
fn default_tail_infl_ramp_iters() -> usize {
    40
}
fn default_tail_tau_o_ratio() -> f64 {
    0.15
}
fn default_tail_candidate_alpha_start() -> f64 {
    0.5
}
fn default_tail_candidate_stride() -> usize {
    5
}
fn default_tail_ref_dilate_beta() -> f64 {
    0.15
}
fn default_tail_cover_tau() -> f64 {
    0.70
}
fn default_tail_cover_softness() -> f64 {
    0.08
}
fn default_tail_cover_lambda_ratio() -> f64 {
    0.10
}
fn default_tail_deoverlap_enable() -> bool {
    true
}
fn default_tail_deoverlap_grid_fine() -> f64 {
    50.0
}
fn default_tail_deoverlap_ring_fine() -> f64 {
    5000.0
}
fn default_tail_deoverlap_grid_coarse() -> f64 {
    200.0
}
fn default_tail_deoverlap_ring_coarse() -> f64 {
    20000.0
}
fn default_tail_deoverlap_infl() -> f64 {
    1.05
}
fn default_tail_deoverlap_search_kappa() -> f64 {
    2.0
}
fn default_tail_deoverlap_lambda_expand() -> f64 {
    1.0
}
fn default_tail_deoverlap_lambda_area() -> f64 {
    1.0
}
fn default_tail_deoverlap_rho_max() -> f64 {
    0.15
}
fn default_tail_deoverlap_use_elliptical_search() -> bool {
    true
}
fn default_tail_deoverlap_lambda_topo() -> f64 {
    0.0
}
fn default_lcr_honor_drc_floor() -> bool {
    // On by default: the rule deck is the authority on how close two devices may sit.
    // Honouring the clearance floor during de-overlap costs a little wirelength on
    // the cases where it binds, and it removes device pairs that a real rule deck
    // reports as spacing violations. Setting this key to false restores the earlier
    // behaviour, in which the floor was applied only by the tail phase.
    true
}
fn default_lcr_retry_enable() -> bool {
    true
}
fn default_lcr_retry_max_rounds() -> usize {
    // Measured on the largest example case, with the de-overlap clearance requirement
    // raised until local conflict resolution genuinely fails: a realistic failure
    // (15 residual pairs) converges in 3 rounds,
    // a harsh one (70 pairs) in 6. An artificial extreme (157 pairs) still has not
    // converged at 12, so a larger cap buys nothing for realistic failures while
    // costing a full multi-phase pass each. 6 is that knee.
    6
}
fn default_lcr_retry_infl_step() -> f64 {
    1.1
}
fn default_lcr_retry_cross_cover_frac() -> f64 {
    // Proposed, not ruled on yet: half of a device. Thresholds of 0.25, 0.5 and
    // 0.75 were all measured; this is the middle one.
    0.5
}
fn default_lcr_retry_cross_criterion() -> String {
    // Maintainer ruling 2026-09-23: `either` (order-inversion OR swept-area coverage).
    // It is the arm the 20-point sweep recommends -- zero residual overlap at every
    // point, mean HPWL -1.9%, one fewer out-of-region case -- and it catches both
    // "slipped past a neighbour" (order) and "ploughed over one" (cover). The trigger
    // itself stays off by default (see below), so this only says what runs when a user
    // turns it on.
    "either".to_string()
}
fn default_lcr_retry_on_crossing() -> bool {
    // Off by maintainer ruling (2026-09-23): the second trigger ships as an opt-in test
    // feature, in the released version too, because its convergence cost is unproven on
    // real designs (the loop mostly runs to the round cap). Off is bit-for-bit the
    // pre-2026-09-22 behaviour, which is the state every committed reference and the
    // released version's zero-overlap claim are measured in. When turned on, the
    // predicate is `lcrRetryCrossCriterion` (default `either`).
    false
}
fn default_tail_deoverlap_final_phi_iters() -> usize {
    300
}

fn multiphase_only_keys() -> &'static [&'static str] {
    &[
        "rho",
        "tauORatio",
        "tauSRatio",
        "aO",
        "bO",
        "aS",
        "bS",
        "lambdaOvlMin",
        "lambdaOvlMax",
        "lambdaSymMin",
        "lambdaSymMax",
        "sLambdaBase",
        "sLambdaTail",
        "tauGateHiRatio",
        "tauGateLoRatio",
        "etaAp",
        "lambdaApMax",
        "anchorBaseRatio",
        "anchorTailRatio",
        "anchorMin",
        "anchorMaxBase",
        "anchorMaxTail",
        "anchorRampIters",
        "mobilityTriggerRatio",
        "mobilityRampIters",
        "cDeltaBase",
        "cDeltaTail",
        "candidateStride",
        "tailInflRatio",
        "tailInflRampIters",
        "tailTauORatio",
        "tailCandidateAlphaStart",
        "tailCandidateStride",
        "tailRefDilateBeta",
        "tailCoverTau",
        "tailCoverSoftness",
        "tailCoverLambdaRatio",
        "tailDeoverlapEnable",
        "tailDeoverlapGridFine",
        "tailDeoverlapRingFine",
        "tailDeoverlapGridCoarse",
        "tailDeoverlapRingCoarse",
        "tailDeoverlapInfl",
        "tailDeoverlapSearchKappa",
        "tailDeoverlapLambdaExpand",
        "tailDeoverlapLambdaArea",
        "tailDeoverlapRhoMax",
        "tailDeoverlapUseEllipticalSearch",
        "tailDeoverlapLambdaTopo",
        "tailDeoverlapFinalPhiIters",
        "lcrHonorDrcFloor",
        "lcrRetryEnable",
        "lcrRetryMaxRounds",
        "lcrRetryInflStep",
        "lcrRetryOnCrossing",
        "lcrRetryCrossCoverFrac",
        "lcrRetryCrossCriterion",
    ]
}

fn grouped_global_multiphase_keys() -> &'static [&'static str] {
    &[
        "lambdaApMax",
        "etaAp",
        "mobilityTriggerRatio",
        "mobilityRampIters",
        "candidateStride",
    ]
}

fn grouped_base_multiphase_keys() -> &'static [&'static str] {
    &[
        "rho",
        "tauORatio",
        "tauSRatio",
        "aO",
        "bO",
        "aS",
        "bS",
        "lambdaOvlMin",
        "lambdaOvlMax",
        "lambdaSymMin",
        "lambdaSymMax",
        "sLambdaBase",
        "tauGateHiRatio",
        "tauGateLoRatio",
        "anchorBaseRatio",
        "anchorMin",
        "anchorMaxBase",
        "anchorRampIters",
        "cDeltaBase",
    ]
}

fn grouped_tail_multiphase_keys() -> &'static [&'static str] {
    &[
        "sLambdaTail",
        "anchorTailRatio",
        "anchorMaxTail",
        "cDeltaTail",
        "tailInflRatio",
        "tailInflRampIters",
        "tailTauORatio",
        "tailCandidateAlphaStart",
        "tailCandidateStride",
        "tailRefDilateBeta",
        "tailCoverTau",
        "tailCoverSoftness",
        "tailCoverLambdaRatio",
    ]
}

fn grouped_deoverlap_multiphase_keys() -> &'static [&'static str] {
    &[
        "tailDeoverlapEnable",
        "tailDeoverlapGridFine",
        "tailDeoverlapRingFine",
        "tailDeoverlapGridCoarse",
        "tailDeoverlapRingCoarse",
        "tailDeoverlapInfl",
        "tailDeoverlapSearchKappa",
        "tailDeoverlapLambdaExpand",
        "tailDeoverlapLambdaArea",
        "tailDeoverlapRhoMax",
        "tailDeoverlapUseEllipticalSearch",
        "tailDeoverlapLambdaTopo",
        "tailDeoverlapFinalPhiIters",
        "lcrHonorDrcFloor",
        "lcrRetryEnable",
        "lcrRetryMaxRounds",
        "lcrRetryInflStep",
        "lcrRetryOnCrossing",
        "lcrRetryCrossCoverFrac",
        "lcrRetryCrossCriterion",
    ]
}

fn mode_values() -> &'static [&'static str] {
    &["multiphase"]
}

fn levenshtein(a: &str, b: &str) -> usize {
    if a == b {
        return 0;
    }
    if a.is_empty() {
        return b.chars().count();
    }
    if b.is_empty() {
        return a.chars().count();
    }
    let b_chars: Vec<char> = b.chars().collect();
    let mut prev: Vec<usize> = (0..=b_chars.len()).collect();
    let mut curr = vec![0usize; b_chars.len() + 1];
    for (i, ca) in a.chars().enumerate() {
        curr[0] = i + 1;
        for (j, cb) in b_chars.iter().enumerate() {
            let cost = if ca == *cb { 0 } else { 1 };
            curr[j + 1] = (prev[j + 1] + 1).min(curr[j] + 1).min(prev[j] + cost);
        }
        std::mem::swap(&mut prev, &mut curr);
    }
    prev[b_chars.len()]
}

fn did_you_mean<'a, I>(key: &str, candidates: I) -> Option<String>
where
    I: IntoIterator<Item = &'a str>,
{
    let mut best: Option<(&str, usize)> = None;
    for cand in candidates {
        let d = levenshtein(key, cand);
        if best.map(|(_, bd)| d < bd).unwrap_or(true) {
            best = Some((cand, d));
        }
    }
    best.and_then(|(cand, d)| {
        if d <= 3 || cand.starts_with(key) || key.starts_with(cand) {
            Some(cand.to_string())
        } else {
            None
        }
    })
}

/// Parameter keys that were renamed for the 1.0 release, and what they are called now.
///
/// A file written against an old spelling is refused, not translated. Accepting it
/// would mean loading a configuration with that parameter silently left at its
/// default, which is a different placement and no warning.
const RETIRED_PARAM_KEYS: [(&str, &str); 17] = [
    ("maxIters", "maxIterations"),
    ("optimAlg", "optimizer"),
    ("adamLr", "learningRate"),
    ("useLsewl", "enableWirelength"),
    ("useOverlap", "enableOverlap"),
    ("useSym", "enableSymmetry"),
    ("useArray", "enableArray"),
    ("useBoundary", "enableBoundary"),
    ("initWlGamma", "wlSmoothingInit"),
    ("thresOvl", "overlapTarget"),
    ("thresSym", "symmetryTarget"),
    ("thresBnd", "boundaryTarget"),
    ("ssStrategy", "stepStrategy"),
    ("initAlpha", "lsStepInit"),
    ("multipAlpha", "lsStepShrink"),
    ("minAlpha", "lsStepMin"),
    ("lsConst1", "armijoC1"),
];

/// The new name of a retired key, if `key` is one.
fn retired_param_key(key: &str) -> Option<&'static str> {
    RETIRED_PARAM_KEYS
        .iter()
        .find(|(old, _)| *old == key)
        .map(|(_, new)| *new)
}

/// The sentence appended to an "unknown key" error: the new name when the key is one
/// this release renamed, otherwise the nearest key that does exist.
fn unknown_key_hint<'a, I>(key: &str, candidates: I) -> String
where
    I: IntoIterator<Item = &'a str>,
{
    if let Some(new_name) = retired_param_key(key) {
        return format!(" `{}` was renamed to `{}`.", key, new_name);
    }
    did_you_mean(key, candidates)
        .map(|s| format!(" Did you mean `{}`?", s))
        .unwrap_or_default()
}

impl PlacementConfig {
    fn all_param_keys() -> PlaceResult<BTreeSet<String>> {
        let v = serde_json::to_value(PlacementConfig::default())?;
        let obj = v.as_object().ok_or_else(|| {
            PlaceError::InvalidParameter("failed to derive default parameter key set".to_string())
        })?;
        Ok(obj.keys().cloned().collect())
    }

    fn section_for_key(key: &str, all_keys: &BTreeSet<String>) -> Option<ParamSection> {
        if multiphase_only_keys().contains(&key) {
            return Some(ParamSection::MultiPhase);
        }
        if all_keys.contains(key) {
            return Some(ParamSection::Common);
        }
        None
    }

    fn grouped_section_for_key(key: &str, all_keys: &BTreeSet<String>) -> Option<&'static str> {
        if grouped_deoverlap_multiphase_keys().contains(&key) {
            return Some("deoverlap");
        }
        if grouped_tail_multiphase_keys().contains(&key) {
            return Some("tail");
        }
        if grouped_base_multiphase_keys().contains(&key) {
            return Some("base");
        }
        if grouped_global_multiphase_keys().contains(&key) {
            return Some("global");
        }
        if all_keys.contains(key) {
            return Some("global");
        }
        None
    }

    fn ensure_allowed_case_top_keys(
        obj: &serde_json::Map<String, serde_json::Value>,
    ) -> PlaceResult<()> {
        let allowed = ["schemaVersion", "placement"];
        for key in obj.keys() {
            if allowed.contains(&key.as_str()) {
                continue;
            }
            let hint = unknown_key_hint(key, allowed.iter().copied());
            return Err(PlaceError::InvalidParameter(format!(
                "unknown top-level key `{}` in params.json.{}",
                key, hint
            )));
        }
        Ok(())
    }

    fn ensure_allowed_case_placement_keys(
        obj: &serde_json::Map<String, serde_json::Value>,
    ) -> PlaceResult<()> {
        let allowed = [
            "schemaVersion",
            "mode",
            "global",
            "base",
            "tail",
            "deoverlap",
        ];
        for key in obj.keys() {
            if allowed.contains(&key.as_str()) {
                continue;
            }
            let hint = unknown_key_hint(key, allowed.iter().copied());
            return Err(PlaceError::InvalidParameter(format!(
                "unknown key `{}` in `placement` section of params.json.{}",
                key, hint
            )));
        }
        Ok(())
    }

    fn ensure_allowed_top_keys(
        obj: &serde_json::Map<String, serde_json::Value>,
    ) -> PlaceResult<()> {
        let allowed = ["schemaVersion", "mode", "common", "multiphase"];
        for key in obj.keys() {
            if allowed.contains(&key.as_str()) {
                continue;
            }
            let hint = unknown_key_hint(key, allowed.iter().copied());
            return Err(PlaceError::InvalidParameter(format!(
                "unknown top-level key `{}` in the placement configuration.{}",
                key, hint
            )));
        }
        Ok(())
    }

    fn parse_case_section_map(
        root: &serde_json::Map<String, serde_json::Value>,
        name: &str,
    ) -> PlaceResult<serde_json::Map<String, serde_json::Value>> {
        match root.get(name) {
            None => Ok(serde_json::Map::new()),
            Some(serde_json::Value::Object(obj)) => {
                let mut out = obj.clone();
                Self::apply_case_section_aliases(name, &mut out)?;
                Ok(out)
            }
            Some(_) => Err(PlaceError::InvalidParameter(format!(
                "`placement.{}` must be a JSON object in params.json",
                name
            ))),
        }
    }

    fn apply_case_section_aliases(
        section_name: &str,
        map: &mut serde_json::Map<String, serde_json::Value>,
    ) -> PlaceResult<()> {
        let alias_pairs: Vec<(&str, &str)> = if section_name == "deoverlap" {
            vec![
                ("deoverlap_search_kappa", "tailDeoverlapSearchKappa"),
                ("deoverlap_lambda_expand", "tailDeoverlapLambdaExpand"),
                ("deoverlap_lambda_area", "tailDeoverlapLambdaArea"),
                ("deoverlap_rho_max", "tailDeoverlapRhoMax"),
                ("deoverlap_infl", "tailDeoverlapInfl"),
                ("infl_deoverlap", "tailDeoverlapInfl"),
                (
                    "deoverlap_use_elliptical_search",
                    "tailDeoverlapUseEllipticalSearch",
                ),
                ("deoverlap_lambda_topo", "tailDeoverlapLambdaTopo"),
                ("deoverlap_final_phi_iters", "tailDeoverlapFinalPhiIters"),
                ("lcr_honor_drc_floor", "lcrHonorDrcFloor"),
                ("lcr_retry_enable", "lcrRetryEnable"),
                ("lcr_retry_max_rounds", "lcrRetryMaxRounds"),
                ("lcr_retry_infl_step", "lcrRetryInflStep"),
                ("lcr_retry_on_crossing", "lcrRetryOnCrossing"),
                ("lcr_retry_cross_cover_frac", "lcrRetryCrossCoverFrac"),
                ("lcr_retry_cross_criterion", "lcrRetryCrossCriterion"),
            ]
        } else if section_name == "global" {
            vec![
                ("pmos_hard_expand_for_gr", "pmosHardExpandForGr"),
                ("tail_inflation_safety_margin", "tailInflationSafetyMargin"),
            ]
        } else {
            Vec::new()
        };
        for (alias, canonical) in alias_pairs {
            let Some(alias_val) = map.remove(alias) else {
                continue;
            };
            if let Some(existing) = map.get(canonical) {
                if existing != &alias_val {
                    return Err(PlaceError::InvalidParameter(format!(
                        "conflicting values in `placement.deoverlap`: `{}` and `{}`",
                        alias, canonical
                    )));
                }
                continue;
            }
            map.insert(canonical.to_string(), alias_val);
        }
        Ok(())
    }

    fn parse_section_map(
        root: &serde_json::Map<String, serde_json::Value>,
        name: &str,
    ) -> PlaceResult<serde_json::Map<String, serde_json::Value>> {
        match root.get(name) {
            None => Ok(serde_json::Map::new()),
            Some(serde_json::Value::Object(obj)) => Ok(obj.clone()),
            Some(_) => Err(PlaceError::InvalidParameter(format!(
                "`{}` must be a JSON object",
                name
            ))),
        }
    }

    fn verify_section_keys(
        section_name: &str,
        section_map: &serde_json::Map<String, serde_json::Value>,
        expected: ParamSection,
        all_keys: &BTreeSet<String>,
    ) -> PlaceResult<()> {
        for key in section_map.keys() {
            let Some(actual) = Self::section_for_key(key, all_keys) else {
                let hint = unknown_key_hint(key, all_keys.iter().map(|s| s.as_str()));
                return Err(PlaceError::InvalidParameter(format!(
                    "unknown key `{}` in section `{}`.{}",
                    key, section_name, hint
                )));
            };
            if actual != expected {
                let dst = match actual {
                    ParamSection::Common => "common",
                    ParamSection::MultiPhase => "multiphase",
                };
                return Err(PlaceError::InvalidParameter(format!(
                    "key `{}` is not allowed in section `{}`; move it to `{}`",
                    key, section_name, dst
                )));
            }
        }
        Ok(())
    }

    fn verify_grouped_section_keys(
        section_name: &str,
        section_map: &serde_json::Map<String, serde_json::Value>,
        all_keys: &BTreeSet<String>,
    ) -> PlaceResult<()> {
        for key in section_map.keys() {
            if key == "gpControllerMode" {
                return Err(PlaceError::InvalidParameter(
                    "`gpControllerMode` is not allowed in params.json groups; use `placement.mode`"
                        .to_string(),
                ));
            }
            let Some(dst) = Self::grouped_section_for_key(key, all_keys) else {
                let hint = unknown_key_hint(key, all_keys.iter().map(|s| s.as_str()));
                return Err(PlaceError::InvalidParameter(format!(
                    "unknown key `{}` in section `{}`.{}",
                    key, section_name, hint
                )));
            };
            if dst != section_name {
                return Err(PlaceError::InvalidParameter(format!(
                    "key `{}` is not allowed in section `{}`; move it to `{}`",
                    key, section_name, dst
                )));
            }
        }
        Ok(())
    }

    fn parse_v2_document(v: serde_json::Value) -> PlaceResult<PlacementConfigV2> {
        let root = v.as_object().ok_or_else(|| {
            PlaceError::InvalidParameter(
                "a placement configuration must be a JSON object".to_string(),
            )
        })?;
        if !root.contains_key("schemaVersion") {
            return Err(PlaceError::InvalidParameter(
                "a placement configuration must declare schemaVersion=2".to_string(),
            ));
        }
        Self::ensure_allowed_top_keys(root)?;
        let schema_version = root
            .get("schemaVersion")
            .and_then(|v| v.as_u64())
            .ok_or_else(|| {
                PlaceError::InvalidParameter("`schemaVersion` must be an integer".to_string())
            })? as u32;
        if schema_version != 2 {
            return Err(PlaceError::InvalidParameter(format!(
                "unsupported schemaVersion `{}` (expected 2)",
                schema_version
            )));
        }

        let mode = root
            .get("mode")
            .and_then(|v| v.as_str())
            .ok_or_else(|| PlaceError::InvalidParameter("`mode` must be `multiphase`".to_string()))?
            .to_string();
        if !mode_values().contains(&mode.as_str()) {
            return Err(PlaceError::InvalidParameter(format!(
                "`mode` must be one of: {}",
                mode_values().join(", ")
            )));
        }

        let common = Self::parse_section_map(root, "common")?;
        let multiphase = Self::parse_section_map(root, "multiphase")?;

        let all_keys = Self::all_param_keys()?;
        Self::verify_section_keys("common", &common, ParamSection::Common, &all_keys)?;
        Self::verify_section_keys(
            "multiphase",
            &multiphase,
            ParamSection::MultiPhase,
            &all_keys,
        )?;

        Ok(PlacementConfigV2 {
            schema_version,
            mode,
            common,
            multiphase,
        })
    }

    fn parse_case_params_document(v: serde_json::Value) -> PlaceResult<CasePlacementParamsV1> {
        let root = v.as_object().ok_or_else(|| {
            PlaceError::InvalidParameter("params.json must be a JSON object".to_string())
        })?;
        Self::ensure_allowed_case_top_keys(root)?;

        let top_schema_version = root
            .get("schemaVersion")
            .and_then(|v| v.as_u64())
            .ok_or_else(|| {
                PlaceError::InvalidParameter(
                    "`schemaVersion` must be an integer in params.json".to_string(),
                )
            })? as u32;
        if top_schema_version != 1 {
            return Err(PlaceError::InvalidParameter(format!(
                "unsupported params.json schemaVersion `{}` (expected 1)",
                top_schema_version
            )));
        }

        let placement_obj = root
            .get("placement")
            .and_then(|v| v.as_object())
            .ok_or_else(|| {
                PlaceError::InvalidParameter(
                    "missing or invalid `placement` object in params.json".to_string(),
                )
            })?;
        Self::ensure_allowed_case_placement_keys(placement_obj)?;

        let placement_schema_version = placement_obj
            .get("schemaVersion")
            .and_then(|v| v.as_u64())
            .ok_or_else(|| {
                PlaceError::InvalidParameter(
                    "`placement.schemaVersion` must be an integer".to_string(),
                )
            })? as u32;
        if placement_schema_version != 1 {
            return Err(PlaceError::InvalidParameter(format!(
                "unsupported placement schemaVersion `{}` (expected 1)",
                placement_schema_version
            )));
        }

        let mode = placement_obj
            .get("mode")
            .and_then(|v| v.as_str())
            .ok_or_else(|| {
                PlaceError::InvalidParameter("`placement.mode` must be `multiphase`".to_string())
            })?
            .to_string();
        if !mode_values().contains(&mode.as_str()) {
            return Err(PlaceError::InvalidParameter(format!(
                "`placement.mode` must be one of: {}",
                mode_values().join(", ")
            )));
        }

        let global = Self::parse_case_section_map(placement_obj, "global")?;
        let base = Self::parse_case_section_map(placement_obj, "base")?;
        let tail = Self::parse_case_section_map(placement_obj, "tail")?;
        let deoverlap = Self::parse_case_section_map(placement_obj, "deoverlap")?;

        let all_keys = Self::all_param_keys()?;
        Self::verify_grouped_section_keys("global", &global, &all_keys)?;
        Self::verify_grouped_section_keys("base", &base, &all_keys)?;
        Self::verify_grouped_section_keys("tail", &tail, &all_keys)?;
        Self::verify_grouped_section_keys("deoverlap", &deoverlap, &all_keys)?;

        Ok(CasePlacementParamsV1 {
            top_schema_version,
            placement_schema_version,
            mode,
            global,
            base,
            tail,
            deoverlap,
        })
    }

    fn parse_override_value(raw: &str) -> PlaceResult<serde_json::Value> {
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            return Err(PlaceError::InvalidParameter(
                "override value cannot be empty".to_string(),
            ));
        }
        if trimmed.starts_with('"')
            || trimmed.starts_with('{')
            || trimmed.starts_with('[')
            || trimmed == "true"
            || trimmed == "false"
            || trimmed == "null"
        {
            let parsed: serde_json::Value = serde_json::from_str(trimmed)?;
            return Ok(parsed);
        }
        if let Ok(i) = trimmed.parse::<i64>() {
            return Ok(serde_json::Value::Number(i.into()));
        }
        if let Ok(f) = trimmed.parse::<f64>() {
            if let Some(n) = serde_json::Number::from_f64(f) {
                return Ok(serde_json::Value::Number(n));
            }
        }
        Ok(serde_json::Value::String(trimmed.to_string()))
    }

    fn parse_cli_override_entry(
        entry: &str,
        mode: &str,
        all_keys: &BTreeSet<String>,
    ) -> PlaceResult<(String, serde_json::Value)> {
        let (raw_key, raw_value) = entry.split_once('=').ok_or_else(|| {
            PlaceError::InvalidParameter(format!("invalid --set `{}`; expected `key=value`", entry))
        })?;
        let mut key = raw_key.trim();
        if key.starts_with("common.") {
            key = &key["common.".len()..];
        } else if key.starts_with("multiphase.") {
            if mode != "multiphase" {
                return Err(PlaceError::InvalidParameter(format!(
                    "override `{}` targets multiphase section but mode is `{}`",
                    entry, mode
                )));
            }
            key = &key["multiphase.".len()..];
        }
        if key == "gpControllerMode" {
            return Err(PlaceError::InvalidParameter(
                "override `gpControllerMode` is not allowed; use the top-level `mode` instead"
                    .to_string(),
            ));
        }
        let Some(_section) = Self::section_for_key(key, all_keys) else {
            let hint = unknown_key_hint(key, all_keys.iter().map(|s| s.as_str()));
            return Err(PlaceError::InvalidParameter(format!(
                "unknown override key `{}`.{}",
                key, hint
            )));
        };
        Ok((key.to_string(), Self::parse_override_value(raw_value)?))
    }

    fn apply_map_with_source(
        merged: &mut serde_json::Map<String, serde_json::Value>,
        source: &mut BTreeMap<String, String>,
        map: &serde_json::Map<String, serde_json::Value>,
        source_name: &str,
    ) {
        for (k, v) in map {
            merged.insert(k.clone(), v.clone());
            source.insert(k.clone(), source_name.to_string());
        }
    }

    fn resolve_with_sections(
        schema_version: u32,
        mode: String,
        sections: Vec<(String, serde_json::Map<String, serde_json::Value>)>,
        cli_overrides: &[String],
    ) -> PlaceResult<(Self, PlacementConfigResolveMeta)> {
        let mut merged = serde_json::to_value(Self::default())?
            .as_object()
            .cloned()
            .ok_or_else(|| {
                PlaceError::InvalidParameter(
                    "failed to initialize default params object".to_string(),
                )
            })?;
        let mut sources = BTreeMap::new();
        for k in merged.keys() {
            sources.insert(k.clone(), "default".to_string());
        }

        for (source_name, map) in sections {
            Self::apply_map_with_source(&mut merged, &mut sources, &map, &source_name);
        }

        let all_keys = Self::all_param_keys()?;
        for entry in cli_overrides {
            let (key, value) = Self::parse_cli_override_entry(entry, &mode, &all_keys)?;
            merged.insert(key.clone(), value);
            sources.insert(key, "cli".to_string());
        }

        merged.insert(
            "gpControllerMode".to_string(),
            serde_json::Value::String(mode.clone()),
        );
        sources.insert("gpControllerMode".to_string(), "derived.mode".to_string());

        let params: PlacementConfig = serde_json::from_value(serde_json::Value::Object(merged))?;
        params.validate()?;
        Ok((
            params,
            PlacementConfigResolveMeta {
                schema_version,
                mode,
                sources,
                cli_overrides: cli_overrides.to_vec(),
            },
        ))
    }

    fn split_grouped_global_map(
        global: serde_json::Map<String, serde_json::Value>,
    ) -> (
        serde_json::Map<String, serde_json::Value>,
        serde_json::Map<String, serde_json::Value>,
    ) {
        let mut common = serde_json::Map::new();
        let mut multiphase_shared = serde_json::Map::new();
        for (k, v) in global {
            if grouped_global_multiphase_keys().contains(&k.as_str()) {
                multiphase_shared.insert(k, v);
            } else {
                common.insert(k, v);
            }
        }
        (common, multiphase_shared)
    }

    pub fn from_json_with_overrides<P: AsRef<Path>>(
        path: P,
        cli_overrides: &[String],
    ) -> PlaceResult<(Self, PlacementConfigResolveMeta)> {
        let content = fs::read_to_string(path)?;
        let raw_v: serde_json::Value = serde_json::from_str(&content)?;
        let v2 = Self::parse_v2_document(raw_v)?;
        let mut sections = Vec::new();
        sections.push(("file.common".to_string(), v2.common));
        if v2.mode == "multiphase" {
            sections.push(("file.multiphase".to_string(), v2.multiphase));
        }
        Self::resolve_with_sections(v2.schema_version, v2.mode, sections, cli_overrides)
    }

    /// Load parameters from JSON file
    pub fn from_json<P: AsRef<Path>>(path: P) -> PlaceResult<Self> {
        let (params, _) = Self::from_json_with_overrides(path, &[])?;
        Ok(params)
    }

    pub fn from_case_params_json_with_overrides<P: AsRef<Path>>(
        path: P,
        cli_overrides: &[String],
    ) -> PlaceResult<(Self, PlacementConfigResolveMeta)> {
        let content = fs::read_to_string(path)?;
        let raw_v: serde_json::Value = serde_json::from_str(&content)?;
        let case_v1 = Self::parse_case_params_document(raw_v)?;
        let (global_common, global_multiphase_shared) =
            Self::split_grouped_global_map(case_v1.global);

        let mut sections = Vec::new();
        sections.push(("file.placement.global".to_string(), global_common));
        if case_v1.mode == "multiphase" {
            sections.push((
                "file.placement.global".to_string(),
                global_multiphase_shared,
            ));
            sections.push(("file.placement.base".to_string(), case_v1.base));
            sections.push(("file.placement.tail".to_string(), case_v1.tail));
            sections.push(("file.placement.deoverlap".to_string(), case_v1.deoverlap));
        }

        let schema_version = case_v1
            .top_schema_version
            .max(case_v1.placement_schema_version);
        Self::resolve_with_sections(schema_version, case_v1.mode, sections, cli_overrides)
    }

    pub fn from_case_params_json<P: AsRef<Path>>(path: P) -> PlaceResult<Self> {
        let (params, _) = Self::from_case_params_json_with_overrides(path, &[])?;
        Ok(params)
    }

    /// Save parameters to JSON file
    pub fn to_json<P: AsRef<Path>>(&self, path: P) -> PlaceResult<()> {
        let content = serde_json::to_string_pretty(self)?;
        fs::write(path, content)?;
        Ok(())
    }

    /// Validate parameter correctness
    pub fn validate(&self) -> PlaceResult<()> {
        if self.max_iterations == 0 {
            return Err(PlaceError::InvalidParameter(
                "max_iterations must be greater than 0".to_string(),
            ));
        }

        if self.learning_rate <= 0.0 {
            return Err(PlaceError::InvalidParameter(
                "learning_rate must be positive".to_string(),
            ));
        }

        if !(0.0..1.0).contains(&self.adam_beta1) {
            return Err(PlaceError::InvalidParameter(
                "adam_beta1 must be in (0, 1)".to_string(),
            ));
        }

        if !(0.0..1.0).contains(&self.adam_beta2) {
            return Err(PlaceError::InvalidParameter(
                "adam_beta2 must be in (0, 1)".to_string(),
            ));
        }

        if self.wl_smoothing_init <= 0.0 {
            return Err(PlaceError::InvalidParameter(
                "wl_smoothing_init must be positive".to_string(),
            ));
        }

        if self.ap_exponent_m < 2.0 {
            return Err(PlaceError::InvalidParameter(
                "ap_exponent_m must be >= 2".to_string(),
            ));
        }

        if self.ap_eps <= 0.0 {
            return Err(PlaceError::InvalidParameter(
                "ap_eps must be positive".to_string(),
            ));
        }

        if self.overlap_target < 0.0 {
            return Err(PlaceError::InvalidParameter(
                "overlap_target must be non-negative".to_string(),
            ));
        }

        if self.aspect_inflation <= 0.0 {
            return Err(PlaceError::InvalidParameter(
                "aspect_inflation must be positive".to_string(),
            ));
        }

        if self.boundary_weight <= 0.0 {
            return Err(PlaceError::InvalidParameter(
                "boundary_weight must be positive".to_string(),
            ));
        }

        if self.sym_self_weight <= 0.0 {
            return Err(PlaceError::InvalidParameter(
                "sym_self_weight must be positive".to_string(),
            ));
        }

        if self.compaction_weight < 0.0 {
            return Err(PlaceError::InvalidParameter(
                "compaction_weight must be non-negative".to_string(),
            ));
        }

        if !(0.0..=1.0).contains(&self.init_jitter_ratio) {
            return Err(PlaceError::InvalidParameter(
                "init_jitter_ratio must be in [0, 1]".to_string(),
            ));
        }
        if !(0.0..=1.0).contains(&self.init_spread_ratio) {
            return Err(PlaceError::InvalidParameter(
                "init_spread_ratio must be in [0, 1]".to_string(),
            ));
        }

        if self.mass_damping_exp < 0.0 {
            return Err(PlaceError::InvalidParameter(
                "mass_damping_exp must be non-negative".to_string(),
            ));
        }

        if self.tail_ema_alpha <= 0.0 || self.tail_ema_alpha > 1.0 {
            return Err(PlaceError::InvalidParameter(
                "tail_ema_alpha must be in (0, 1]".to_string(),
            ));
        }
        if self.tail_window == 0 {
            return Err(PlaceError::InvalidParameter(
                "tail_window must be > 0".to_string(),
            ));
        }
        if self.tail_diag_interval == 0 {
            return Err(PlaceError::InvalidParameter(
                "tail_diag_interval must be > 0".to_string(),
            ));
        }
        if self.tail_anchor_strong < 0.0
            || self.tail_anchor_mid < 0.0
            || self.tail_anchor_weak < 0.0
        {
            return Err(PlaceError::InvalidParameter(
                "tail anchor weights must be non-negative".to_string(),
            ));
        }
        if self.mobility_min <= 0.0
            || self.mobility_max <= 0.0
            || self.mobility_min > self.mobility_max
        {
            return Err(PlaceError::InvalidParameter(
                "mobility_min/max invalid".to_string(),
            ));
        }
        if self.tail_inflation_step < 0.0 || self.tail_inflation_max < 1.0 {
            return Err(PlaceError::InvalidParameter(
                "tail inflation parameters invalid".to_string(),
            ));
        }
        if self.pmos_hard_expand_for_gr < 0.0 {
            return Err(PlaceError::InvalidParameter(
                "pmos_hard_expand_for_gr must be >= 0".to_string(),
            ));
        }
        if self.tail_inflation_safety_margin < 0.0 {
            return Err(PlaceError::InvalidParameter(
                "tail_inflation_safety_margin must be >= 0".to_string(),
            ));
        }
        if self.tail_hard_inflation_floor < 0.0 {
            return Err(PlaceError::InvalidParameter(
                "tail_hard_inflation_floor must be >= 0".to_string(),
            ));
        }

        if self.gp_controller_mode != "multiphase" {
            return Err(PlaceError::InvalidParameter(
                "gp_controller_mode must be: multiphase".to_string(),
            ));
        }
        if self.rho <= 0.0 || self.rho > 1.0 {
            return Err(PlaceError::InvalidParameter(
                "rho must be in (0, 1]".to_string(),
            ));
        }
        if self.tau_o_ratio <= 0.0 || self.tau_s_ratio <= 0.0 {
            return Err(PlaceError::InvalidParameter(
                "tau_o_ratio and tau_s_ratio must be positive".to_string(),
            ));
        }
        if self.lambda_ovl_min <= 0.0
            || self.lambda_ovl_max < self.lambda_ovl_min
            || self.lambda_sym_min <= 0.0
            || self.lambda_sym_max < self.lambda_sym_min
        {
            return Err(PlaceError::InvalidParameter(
                "multiphase lambda min/max bounds invalid".to_string(),
            ));
        }
        if self.tau_gate_hi_ratio <= self.tau_gate_lo_ratio || self.tau_gate_lo_ratio <= 0.0 {
            return Err(PlaceError::InvalidParameter(
                "multiphase gate ratios require hi > lo > 0".to_string(),
            ));
        }
        if self.eta_ap < 0.0 || !(0.0..=1.0).contains(&self.lambda_ap_max) {
            return Err(PlaceError::InvalidParameter(
                "multiphase AP parameters invalid".to_string(),
            ));
        }
        if self.anchor_base_ratio <= 0.0
            || self.anchor_tail_ratio <= 0.0
            || self.anchor_min <= 0.0
            || self.anchor_max_base < self.anchor_min
            || self.anchor_max_tail < self.anchor_max_base
            || self.anchor_ramp_iters == 0
        {
            return Err(PlaceError::InvalidParameter(
                "multiphase anchor parameters invalid".to_string(),
            ));
        }
        if self.mobility_trigger_ratio <= 0.0
            || self.mobility_trigger_ratio > 1.0
            || self.mobility_ramp_iters == 0
        {
            return Err(PlaceError::InvalidParameter(
                "multiphase mobility parameters invalid".to_string(),
            ));
        }
        if self.c_delta_base <= 0.0 || self.c_delta_tail <= 0.0 {
            return Err(PlaceError::InvalidParameter(
                "multiphase c_delta values must be positive".to_string(),
            ));
        }
        if self.candidate_stride == 0 {
            return Err(PlaceError::InvalidParameter(
                "candidate_stride must be > 0".to_string(),
            ));
        }
        if self.tail_infl_ratio < 0.0 {
            return Err(PlaceError::InvalidParameter(
                "tail_infl_ratio must be >= 0".to_string(),
            ));
        }
        if self.tail_infl_ramp_iters == 0 {
            return Err(PlaceError::InvalidParameter(
                "tail_infl_ramp_iters must be > 0".to_string(),
            ));
        }
        if self.tail_tau_o_ratio <= 0.0 {
            return Err(PlaceError::InvalidParameter(
                "tail_tau_o_ratio must be > 0".to_string(),
            ));
        }
        if !(0.0..=1.0).contains(&self.tail_candidate_alpha_start) {
            return Err(PlaceError::InvalidParameter(
                "tail_candidate_alpha_start must be in [0, 1]".to_string(),
            ));
        }
        if self.tail_candidate_stride == 0 {
            return Err(PlaceError::InvalidParameter(
                "tail_candidate_stride must be > 0".to_string(),
            ));
        }
        if self.tail_ref_dilate_beta < 0.0 {
            return Err(PlaceError::InvalidParameter(
                "tail_ref_dilate_beta must be >= 0".to_string(),
            ));
        }
        if !(0.0..=1.0).contains(&self.tail_cover_tau) {
            return Err(PlaceError::InvalidParameter(
                "tail_cover_tau must be in [0, 1]".to_string(),
            ));
        }
        if self.tail_cover_softness <= 0.0 {
            return Err(PlaceError::InvalidParameter(
                "tail_cover_softness must be > 0".to_string(),
            ));
        }
        if self.tail_cover_lambda_ratio < 0.0 {
            return Err(PlaceError::InvalidParameter(
                "tail_cover_lambda_ratio must be >= 0".to_string(),
            ));
        }
        if self.tail_deoverlap_grid_fine <= 0.0
            || self.tail_deoverlap_ring_fine <= 0.0
            || self.tail_deoverlap_grid_coarse <= 0.0
            || self.tail_deoverlap_ring_coarse <= 0.0
        {
            return Err(PlaceError::InvalidParameter(
                "multiphase tail de-overlap grid/ring values must be > 0".to_string(),
            ));
        }
        if self.tail_deoverlap_ring_fine < self.tail_deoverlap_grid_fine
            || self.tail_deoverlap_ring_coarse < self.tail_deoverlap_grid_coarse
        {
            return Err(PlaceError::InvalidParameter(
                "multiphase tail de-overlap ring must be >= grid step".to_string(),
            ));
        }
        if !matches!(
            self.lcr_retry_cross_criterion.as_str(),
            "order" | "cover" | "either"
        ) {
            return Err(PlaceError::InvalidParameter(format!(
                "lcr_retry_cross_criterion must be order | cover | either, got {:?}",
                self.lcr_retry_cross_criterion
            )));
        }
        if !self.lcr_retry_cross_cover_frac.is_finite()
            || self.lcr_retry_cross_cover_frac <= 0.0
            || self.lcr_retry_cross_cover_frac > 1.0
        {
            return Err(PlaceError::InvalidParameter(
                "lcr_retry_cross_cover_frac must be finite and in (0, 1]".to_string(),
            ));
        }
        if !self.lcr_retry_infl_step.is_finite() || self.lcr_retry_infl_step <= 1.0 {
            return Err(PlaceError::InvalidParameter(
                "lcr_retry_infl_step must be finite and > 1 (growth must be monotone)".to_string(),
            ));
        }
        if self.tail_deoverlap_infl < 1.0 {
            return Err(PlaceError::InvalidParameter(
                "tail_deoverlap_infl must be >= 1".to_string(),
            ));
        }
        if self.tail_deoverlap_search_kappa < 1.0 {
            return Err(PlaceError::InvalidParameter(
                "tail_deoverlap_search_kappa must be >= 1".to_string(),
            ));
        }
        if self.tail_deoverlap_lambda_expand < 0.0
            || self.tail_deoverlap_lambda_area < 0.0
            || self.tail_deoverlap_lambda_topo < 0.0
        {
            return Err(PlaceError::InvalidParameter(
                "multiphase tail de-overlap lambda values must be >= 0".to_string(),
            ));
        }
        if self.tail_deoverlap_rho_max < 0.0 {
            return Err(PlaceError::InvalidParameter(
                "tail_deoverlap_rho_max must be >= 0".to_string(),
            ));
        }
        if self.tail_deoverlap_final_phi_iters == 0 {
            return Err(PlaceError::InvalidParameter(
                "tail_deoverlap_final_phi_iters must be >= 1".to_string(),
            ));
        }

        Ok(())
    }

    /// Create default parameters (for testing)
    // An inherent `default()`; every caller spells `PlacementConfig::default()`.
    // Moving it onto the `Default` trait is an upstream API decision.
    #[allow(clippy::should_implement_trait)]
    pub fn default() -> Self {
        Self {
            max_iterations: 2000,
            optimizer: "Adam".to_string(),
            learning_rate: 200.0,
            adam_beta1: 0.9,
            adam_beta2: 0.999,
            enable_wirelength: true,
            use_ap_wl: false,
            enable_overlap: true,
            enable_symmetry: true,
            enable_boundary: false,
            use_well_cluster: false,
            enable_array: false,
            use_compaction: false,
            use_mass_damping: true,
            wl_smoothing_init: 1.0,
            wl_gamma_decay: 0.95,
            ap_exponent_m: 6.0,
            ap_eps: 1e-6,
            ap_stable_weight: 0.0,
            step_strategy: "none".to_string(),
            ls_step_init: 1.0,
            ls_step_shrink: 0.5,
            ls_step_min: 1e-6,
            armijo_c1: 1e-4,
            bias_position_unit: 1.0,
            aspect_inflation: 1.0,
            boundary_weight: 1.0,
            sym_self_weight: 2.0,
            compaction_weight: 0.0,
            random_seed: Some(1),
            randomize_initial_placement: false,
            use_best_solution: false,
            init_jitter_ratio: 0.1,
            init_spread_ratio: 1.0,
            mass_damping_exp: 0.5,
            grad_tol: 1e-4,
            loss_rtol: 1e-5,
            move_rtol: 1e-4,
            move_max_rtol: 5e-4,
            overlap_target: 5e-4,
            symmetry_target: 1e-2,
            boundary_target: 0.0,
            convergence_window: 10,
            enable_gp_tail: true,
            tail_iters: 500,
            tail_ema_alpha: 0.2,
            tail_tau_ovl_good: 0.01,
            tail_d_soft: 0.06,
            tail_d_hard: 0.12,
            tail_s_soft: 0.02,
            tail_s_hard: 0.06,
            tail_lambda_growth_cap: 0.08,
            tail_lambda_decay_cap: 0.03,
            tail_tau_accept_ovl: 0.005,
            tail_tau_min: 0.001,
            tail_d_accept: 0.10,
            tail_d_rollback: 0.16,
            tail_s_accept: 0.05,
            tail_s_rollback: 0.10,
            tail_window: 20,
            tail_anchor_strong: 1.0,
            tail_anchor_mid: 0.35,
            tail_anchor_weak: 0.15,
            mobility_min: 0.6,
            mobility_max: 3.0,
            mobility_sym_anchor_mult: 1.3,
            mobility_large_mult: 1.3,
            mobility_small_mult: 0.8,
            tail_selective_inflation: true,
            tail_inflation_step: 0.05,
            tail_inflation_max: 1.10,
            pmos_hard_expand_for_gr: 650.0,
            tail_inflation_safety_margin: 0.01,
            tail_hard_inflation_floor: 0.0,
            tail_diag_interval: 5,
            tail_diag_enable_plots: true,
            gp_controller_mode: "multiphase".to_string(),
            rho: 0.15,
            tau_o_ratio: 0.20,
            tau_s_ratio: 0.50,
            a_o: 1.0,
            b_o: 0.25,
            a_s: 0.35,
            b_s: 0.10,
            lambda_ovl_min: 1e-4,
            lambda_ovl_max: 30.0,
            lambda_sym_min: 1e-5,
            lambda_sym_max: 10.0,
            s_lambda_base: (1.01f64).ln(),
            s_lambda_tail: (1.015f64).ln(),
            tau_gate_hi_ratio: 0.25,
            tau_gate_lo_ratio: 0.10,
            eta_ap: 0.001,
            lambda_ap_max: 0.7,
            anchor_base_ratio: 1e-3,
            anchor_tail_ratio: 5e-3,
            anchor_min: 1e-6,
            anchor_max_base: 2.0,
            anchor_max_tail: 10.0,
            anchor_ramp_iters: 20,
            mobility_trigger_ratio: 0.3,
            mobility_ramp_iters: 50,
            c_delta_base: 0.05,
            c_delta_tail: 0.08,
            candidate_stride: 5,
            tail_infl_ratio: 0.15,
            tail_infl_ramp_iters: 100,
            tail_tau_o_ratio: 0.15,
            tail_candidate_alpha_start: 0.5,
            tail_candidate_stride: 5,
            tail_ref_dilate_beta: 0.20,
            tail_cover_tau: 0.70,
            tail_cover_softness: 0.08,
            tail_cover_lambda_ratio: 0.10,
            tail_deoverlap_enable: true,
            tail_deoverlap_grid_fine: 50.0,
            tail_deoverlap_ring_fine: 5000.0,
            tail_deoverlap_grid_coarse: 200.0,
            tail_deoverlap_ring_coarse: 20000.0,
            tail_deoverlap_infl: 1.05,
            tail_deoverlap_search_kappa: 2.0,
            tail_deoverlap_lambda_expand: 1.0,
            tail_deoverlap_lambda_area: 1.0,
            tail_deoverlap_rho_max: 0.15,
            tail_deoverlap_use_elliptical_search: true,
            tail_deoverlap_lambda_topo: 0.0,
            tail_deoverlap_final_phi_iters: 300,
            lcr_honor_drc_floor: default_lcr_honor_drc_floor(),
            lcr_retry_enable: default_lcr_retry_enable(),
            lcr_retry_max_rounds: default_lcr_retry_max_rounds(),
            lcr_retry_infl_step: default_lcr_retry_infl_step(),
            lcr_retry_on_crossing: default_lcr_retry_on_crossing(),
            lcr_retry_cross_cover_frac: default_lcr_retry_cross_cover_frac(),
            lcr_retry_cross_criterion: default_lcr_retry_cross_criterion(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn write_temp_json(v: &serde_json::Value) -> PathBuf {
        let dir = std::env::temp_dir();
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        static SEQ: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let seq = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let path = dir.join(format!(
            "placement_config_v2_test_{}_{}_{}.json",
            std::process::id(),
            nanos,
            seq
        ));
        std::fs::write(&path, serde_json::to_string_pretty(v).unwrap()).unwrap();
        path
    }

    #[test]
    fn test_default_params() {
        let params = PlacementConfig::default();
        assert!(params.validate().is_ok());
        assert_eq!(params.max_iterations, 2000);
        assert_eq!(params.learning_rate, 200.0);
    }

    #[test]
    fn test_invalid_lr() {
        let mut params = PlacementConfig::default();
        params.learning_rate = -1.0;
        assert!(params.validate().is_err());
    }

    #[test]
    fn test_invalid_beta() {
        let mut params = PlacementConfig::default();
        params.adam_beta1 = 1.5;
        assert!(params.validate().is_err());
    }

    #[test]
    fn test_load_ota2_placement_config_v2() {
        let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let repo_root = manifest_dir.parent().unwrap().parent().unwrap();
        let placement_config = repo_root
            .join("src")
            .join("placer")
            .join("placement_config.example.json");
        assert!(
            placement_config.exists(),
            "missing {}",
            placement_config.display()
        );

        let raw: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&placement_config).unwrap()).unwrap();
        let expected_max_iterations = raw
            .get("common")
            .and_then(|x| x.get("maxIterations"))
            .and_then(|v| v.as_u64())
            .unwrap_or(2000) as usize;
        let expected_lambda_ap_max = raw
            .get("multiphase")
            .and_then(|x| x.get("lambdaApMax"))
            .and_then(|v| v.as_f64())
            .unwrap_or(PlacementConfig::default().lambda_ap_max);

        let params = PlacementConfig::from_json(&placement_config).unwrap();
        assert_eq!(params.max_iterations, expected_max_iterations);
        assert_eq!(params.gp_controller_mode, "multiphase");
        assert!(params.enable_wirelength);
        assert!(params.enable_overlap);
        assert!(params.enable_symmetry);
        assert!(!params.enable_boundary);
        assert!(!params.use_well_cluster);
        assert!(!params.enable_array);
        assert!(params.use_ap_wl);
        assert_eq!(params.wl_smoothing_init, 1.0);
        assert!((params.overlap_target - 5e-4).abs() < 1e-12);
        assert!((params.lambda_ap_max - expected_lambda_ap_max).abs() < 1e-12);
    }

    #[test]
    fn test_load_ota2_case_params_json() {
        let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let repo_root = manifest_dir.parent().unwrap().parent().unwrap();
        let params_json = repo_root
            .join("benchmark")
            .join("abstract")
            .join("ota2")
            .join("params.json");
        assert!(params_json.exists(), "missing {}", params_json.display());

        let raw: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&params_json).unwrap()).unwrap();
        let expected_max_iterations = raw
            .get("placement")
            .and_then(|x| x.get("global"))
            .and_then(|x| x.get("maxIterations"))
            .and_then(|v| v.as_u64())
            .unwrap_or(2000) as usize;
        let expected_lambda_ap_max = raw
            .get("placement")
            .and_then(|x| x.get("global"))
            .and_then(|x| x.get("lambdaApMax"))
            .and_then(|v| v.as_f64())
            .unwrap_or(PlacementConfig::default().lambda_ap_max);

        let params = PlacementConfig::from_case_params_json(&params_json).unwrap();
        assert_eq!(params.max_iterations, expected_max_iterations);
        assert_eq!(params.gp_controller_mode, "multiphase");
        assert!((params.lambda_ap_max - expected_lambda_ap_max).abs() < 1e-12);
    }

    #[test]
    fn test_reject_legacy_flat_json() {
        let v = serde_json::json!({
            "maxIterations": 1000
        });
        let path = write_temp_json(&v);
        let err = PlacementConfig::from_json(&path).unwrap_err();
        let _ = std::fs::remove_file(path);
        assert!(format!("{err}").contains("schemaVersion=2"));
    }

    #[test]
    fn test_reject_legacy_anchor_max_key_in_v2() {
        let v = serde_json::json!({
            "schemaVersion": 2,
            "mode": "multiphase",
            "common": {
                "maxIterations": 1000
            },
            "multiphase": {
                "anchorMax": 10.0
            }
        });
        let path = write_temp_json(&v);
        let err = PlacementConfig::from_json(&path).unwrap_err();
        let _ = std::fs::remove_file(path);
        assert!(format!("{err}").contains("unknown key `anchorMax`"));
    }

    #[test]
    fn test_case_params_missing_placement() {
        let v = serde_json::json!({
            "schemaVersion": 1
        });
        let path = write_temp_json(&v);
        let err = PlacementConfig::from_case_params_json(&path).unwrap_err();
        let _ = std::fs::remove_file(path);
        assert!(format!("{err}").contains("missing or invalid `placement` object"));
    }

    #[test]
    fn test_case_params_reject_unknown_group() {
        let v = serde_json::json!({
            "schemaVersion": 1,
            "placement": {
                "schemaVersion": 1,
                "mode": "multiphase",
                "globals": {}
            }
        });
        let path = write_temp_json(&v);
        let err = PlacementConfig::from_case_params_json(&path).unwrap_err();
        let _ = std::fs::remove_file(path);
        assert!(format!("{err}").contains("unknown key `globals`"));
    }

    #[test]
    fn test_case_params_reject_type_error() {
        let v = serde_json::json!({
            "schemaVersion": 1,
            "placement": {
                "schemaVersion": 1,
                "mode": "multiphase",
                "global": []
            }
        });
        let path = write_temp_json(&v);
        let err = PlacementConfig::from_case_params_json(&path).unwrap_err();
        let _ = std::fs::remove_file(path);
        assert!(format!("{err}").contains("`placement.global` must be a JSON object"));
    }

    #[test]
    fn test_case_params_default_fallback() {
        let v = serde_json::json!({
            "schemaVersion": 1,
            "placement": {
                "schemaVersion": 1,
                "mode": "multiphase",
                "global": {
                    "maxIterations": 1111
                }
            }
        });
        let path = write_temp_json(&v);
        let params = PlacementConfig::from_case_params_json(&path).unwrap();
        let _ = std::fs::remove_file(path);
        assert_eq!(params.max_iterations, 1111);
        assert_eq!(params.tau_o_ratio, PlacementConfig::default().tau_o_ratio);
    }

    #[test]
    fn test_case_params_deoverlap_snake_aliases() {
        let v = serde_json::json!({
            "schemaVersion": 1,
            "placement": {
                "schemaVersion": 1,
                "mode": "multiphase",
                "deoverlap": {
                    "deoverlap_search_kappa": 1.8,
                    "deoverlap_lambda_expand": 0.7,
                    "deoverlap_lambda_area": 1.2,
                    "deoverlap_rho_max": 0.25,
                    "infl_deoverlap": 1.15,
                    "deoverlap_use_elliptical_search": false,
                    "deoverlap_lambda_topo": 0.0,
                    "deoverlap_final_phi_iters": 123
                }
            }
        });
        let path = write_temp_json(&v);
        let params = PlacementConfig::from_case_params_json(&path).unwrap();
        let _ = std::fs::remove_file(path);
        assert!((params.tail_deoverlap_search_kappa - 1.8).abs() < 1e-12);
        assert!((params.tail_deoverlap_lambda_expand - 0.7).abs() < 1e-12);
        assert!((params.tail_deoverlap_lambda_area - 1.2).abs() < 1e-12);
        assert!((params.tail_deoverlap_rho_max - 0.25).abs() < 1e-12);
        assert!((params.tail_deoverlap_infl - 1.15).abs() < 1e-12);
        assert!(!params.tail_deoverlap_use_elliptical_search);
        assert!((params.tail_deoverlap_lambda_topo - 0.0).abs() < 1e-12);
        assert_eq!(params.tail_deoverlap_final_phi_iters, 123);
    }

    #[test]
    fn test_case_params_deoverlap_alias_conflict_rejected() {
        let v = serde_json::json!({
            "schemaVersion": 1,
            "placement": {
                "schemaVersion": 1,
                "mode": "multiphase",
                "deoverlap": {
                    "deoverlap_lambda_expand": 0.7,
                    "tailDeoverlapLambdaExpand": 1.0
                }
            }
        });
        let path = write_temp_json(&v);
        let err = PlacementConfig::from_case_params_json(&path).unwrap_err();
        let _ = std::fs::remove_file(path);
        assert!(format!("{err}").contains("conflicting values"));
    }

    #[test]
    fn test_case_params_deoverlap_final_phi_iters_reject_zero() {
        let v = serde_json::json!({
            "schemaVersion": 1,
            "placement": {
                "schemaVersion": 1,
                "mode": "multiphase",
                "deoverlap": {
                    "tailDeoverlapFinalPhiIters": 0
                }
            }
        });
        let path = write_temp_json(&v);
        let err = PlacementConfig::from_case_params_json(&path).unwrap_err();
        let _ = std::fs::remove_file(path);
        assert!(format!("{err}").contains("tail_deoverlap_final_phi_iters"));
    }

    #[test]
    fn test_case_params_global_pmos_alias() {
        let v = serde_json::json!({
            "schemaVersion": 1,
            "placement": {
                "schemaVersion": 1,
                "mode": "multiphase",
                "global": {
                    "pmos_hard_expand_for_gr": 700.0
                }
            }
        });
        let path = write_temp_json(&v);
        let params = PlacementConfig::from_case_params_json(&path).unwrap();
        let _ = std::fs::remove_file(path);
        assert!((params.pmos_hard_expand_for_gr - 700.0).abs() < 1e-12);
    }

    #[test]
    fn test_case_params_global_randomize_initial_placement() {
        assert!(!PlacementConfig::default().randomize_initial_placement);

        let v = serde_json::json!({
            "schemaVersion": 1,
            "placement": {
                "schemaVersion": 1,
                "mode": "multiphase",
                "global": {
                    "randomizeInitialPlacement": true
                }
            }
        });
        let path = write_temp_json(&v);
        let params = PlacementConfig::from_case_params_json(&path).unwrap();
        let _ = std::fs::remove_file(path);
        assert!(params.randomize_initial_placement);
    }

    #[test]
    fn test_anchor_base_tail_bounds() {
        let mut p = PlacementConfig::default();
        p.anchor_max_base = 2.0;
        p.anchor_max_tail = 10.0;
        assert!(p.validate().is_ok());
        p.anchor_max_tail = 1.0;
        assert!(p.validate().is_err());
    }

    #[test]
    fn test_cli_override_precedence() {
        let v = serde_json::json!({
            "schemaVersion": 2,
            "mode": "multiphase",
            "common": {
                "maxIterations": 1200
            },
            "multiphase": {}
        });
        let path = write_temp_json(&v);
        let (params, meta) =
            PlacementConfig::from_json_with_overrides(&path, &[String::from("maxIterations=1000")])
                .unwrap();
        let _ = std::fs::remove_file(path);
        assert_eq!(params.max_iterations, 1000);
        assert_eq!(meta.sources.get("maxIterations"), Some(&"cli".to_string()));
        assert_eq!(
            meta.sources.get("gpControllerMode"),
            Some(&"derived.mode".to_string())
        );
    }
}
