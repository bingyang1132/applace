use serde::{Deserialize, Serialize};

pub const OUTPUT_SCHEMA_VERSION: u32 = 2;
pub const CONTROLLER_VERSION: &str = "tail_v2_2";
pub const TAIL_MODE: &str = "tail_v2_2_inflation_plus_dilated_reference_plus_cover";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Phase {
    Base,
    Tail,
}

impl Phase {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Base => "base",
            Self::Tail => "tail",
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct ConflictMetrics {
    pub num_components: usize,
    pub largest_component_size: usize,
    pub largest_component_area: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct CoverMetrics {
    pub num_pairs: usize,
    pub max_ratio: f64,
    pub mean_ratio: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct LambdaSnapshot {
    pub lambda_ovl: f64,
    pub lambda_sym: f64,
    pub ap_mix: f64,
    pub lambda_anchor: f64,
    pub lambda_cover_tail: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct IterRow {
    pub iter: usize,
    pub stage: String,
    pub loss_total: f64,
    pub wl_base: f64,
    pub wl_ap: f64,
    pub wl_mix_real: f64,
    pub ovl_raw_real: f64,
    pub ovl_raw_infl: f64,
    pub q_o_real: f64,
    pub q_o_infl: f64,
    pub sym_raw: f64,
    pub q_s: f64,
    pub lambda_ovl: f64,
    pub lambda_sym: f64,
    pub ap_mix: f64,
    pub lambda_anchor: f64,
    pub lambda_cover_tail: f64,
    pub alpha_infl: f64,
    pub ref_scale_s: f64,
    pub grad_norm_total: f64,
    pub grad_norm_wl: f64,
    pub grad_norm_ovl: f64,
    pub grad_norm_sym: f64,
    pub grad_norm_cover: f64,
    pub h_o: f64,
    pub h_s: f64,
    pub d_o: f64,
    pub d_s: f64,
    pub step_norm: f64,
    pub disp_rms: f64,
    pub disp_max: f64,
    pub disp_to_ckpt_rms: f64,
    pub disp_to_ckpt_max: f64,
    pub disp_to_tail_ckpt_rms: f64,
    pub disp_to_tail_ckpt_max: f64,
    pub disp_to_ref_rms: f64,
    pub disp_to_ref_max: f64,
    pub phi_step_rms: f64,
    pub phi_step_max: f64,
    pub clip_ratio_all: f64,
    pub clip_ratio_xy: f64,
    pub clip_ratio_phi: f64,
    pub mobility_enabled: bool,
    pub num_conflict_components_real: usize,
    pub largest_conflict_component_size_real: usize,
    pub largest_conflict_component_area_real: f64,
    pub num_conflict_components_infl: usize,
    pub largest_conflict_component_area_infl: f64,
    pub num_cover_pairs_real: usize,
    pub max_cover_ratio_real: f64,
    pub mean_cover_ratio_real: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct InflatedGeometrySummary {
    pub alpha_infl: f64,
    pub delta_0: f64,
    pub min_width_infl: f64,
    pub max_width_infl: f64,
    pub min_height_infl: f64,
    pub max_height_infl: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Candidate {
    pub label: String,
    pub stage: String,
    pub iter: usize,
    pub q_o_real: f64,
    pub q_o_infl: f64,
    pub q_s: f64,
    pub wl_mix_real: f64,
    pub ovl_raw_real: f64,
    pub ovl_raw_infl: f64,
    pub conflict_real: ConflictMetrics,
    pub conflict_infl: ConflictMetrics,
    pub cover_real: CoverMetrics,
    pub ap_mix: f64,
    pub alpha_infl: f64,
    pub ref_scale_s: f64,
    pub disp_to_ref_rms: f64,
    pub disp_to_ref_max: f64,
    pub lambdas: LambdaSnapshot,
    pub reduced_vars: Vec<f64>,
    pub full_positions: Vec<f64>,
    pub phi_tilde: Vec<f64>,
    pub inflated_geometry: InflatedGeometrySummary,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct CandidateSummaryEntry {
    pub label: String,
    pub stage: String,
    pub iter: usize,
    pub q_o_real: f64,
    pub q_o_infl: f64,
    pub q_s: f64,
    pub wl_mix_real: f64,
    pub num_conflict_components_real: usize,
    pub largest_conflict_component_area_real: f64,
    pub num_conflict_components_infl: usize,
    pub largest_conflict_component_area_infl: f64,
    pub num_cover_pairs_real: usize,
    pub max_cover_ratio_real: f64,
    pub mean_cover_ratio_real: f64,
    pub ref_scale_s: f64,
    pub disp_to_ref_rms: f64,
    pub alpha_infl: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct RunSummary {
    #[serde(rename = "schemaVersion")]
    pub schema_version: u32,
    pub initialization_mode: String,
    pub prior_init_used: bool,
    pub symmetry_binding_used: bool,
    pub base_end_iter: Option<usize>,
    pub tail_end_iter: Option<usize>,
    pub gp_end_iter: Option<usize>,
    pub mobility_activation_iteration: Option<usize>,
    pub ap_activation_iteration: Option<usize>,
    pub selected_final_candidate_label: Option<String>,
    pub selected_base_end_label: Option<String>,
    pub selected_tail_seed_label: Option<String>,
    pub controller_version: String,
    pub tail_mode: String,
    pub r_infl: f64,
    pub delta_0: f64,
    pub beta: f64,
    pub n_infl_ramp: usize,
    pub tau_cover: f64,
    pub s_cover: f64,
    pub lambda_cover_tail: f64,
    pub ap_mix_tail_ckpt: f64,
    pub phi_hard_frozen: bool,
    pub candidate_summary: Vec<CandidateSummaryEntry>,
}

pub fn better_base_composite(a: &Candidate, b: &Candidate) -> bool {
    let ta = (
        a.conflict_real.num_components,
        ordered_f64(a.conflict_real.largest_component_area),
        ordered_f64(a.q_o_real),
        ordered_f64(a.q_s),
        ordered_f64(a.wl_mix_real),
    );
    let tb = (
        b.conflict_real.num_components,
        ordered_f64(b.conflict_real.largest_component_area),
        ordered_f64(b.q_o_real),
        ordered_f64(b.q_s),
        ordered_f64(b.wl_mix_real),
    );
    ta < tb
}

pub fn better_tail_composite(a: &Candidate, b: &Candidate) -> bool {
    let ta = (
        a.cover_real.num_pairs,
        ordered_f64(a.cover_real.max_ratio),
        a.conflict_infl.num_components,
        ordered_f64(a.conflict_infl.largest_component_area),
        ordered_f64(a.q_o_infl),
        ordered_f64(a.disp_to_ref_rms),
        ordered_f64(a.wl_mix_real),
    );
    let tb = (
        b.cover_real.num_pairs,
        ordered_f64(b.cover_real.max_ratio),
        b.conflict_infl.num_components,
        ordered_f64(b.conflict_infl.largest_component_area),
        ordered_f64(b.q_o_infl),
        ordered_f64(b.disp_to_ref_rms),
        ordered_f64(b.wl_mix_real),
    );
    ta < tb
}

fn ordered_f64(v: f64) -> u64 {
    if v.is_nan() {
        return u64::MAX;
    }
    let bits = v.to_bits();
    if (bits >> 63) == 0 {
        bits | (1 << 63)
    } else {
        !bits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stage_as_str() {
        assert_eq!(Phase::Base.as_str(), "base");
        assert_eq!(Phase::Tail.as_str(), "tail");
    }

    #[test]
    fn test_better_composite_lexicographic() {
        let a = Candidate {
            label: "a".to_string(),
            stage: "tail".to_string(),
            iter: 1,
            q_o_real: 0.2,
            q_o_infl: 0.3,
            q_s: 0.3,
            wl_mix_real: 10.0,
            conflict_real: ConflictMetrics {
                num_components: 2,
                largest_component_size: 5,
                largest_component_area: 100.0,
            },
            conflict_infl: ConflictMetrics {
                num_components: 2,
                largest_component_size: 5,
                largest_component_area: 100.0,
            },
            cover_real: CoverMetrics {
                num_pairs: 1,
                max_ratio: 0.85,
                mean_ratio: 0.85,
            },
            disp_to_ref_rms: 1.0,
            ..Default::default()
        };

        let mut b = a.clone();
        b.cover_real.num_pairs = 2;
        assert!(better_tail_composite(&a, &b));

        let mut c = a.clone();
        c.cover_real.num_pairs = 1;
        c.cover_real.max_ratio = 0.75;
        assert!(!better_tail_composite(&a, &c));
    }
}
