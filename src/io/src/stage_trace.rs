use std::fmt;
use std::sync::OnceLock;
use std::time::Instant;

#[derive(Debug, Clone, Copy)]
pub enum Stage {
    ReadInput,
    BuildDb,
    Init,
    SolveIter,
    LambdaUpdate,
    ExportJson,
    ExportGds,
    Other,
}

impl fmt::Display for Stage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Stage::ReadInput => "ReadInput",
            Stage::BuildDb => "BuildDB",
            Stage::Init => "Init",
            Stage::SolveIter => "SolveIter",
            Stage::LambdaUpdate => "LambdaUpdate",
            Stage::ExportJson => "ExportJson",
            Stage::ExportGds => "ExportGds",
            Stage::Other => "Other",
        };
        f.write_str(s)
    }
}

fn tracing_enabled() -> bool {
    matches!(
        std::env::var("HIER_STAGE_TRACE").ok().as_deref(),
        Some("1" | "true" | "TRUE" | "yes" | "YES")
    )
}

fn start_time() -> &'static Instant {
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now)
}

pub fn trace_stage(stage: Stage, context: &str) {
    if !tracing_enabled() {
        return;
    }
    let elapsed_ms = start_time().elapsed().as_millis();
    eprintln!("[stage {:>8}ms] {:<12} {}", elapsed_ms, stage, context);
}
