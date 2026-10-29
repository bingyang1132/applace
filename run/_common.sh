#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MANIFEST="$ROOT_DIR/src/Cargo.toml"

if [[ ! -f "$MANIFEST" ]]; then
  echo "[error] missing Cargo manifest: $MANIFEST" >&2
  exit 1
fi

CARGO_BIN="${CARGO_BIN:-$(command -v cargo || true)}"
if [[ -z "$CARGO_BIN" && -x "$HOME/.cargo/bin/cargo" ]]; then
  CARGO_BIN="$HOME/.cargo/bin/cargo"
fi
if [[ -z "$CARGO_BIN" ]]; then
  echo "[error] cargo not found" >&2
  exit 1
fi

DEFAULT_CASE="$ROOT_DIR/benchmark/ota2"
CASE_ARG="${1:-}"
OUT_ARG="${2:-}"

if [[ -z "$CASE_ARG" ]]; then
  CASE_DIR="$DEFAULT_CASE"
elif [[ -d "$ROOT_DIR/benchmark/$CASE_ARG" ]]; then
  # Support short benchmark names, e.g. "ota1", "ota2", "ota3".
  CASE_DIR="$ROOT_DIR/benchmark/$CASE_ARG"
else
  # Fall back to caller-provided directory path.
  CASE_DIR="$CASE_ARG"
fi

if [[ ! -d "$CASE_DIR" ]]; then
  echo "[error] case dir not found: $CASE_DIR" >&2
  echo "        tip: pass benchmark name (e.g. ota2) or full/relative directory path" >&2
  exit 1
fi
CASE_DIR="$(cd "$CASE_DIR" && pwd)"

if [[ -n "$OUT_ARG" ]]; then
  OUT_DIR="$OUT_ARG"
else
  RUN_TIMESTAMP="${APPLACE_RUN_TIMESTAMP:-$(date +%Y%m%d_%H%M%S_%N)}"
  RUN_PREFIX="${APPLACE_RUN_PREFIX:-}"
  if [[ -n "$RUN_PREFIX" ]]; then
    RUN_TIMESTAMP="${RUN_PREFIX}_${RUN_TIMESTAMP}"
  fi
  OUT_DIR="$CASE_DIR/out/$RUN_TIMESTAMP"
fi

CASE_NAME="$(basename "$CASE_DIR")"
if [[ -f "$CASE_DIR/${CASE_NAME}.sp" ]]; then
  NETLIST="$CASE_DIR/${CASE_NAME}.sp"
else
  shopt -s nullglob
  netlist_candidates=("$CASE_DIR"/*.sp)
  shopt -u nullglob
  if [[ ${#netlist_candidates[@]} -eq 1 ]]; then
    NETLIST="${netlist_candidates[0]}"
  else
    echo "[error] cannot determine netlist .sp in case dir: $CASE_DIR" >&2
    if [[ ${#netlist_candidates[@]} -gt 1 ]]; then
      echo "        found multiple candidates: ${netlist_candidates[*]}" >&2
    fi
    exit 1
  fi
fi
PCELL="$CASE_DIR/custom"
PARAMS_JSON="$CASE_DIR/params.json"
SYM_JSON="$CASE_DIR/sym.json"

# Rule deck selection, in order:
#   APPLACE_PDK=<path>   an absolute path, or one relative to the repo root
#   <case>/lef.json      a deck sitting next to the design
#   pdk/example_lef.json the placeholder deck shipped with this repository
#
# The shipped deck contains placeholder values only. Supply your own deck to run
# against a real process; docs/INPUT_FORMAT.md documents the fields that are read.
if [[ -n "${APPLACE_PDK:-}" ]]; then
  if [[ "$APPLACE_PDK" = /* ]]; then
    PDK_JSON="$APPLACE_PDK"
  else
    PDK_JSON="$ROOT_DIR/$APPLACE_PDK"
  fi
elif [[ -f "$CASE_DIR/lef.json" ]]; then
  PDK_JSON="$CASE_DIR/lef.json"
else
  PDK_JSON="$ROOT_DIR/pdk/example_lef.json"
fi
if [[ ! -f "$NETLIST" ]]; then
  echo "[error] netlist not found: $NETLIST" >&2
  exit 1
fi
if [[ ! -d "$PCELL" ]]; then
  echo "[error] custom pcell dir not found: $PCELL" >&2
  exit 1
fi
if [[ ! -f "$PDK_JSON" ]]; then
  echo "[error] pdk json not found: $PDK_JSON" >&2
  exit 1
fi
if [[ ! -f "$PARAMS_JSON" ]]; then
  echo "[error] params json not found: $PARAMS_JSON" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

TARGET_DIR="${TARGET_DIR:-$ROOT_DIR/target}"
PROFILE="${PROFILE:-release}"
BIN_DIR="$TARGET_DIR/$PROFILE"
mkdir -p "$TARGET_DIR"

APPLACE_DEBUG="${APPLACE_DEBUG:-${DEBUG:-0}}"

debug_enabled() {
  [[ "$APPLACE_DEBUG" == "1" || "$APPLACE_DEBUG" == "true" || "$APPLACE_DEBUG" == "TRUE" || "$APPLACE_DEBUG" == "yes" || "$APPLACE_DEBUG" == "YES" ]]
}

prune_outputs_unless_debug() {
  debug_enabled && return 0
  local dir="$1"; shift
  local keep
  local path
  local name
  [[ -d "$dir" ]] || return 0
  find "$dir" -mindepth 1 -maxdepth 1 | while IFS= read -r path; do
    name="$(basename "$path")"
    keep=0
    for allowed in "$@"; do
      if [[ "$name" == "$allowed" ]]; then
        keep=1
        break
      fi
    done
    if [[ $keep -eq 0 ]]; then
      rm -rf "$path"
    fi
  done
}

keep_multiphase_enabled() {
  [[ "${APPLACE_KEEP_MULTIPHASE:-${APPLACE_KEEP_INTERNAL:-0}}" == "1" \
    || "${APPLACE_KEEP_MULTIPHASE:-${APPLACE_KEEP_INTERNAL:-0}}" == "true" \
    || "${APPLACE_KEEP_MULTIPHASE:-${APPLACE_KEEP_INTERNAL:-0}}" == "TRUE" \
    || "${APPLACE_KEEP_MULTIPHASE:-${APPLACE_KEEP_INTERNAL:-0}}" == "yes" \
    || "${APPLACE_KEEP_MULTIPHASE:-${APPLACE_KEEP_INTERNAL:-0}}" == "YES" ]]
}

checkpoint_json_path() {
  local label="$1"
  printf '%s\n' "$OUT_DIR/multiphase_controller/checkpoints/${label}.gp.json"
}

checkpoint_gds_path() {
  local label="$1"
  printf '%s\n' "$OUT_DIR/multiphase_controller/checkpoints/${label}.gds"
}

ensure_checkpoint() {
  local label="$1"
  local src_json
  local src_gds
  src_json="$(checkpoint_json_path "$label")"
  src_gds="$(checkpoint_gds_path "$label")"
  if [[ -f "$src_json" && -f "$src_gds" ]]; then
    return 0
  fi

  echo "[info] missing multi-phase checkpoint ${label}; running Global Placement with internal checkpoints kept"
  APPLACE_KEEP_MULTIPHASE=1 bash "$SCRIPT_DIR/01-global-placement.sh" "$CASE_DIR" "$OUT_DIR"

  if [[ ! -f "$src_json" || ! -f "$src_gds" ]]; then
    echo "[error] missing multi-phase checkpoint after Global Placement: $src_json / $src_gds" >&2
    return 1
  fi
}

materialize_checkpoint() {
  local label="$1"
  local out_json="$2"
  local out_gds="$3"
  local src_json
  local src_gds

  ensure_checkpoint "$label"
  src_json="$(checkpoint_json_path "$label")"
  src_gds="$(checkpoint_gds_path "$label")"
  mkdir -p "$(dirname "$out_json")" "$(dirname "$out_gds")"
  cp -f "$src_json" "$out_json"
  cp -f "$src_gds" "$out_gds"
  echo "[materialize] ${label} -> ${out_json}"
  echo "[materialize] ${label} -> ${out_gds}"
}

cargo_build_bin() {
  local bin="$1"; shift
  local profile_flags=()
  # With PROFILE=release the build must really be a release build, otherwise BIN_DIR points
  # at target/release while nothing ever puts a binary there (perf runs must use release).
  if [[ "$PROFILE" == "release" ]]; then
    profile_flags+=(--release)
  fi
  "$CARGO_BIN" build \
    --manifest-path "$MANIFEST" \
    --target-dir "$TARGET_DIR" \
    -p applace-io \
    --bin "$bin" \
    "${profile_flags[@]}" \
    "$@"
}

# Always ask cargo whether the binary on disk was built from the sources on
# disk: it is the only thing here that knows, and it is a no-op when nothing
# changed. A plain existence test would happily run a binary compiled from
# code that has since been edited, and say nothing about it.
run_stage_bin() {
  local bin="$1"; shift
  local exe="$BIN_DIR/$bin"
  echo "[build] cargo build $bin ($PROFILE)"
  cargo_build_bin "$bin"
  "$exe" "$@"
}


run_stage_with_log() {
  local stage="$1"; shift
  local log_dir="${STAGE_LOG_DIR:-$OUT_DIR}"
  mkdir -p "$log_dir"
  local logfile="$log_dir/${stage}.log"
  local start_iso
  local end_iso
  local start_epoch
  local end_epoch
  local elapsed_sec

  # Truncate on each run; keep deterministic file name.
  : > "$logfile"
  echo "[log] stage=${stage} file=${logfile}" | tee -a "$logfile"
  echo "[input] ROOT_DIR=${ROOT_DIR}" | tee -a "$logfile"
  echo "[input] CASE_DIR=${CASE_DIR}" | tee -a "$logfile"
  echo "[input] OUT_DIR=${OUT_DIR}" | tee -a "$logfile"
  echo "[input] NETLIST=${NETLIST}" | tee -a "$logfile"
  echo "[input] NETLIST_DIR=$(dirname "$NETLIST")" | tee -a "$logfile"
  echo "[input] PCELL=${PCELL}" | tee -a "$logfile"
  echo "[input] PCELL_DIR=${PCELL}" | tee -a "$logfile"
  echo "[input] PDK_JSON=${PDK_JSON}" | tee -a "$logfile"
  echo "[input] PDK_DIR=$(dirname "$PDK_JSON")" | tee -a "$logfile"
  echo "[input] PARAMS_JSON=${PARAMS_JSON}" | tee -a "$logfile"
  echo "[input] PARAMS_DIR=$(dirname "$PARAMS_JSON")" | tee -a "$logfile"
  if [[ -f "$SYM_JSON" ]]; then
    echo "[input] SYM_JSON=${SYM_JSON}" | tee -a "$logfile"
    echo "[input] SYM_DIR=$(dirname "$SYM_JSON")" | tee -a "$logfile"
  else
    echo "[input] SYM_JSON=(missing, optional)" | tee -a "$logfile"
  fi
  start_iso="$(date -Iseconds)"
  start_epoch="$(date +%s)"
  echo "[timing] stage=${stage} start=${start_iso}" | tee -a "$logfile"

  set +e
  "$@" 2>&1 | tee -a "$logfile"
  local cmd_status=${PIPESTATUS[0]}
  set -e
  end_iso="$(date -Iseconds)"
  end_epoch="$(date +%s)"
  elapsed_sec=$((end_epoch - start_epoch))
  echo "[timing] stage=${stage} end=${end_iso} elapsed_sec=${elapsed_sec}" | tee -a "$logfile"

  if [[ $cmd_status -ne 0 ]]; then
    echo "[error] ${stage} failed (exit=${cmd_status}). See log: ${logfile}" | tee -a "$logfile" >&2
    return $cmd_status
  fi

  return 0
}
