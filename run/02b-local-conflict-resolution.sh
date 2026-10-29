#!/usr/bin/env bash
set -euo pipefail
export APPLACE_RUN_PREFIX="${APPLACE_RUN_PREFIX:-lg}"
source "$(cd "$(dirname "$0")" && pwd)/_common.sh" "${1:-}" "${2:-}"

RGE_JSON="$OUT_DIR/reference_guided_expansion.json"
LCR_JSON="$OUT_DIR/local_conflict_resolution.json"
LCR_GDS="$OUT_DIR/local_conflict_resolution.gds"
DEOVERLAP_GP_JSON="$OUT_DIR/multiphase_controller/checkpoints/deoverlap_end.gp.json"
DEOVERLAP_GDS="$OUT_DIR/multiphase_controller/checkpoints/deoverlap_end.gds"
export STAGE_LOG_DIR="$OUT_DIR"

if [[ ! -f "$RGE_JSON" ]]; then
  APPLACE_KEEP_MULTIPHASE=1 bash "$SCRIPT_DIR/02a-reference-guided-expansion.sh" "$CASE_DIR" "$OUT_DIR" >/dev/null
elif [[ ! -f "$DEOVERLAP_GP_JSON" || ! -f "$DEOVERLAP_GDS" ]]; then
  APPLACE_KEEP_MULTIPHASE=1 bash "$SCRIPT_DIR/01-global-placement.sh" "$CASE_DIR" "$OUT_DIR" >/dev/null
fi

run_stage_with_log local_conflict_resolution \
  materialize_checkpoint deoverlap_end "$LCR_JSON" "$LCR_GDS"

keep=(
  local_conflict_resolution.json local_conflict_resolution.gds local_conflict_resolution.log
  placement.log
)
if [[ "${APPLACE_KEEP_GLOBAL:-0}" == "1" ]]; then
  keep+=(global_placement.json global_placement.gds global_placement.log)
fi
if [[ "${APPLACE_KEEP_RGE:-0}" == "1" ]]; then
  keep+=(reference_guided_expansion.json reference_guided_expansion.gds reference_guided_expansion.log)
fi
if keep_multiphase_enabled; then
  keep+=(multiphase_controller)
fi
if [[ "${APPLACE_KEEP_LEGALIZATION_LOG:-0}" == "1" ]]; then
  keep+=(legalization.log)
fi
prune_outputs_unless_debug "$OUT_DIR" "${keep[@]}"

echo "[ok] Local Conflict Resolution done"
echo "  json: $LCR_JSON"
echo "  gds:  $LCR_GDS"
echo "  log:  $OUT_DIR/local_conflict_resolution.log"
