#!/usr/bin/env bash
set -euo pipefail
export APPLACE_RUN_PREFIX="${APPLACE_RUN_PREFIX:-lg}"
source "$(cd "$(dirname "$0")" && pwd)/_common.sh" "${1:-}" "${2:-}"

GP_JSON="$OUT_DIR/global_placement.json"
RGE_JSON="$OUT_DIR/reference_guided_expansion.json"
RGE_GDS="$OUT_DIR/reference_guided_expansion.gds"
TAIL_GP_JSON="$OUT_DIR/multiphase_controller/checkpoints/tail_end.gp.json"
TAIL_GDS="$OUT_DIR/multiphase_controller/checkpoints/tail_end.gds"
export STAGE_LOG_DIR="$OUT_DIR"

if [[ ! -f "$GP_JSON" || ! -f "$TAIL_GP_JSON" || ! -f "$TAIL_GDS" ]]; then
  APPLACE_KEEP_MULTIPHASE=1 bash "$SCRIPT_DIR/01-global-placement.sh" "$CASE_DIR" "$OUT_DIR" >/dev/null
fi

run_stage_with_log reference_guided_expansion \
  materialize_checkpoint tail_end "$RGE_JSON" "$RGE_GDS"

keep=(reference_guided_expansion.json reference_guided_expansion.gds reference_guided_expansion.log placement.log)
if [[ "${APPLACE_KEEP_GLOBAL:-0}" == "1" ]]; then
  keep+=(global_placement.json global_placement.gds global_placement.log)
fi
if keep_multiphase_enabled; then
  keep+=(multiphase_controller)
fi
if [[ "${APPLACE_KEEP_LEGALIZATION_LOG:-0}" == "1" ]]; then
  keep+=(legalization.log)
fi
prune_outputs_unless_debug "$OUT_DIR" "${keep[@]}"

echo "[ok] Reference-Guided Expansion done"
echo "  json: $RGE_JSON"
echo "  gds:  $RGE_GDS"
echo "  log:  $OUT_DIR/reference_guided_expansion.log"
