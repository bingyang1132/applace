#!/usr/bin/env bash
set -euo pipefail
export APPLACE_RUN_PREFIX="${APPLACE_RUN_PREFIX:-gp}"
source "$(cd "$(dirname "$0")" && pwd)/_common.sh" "${1:-}" "${2:-}"

GP_JSON="$OUT_DIR/global_placement.json"
GP_GDS="$OUT_DIR/global_placement.gds"
export STAGE_LOG_DIR="$OUT_DIR"

if [[ -f "$SYM_JSON" ]]; then
  run_stage_with_log global_placement \
    run_stage_bin run_placement_stage \
    "$NETLIST" "$PCELL" "$PDK_JSON" "$PARAMS_JSON" "$GP_JSON" "$GP_GDS" "$SYM_JSON"
else
  run_stage_with_log global_placement \
    run_stage_bin run_placement_stage \
    "$NETLIST" "$PCELL" "$PDK_JSON" "$PARAMS_JSON" "$GP_JSON" "$GP_GDS"
fi

keep=(global_placement.json global_placement.gds global_placement.log placement.log legalization.log)
if keep_multiphase_enabled; then
  keep+=(multiphase_controller)
fi
prune_outputs_unless_debug "$OUT_DIR" \
  "${keep[@]}"

echo "[ok] Global Placement done"
echo "  json: $GP_JSON"
echo "  gds:  $GP_GDS"
echo "  log:  $OUT_DIR/global_placement.log"
