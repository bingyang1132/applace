#!/usr/bin/env bash
set -euo pipefail

export APPLACE_RUN_PREFIX="${APPLACE_RUN_PREFIX:-lg}"
source "$(cd "$(dirname "$0")" && pwd)/_common.sh" "${1:-}" "${2:-}"

LEGALIZATION_LOG="$OUT_DIR/legalization.log"
: > "$LEGALIZATION_LOG"

if [[ ! -f "$OUT_DIR/global_placement.json" ]]; then
  APPLACE_KEEP_MULTIPHASE=1 bash "$SCRIPT_DIR/01-global-placement.sh" "$CASE_DIR" "$OUT_DIR" >> "$LEGALIZATION_LOG" 2>&1
fi
APPLACE_KEEP_MULTIPHASE=1 APPLACE_KEEP_LEGALIZATION_LOG=1 bash "$SCRIPT_DIR/02a-reference-guided-expansion.sh" "$CASE_DIR" "$OUT_DIR" >> "$LEGALIZATION_LOG" 2>&1
APPLACE_KEEP_RGE=1 APPLACE_KEEP_MULTIPHASE=1 APPLACE_KEEP_LEGALIZATION_LOG=1 bash "$SCRIPT_DIR/02b-local-conflict-resolution.sh" "$CASE_DIR" "$OUT_DIR" >> "$LEGALIZATION_LOG" 2>&1

prune_outputs_unless_debug "$OUT_DIR" \
  reference_guided_expansion.json reference_guided_expansion.gds \
  local_conflict_resolution.json local_conflict_resolution.gds \
  legalization.log

echo "[ok] APPlace legalization done"
echo "  reference-guided expansion: $OUT_DIR/reference_guided_expansion.json"
echo "  local conflict resolution:  $OUT_DIR/local_conflict_resolution.json"
echo "  log:                        $LEGALIZATION_LOG"
exit 0
