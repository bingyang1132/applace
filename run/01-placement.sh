#!/usr/bin/env bash
set -euo pipefail

export APPLACE_RUN_PREFIX="${APPLACE_RUN_PREFIX:-lg}"
source "$(cd "$(dirname "$0")" && pwd)/_common.sh" "${1:-}" "${2:-}"

PLACEMENT_LOG="$OUT_DIR/placement.log"
: > "$PLACEMENT_LOG"

APPLACE_KEEP_MULTIPHASE=1 bash "$SCRIPT_DIR/01-global-placement.sh" "$CASE_DIR" "$OUT_DIR" >> "$PLACEMENT_LOG" 2>&1
APPLACE_KEEP_GLOBAL=1 APPLACE_KEEP_MULTIPHASE=1 bash "$SCRIPT_DIR/02a-reference-guided-expansion.sh" "$CASE_DIR" "$OUT_DIR" >> "$PLACEMENT_LOG" 2>&1
APPLACE_KEEP_GLOBAL=1 APPLACE_KEEP_RGE=1 APPLACE_KEEP_MULTIPHASE=1 bash "$SCRIPT_DIR/02b-local-conflict-resolution.sh" "$CASE_DIR" "$OUT_DIR" >> "$PLACEMENT_LOG" 2>&1

prune_outputs_unless_debug "$OUT_DIR" \
  global_placement.json global_placement.gds \
  reference_guided_expansion.json reference_guided_expansion.gds \
  local_conflict_resolution.json local_conflict_resolution.gds \
  placement.log
# The multi-phase checkpoint directory is deliberately not in the keep list:
# it is internal to global placement. Set APPLACE_DEBUG=1 to keep everything.

echo "[ok] APPlace placement done"
echo "  global placement:              $OUT_DIR/global_placement.json"
echo "  reference-guided expansion:    $OUT_DIR/reference_guided_expansion.json"
echo "  local conflict resolution:     $OUT_DIR/local_conflict_resolution.json"
echo "  log:                           $PLACEMENT_LOG"
exit 0
