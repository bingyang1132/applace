#!/usr/bin/env bash
# Pre-build the stage binaries so the first run does not pay for compilation.
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/_common.sh" "${1:-}" "${2:-}"

cargo_build_bin run_placement_stage
cargo_build_bin run_abstract_placement

echo "[ok] built APPlace stage binaries under: $BIN_DIR"
ls -lh "$BIN_DIR"/run_placement_stage 2>/dev/null || true
ls -lh "$BIN_DIR"/run_abstract_placement 2>/dev/null || true
