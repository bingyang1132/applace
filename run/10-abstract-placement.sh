#!/usr/bin/env bash
# Form two: placement from a purely geometric abstract case.
#
# Unlike every other script in run/, this one needs no PDK, no PCell GDS and no SPICE
# netlist -- the abstract case dir under benchmark/abstract/<case>/ is the whole input.
# That is what makes it the demo an outside user can run immediately.
#
#   bash run/10-abstract-placement.sh <case> [out_dir] [--set key=value]...
#
# <case> = ota1|ota2|ota3|comp, or a path to an abstract case dir.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

CASE_ARG="${1:-ota2}"
shift || true
OUT_ARG="${1:-}"
if [[ -n "$OUT_ARG" && "$OUT_ARG" != --* ]]; then
  shift
else
  OUT_ARG=""
fi

if [[ -d "$ROOT_DIR/benchmark/abstract/$CASE_ARG" ]]; then
  CASE_DIR="$ROOT_DIR/benchmark/abstract/$CASE_ARG"
elif [[ -d "$CASE_ARG" ]]; then
  CASE_DIR="$(cd "$CASE_ARG" && pwd)"
else
  echo "[error] abstract case not found: $CASE_ARG" >&2
  echo "        expected benchmark/abstract/<case> or a directory path" >&2
  exit 1
fi
CASE_NAME="$(basename "$CASE_DIR")"

for f in bbox.json pin.json sym.json params.json; do
  [[ -f "$CASE_DIR/$f" ]] || { echo "[error] missing $CASE_DIR/$f" >&2; exit 1; }
done

OUT_DIR="${OUT_ARG:-$ROOT_DIR/out/abstract/$CASE_NAME}"
mkdir -p "$OUT_DIR"

CARGO_BIN="${CARGO_BIN:-$(command -v cargo || echo "$HOME/.cargo/bin/cargo")}"
TARGET_DIR="${TARGET_DIR:-$ROOT_DIR/target}"
PROFILE="${PROFILE:-release}"
BIN="$TARGET_DIR/$PROFILE/run_abstract_placement"


# Unconditional: cargo, not the presence of a file, decides whether the binary
# on disk was built from the sources on disk. It is a no-op when nothing
# changed, and the alternative is measuring a binary that no longer
# corresponds to the code in the tree.
echo "[build] cargo build run_abstract_placement ($PROFILE)"
profile_flag=()
[[ "$PROFILE" == "release" ]] && profile_flag=(--release)
CARGO_TARGET_DIR="$TARGET_DIR" "$CARGO_BIN" build "${profile_flag[@]}" \
  --manifest-path "$ROOT_DIR/src/Cargo.toml" \
  --bin run_abstract_placement

"$BIN" \
  "$CASE_DIR/bbox.json" "$CASE_DIR/pin.json" "$CASE_DIR/sym.json" \
  "$CASE_DIR/params.json" "$OUT_DIR/placement.json" "$@"

echo "[ok] abstract placement done"
echo "  case:      $CASE_DIR"
echo "  placement: $OUT_DIR/placement.json"
echo "  layout:    $OUT_DIR/placement.gds   (outline level: 1/0 outline, 1/1 label,"
echo "                                        2/0 pin anchor, 3/0 region,"
echo "                                        4/0 PMOS guard-ring reservation,"
echo "                                        5/0 PMOS marker, 5/1 \"PMOS\" text)"
echo "  summary:   $OUT_DIR/placement_summary.json"
echo ""
echo "  annotated picture:"
echo "    python3 scripts/plot_placement.py $OUT_DIR --case $CASE_DIR --out $OUT_DIR/placement.png"
