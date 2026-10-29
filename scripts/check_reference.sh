#!/usr/bin/env bash
# Check the four example placements against what this repository says they should be.
#
#   bash scripts/check_reference.sh              # invariants, on any machine
#   bash scripts/check_reference.sh ota2 comp    # just these cases
#   bash scripts/check_reference.sh --strict     # byte-for-byte, same machine only
#
# DEFAULT MODE checks the properties a placement either has or does not have, against
# `benchmark/abstract/<case>/reference/placement_summary.json`:
#
#   * the run completes and writes a placement;
#   * device_count, pmos_count and sym_pairs match the reference exactly -- the problem
#     the placer was handed is the problem it solved;
#   * sym_deviation is 0 -- every declared symmetry came out exact;
#   * overlap_pairs is 0 -- no two devices overlap;
#   * every device lies inside the declared region, when the case declares one;
#   * HPWL is within [0.6, 1.6] of the reference. That is a crash barrier, not a
#     precision gate: it catches a placement that has fallen apart, and deliberately
#     does not care about a few per cent of quality either way.
#
# STRICT MODE (`--strict`) compares `placement.json` byte for byte against
# `benchmark/abstract/<case>/reference/placement.json`. That comparison only holds on
# the machine and toolchain the reference files were generated on. Gradient descent is
# sensitive to the last bit of a floating-point result, and a different libm or a
# different code generator moves the run into a different -- equally good -- local
# optimum, tens of nanometres or more away. Cross-platform CI therefore runs the
# default mode. See section 5 (Determinism) of docs/OUTPUT.md.
#
# Either way, `placement.gds` is not compared: GDSII stores a wall-clock timestamp in
# its BGNLIB and BGNSTR records, so a GDS file is never identical even to itself. The
# JSON carries the same coordinates.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

STRICT=0
CASES=()
for arg in "$@"; do
  case "$arg" in
    --strict) STRICT=1 ;;
    -h|--help) sed -n '2,31p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "unknown option: $arg" >&2; exit 2 ;;
    *)  CASES+=("$arg") ;;
  esac
done
if [[ ${#CASES[@]} -eq 0 ]]; then
  CASES=(ota1 ota2 ota3 comp)
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "[error] python3 is required to read the placement summaries" >&2
  exit 2
fi

OUT_ROOT="$(mktemp -d)"
trap 'rm -rf "$OUT_ROOT"' EXIT

failed=0
for case in "${CASES[@]}"; do
  ref_dir="$ROOT_DIR/benchmark/abstract/$case/reference"
  out="$OUT_ROOT/$case"
  if ! bash "$ROOT_DIR/run/10-abstract-placement.sh" "$case" "$out" >"$out.log" 2>&1; then
    echo "  [FAIL] $case: the run itself failed; see $out.log" >&2
    tail -20 "$out.log" >&2
    failed=1
    continue
  fi
  # Which binary actually ran, and how it was built. A mismatch here (a debug
  # binary, a different toolchain) explains most "the numbers differ" reports
  # before anyone looks at the placement itself. The run script always invokes
  # cargo -- it is the only thing that knows whether the binary on disk was built
  # from the sources on disk -- so this line is present on every run.
  grep -E '^\[build\]' "$out.log" >&2 \
    || echo "  [warn] $case: the run printed no build line; which binary ran is unknown" >&2

  if (( STRICT )); then
    ref="$ref_dir/placement.json"
    if [[ ! -f "$ref" ]]; then
      echo "  [FAIL] $case: no reference at $ref" >&2; failed=1; continue
    fi
    if cmp -s "$ref" "$out/placement.json"; then
      echo "  [ok]   $case: placement.json matches the reference byte for byte"
    else
      echo "  [FAIL] $case: placement.json differs from the reference" >&2
      diff <(head -40 "$ref") <(head -40 "$out/placement.json") | head -20 >&2 || true
      failed=1
    fi
    continue
  fi

  ref="$ref_dir/placement_summary.json"
  if [[ ! -f "$ref" ]]; then
    echo "  [FAIL] $case: no reference summary at $ref" >&2; failed=1; continue
  fi
  if python3 - "$case" "$ref" "$out/placement_summary.json" <<'PY'
import json
import sys

case, ref_path, got_path = sys.argv[1:4]
ref = json.load(open(ref_path))["global"]
got_doc = json.load(open(got_path))
got = got_doc["global"]

problems = []

for key in ("device_count", "pmos_count", "sym_pairs"):
    if got[key] != ref[key]:
        problems.append(f"{key}: {got[key]}, reference says {ref[key]}")

if got["sym_deviation"] != 0:
    problems.append(f"sym_deviation: {got['sym_deviation']} DBU, want 0 "
                    "(a declared symmetry did not come out symmetric)")

if got["overlap_pairs"] != 0:
    problems.append(f"overlap_pairs: {got['overlap_pairs']}, want 0")

bbox, region = got.get("layout_bbox"), got.get("region")
if bbox is None:
    problems.append("layout_bbox is null: the placement has no extent")
elif region is not None:
    for lo, hi in (("xl", "xh"), ("yl", "yh")):
        if bbox[lo] < region[lo] or bbox[hi] > region[hi]:
            problems.append(
                f"devices leave the declared region on {lo[0]}: "
                f"[{bbox[lo]}, {bbox[hi]}] outside [{region[lo]}, {region[hi]}]")

lo_band, hi_band = 0.6, 1.6
ratio = got["hpwl"] / ref["hpwl"] if ref["hpwl"] > 0 else 1.0
if not lo_band <= ratio <= hi_band:
    problems.append(f"hpwl: {got['hpwl']:.0f}, {ratio:.2f}x the reference "
                    f"{ref['hpwl']:.0f}, outside [{lo_band}, {hi_band}]")

if problems:
    print(f"  [FAIL] {case}:", file=sys.stderr)
    for p in problems:
        print(f"         {p}", file=sys.stderr)
    sys.exit(1)

print(f"  [ok]   {case}: {got['device_count']} devices, "
      f"{got['sym_pairs']} symmetry pairs exact, no overlap, "
      f"hpwl {ratio:.2f}x reference")
PY
  then :; else failed=1; fi
done

if (( failed )); then
  echo >&2
  if (( STRICT )); then
    echo "Strict mode compares bytes, and bytes only survive one machine and one" >&2
    echo "toolchain. If this is CI, or a different OS or compiler, run without" >&2
    echo "--strict. If it is the machine the references came from, then either your" >&2
    echo "build is not this build, or a change moved a device -- and if it is the" >&2
    echo "second, regenerate the reference files in the same commit and say in the" >&2
    echo "pull request what moved and why." >&2
  else
    echo "An invariant failure is a real defect: overlap, a broken symmetry, a device" >&2
    echo "outside its region, or a placement that has come apart. It is not a" >&2
    echo "floating-point difference -- those cannot move any of these numbers." >&2
  fi
  exit 1
fi

if (( STRICT )); then
  echo "[ok] every reference placement reproduced byte for byte"
else
  echo "[ok] every example case placed, with every invariant met"
fi
