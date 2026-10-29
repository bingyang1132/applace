# Output format

Every APPlace run writes a coordinate map as JSON and the same placement as an
outline-level GDS layout. The geometric form additionally writes a summary of the
placement as data, and a script is shipped that turns any of it into an annotated
picture. Coordinates are in database units; one database unit is one nanometre.

---

## 1. `placement.json` -- the coordinate map

```json
{
  "OTA_2_C0": [18712.529164854262, 43312.281093629594],
  "OTA_2_C1": [59810.47083514574,  43312.281093629594],
  "OTA_2_C2": [13427.301969805674, 17492.65993431208]
}
```

An object keyed by device name, exactly the names you supplied. Each value is a
two-element array `[x, y]`: the **lower-left corner** of the device outline, in
database units.

The values are unrounded. The placer solves in floating point and this file
reports what it solved; if you need integers, round at the point of use. The GDS
output rounds to the nearest unit, so the two files can differ by up to half a
unit per coordinate. That is expected, and the JSON is the authoritative answer.

The device flow writes the same schema, once per stage, under the stage's name
rather than `placement.json` -- see section 3.

---

## 2. `placement.gds` -- the outline-level layout

The same placement, as a layout you can open in any GDS viewer. It contains one
structure holding, for every device, a rectangle at the placed position with the
device's width and height, a text label carrying the device name, and a small
anchor mark at the device centre. The declared placement region is drawn as one
more rectangle.

This is deliberately an **outline-level** view: it shows where devices landed and
how much room they take, not what is inside them. No PCell polygons are copied
into it, and no process layer map is applied.

| Purpose | Layer | Datatype |
| --- | --- | --- |
| Device outline | 1 | 0 |
| Device name label | 1 | 1 |
| Pin anchor at the device centre | 2 | 0 |
| Placement region | 3 | 0 |
| PMOS guard-ring reservation | 4 | 0 |
| PMOS marker (a copy of the outline) | 5 | 0 |
| The text `PMOS` | 5 | 1 |

Layers 4 and 5 are what makes the picture readable. Open a placement with only
the outlines and the obvious question has no answer on the screen: which of these
is a PMOS, and why is there a gap around it? Layer 5 answers the first. Layer 4
answers the second -- it is the **guard-ring reservation**, the room the placer
kept free around every PMOS so a well ring can be dropped in afterwards, and it
is the usual reason two devices sit further apart than their outlines require.

The reservation is drawn as the band itself -- four rectangles, the ring between
the device outline and the outline grown by the reservation -- rather than as one
grown box, so what you see is exactly the area that was reserved and nothing
covers the device it belongs to. It is omitted entirely when the reservation is
zero: the drawing never implies room that was not kept.

These numbers are local to this output. They are not a foundry layer map and
carry no process meaning; pick any viewer layer colours you like.

Coordinates are integers, rounded from the solved positions as described above.

---

## 2a. `placement_summary.json` -- the placement as data

Written by the geometric form beside `placement.json`. It is the same placement
with the facts needed to judge it attached: what each device is, how much room
was reserved for it, and which pair it is mirrored with.

```json
{
  "schema": "applace.placement_summary/1",
  "form": "abstract",
  "devices": [
    {
      "name": "COMPARATOR_test_flow_M10",
      "x": 6571.680376257122, "y": 8245.145198213822,
      "w": 880.0, "h": 1610.0,
      "isPmos": true,
      "gr_reservation": 650.0,
      "symmetry_group": "COMPARATOR_test_flow",
      "symmetry_partner": "COMPARATOR_test_flow_M12",
      "symmetry_axis": "vertical"
    }
  ],
  "global": {
    "device_count": 17, "pmos_count": 8, "net_count": 12, "sym_pairs": 9,
    "sym_deviation": 0.0,
    "gr_reservation": 650.0,
    "spacing_floor": 0.0,
    "region": {"xl": 0, "yl": 0, "xh": 17290, "yh": 17290},
    "layout_bbox": {"xl": 4421.63, "yl": 151.32, "xh": 12868.37, "yh": 17083.23},
    "hpwl": 103042.61165848881,
    "overlap_pairs": 0,
    "unit": "DBU = nm"
  }
}
```

Per device:

| Field | Meaning |
| --- | --- |
| `name` | The device name, exactly as you supplied it. |
| `x`, `y` | Lower-left corner of the outline, unrounded -- the same numbers as `placement.json`. |
| `w`, `h` | Outline size, from `bbox.json`. |
| `isPmos` | Whether the device is a PMOS. Decides the guard-ring reservation. |
| `gr_reservation` | Room kept free on **each side** of this device, in database units. `0` for anything that is not a PMOS. |
| `symmetry_group` | The `sym.json` group this device's pair was declared in, or `null`. |
| `symmetry_partner` | The device it is mirrored with, or `null` when it is self-symmetric (declared symmetric about the axis on its own) or in no pair. |
| `symmetry_axis` | `"vertical"` or `"horizontal"` -- the group's axis. |

Globally:

| Field | Meaning |
| --- | --- |
| `device_count`, `pmos_count`, `net_count`, `sym_pairs` | Sizes of the problem that was solved. |
| `sym_deviation` | How far, in database units, the worst symmetric device sits from where its group's axis puts it. `0` means every declared symmetry came out exact, which is what the legalizer is for; anything else is a constraint that was not met. |
| `gr_reservation` | The per-side PMOS guard-ring reservation this run used. |
| `spacing_floor` | The minimum clean gap legalization keeps between any two devices. `0` means none was configured, which is the geometric form's default: it ships no deck to derive one from. |
| `region` | The placement region declared in `bbox.json`, or `null`. |
| `layout_bbox` | The box the placement actually occupies. Compare it with `region` to see how much of the declared area was used. |
| `hpwl` | Half-perimeter wire length summed over nets, measured between **device centres**. Centres rather than pins because the geometric form states no pin positions -- a pin may sit anywhere on its device -- so a pin-based number would report the optimizer's internal variables instead of the placement. |
| `overlap_pairs` | Device pairs whose outlines still overlap with positive area. `0` is what you want. Touching edge to edge is not an overlap. |

`hpwl`, `overlap_pairs` and `sym_deviation` are recomputed from the boxes in this
same file, so they describe the placement as published rather than any internal
bookkeeping. They are also what `scripts/check_reference.sh` compares by default:
see section 5 below for why that check is invariants rather
than bytes.

---

## 2b. `scripts/plot_placement.py` -- the annotated picture

```bash
python3 scripts/plot_placement.py out/abstract/comp --out comp.png
```

Draws what the summary describes: every device as a labelled box, PMOS devices
distinctly coloured with their guard-ring reservation as a dashed frame,
symmetric pairs sharing a colour with the realized axis drawn through them, the
declared region, the box the placement used, and -- in red -- any device still in
an overlapping pair. Only matplotlib is needed.

For the device flow, which writes no summary, point it at a stage output and tell
it where the device geometry lives:

```bash
python3 scripts/plot_placement.py out/run/local_conflict_resolution.json \
    --case <dir with bbox.json, sym.json, pin.json> --no-region
```

`--no-region` because the region in a geometric case directory is not the one the
device flow used: the device flow derives its own from total device area and
never reads that field.

Pass more than one run to get them side by side on a single common scale, which
is the quickest way to see what a parameter change actually did:

```bash
python3 scripts/plot_placement.py runA runB --out compare.png
```

---

## 3. Stage outputs from the device flow

`run/01-placement.sh` runs three successive stages and materializes each one as a
JSON coordinate map and a GDS layout:

| Stage | What it is | Files |
| --- | --- | --- |
| Global Placement | the gradient-based placement seed; devices are positioned but may still overlap | `global_placement.json`, `global_placement.gds` |
| Reference-Guided Expansion | the first legalization phase, which spreads the seed toward a legal arrangement | `reference_guided_expansion.json`, `reference_guided_expansion.gds` |
| Local Conflict Resolution | the final discrete pass that repairs remaining overlap | `local_conflict_resolution.json`, `local_conflict_resolution.gds` |

`local_conflict_resolution.json` is the placement you ship downstream. The two
earlier files are there so you can see what the optimizer did and diff between
phases.

Alongside them the run writes `placement.log`, which records the resolved inputs
(netlist, layout directory, deck, parameters, symmetry) and each stage's start,
end and elapsed time. If a run surprises you, that log tells you which files it
actually read.

The three stages above are checkpoints of one multi-phase optimization, and the
run keeps that optimization's own working artifacts in a `multiphase_controller/`
subdirectory: `checkpoints/<phase>.gp.json` and `.gds` for each phase boundary,
and the per-iteration trace. Nothing downstream reads them -- they are there to be
looked at when a placement is not what you expected -- and the individual stage
scripts keep only the ones they publish unless `APPLACE_DEBUG=1` is set.

The individual stages can also be run on their own:

```bash
bash run/01-global-placement.sh <case> <out>
bash run/02a-reference-guided-expansion.sh <case> <out>
bash run/02b-local-conflict-resolution.sh <case> <out>
bash run/02-legalization.sh <case> <out>
```

Only global placement executes the placer; the two legalization scripts publish
phases of that same run, so running them against an output directory that has no
prior run triggers the run for you.

---

## 4. `placement.lcr_retry.json` -- the retry record

The final conflict-resolution pass has a retry loop: if it finishes with devices
still overlapping, it inflates the keep-out halo around the devices involved and
runs again, keeping the best result rather than the last one. When that loop
fires, a record of it is written next to the placement.

**This file is absent when the first pass resolved everything.** Its presence
means the placer had to work for the result, and it tells you how hard.

```json
{
  "enabled": true,
  "max_rounds": 6,
  "infl_step": 1.1,
  "rounds_run": 2,
  "converged": true,
  "overlap_pairs_after_by_round": [780, 12, 0],
  "unresolved_components_by_round": [1, 1, 0],
  "inflated_cells_by_round": [0, 18, 26],
  "inflated_cells": [[0, 1.21], [1, 1.21], [2, 1.1]],
  "accepted_round": 2,
  "best_overlap_pairs_after": 0,
  "exhausted": false
}
```

| Field | Meaning |
| --- | --- |
| `enabled` | Whether the retry loop was switched on at all. |
| `max_rounds` | The configured ceiling on extra passes. |
| `infl_step` | The multiplier applied to a device's halo each time it is inflated. |
| `rounds_run` | How many **extra** passes ran. `0` means the feature was on but never needed. |
| `converged` | True when some pass reached zero overlapping pairs. |
| `overlap_pairs_after_by_round` | Overlapping device pairs at the end of each pass, first pass first. |
| `unresolved_components_by_round` | Connected groups of still-conflicting devices at the end of each pass. |
| `inflated_cells_by_round` | How many devices carried an inflated halo entering each pass. |
| `inflated_cells` | Final inflation factor per device that was ever inflated, as `[device index, factor]`. Indices are positions in `bbox.json`'s `cells` array. |
| `accepted_round` | Which pass produced the placement that was returned -- the best one, not necessarily the last. |
| `best_overlap_pairs_after` | Overlapping pairs in that accepted pass. The number to check. |
| `exhausted` | True when the loop hit `max_rounds` with conflicts still present. |

The two fields worth reading first are `best_overlap_pairs_after` -- zero means a
clean placement -- and `exhausted`, which if true means the placer gave you its
best attempt rather than a solution. When `exhausted` is true, the usual remedies
are a lower target density (a larger `boundary`), a higher `max_rounds`, or a
larger `infl_step`; see `docs/PARAMETERS.md`.

---

## 5. Determinism

The same binary on the same input produces byte-identical output, in both the
JSON and the GDS. This is a hard property of the placer, not an accident: no
iteration over unordered containers reaches the result, and ties are broken by a
sorted rule.

It holds **within one build profile**. Debug and release builds differ in
floating-point reduction order and will disagree in the low digits. Compare
release against release.

It is **not** promised across platforms or toolchains. A different libm or code
generator changes the last bit of a result, and a gradient method amplifies that
into a different, equally good local optimum. That is why
`scripts/check_reference.sh` checks invariants and a quality band by default, and
why its `--strict` byte comparison only holds on the machine and toolchain the
reference files were generated on.
