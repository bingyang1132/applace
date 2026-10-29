[Read this in Chinese](README.zh-CN.md)

# APPlace

We hope APPlace can contribute to the open-source community and to analog design
automation. It has not been tested at scale outside our experimental environment
and the PDKs we used, so running into problems is to be expected. Please report
them in the issue tracker; we will fix what we can, and pull requests are welcome.
With a report or a pull request, please attach the concrete input that shows the
problem, so that it can join the long-term regression set. If your design cannot
be shared, a case in the geometric form (`docs/INPUT_FORMAT.md`), which carries no
process data, is enough.

**APPlace** is a routing-aware analytical placement framework for analog
circuits. Unlike conventional analytical placement, which optimizes device
coordinates only and leaves access-point (AP) selection to the router, APPlace
co-optimizes **device locations and access points** during global placement, so
that the interconnect geometry seen by the placer is close to the connection
positions the router will eventually use.

APPlace has two main stages:

1. **Routing-Aware Global Placement**  
   Device coordinates and AP variables enter one continuous optimization.
   APPlace uses a differentiable terminal/AP geometry, an AP-aware wirelength
   objective, and adaptive multi-objective control, so device positions and their
   most suitable access positions evolve together.

2. **Structure-Preserving Legalization**  
   While removing overlap and enforcing symmetry, this stage preserves as much as
   possible of the routing-relevant structure formed in global placement. It
   consists of reference-guided expansion followed by bounded local conflict
   resolution.

The experiments in the paper evaluate APPlace on four real analog benchmarks.
Experiments and ablations show that both AP co-optimization and
structure-preserving legalization have a significant effect on final post-layout
quality.

> **Paper:** *APPlace: Co-Optimizing Device Locations and Access Points for
> Routing-Aware Analog Placement*, accepted to ASP-DAC 2027.

This repository provides the Rust implementation of APPlace, abstract benchmarks
that run without any PDK, and the input interfaces for connecting real PCell /
PDK data. The repository covers placement only; routing, parasitic extraction and
post-layout simulation are out of scope.

## What is in the box

- Three crates under `src/`: `applace-core` (the geometry and netlist model),
  `applace-placer` (the gradient placer and legalizer), `applace-io` (parsers,
  writers, and the command-line entry points).
- Four example designs under `benchmark/abstract/`, runnable with no setup
  beyond a Rust toolchain, each with the placement this repository produces for
  it under `reference/`.
- Shell entry points under `run/`, and `scripts/` for the two things you run
  afterwards: `plot_placement.py` to look at a result and `check_reference.sh`
  to check your build against the reference invariants.

## Install

APPlace needs a stock Rust toolchain (edition 2021) and nothing else. There is no
solver, no C++ dependency and no cargo feature to choose: the build you get is the
build there is.

```bash
# Rust, if you do not have it
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Build
cargo build --release --manifest-path src/Cargo.toml --workspace
```

Build output goes to `./target` by default; set `TARGET_DIR` to move it.

## Quickstart

One command, no other inputs:

```bash
bash run/10-abstract-placement.sh ota2 /tmp/applace-ota2
```

This places the `ota2` example -- 49 devices, 35 nets, 24 symmetry pairs -- and
writes two files:

```
/tmp/applace-ota2/placement.json     device -> [x, y], lower-left corner, in nanometres
/tmp/applace-ota2/placement.gds      the same placement as an outline-level layout
```

Open `placement.gds` in any GDS viewer to see the result: one rectangle per
device, a text label with its name, a pin anchor, and the placement region.
`docs/OUTPUT.md` gives the layer assignment and the exact JSON schema.

Three more examples ship alongside it: `ota1`, `ota3`, `comp`. Substitute the
name.

## The two ways in

APPlace accepts a design in either of two shapes.

| | Geometric input | Device input |
| --- | --- | --- |
| What you supply | four small JSON files: device outlines, a pin and net table, symmetry pairs, parameters | a SPICE netlist, one PCell GDS per device, and a rule deck |
| Needs a PDK | no | yes |
| Needs foundry PCell geometry | no | yes |
| Entry point | `run/10-abstract-placement.sh` | `run/01-placement.sh` |
| Example data included | yes, four designs | no -- bring your own |

**Geometric input** is the quickstart above. It is the complete placement problem
reduced to what the placer actually needs: how big each device is, what connects
to what, and what must stay symmetric. The four shipped examples use this form.

**Device input** takes a real design: a hierarchical SPICE netlist with device
sizing, a directory of per-device PCell layout, and a rule deck describing the
process. APPlace parses the PCell geometry to derive device outlines and pin
positions, then runs the same placer. No design of this kind ships with the
repository. `docs/INPUT_FORMAT.md` specifies every file the device flow reads, so
you can assemble your own case directory.

```bash
bash run/01-placement.sh /path/to/your/case /tmp/out
```

## Outputs

The geometric flow writes `placement.json` and `placement.gds`, plus
`placement.lcr_retry.json` when the final legalization step had to retry.

The device flow writes, for each stage, a JSON coordinate map and a GDS layout:

| Stage | What it is | Files |
| --- | --- | --- |
| Global Placement | the gradient-based placement seed | `global_placement.{json,gds}` |
| Reference-Guided Expansion | the first legalization phase | `reference_guided_expansion.{json,gds}` |
| Local Conflict Resolution | the final overlap repair | `local_conflict_resolution.{json,gds}` |

`docs/OUTPUT.md` documents all of them, including the GDS layer numbers and the
retry record.

## Reading the picture

`scripts/plot_placement.py` turns a run into an annotated layout. Run it on any
output directory:

```bash
bash run/10-abstract-placement.sh comp out/comp
python3 scripts/plot_placement.py out/comp --out comp.png
```

A placement is a list of coordinates, and coordinates alone do not tell you
whether the result is sensible. Two things usually explain a layout that looks
wrong at first glance, and both are drawn.

**Guard-ring reservation.** Analog PMOS devices sit inside an N-well ring that is
added after placement, so the placer keeps a band of empty space around every
PMOS. That band is the reason two devices sit further apart than their outlines
require. It is drawn as a dashed frame around each PMOS, and it is the area that
was actually reserved -- not a margin the drawing invented.

**Symmetry.** Analog circuits are placed about mirror axes. Each matched pair
shares a fill colour, and the axis drawn through them is the one the placement
realized, not the one that was declared. If a pair is not mirrored about its
axis, you can see it.

| Mark | Meaning |
| --- | --- |
| Orange box, thick edge, `PMOS` label | a PMOS device |
| Grey box | any other device |
| Dashed orange frame | guard-ring reservation kept free around that PMOS |
| Coloured fill, one colour per pair | a symmetric pair |
| Purple dash-dot line | the realized symmetry axis |
| Blue dashed box | the bounding box the placement actually occupies |
| Green dotted box | the placement region you declared |
| Red outline | a device still overlapping another one -- should not appear |
| Grey star lines | net connectivity, drawn from device centres |
| Small dark dot | the device centre, where the geometric form seeds its pins |

The title line carries the numbers to check first: device and PMOS counts, the
bounding box, utilization, **overlapping pairs (zero is what you want)**, and
centre-to-centre HPWL. The same numbers are in `placement_summary.json` if you
would rather read them than look at them.

The same layout opens in any GDS viewer as `placement.gds`; layers 4 and 5 carry
the reservation and the PMOS marking (see `docs/OUTPUT.md` for the full layer
table).

## Parameters

Every optimizer setting is read from the case's `params.json`, and any of them
can be overridden on the command line:

```bash
bash run/10-abstract-placement.sh ota2 /tmp/out --set symWeight=2.0
```

`docs/PARAMETERS.md` lists every key with its type, default and effect.

## Limitations

Read these before deciding whether APPlace fits your problem.

- **Placement only.** No routing, no post-placement geometry (guard rings, power
  stripes, well taps), no parasitic extraction, no simulation.
- **No process data is included.** Every rule value in `pdk/example_lef.json` is
  an invented placeholder. Its `pmos`/`nmos`/`cap`/`res` lists are the exception:
  those name device models, and they have to match the model strings in your own
  netlist for device classification to work. Running against a real process means
  supplying your own PDK data and PCell GDS; `docs/INPUT_FORMAT.md` documents the
  details.
- **No example design for the device flow.** Only the geometric examples ship.
- **The geometric form applies no process spacing.** Without a rule deck there is
  no minimum device-to-device spacing, so outlines in a geometric-form placement
  may touch. Only the device form, given a deck, derives a legalization spacing
  floor from the process rules; DRC meaning comes from that path.
- **Symmetry constraints.** At present a symmetry group shares one vertical or
  horizontal axis. Rotational symmetry, and hierarchical symmetry beyond a shared
  axis, are not part of the model implemented in this repository.
- **Non-hierarchical placement.** Hierarchy in the netlist is flattened to devices
  before placement; APPlace does not yet place blocks as blocks.
- **Two parameters are reserved and currently do nothing.** `useWellCluster`
  and `enableArray` are accepted and validated, but the penalty terms they name are
  not implemented: each contributes exactly zero, and every example case produces
  a byte-identical placement whichever way they are set.
  They are documented as reserved in `docs/PARAMETERS.md` rather than removed, so
  that existing parameter files keep loading.

## Citation

If APPlace is useful in your research, please cite:

```bibtex
@inproceedings{liu2027applace,
  author    = {Liu, Bingyang and Gao, Xiaohan and Poddar, Souradip and Pan, David Z.},
  title     = {{APPlace}: Co-Optimizing Device Locations and Access Points for Routing-Aware Analog Placement},
  booktitle = {Proceedings of the Asia and South Pacific Design Automation Conference (ASP-DAC)},
  year      = {2027},
  note      = {To appear},
}
```

## Acknowledgments

We thank Yibo Lin and Haoyi Zhang (Peking University) for their help.

## License

Copyright (c) 2026 The University of Texas at Austin
All rights reserved.

Licensed under the GNU General Public License v3.0 (GPL-3.0); see
[LICENSE](LICENSE).

Unless stated otherwise, any contribution submitted for inclusion is licensed
under the same terms, without additional terms.
