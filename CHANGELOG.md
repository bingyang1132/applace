# Changelog

All notable changes to this project are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- **One circuit database, one level.** A netlist is flattened into devices as it is
  read, and the database holds the design's devices, nets, pins, ports and symmetry
  groups directly; the subcircuit tree it used to carry was never read by
  placement. The example references are unchanged byte for byte. A device-flow
  run is not byte-identical to 0.1.0: the old top subcircuit held a zero-size slot
  in every placer array, and without it the optimizer settles in a different,
  equally good placement (no overlap, exact symmetry).

- **Licence.** APPlace is now distributed under the GNU General Public License
  v3.0 (`GPL-3.0`, see `LICENSE`), replacing the previous permissive terms.
  Copyright (c) 2026 The University of Texas at Austin.

### Removed

- **`enableHierarchy`.** The reserved parameter for a hierarchy penalty that was never
  implemented. A parameter file that still sets it is refused as an unknown key.

### Fixed

- **`ota3` example connectivity.** The compensation capacitors were on the wrong
  nets (C1 tied to VOP2 and C5 to VOM2, a cross-coupled pair rather than a Miller
  capacitor across each output stage). `benchmark/abstract/ota3/pin.json` now
  reads C1 = VOM2 <-> OUTP and C5 = VOP2 <-> OUTM, and the `ota3` reference
  placement is re-recorded on the corrected netlist. The other three references
  are unchanged.

## [0.1.0] - 2026-09-17

First public release: the placement half of APPlace, as described in the
ASP-DAC 2027 paper.

### Added

- **Routing-aware global placement.** A gradient placer that optimises device
  locations and per-pin access points in the same continuous problem
  (`src/placer`).
- **Structure-preserving legalization.** Reference-guided expansion followed by
  bounded local conflict resolution, with a retry loop that widens clearance
  when the discrete pass cannot resolve a conflict.
- **Two input forms.** The geometric form takes four small JSON files (device
  outlines, pins and nets, symmetry pairs, parameters) and needs no process
  data; the device form reads a SPICE netlist, one PCell layout per device and a
  rule deck. `docs/INPUT_FORMAT.md` specifies both.
- **Four runnable example designs** in the geometric form, under
  `benchmark/abstract/`, with reference placements under
  `benchmark/abstract/<case>/reference/` and `scripts/check_reference.sh` to
  check a build against their invariants (and, with `--strict`, byte for byte).
- **Annotated output.** `placement.gds` carries device outlines, names, pin
  anchors, the placement region, the PMOS guard-ring reservation and a PMOS
  marker; `placement_summary.json` carries the same facts as data; and
  `scripts/plot_placement.py` draws them.
- **Deck-declared device classes.** `lef.json` declares which model names are
  PMOS, NMOS, capacitors and resistors (`device_types`). A model the declaration
  does not cover is a hard error naming it, rather than a silent "not a PMOS"
  that loses the guard-ring reservation.
- **Determinism as a property, not an aspiration.** The same binary on the same
  machine produces byte-identical output; the test suite and CI both check it.
  Byte identity across platforms and toolchains is not claimed -- a gradient
  method amplifies a last-bit difference into a different local optimum -- so
  `scripts/check_reference.sh` checks invariants and a quality band by default.
- A placeholder rule deck at `pdk/example_lef.json`: every rule value is
  invented and every model name is generic, so the repository carries no process
  data.

### Not included

Routing, post-placement geometry (guard rings, power stripes, well taps),
parasitic extraction and simulation are outside the scope of this repository.
Three parameters -- `enableHierarchy`, `useWellCluster` and `enableArray` -- are
reserved: they are accepted and validated, and the penalty terms they name are
not implemented. See the Limitations section of `README.md`.

### Archival

An archived copy of this release with a citable DOI is planned; the identifier
will be added here and to `CITATION.cff` once it is minted.

[Unreleased]: https://github.com/bingyang1132/applace/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/bingyang1132/applace/releases/tag/v0.1.0
