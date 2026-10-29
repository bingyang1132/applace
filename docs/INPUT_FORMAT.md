# Input format

APPlace reads a design in one of two shapes. The **geometric** form is four JSON
files and needs nothing else; the **device** form reads a SPICE netlist, a
directory of per-device layout, and a rule deck. Both feed the same placer.

All lengths are integers or reals in database units. One database unit is one
nanometre.

---

## 1. Geometric input

A case is a directory holding exactly four files:

```
<case>/
  bbox.json     device outlines
  pin.json      pins, the device each belongs to, and the net each is on
  sym.json      symmetry groups
  params.json   optimizer parameters
```

Run it with:

```bash
bash run/10-abstract-placement.sh /path/to/<case> /path/to/out
```

The four example designs under `benchmark/abstract/` are exactly this shape, and
are the best reference to read alongside this section.

### 1.1 `bbox.json`

```json
{
  "cells": [
    {
      "cell_name": "OTA_2_M0",
      "bbox": { "xl": 0, "yl": 0, "xh": 5870, "yh": 2280 },
      "isPmos": false
    }
  ],
  "boundary": { "xl": 0, "yl": 0, "xh": 92309, "yh": 92309 }
}
```

| Field | Required | Meaning |
| --- | --- | --- |
| `cells[].cell_name` | yes | Unique device name. This is the key you get back in `placement.json`. |
| `cells[].bbox` | yes | The device outline as `xl`/`yl`/`xh`/`yh`. Only the width and height carry information; the placer computes a position of its own, so the box is conventionally normalized to the origin. |
| `cells[].isPmos` | no, default `false` | Marks the device as PMOS. PMOS devices get extra room reserved around them for a well ring that a later flow stage may add; see `pmosHardExpandForGr` in `docs/PARAMETERS.md`. |
| `boundary` | no, but supply it | The rectangular region devices are placed into. If you omit it the placer falls back to the union of the outlines plus a small margin, which for origin-normalized boxes degenerates to the single largest device. |

The order of `cells` is the device index order used internally, and it is the
order the GDS output is written in. Keep it stable if you want to diff runs.

Device names must be unique, and the **short name** of each -- the part after the
last underscore, so `OTA_2_M21` has short name `M21` -- must also be unique,
because `sym.json` matches on short names. A collision is a hard error naming
every conflicting device rather than a silent arbitrary choice.

### 1.2 `pin.json`

```json
{
  "pins": [
    {
      "cell_name": "OTA_2_C0",
      "name": "OTA_2_C0_0",
      "nets": ["OTA_2_net0118"],
      "offset_x": 4895.0,
      "offset_y": 4490.0,
      "isIO": false
    }
  ]
}
```

| Field | Required | Meaning |
| --- | --- | --- |
| `cell_name` | yes | The device this pin belongs to; must match a `cells[].cell_name`. An empty string marks a pin that belongs to no device. |
| `name` | yes | Pin name. Informational; uniqueness is not enforced. |
| `nets` | yes | Net names. Only the first entry is used: a pin sits on one net. |
| `offset_x`, `offset_y` | yes | Pin position relative to the device origin. |
| `isIO` | no, default `false` | Marks a top-level port. |

Nets are created implicitly from the names appearing here, sorted, so the net set
is whatever the pin table mentions.

**On pin offsets.** The placer does not treat the offset as a fixed pin location.
Each pin carries a free angular variable that slides it around its device's
outline, solved together with the placement, so what you supply is a starting
hint and not a constraint. The shipped examples write the device centre, which
projects to a neutral start. If you know where a pin really is, giving the true
offset is a better start; if you do not, the centre is fine.

### 1.3 `sym.json`

```json
{
  "OTA_2": {
    "axis": "vertical",
    "pairs": {
      "C0": "C1",
      "C2": "C3",
      "M1": "M1"
    }
  }
}
```

Each top-level key names a symmetry group. Inside it:

- `axis` -- `"vertical"` for devices that mirror left/right, `"horizontal"` for
  devices that mirror top/bottom. Omitted means vertical.
- `pairs` -- the symmetry constraints, each one naming two devices by **short
  name**. `"C0": "C1"` means those two must be mirror images about the group's
  axis. A pair whose two sides are the same device, `"M1": "M1"`, is a
  self-symmetric device: it must sit centred on the axis.

Any other key in a block is an error rather than a shrug, so a misspelling is
reported instead of being silently dropped. A pair naming a device that is not in
`bbox.json` is skipped.

Each group gets its own axis variable. Groups that should share one axis converge
to it through the objective rather than by declaration.

`sym.json` may be an empty object `{}` if the design has no symmetry.

### 1.4 `params.json`

```json
{
  "schemaVersion": 2,
  "placement": {
    "global": { },
    "base":   { },
    "tail":   { },
    "deoverlap": { }
  }
}
```

Optimizer settings, keyed by name under the `placement` object. Every key is
optional and falls back to a documented default. `docs/PARAMETERS.md` is the
complete reference. Individual keys can be overridden per run:

```bash
bash run/10-abstract-placement.sh ota2 /tmp/out --set symWeight=2.0
```

You can copy `benchmark/abstract/ota2/params.json` as a starting point.

The library also understands a second layout, in which the same keys are grouped
under `common` and `multiphase` rather than by phase;
`src/placer/placement_config.example.json` is a complete worked example of it. The shipped
binaries do not read it -- both entry points load a case `params.json` in the form
above. It is reachable only through the library API, so treat it as a detail of
`applace-placer`, not as a second file format for the tool.

---

## 2. Device input

A case directory for the device flow:

```
<case>/
  <case>.sp         hierarchical SPICE netlist, with device sizing
  custom/           one GDS file per device (the PCell layout)
  params.json       optimizer parameters, same schema as above
  sym.json          symmetry constraints (optional)
  lef.json          rule deck (optional; see the resolution order below)
```

Run it with:

```bash
bash run/01-placement.sh /path/to/<case> /path/to/out
```

**No design of this kind ships with this repository.** The netlists and PCell
layout APPlace was developed against are not ours to publish. This section
specifies the interface so you can point the flow at your own.

### 2.1 The netlist

A hierarchical SPICE netlist. Subcircuit definitions are read, instances are
expanded, and the result is flattened to devices before placement -- APPlace
places devices, not blocks. The netlist supplies the connectivity and the device
type string (which is what decides PMOS-ness), and the sizing it carries reaches
placement only through the PCell geometry.

If the file is not named `<case>.sp`, the flow accepts the case directory as long
as it contains exactly one `.sp` file.

### 2.2 `custom/` -- PCell layout

One GDS file per device instance in the netlist. APPlace reads each one to derive
two things: the device's outline, and the position of each of its pins. The
polygons themselves are not carried into placement.

The directory is normalized on first use into a sibling working directory; that
output is regenerated on demand and is not part of your input.

### 2.3 `sym.json` for the device flow

The device flow accepts the flat schema of section 1.3 and also a nested form in
which a group holds a `pairs` object and a `children` list of subgroups. Group
names are matched first against subcircuit instance names; a group naming no
instance falls back to matching its pairs by flattened device short name, which
is the same rule the geometric form uses. An ambiguous match is a hard error.

### 2.4 The rule deck

The deck is resolved in this order:

1. `APPLACE_PDK` -- an absolute path, or one relative to the repository root
2. `<case>/lef.json` -- a deck sitting next to the design
3. `pdk/example_lef.json` -- the placeholder deck shipped here

**The shipped deck contains placeholder values throughout.** It exists so the
device flow has a well-formed deck to parse, not so it can produce results valid
for any real process. Supply your own.

The deck is JSON. The fields placement reads:

| Key | Type | Meaning |
| --- | --- | --- |
| `unit_r` | number | The deck's length unit, in metres. `1e-06` means deck values are in micrometres. |
| `unit_n` | number | The database unit, in metres. `1e-09` means output coordinates are in nanometres. |
| `grid` | number | Manufacturing grid, in deck units. |
| `layer_map` | object | Layer name to GDS layer number, for every layer named elsewhere in the deck and in the PCell layout. |
| `layer_datatype` | object | Layer name to GDS datatype. |
| `min_w_rules` | object | Minimum width per layer. |
| `sp_rules` | object of objects | Minimum spacing, `sp_rules[a][b]` between layer `a` and layer `b`. |
| `en_rules` | object of objects | Minimum enclosure, `en_rules[outer][inner]`. |
| `ex_rules` | object of objects | Minimum extension, `ex_rules[a][b]`. |
| `nwell_gr`, `sub_gr` | boolean | Whether the process wants an N-well ring and a substrate ring. |
| `nw_od`, `np_od` | number | N-well to diffusion, and N-plus to diffusion, spacings. These set the room reserved around PMOS devices. |
| `_meta.physical_grid` | boolean | Whether `grid` is a real manufacturing grid. `false` disables grid snapping and its preconditions -- which is what the shipped placeholder deck declares, because its `grid` is a stand-in. |
| `device_types` | object | Which device model names are PMOS, NMOS, capacitors and resistors. See below. |

Values are in deck units (`unit_r`) unless stated otherwise.

Keys beyond this list may appear; placement ignores what it does not read. A key
that placement does need and cannot find is a hard error naming the key -- the
deck is never silently filled in with a guessed value.

#### `device_types` -- telling the tool what your devices are

Placement has to know which devices are PMOS. Guard-ring room is reserved around
them and nowhere else, legalization keeps them apart by a wider margin, and the
N-well clustering that follows placement is built out of them. Getting that
wrong does not produce an error; it produces a placement with no room where the
room was needed.

The device model string comes from your netlist, and its spelling is your
process's. Declare it:

```jsonc
"device_types": {
  "match": "prefix",             // optional: "prefix" (default) or "exact"
  "pmos":      ["sky130_fd_pr__pfet"],
  "nmos":      ["sky130_fd_pr__nfet"],
  "capacitor": ["sky130_fd_pr__cap"],
  "resistor":  ["sky130_fd_pr__res"],
  "other":     []                // deliberately none of the four
}
```

| Rule | |
| --- | --- |
| Matching | Case-insensitive. `prefix` (the default) matches a model *family*: `pfet` covers `pfet_hv`, `pfet_v2`, `pfet_v2_wide`. `exact` requires the whole model string. |
| Overlaps | The longest listed name wins, so `"pmos": ["pfet"]` with `"other": ["pfet_dummy"]` puts `pfet_dummy_x` in `other`. |
| Contradictions | The same name under two categories is a hard error when the deck is read. |
| Coverage | **Once you declare this section, every device model in your netlist must be listed under some category.** A model listed nowhere is a hard error naming it. Put devices that are genuinely none of the four under `other`. |
| Omitting the section | A hard error. The tool has no built-in idea of which model names are PMOS in your process, so a deck without `device_types` is refused with a message pointing here. Declare the section for every deck you bring. |

The shipped placeholder deck declares the section, so it is also a worked
example. The section names no rule values and is safe to keep in a deck you
publish.

---

## 3. Which form should you use

Use the geometric form when you want to evaluate the placer, benchmark against
it, or place a problem you can describe as boxes and nets. It is the whole placer
with none of the setup.

Use the device form when you have a real design and a deck for its process, and
you want outlines and pin positions derived from the actual PCell layout rather
than supplied by hand.

The two are not interchangeable. The same circuit expressed both ways will place
differently, because the device form knows the real outline and pin location of
every PCell while the geometric form knows only the boxes you hand it. Both
produce placements of the same quality.
