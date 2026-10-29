#!/usr/bin/env python3
"""Draw an annotated picture of an APPlace placement.

A placement is a list of coordinates, and a list of coordinates does not tell you
whether it is any good. This draws the things that make it readable:

  * every device as its outline box, labelled with its name;
  * PMOS devices filled in a distinct colour and, around each, the **guard-ring
    reservation** -- the room the placer deliberately kept free so a well ring can be
    dropped in later. That reservation is the usual reason two devices sit further
    apart than their outlines suggest they need to;
  * symmetric pairs in a shared colour, with the realized symmetry axis;
  * the declared placement region, and the bounding box the placement actually used;
  * devices that still overlap, outlined in red.

Usage
-----
    # form two (geometric demo): the run directory is self-describing
    python3 scripts/plot_placement.py out/abstract/comp

    # form one (device flow): positions come from the stage output, device
    # geometry and symmetry from the matching case
    python3 scripts/plot_placement.py out/run/local_conflict_resolution.json \\
        --case benchmark/abstract/comp --title "comp - device flow"

    # two runs side by side, on one common data scale
    python3 scripts/plot_placement.py runA runB --out compare.png

Inputs
------
`placement_summary.json` (written by `run/10-abstract-placement.sh`) carries everything
this script needs. When it is absent -- the device flow does not write one -- pass
`--case <dir>` pointing at a directory holding `bbox.json`, `sym.json` and optionally
`pin.json`; device sizes, the PMOS flags and the symmetry pairs are read from there and
the positions from the placement JSON.

Only matplotlib is required.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Rectangle
except ImportError:  # pragma: no cover - environment problem, not a code path
    sys.exit("plot_placement.py needs matplotlib: pip install matplotlib")

PLACEMENT_NAMES = ("placement.json", "local_conflict_resolution.json",
                   "reference_guided_expansion.json", "global_placement.json")

PMOS_FILL = "#f2c9a0"
PMOS_EDGE = "#b06000"
PLAIN_FILL = "#dcdcdc"
PLAIN_EDGE = "#666666"
BAD_EDGE = "#d00000"
AXIS_COLOR = "#8000a0"
BBOX_COLOR = "#0050c0"
REGION_COLOR = "#108020"
NET_COLOR = "0.78"
ANCHOR_COLOR = "#003366"


class Placement:
    """One placement, resolved into everything the drawing needs."""

    def __init__(self, label, devices, region, gr, spacing_floor, nets, source):
        self.label = label
        self.devices = devices          # list of dicts: name x y w h isPmos gr_reservation ...
        self.region = region            # {xl,yl,xh,yh} or None
        self.gr = gr                    # per-side guard-ring reservation, DBU
        self.spacing_floor = spacing_floor
        self.nets = nets                # list of device-name lists, may be empty
        self.source = source
        self.by_name = {d["name"]: d for d in devices}

    # -- geometry helpers -------------------------------------------------------
    def centre(self, d):
        return d["x"] + d["w"] / 2.0, d["y"] + d["h"] / 2.0

    def bbox(self):
        xl = min(d["x"] for d in self.devices)
        yl = min(d["y"] for d in self.devices)
        xh = max(d["x"] + d["w"] for d in self.devices)
        yh = max(d["y"] + d["h"] for d in self.devices)
        return xl, yl, xh, yh

    def overlapping(self):
        """Names of devices in at least one positive-area overlap."""
        bad = set()
        ds = self.devices
        for i in range(len(ds)):
            for j in range(i + 1, len(ds)):
                a, b = ds[i], ds[j]
                ox = min(a["x"] + a["w"], b["x"] + b["w"]) - max(a["x"], b["x"])
                oy = min(a["y"] + a["h"], b["y"] + b["h"]) - max(a["y"], b["y"])
                if ox > 1e-9 and oy > 1e-9:
                    bad.add(a["name"])
                    bad.add(b["name"])
        return bad

    def hpwl(self):
        total = 0.0
        for members in self.nets:
            pts = [self.centre(self.by_name[n]) for n in members if n in self.by_name]
            if len(pts) < 2:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            total += (max(xs) - min(xs)) + (max(ys) - min(ys))
        return total

    def pair_groups(self):
        """{group name: [(a, b), ...]} over devices that are actually placed."""
        groups: dict[str, list[tuple[str, str]]] = {}
        seen: set[tuple[str, str]] = set()
        for d in self.devices:
            g, p = d.get("symmetry_group"), d.get("symmetry_partner")
            if not g or not p or p not in self.by_name:
                continue
            key = tuple(sorted((d["name"], p)))
            if key in seen:
                continue
            seen.add(key)
            groups.setdefault(g, []).append(key)
        return groups

    def axis_is_vertical(self, group):
        for d in self.devices:
            if d.get("symmetry_group") == group:
                return d.get("symmetry_axis", "vertical") != "horizontal"
        return True


# ---------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------

def _read_json(p: Path):
    return json.loads(Path(p).read_text())


def _positions(path: Path) -> dict:
    """`{device: [x, y]}` -- the format every placement stage writes."""
    raw = _read_json(path)
    return {k: (float(v[0]), float(v[1])) for k, v in raw.items()}


def _nets_from_pin_json(path: Path) -> list[list[str]]:
    by_net: dict[str, set[str]] = {}
    for p in _read_json(path).get("pins", []):
        for n in p.get("nets") or []:
            by_net.setdefault(n, set()).add(p["cell_name"])
    return [sorted(v) for _, v in sorted(by_net.items())]


def _sym_from_case(sym_path: Path, names: list[str]):
    """Resolve `sym.json`'s short device names against full instance names.

    Same rule the abstract loader uses: a device's short name is the last `_`-separated
    segment of its full instance path. A short name claimed by two devices is skipped
    rather than bound to an arbitrary one.
    """
    short: dict[str, list[str]] = {}
    for full in names:
        short.setdefault(full.rsplit("_", 1)[-1], []).append(full)
    resolve = {k: v[0] for k, v in short.items() if len(v) == 1}

    out: dict[str, tuple[str, bool]] = {}   # device -> (group, is_vert)
    partner: dict[str, str] = {}
    for group, body in _read_json(sym_path).items():
        if not isinstance(body, dict):
            continue
        is_vert = body.get("axis", "vertical") != "horizontal"
        pairs = body.get("pairs") or {}
        for k, v in pairs.items():
            if not isinstance(v, str):
                continue
            a, b = resolve.get(k), resolve.get(v)
            if not a or not b:
                continue
            for x, y in ((a, b), (b, a)):
                out.setdefault(x, (group, is_vert))
                if x != y:
                    partner.setdefault(x, y)
    return out, partner


def load(target: Path, case: Path | None, label: str | None, gr_override) -> Placement:
    target = Path(target)
    placement_json = None
    summary_json = None

    if target.is_dir():
        if (target / "placement_summary.json").exists():
            summary_json = target / "placement_summary.json"
        for n in PLACEMENT_NAMES:
            if (target / n).exists():
                placement_json = target / n
                break
        if placement_json is None and summary_json is None:
            sys.exit(f"{target}: no placement JSON here (looked for {', '.join(PLACEMENT_NAMES)})")
    else:
        placement_json = target
        sibling = target.with_name("placement_summary.json")
        if sibling.exists():
            summary_json = sibling

    name = label or (target.name if target.is_dir() else target.parent.name)

    # -- the self-describing path ---------------------------------------------
    if summary_json is not None:
        s = _read_json(summary_json)
        g = s.get("global", {})
        devices = [dict(d) for d in s["devices"]]
        nets = []
        if case is not None and (Path(case) / "pin.json").exists():
            nets = _nets_from_pin_json(Path(case) / "pin.json")
        gr = gr_override if gr_override is not None else g.get("gr_reservation", 0.0)
        return Placement(name, devices, g.get("region"), gr,
                         g.get("spacing_floor", 0.0), nets, summary_json)

    # -- placement JSON + a case directory ------------------------------------
    if case is None:
        sys.exit(f"{placement_json} has no placement_summary.json beside it; "
                 "pass --case <dir> holding bbox.json and sym.json")
    case = Path(case)
    bbox = _read_json(case / "bbox.json")
    pos = _positions(placement_json)

    size = {}
    pmos = {}
    for c in bbox["cells"]:
        b = c["bbox"]
        size[c["cell_name"]] = (abs(b["xh"] - b["xl"]), abs(b["yh"] - b["yl"]))
        pmos[c["cell_name"]] = bool(c.get("isPmos", False))

    missing = sorted(set(pos) - set(size))
    if missing:
        sys.exit(f"{placement_json}: {len(missing)} placed device(s) are not in "
                 f"{case / 'bbox.json'}, e.g. {missing[0]}")

    groups, partner = ({}, {})
    if (case / "sym.json").exists():
        groups, partner = _sym_from_case(case / "sym.json", sorted(pos))

    gr = gr_override if gr_override is not None else 650.0
    devices = []
    for n in sorted(pos):
        w, h = size[n]
        g = groups.get(n)
        devices.append({
            "name": n, "x": pos[n][0], "y": pos[n][1], "w": w, "h": h,
            "isPmos": pmos[n],
            "gr_reservation": gr if pmos[n] else 0.0,
            "symmetry_group": g[0] if g else None,
            "symmetry_partner": partner.get(n),
            "symmetry_axis": ("vertical" if (g is None or g[1]) else "horizontal"),
        })
    nets = _nets_from_pin_json(case / "pin.json") if (case / "pin.json").exists() else []
    region = bbox.get("boundary")
    return Placement(name, devices, region, gr, 0.0, nets, placement_json)


# ---------------------------------------------------------------------------
# drawing
# ---------------------------------------------------------------------------

def draw(ax, pl: Placement, show_names=True, show_nets=True, show_region=True,
         used: set | None = None):
    used = used if used is not None else set()
    used.update({"bbox", "centre"})
    bad = pl.overlapping()
    groups = pl.pair_groups()
    if bad:
        used.add("bad")
    if groups:
        used.update({"pair", "axis"})

    # one colour per symmetric pair, stable across runs of the same case
    cmap = plt.get_cmap("tab20")
    pair_colour = {}
    for i, key in enumerate(sorted(k for v in groups.values() for k in v)):
        for d in key:
            pair_colour[d] = cmap(i % 20)

    if show_nets and pl.nets:
        used.add("nets")
        for members in pl.nets:
            pts = [pl.centre(pl.by_name[n]) for n in members if n in pl.by_name]
            if len(pts) < 2:
                continue
            mx = sum(p[0] for p in pts) / len(pts)
            my = sum(p[1] for p in pts) / len(pts)
            for px, py in pts:
                ax.plot([px, mx], [py, my], color=NET_COLOR, lw=0.35, zorder=1)

    for d in pl.devices:
        x, y, w, h = d["x"], d["y"], d["w"], d["h"]
        res = d.get("gr_reservation") or 0.0
        used.add("pmos" if d["isPmos"] else "plain")
        if res > 0:
            used.add("gr")
        if res > 0:
            ax.add_patch(Rectangle((x - res, y - res), w + 2 * res, h + 2 * res,
                                   fill=False, ls=(0, (2, 2)), lw=0.6,
                                   ec=PMOS_EDGE, zorder=2))
        fc = pair_colour.get(d["name"])
        if fc is not None:
            fc = (fc[0], fc[1], fc[2], 0.55)
        else:
            fc = PMOS_FILL if d["isPmos"] else PLAIN_FILL
        ec = BAD_EDGE if d["name"] in bad else (PMOS_EDGE if d["isPmos"] else PLAIN_EDGE)
        lw = 1.8 if d["name"] in bad else (1.5 if d["isPmos"] else 0.8)
        ax.add_patch(Rectangle((x, y), w, h, fc=fc, ec=ec, lw=lw, zorder=3))

        # Name above the centre, "PMOS" below it, the centre dot in between, so the
        # three never sit on top of each other.
        cx, cy = pl.centre(d)
        ax.plot([cx], [cy], marker="o", ms=1.8, color=ANCHOR_COLOR, zorder=5)
        if show_names:
            ax.text(cx, cy + h * 0.20, d["name"].rsplit("_", 1)[-1],
                    ha="center", va="center", fontsize=5.0, zorder=6, color="black")
        if d["isPmos"]:
            ax.text(cx, cy - h * 0.20, "PMOS", ha="center", va="center",
                    fontsize=4.0, zorder=6, color=PMOS_EDGE)

    # realized symmetry axes -- the midpoint of the pairs as placed, not as declared
    for group, pairs in groups.items():
        vert = pl.axis_is_vertical(group)
        mids = []
        for a, b in pairs:
            ca, cb = pl.centre(pl.by_name[a]), pl.centre(pl.by_name[b])
            mids.append((ca[0] + cb[0]) / 2.0 if vert else (ca[1] + cb[1]) / 2.0)
        m = sum(mids) / len(mids)
        (ax.axvline if vert else ax.axhline)(m, color=AXIS_COLOR, ls="-.", lw=0.8, zorder=4)

    xl, yl, xh, yh = pl.bbox()
    ax.add_patch(Rectangle((xl, yl), xh - xl, yh - yl, fill=False, ec=BBOX_COLOR,
                           ls="--", lw=1.0, zorder=4))
    region = pl.region if show_region else None
    if region:
        used.add("region")
        ax.add_patch(Rectangle((region["xl"], region["yl"]),
                               region["xh"] - region["xl"], region["yh"] - region["yl"],
                               fill=False, ec=REGION_COLOR, ls=":", lw=1.2, zorder=4))

    # Limits explicitly: patch-driven autoscale misses the region rectangle, and the
    # reservation bands must stay inside the frame or the picture hides the very thing
    # it is drawn to show.
    pad = max(d.get("gr_reservation") or 0.0 for d in pl.devices) if pl.devices else 0.0
    vx = [xl - pad, xh + pad]
    vy = [yl - pad, yh + pad]
    if region:
        vx += [region["xl"], region["xh"]]
        vy += [region["yl"], region["yh"]]
    mx = 0.04 * max(max(vx) - min(vx), max(vy) - min(vy), 1.0)
    ax.set_xlim(min(vx) - mx, max(vx) + mx)
    ax.set_ylim(min(vy) - mx, max(vy) + mx)

    area = sum(d["w"] * d["h"] for d in pl.devices)
    util = area / max((xh - xl) * (yh - yl), 1.0)
    n_pmos = sum(1 for d in pl.devices if d["isPmos"])
    subtitle = (f"{len(pl.devices)} devices ({n_pmos} PMOS)   "
                f"bbox {xh - xl:,.0f} x {yh - yl:,.0f} DBU   util {util:.2f}   "
                f"overlaps {len(bad) // 2 if bad else 0} pair(s)")
    if pl.nets:
        subtitle += f"   HPWL(centre) {pl.hpwl():,.0f}"
    ax.set_title(f"{pl.label}\n{subtitle}", fontsize=8)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=6)
    ax.set_xlabel("x (DBU = nm)", fontsize=7)
    ax.set_ylabel("y (DBU = nm)", fontsize=7)
    return (xl, yl, xh, yh)


# Legend entries, in the order they should read. Only the ones something was actually
# drawn for are shown -- a legend that promises a mark the picture does not contain is
# worse than no legend.
LEGEND_ENTRIES = [
    ("pmos", lambda: Rectangle((0, 0), 1, 1, fc=PMOS_FILL, ec=PMOS_EDGE,
                               label="PMOS device")),
    ("plain", lambda: Rectangle((0, 0), 1, 1, fc=PLAIN_FILL, ec=PLAIN_EDGE,
                                label="other device")),
    ("gr", lambda: Line2D([], [], color=PMOS_EDGE, ls=(0, (2, 2)),
                          label="guard-ring reservation (PMOS)")),
    ("pair", lambda: Rectangle((0, 0), 1, 1, fc=(0.4, 0.6, 0.9, 0.55), ec="#222222",
                               label="symmetric pair (one colour per pair)")),
    ("axis", lambda: Line2D([], [], color=AXIS_COLOR, ls="-.",
                            label="realized symmetry axis")),
    ("bbox", lambda: Line2D([], [], color=BBOX_COLOR, ls="--",
                            label="placement bounding box")),
    ("region", lambda: Line2D([], [], color=REGION_COLOR, ls=":",
                              label="declared region")),
    ("bad", lambda: Line2D([], [], color=BAD_EDGE, lw=1.6,
                           label="device in an overlapping pair")),
    ("nets", lambda: Line2D([], [], color=NET_COLOR, lw=0.6,
                            label="net star (topology)")),
    ("centre", lambda: Line2D([], [], marker="o", ls="", ms=3, color=ANCHOR_COLOR,
                              label="device centre")),
]


def legend_handles(used: set):
    return [make() for key, make in LEGEND_ENTRIES if key in used]


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", nargs="+", type=Path,
                    help="run directory, or a placement JSON. More than one draws them "
                         "side by side on a common data scale.")
    ap.add_argument("--case", type=Path, action="append", default=None,
                    help="case directory with bbox.json / sym.json / pin.json. Required "
                         "for a placement JSON that has no placement_summary.json beside "
                         "it; repeat once per target, or give one for all of them.")
    ap.add_argument("--label", action="append", default=None,
                    help="panel title; repeat once per target")
    ap.add_argument("--out", type=Path, default=None,
                    help="output image (default: placement.png next to the first target)")
    ap.add_argument("--gr", type=float, action="append", default=None,
                    help="override the per-side guard-ring reservation drawn around each "
                         "PMOS, in DBU (default: from placement_summary.json, else 650). "
                         "0 draws no reservation at all. Repeat once per target.")
    ap.add_argument("--no-nets", action="store_true", help="do not draw net stars")
    ap.add_argument("--no-region", action="store_true",
                    help="do not draw the declared region. Use this when the region in "
                         "the case directory is not the one this placement was run "
                         "with -- the device flow derives its own from total device "
                         "area and never reads bbox.json's.")
    ap.add_argument("--no-names", action="store_true", help="do not label devices")
    ap.add_argument("--title", default=None, help="figure title")
    ap.add_argument("--dpi", type=int, default=170)
    args = ap.parse_args()

    def nth(seq, i):
        if not seq:
            return None
        return seq[i] if i < len(seq) else seq[-1]

    plots = [load(t, nth(args.case, i), nth(args.label, i), nth(args.gr, i))
             for i, t in enumerate(args.target)]

    out = args.out
    if out is None:
        first = Path(args.target[0])
        out = (first if first.is_dir() else first.parent) / "placement.png"

    n = len(plots)
    fig, axes = plt.subplots(1, n, figsize=(11 * n if n > 1 else 11, 9))
    axes = [axes] if n == 1 else list(axes)
    used: set = set()
    boxes = [draw(ax, pl, not args.no_names, not args.no_nets, not args.no_region, used)
             for ax, pl in zip(axes, plots)]

    if n > 1:
        span = max(max(b[2] - b[0], b[3] - b[1]) for b in boxes) * 1.12
        for ax, b in zip(axes, boxes):
            cx, cy = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
            ax.set_xlim(cx - span / 2, cx + span / 2)
            ax.set_ylim(cy - span / 2, cy + span / 2)

    axes[-1].legend(handles=legend_handles(used), fontsize=6.5, loc="upper left",
                    bbox_to_anchor=(1.01, 1.0), frameon=False)
    if args.title:
        fig.suptitle(args.title, fontsize=11)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
    else:
        fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=args.dpi)
    plt.close(fig)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
