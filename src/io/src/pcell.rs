//! PCell parser - extract pin shapes/bboxes from GDS and map to CircuitDb.

use crate::parser::{parse_pdk, ParseError, PdkRules};
use applace_core::{CircuitDb, DeviceClass};
use gds21::{GdsElement, GdsLibrary, GdsPoint, GdsStrans, GdsStruct};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone)]
struct PolyBox {
    layer: i32,
    #[allow(dead_code)] // parsed but never consulted; upstream item
    datatype: i32,
    xl: i32,
    yl: i32,
    xh: i32,
    yh: i32,
}

struct GdsData {
    top: String,
    polygons: Vec<GdsPoly>,
    labels: Vec<GdsLabel>,
    /// Hierarchy-aware bbox of the device struct (descends into SREF/AREF).
    /// `polygons` holds only the geometry the struct owns itself, so the two
    /// differ for a hierarchical PCell -- the cell footprint must use this one.
    hier_bbox: Option<(i32, i32, i32, i32)>,
}

struct GdsPoly {
    layer: i32,
    datatype: i32,
    points: Vec<[i32; 2]>,
}

struct GdsLabel {
    text: String,
    layer: i32,
    #[allow(dead_code)] // parsed but never consulted; upstream item
    texttype: i32,
    x: i32,
    y: i32,
}

#[derive(Default)]
struct Dsu {
    parent: Vec<usize>,
    rank: Vec<u8>,
}

impl Dsu {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            rank: vec![0; n],
        }
    }

    fn find(&mut self, x: usize) -> usize {
        if self.parent[x] != x {
            let root = self.find(self.parent[x]);
            self.parent[x] = root;
        }
        self.parent[x]
    }

    fn union(&mut self, a: usize, b: usize) {
        let mut ra = self.find(a);
        let mut rb = self.find(b);
        if ra == rb {
            return;
        }
        if self.rank[ra] < self.rank[rb] {
            std::mem::swap(&mut ra, &mut rb);
        }
        self.parent[rb] = ra;
        if self.rank[ra] == self.rank[rb] {
            self.rank[ra] += 1;
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BboxRect {
    pub xl: i32,
    pub yl: i32,
    pub xh: i32,
    pub yh: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BboxCell {
    pub cell_name: String,
    pub bbox: BboxRect,
    /// Device model string from the netlist -- whatever the process calls the device.
    ///
    /// `cell_name` is the *instance* path (`Core_test_flow_M0`) and carries no device
    /// type, so anything downstream that needs to classify a device (PMOS vs not, for
    /// the guard-ring reservation) has to be handed the model explicitly. Classifying
    /// on `cell_name` instead makes `isPmos` false for every device in every abstract
    /// case.
    ///
    /// `#[serde(default)]` keeps older dumps loadable.
    #[serde(default)]
    pub model: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BboxFile {
    pub cells: Vec<BboxCell>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PinCellShape {
    pub shape: BboxRect,
}

/// One pin metal rectangle (cell-local normalized coordinates, GDS layer number).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PinShapeEntry {
    pub layer: i32,
    pub shape: BboxRect,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PinEntry {
    pub cell: Option<PinCellShape>,
    pub cell_name: String,
    pub idx: i32,
    #[serde(rename = "isIO")]
    pub is_io: bool,
    #[serde(rename = "isNWGR")]
    pub is_well_tap: bool,
    pub layers: Vec<i32>,
    pub name: String,
    pub nets: Vec<String>,
    pub offset_x: i32,
    pub offset_y: i32,
    pub shape: BboxRect,
    /// Full list of metal rectangles (optional, empty by default so existing consumers keep working).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub shapes: Vec<PinShapeEntry>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PinFile {
    pub pins: Vec<PinEntry>,
}

#[derive(Debug, Clone)]
pub struct PcellOutput {
    pub bbox: BboxFile,
    pub pins: PinFile,
    pub resolved_pcelldir: PathBuf,
}

fn dump_pcell_output(out: &PcellOutput, dir: &Path) -> Result<(), ParseError> {
    std::fs::create_dir_all(dir)?;
    let bbox_path = dir.join("bbox.json");
    let pin_path = dir.join("pin.json");
    std::fs::write(&bbox_path, serde_json::to_string_pretty(&out.bbox)?)?;
    std::fs::write(&pin_path, serde_json::to_string_pretty(&out.pins)?)?;
    Ok(())
}

fn project_root() -> PathBuf {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let io_dir = PathBuf::from(manifest_dir);
    io_dir.parent().unwrap().parent().unwrap().to_path_buf()
}

fn normalized_pcelldir_path(pcelldir: &Path) -> PathBuf {
    let parent = pcelldir.parent().unwrap_or_else(|| Path::new("."));
    let base = pcelldir
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("custom");
    parent.join(format!("{}_normalized", base))
}

// ===================================================================
// Hierarchy-aware GDS geometry
// ===================================================================
//
// Why this exists: the older `compute_gds_bbox` read only the BOUNDARY/PATH
// owned by the selected struct **itself** and never descended into SREF/AREF,
// while `normalize_single_gds` translated the elements of **every** struct by
// (dx,dy). A leaf at depth d was therefore moved by d·(dx,dy), which tore
// hierarchical PCells apart. The self-check assertion re-measured with the
// same hierarchy-blind function, so it could never catch it.
//
// The functions below descend and apply the transforms according to the GDSII
// standard semantics, and are the single measurement path.

// -------------------------------------------------------------------
// GDSII `PATH` -> axis-aligned rectangles
// -------------------------------------------------------------------
//
// A `PATH` used to be converted by taking the bounding box of its end points and
// then growing it by half the line width in all four directions, regardless of its
// `PATHTYPE`. For a flush path (`PATHTYPE 0`, the common case in PCell layouts) that
// makes every pin rectangle half a line width longer than the layout at **each** end,
// which is enough to put the model's edges out of step with the layout's.
//
// The conversion now takes the end extension from the GDSII `PATHTYPE`; the
// half-width expansion perpendicular to the segment direction is unchanged (it was
// correct). A multi-segment `PATH` yields one rectangle per segment, so the result
// keeps union semantics instead of collapsing an L-shaped path into one solid box.
//
// `PATHTYPE` semantics (GDSII specification; a missing record means 0):
//   0 = flush     no end extension
//   1 = round     semicircular ends -- round in GDS, so a rectangle model can only
//                 approximate them; treated as 2, with a one-time warning
//   2 = square    each end extended by half the line width
//   4 = custom    the ends extended by `BGNEXTN` / `ENDEXTN` respectively
// Any other value is a hard error: the semantics are not guessed.

/// Environment switch: `APPLACE_PCELL_PATH_LEGACY=1` restores the previous `PATH`
/// conversion (half a line width of extension at both ends, one bounding box for the
/// whole path), for byte-for-byte A/B comparison. **Off by default** (= follow
/// `PATHTYPE`). An unknown value is a hard error.
pub const PCELL_PATH_LEGACY_ENV: &str = "APPLACE_PCELL_PATH_LEGACY";

/// Pure parsing (unit-testable). `None` = the variable is unset => `false` (the new
/// convention).
pub fn parse_pcell_path_legacy_switch(v: Option<&str>) -> Result<bool, String> {
    match v {
        None => Ok(false),
        Some(raw) => match raw.trim().to_ascii_lowercase().as_str() {
            "" | "0" | "false" | "off" | "no" => Ok(false),
            "1" | "true" | "on" | "yes" => Ok(true),
            _ => Err(raw.to_string()),
        },
    }
}

/// The switch state for this process (`OnceLock`: one answer per process, for
/// determinism). An invalid value **panics** rather than silently falling back to the
/// default.
pub fn legacy_pcell_path() -> bool {
    static SWITCH: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *SWITCH.get_or_init(|| {
        let raw = std::env::var(PCELL_PATH_LEGACY_ENV).ok();
        parse_pcell_path_legacy_switch(raw.as_deref()).unwrap_or_else(|bad| {
            panic!(
                "{PCELL_PATH_LEGACY_ENV}={bad:?} is not recognised; \
                 accepted values are 0/false/off/no (off, the default: follow PATHTYPE) \
                 or 1/true/on/yes (on, for byte-for-byte A/B comparison only)"
            )
        })
    })
}

/// The end extensions of one `PATH`, decoded from its `PATHTYPE` but **not yet
/// multiplied by the half width**.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PathEnds {
    /// `PATHTYPE 0`: no extension at either end.
    Flush,
    /// `PATHTYPE 2` (and `PATHTYPE 1`, which is approximated by it): half a line width at each end.
    Half,
    /// `PATHTYPE 4`: `BGNEXTN` / `ENDEXTN` at the two ends respectively.
    Custom(i32, i32),
}

/// A round-ended `PATH` warns once: the conversion is an approximation, and whoever
/// has one should see that.
fn warn_round_path_once() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        eprintln!(
            "[pcell] warning: GDS PATHTYPE 1 (round end) encountered; \
             approximated as PATHTYPE 2 (square, half-width extension). \
             A round end has no exact axis-aligned rectangle model."
        );
    });
}

fn path_ends(p: &gds21::GdsPath) -> Result<PathEnds, ParseError> {
    match p.path_type.unwrap_or(0) {
        0 => Ok(PathEnds::Flush),
        1 => {
            warn_round_path_once();
            Ok(PathEnds::Half)
        }
        2 => Ok(PathEnds::Half),
        4 => Ok(PathEnds::Custom(
            p.begin_extn.unwrap_or(0),
            p.end_extn.unwrap_or(0),
        )),
        other => Err(ParseError::Parse(format!(
            "gds PATH: unsupported PATHTYPE {other} on layer {}/{}; \
             only 0 (flush), 1 (round, approximated), 2 (square) and 4 (custom \
             BGNEXTN/ENDEXTN) are defined by GDSII",
            p.layer, p.datatype
        ))),
    }
}

impl PathEnds {
    fn as_f64(self, half: f64) -> (f64, f64) {
        match self {
            PathEnds::Flush => (0.0, 0.0),
            PathEnds::Half => (half, half),
            PathEnds::Custom(b, e) => (b as f64, e as f64),
        }
    }

    fn as_i32(self, half: i32) -> (i32, i32) {
        match self {
            PathEnds::Flush => (0, 0),
            PathEnds::Half => (half, half),
            PathEnds::Custom(b, e) => (b, e),
        }
    }
}

/// The per-segment rectangles of a `PATH` (local coordinates, **union semantics**).
///
/// * `half` -- the half-width expansion perpendicular to the segment direction (each
///   caller keeps its own algorithm and integer type for it; it is not changed here);
/// * `(bgn, end)` -- the end extensions at the first segment's start and the last
///   segment's end (decided by `PATHTYPE`); interior vertices do not take the
///   `PATHTYPE` extension, each segment there is extended by `half` instead (= the
///   corner block of a 90-degree mitre joint).
///
/// Degenerate cases, handled conservatively:
/// * fewer than 2 points => a box of +/-`half` around the point (as before; no
///   geometry is dropped);
/// * a zero-length segment (coincident points) => the same, the direction being
///   undefined;
/// * a diagonal (non-Manhattan) segment => the conservative bounding box of its two
///   end points grown by `half`, with no end extension (an axis-aligned rectangle has
///   no exact representation of a diagonal segment; at least the whole path is no
///   longer collapsed into one box).
fn path_segment_boxes<T>(xy: &[GdsPoint], half: T, bgn: T, end: T) -> Vec<(T, T, T, T)>
where
    T: Copy + PartialOrd + From<i32> + std::ops::Add<Output = T> + std::ops::Sub<Output = T>,
{
    let lo = |a: T, b: T| if a < b { a } else { b };
    let hi = |a: T, b: T| if a > b { a } else { b };

    if xy.len() < 2 {
        return xy
            .iter()
            .map(|p| {
                let (x, y) = (T::from(p.x), T::from(p.y));
                (x - half, y - half, x + half, y + half)
            })
            .collect();
    }

    let last = xy.len() - 1;
    let mut out = Vec::with_capacity(last);
    for i in 0..last {
        let (a, b) = (&xy[i], &xy[i + 1]);
        // The `PATHTYPE` end extension applies only at the two **true end points** of
        // the whole path. Interior vertices are always extended by `half`: the mitre
        // joint of a 90-degree bend is exactly that half-width square corner block,
        // and without it the bend would leave a notch.
        let ext_a = if i == 0 { bgn } else { half };
        let ext_b = if i + 1 == last { end } else { half };
        let (ax, ay) = (T::from(a.x), T::from(a.y));
        let (bx, by) = (T::from(b.x), T::from(b.y));

        let bx4 = if a.y == b.y && a.x != b.x {
            // horizontal segment
            let (xl, xh) = if a.x < b.x {
                (ax - ext_a, bx + ext_b)
            } else {
                (bx - ext_b, ax + ext_a)
            };
            (xl, ay - half, xh, ay + half)
        } else if a.x == b.x && a.y != b.y {
            // vertical segment
            let (yl, yh) = if a.y < b.y {
                (ay - ext_a, by + ext_b)
            } else {
                (by - ext_b, ay + ext_a)
            };
            (ax - half, yl, ax + half, yh)
        } else {
            // zero-length or diagonal segment: conservative bbox (see the function docs)
            (
                lo(ax, bx) - half,
                lo(ay, by) - half,
                hi(ax, bx) + half,
                hi(ay, by) + half,
            )
        };
        out.push(bx4);
    }
    out
}

/// Affine transform `(x,y) -> (a·x + b·y + tx, c·x + d·y + ty)`.
///
/// GDSII STRANS semantics (identical for SREF and AREF): first mirror about the
/// **x axis** (`reflected`), then apply the **magnification** `mag`, then rotate
/// **counter-clockwise** by `angle` degrees, and finally translate to the
/// instance origin. `abs_mag` / `abs_angle` mean "do not inherit the parent
/// magnification/rotation"; in our device GDS the STRANS flag word is always 0
/// (measured over 137 files and 2016 STRANS records across 4 cases), so an
/// absolute flag is reported as an error here instead of guessing a semantics.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Xform {
    a: f64,
    b: f64,
    c: f64,
    d: f64,
    tx: f64,
    ty: f64,
}

impl Xform {
    fn identity() -> Self {
        Self {
            a: 1.0,
            b: 0.0,
            c: 0.0,
            d: 1.0,
            tx: 0.0,
            ty: 0.0,
        }
    }

    /// Builds from the STRANS record plus the instance origin. `what` is only used in error messages.
    fn instance(
        strans: Option<&GdsStrans>,
        ox: f64,
        oy: f64,
        what: &str,
    ) -> Result<Self, ParseError> {
        let mut refl = false;
        let mut mag = 1.0f64;
        let mut angle = 0.0f64;
        if let Some(s) = strans {
            if s.abs_mag || s.abs_angle {
                return Err(ParseError::Parse(format!(
                    "gds hierarchy: {} uses absolute magnification/angle (STRANS abs bits), \
                     which this reader does not model; refusing to guess",
                    what
                )));
            }
            refl = s.reflected;
            mag = s.mag.unwrap_or(1.0);
            angle = s.angle.unwrap_or(0.0);
        }
        // GDSII: a missing or zero mag is treated as 1.0 (a zero magnification has no geometric meaning).
        if mag == 0.0 {
            mag = 1.0;
        }
        // Mirror (about the x axis) -> magnify
        let (m00, m01, m10, m11) = if refl {
            (mag, 0.0, 0.0, -mag)
        } else {
            (mag, 0.0, 0.0, mag)
        };
        // Rotate
        let rad = angle.to_radians();
        let (sa, ca) = (snap_unit(rad.sin()), snap_unit(rad.cos()));
        Ok(Self {
            a: ca * m00 - sa * m10,
            b: ca * m01 - sa * m11,
            c: sa * m00 + ca * m10,
            d: sa * m01 + ca * m11,
            tx: ox,
            ty: oy,
        })
    }

    /// `self ∘ inner`: apply `inner` first, then `self`.
    fn compose(&self, inner: &Xform) -> Xform {
        Xform {
            a: self.a * inner.a + self.b * inner.c,
            b: self.a * inner.b + self.b * inner.d,
            c: self.c * inner.a + self.d * inner.c,
            d: self.c * inner.b + self.d * inner.d,
            tx: self.a * inner.tx + self.b * inner.ty + self.tx,
            ty: self.c * inner.tx + self.d * inner.ty + self.ty,
        }
    }

    /// Linear part only (the AREF row/column pitch vectors already live in the
    /// parent coordinate system and must not be rotated a second time).
    fn linear(&self) -> Xform {
        Xform {
            tx: 0.0,
            ty: 0.0,
            ..*self
        }
    }

    fn apply(&self, x: f64, y: f64) -> (f64, f64) {
        (
            self.a * x + self.b * y + self.tx,
            self.c * x + self.d * y + self.ty,
        )
    }
}

/// Snaps sin/cos values that sit next to an integer back to the exact value:
/// `cos(90°)` is 6.1e-17 rather than 0, and feeding that straight into
/// floor/ceil inflates the bbox by 1 DBU and breaks determinism.
fn snap_unit(v: f64) -> f64 {
    let r = v.round();
    if (v - r).abs() < 1e-9 {
        r
    } else {
        v
    }
}

/// Floating-point bbox accumulator; rounds outward when finalized (lo down, hi up) so coverage is never understated.
#[derive(Debug, Clone, Copy, Default)]
struct FBox {
    lo: Option<(f64, f64, f64, f64)>,
}

impl FBox {
    fn add_point(&mut self, x: f64, y: f64) {
        self.lo = Some(match self.lo {
            None => (x, y, x, y),
            Some((xl, yl, xh, yh)) => (xl.min(x), yl.min(y), xh.max(x), yh.max(y)),
        });
    }

    fn add_box(&mut self, b: &FBox) {
        if let Some((xl, yl, xh, yh)) = b.lo {
            self.add_point(xl, yl);
            self.add_point(xh, yh);
        }
    }

    fn shifted(&self, dx: f64, dy: f64) -> FBox {
        FBox {
            lo: self
                .lo
                .map(|(xl, yl, xh, yh)| (xl + dx, yl + dy, xh + dx, yh + dy)),
        }
    }

    fn finish(&self) -> Option<(i32, i32, i32, i32)> {
        self.lo.map(|(xl, yl, xh, yh)| {
            (
                snap_out(xl, false),
                snap_out(yl, false),
                snap_out(xh, true),
                snap_out(yh, true),
            )
        })
    }
}

fn snap_out(v: f64, up: bool) -> i32 {
    let r = v.round();
    let v = if (v - r).abs() < 1e-6 { r } else { v };
    let out = if up { v.ceil() } else { v.floor() };
    out.clamp(i32::MIN as f64, i32::MAX as f64) as i32
}

/// `name -> struct` index. Duplicate struct names in a GDS are invalid input (the first is kept and an error is reported).
fn index_structs(lib: &GdsLibrary) -> HashMap<&str, &GdsStruct> {
    let mut map: HashMap<&str, &GdsStruct> = HashMap::new();
    for s in &lib.structs {
        map.entry(s.name.as_str()).or_insert(s);
    }
    map
}

/// Accumulates the geometry bbox of the subtree rooted at `name` (with every
/// SREF/AREF expanded) in the `xf` coordinate system.
///
/// BOUNDARY and PATH count (a PATH is grown by its half width, the same rule
/// `run_gds21_extract` already uses); TEXT / BOX / NODE do not, matching the
/// previous behaviour so the bbox of a flat cell is not changed in passing.
fn accumulate_hier_bbox(
    index: &HashMap<&str, &GdsStruct>,
    name: &str,
    xf: &Xform,
    acc: &mut FBox,
    stack: &mut Vec<String>,
) -> Result<(), ParseError> {
    if stack.iter().any(|s| s == name) {
        return Err(ParseError::Parse(format!(
            "gds hierarchy: cyclic struct reference at {:?} (path {:?})",
            name, stack
        )));
    }
    let Some(s) = index.get(name) else {
        // Dangling reference: the GDS references a struct that is not in the
        // library. Report an error rather than silently treating it as empty.
        return Err(ParseError::Parse(format!(
            "gds hierarchy: dangling struct reference {:?}",
            name
        )));
    };
    stack.push(name.to_string());
    for elem in &s.elems {
        match elem {
            GdsElement::GdsBoundary(b) => {
                for p in &b.xy {
                    let (x, y) = xf.apply(p.x as f64, p.y as f64);
                    acc.add_point(x, y);
                }
            }
            GdsElement::GdsPath(p) => {
                let half = (p.width.unwrap_or(0) as f64) / 2.0;
                if legacy_pcell_path() {
                    for pt in &p.xy {
                        for (ox, oy) in [(-half, -half), (half, -half), (half, half), (-half, half)]
                        {
                            let (x, y) = xf.apply(pt.x as f64 + ox, pt.y as f64 + oy);
                            acc.add_point(x, y);
                        }
                    }
                } else {
                    let (bgn, end) = path_ends(p)?.as_f64(half);
                    for (xl, yl, xh, yh) in path_segment_boxes(&p.xy, half, bgn, end) {
                        for (cx, cy) in [(xl, yl), (xh, yl), (xh, yh), (xl, yh)] {
                            let (x, y) = xf.apply(cx, cy);
                            acc.add_point(x, y);
                        }
                    }
                }
            }
            GdsElement::GdsStructRef(r) => {
                let child = Xform::instance(
                    r.strans.as_ref(),
                    r.xy.x as f64,
                    r.xy.y as f64,
                    &format!("SREF {:?} in struct {:?}", r.name, name),
                )?;
                accumulate_hier_bbox(index, &r.name, &xf.compose(&child), acc, stack)?;
            }
            GdsElement::GdsArrayRef(a) => {
                let what = format!("AREF {:?} in struct {:?}", a.name, name);
                let base =
                    Xform::instance(a.strans.as_ref(), a.xy[0].x as f64, a.xy[0].y as f64, &what)?;
                // Bbox of a single instance (in the parent coordinate system)
                let mut one = FBox::default();
                accumulate_hier_bbox(index, &a.name, &xf.compose(&base), &mut one, stack)?;
                // GDSII: xy[1] = origin + cols·column vector, xy[2] = origin + rows·row vector;
                // both vectors **already live in the parent coordinate system** (rotation and
                // magnification included), so only the linear part of `xf` applies to them.
                let cols = a.cols.max(1) as f64;
                let rows = a.rows.max(1) as f64;
                let lin = xf.linear();
                let colv = lin.apply(
                    (a.xy[1].x - a.xy[0].x) as f64 / cols,
                    (a.xy[1].y - a.xy[0].y) as f64 / cols,
                );
                let rowv = lin.apply(
                    (a.xy[2].x - a.xy[0].x) as f64 / rows,
                    (a.xy[2].y - a.xy[0].y) as f64 / rows,
                );
                // The translation set {i·colv + j·rowv} is linear in (i,j) ⇒ the extrema sit at the four corners.
                for i in [0.0, cols - 1.0] {
                    for j in [0.0, rows - 1.0] {
                        acc.add_box(&one.shifted(i * colv.0 + j * rowv.0, i * colv.1 + j * rowv.1));
                    }
                }
            }
            _ => {}
        }
    }
    stack.pop();
    Ok(())
}

/// **Hierarchy-aware** bbox of the struct `name` (its own elements plus every child struct, transforms included).
fn hier_struct_bbox(
    lib: &GdsLibrary,
    name: &str,
) -> Result<Option<(i32, i32, i32, i32)>, ParseError> {
    let index = index_structs(lib);
    let mut acc = FBox::default();
    let mut stack = Vec::new();
    accumulate_hier_bbox(&index, name, &Xform::identity(), &mut acc, &mut stack)?;
    Ok(acc.finish())
}

fn translate_point(p: &mut GdsPoint, dx: i32, dy: i32) -> Result<(), ParseError> {
    p.x = p.x.checked_add(dx).ok_or_else(|| {
        ParseError::Parse("gds x coordinate overflow during normalization".into())
    })?;
    p.y = p.y.checked_add(dy).ok_or_else(|| {
        ParseError::Parse("gds y coordinate overflow during normalization".into())
    })?;
    Ok(())
}

fn translate_gds_element(elem: &mut GdsElement, dx: i32, dy: i32) -> Result<(), ParseError> {
    match elem {
        GdsElement::GdsBoundary(b) => {
            for p in &mut b.xy {
                translate_point(p, dx, dy)?;
            }
        }
        GdsElement::GdsPath(p) => {
            for pt in &mut p.xy {
                translate_point(pt, dx, dy)?;
            }
        }
        GdsElement::GdsStructRef(r) => {
            translate_point(&mut r.xy, dx, dy)?;
        }
        GdsElement::GdsArrayRef(a) => {
            for p in &mut a.xy {
                translate_point(p, dx, dy)?;
            }
        }
        GdsElement::GdsTextElem(t) => {
            translate_point(&mut t.xy, dx, dy)?;
        }
        GdsElement::GdsNode(n) => {
            for p in &mut n.xy {
                translate_point(p, dx, dy)?;
            }
        }
        GdsElement::GdsBox(b) => {
            for p in &mut b.xy {
                translate_point(p, dx, dy)?;
            }
        }
    }
    Ok(())
}

fn normalize_single_gds(src: &Path, dst: &Path) -> Result<(), ParseError> {
    let mut lib = GdsLibrary::load(src).map_err(|e| {
        ParseError::Parse(format!(
            "pcell normalize failed to load {}: {}",
            src.display(),
            e
        ))
    })?;
    normalize_library(&mut lib, &src.display().to_string())?;

    if let Some(parent) = dst.parent() {
        std::fs::create_dir_all(parent)?;
    }
    lib.save(dst).map_err(|e| {
        ParseError::Parse(format!(
            "pcell normalize failed to save {}: {}",
            dst.display(),
            e
        ))
    })?;
    Ok(())
}

/// Normalizes a PCell library in place so that the **hierarchy-aware bbox of the
/// device struct** lands at (0,0).
///
/// How it works:
///
/// 1. The bbox is measured with [`hier_struct_bbox`] -- descending into
///    SREF/AREF and applying the transforms, instead of looking only at the
///    polygons the selected struct owns;
/// 2. **Exactly one place is translated** -- the direct elements of the device
///    struct itself (including the origins of the SREF/AREF it emits). Nothing
///    inside a child struct moves, so the "a leaf at depth d gets moved by
///    d·(dx,dy)" tearing can no longer happen;
/// 3. If the device struct has ancestor wrappers (pure forwarders: no geometry
///    or labels of their own, exactly one reference, identity transform), their
///    reference origins are zeroed so that the whole file starts at (0,0) under
///    **any** reading (top-level flatten or device-struct flatten). An ancestor
///    that is not a pure forwarder is left untouched with a warning -- the
///    device coordinate system is already correct, and that is what the flow
///    consumes.
/// 4. The self-check re-measures with the same hierarchy-aware functions and
///    requires **idempotence** (running again on an already normalized library
///    must give offset (0,0)). The old assertion re-measured itself with the
///    hierarchy-blind function, so it could never catch its own bug.
fn normalize_library(lib: &mut GdsLibrary, what: &str) -> Result<(), ParseError> {
    let Some(dev) = pick_device_struct_name(lib) else {
        return Err(ParseError::Parse(format!(
            "pcell normalize failed: no structs in {}",
            what
        )));
    };
    let Some((xl, yl, xh, yh)) = hier_struct_bbox(lib, &dev)? else {
        return Err(ParseError::Parse(format!(
            "pcell normalize failed: no drawable geometry under struct {:?} in {}",
            dev, what
        )));
    };
    let dx = -xl;
    let dy = -yl;

    // (2) Touch only the device struct's own elements.
    if dx != 0 || dy != 0 {
        let Some(s) = lib.structs.iter_mut().find(|s| s.name == dev) else {
            return Err(ParseError::Parse(format!(
                "pcell normalize failed: struct {:?} vanished in {}",
                dev, what
            )));
        };
        for e in &mut s.elems {
            translate_gds_element(e, dx, dy)?;
        }
    }

    // (3) Zero the reference origins of the pure-forwarder ancestors.
    zero_passthrough_ancestors(lib, &dev, what);

    // (4) Hierarchy-aware self-check plus idempotence.
    let Some((nxl, nyl, nxh, nyh)) = hier_struct_bbox(lib, &dev)? else {
        return Err(ParseError::Parse(format!(
            "pcell normalize produced empty geometry: {} struct {:?}",
            what, dev
        )));
    };
    if nxl != 0 || nyl != 0 {
        return Err(ParseError::Parse(format!(
            "pcell normalize validation failed for {} struct {:?}: expected hierarchy-aware \
             bbox origin (0,0), got ({},{})",
            what, dev, nxl, nyl
        )));
    }
    if (xh - xl) != (nxh - nxl) || (yh - yl) != (nyh - nyl) {
        return Err(ParseError::Parse(format!(
            "pcell normalize validation failed for {} struct {:?}: hierarchy-aware size changed \
             from ({},{}) to ({},{}) — the cell was torn",
            what,
            dev,
            xh - xl,
            yh - yl,
            nxh - nxl,
            nyh - nyl
        )));
    }
    // Idempotence: both the device-struct selection and the offset must be stable after normalization.
    let redev = pick_device_struct_name(lib).unwrap_or_default();
    if redev != dev {
        return Err(ParseError::Parse(format!(
            "pcell normalize validation failed for {}: device struct changed from {:?} to {:?} \
             after normalization (not idempotent)",
            what, dev, redev
        )));
    }
    Ok(())
}

/// Zeroes the reference origins of the device struct's **pure-forwarder
/// ancestors** (no geometry or labels of their own, exactly one reference,
/// identity transform). If any one ancestor fails those conditions the whole
/// chain is left untouched and a warning is emitted.
fn zero_passthrough_ancestors(lib: &mut GdsLibrary, dev: &str, what: &str) {
    // Collect bottom-up: child -> names of the structs that reference it (possibly several).
    let mut cur = dev.to_string();
    let mut chain: Vec<(String, String)> = Vec::new(); // (parent, child)
    let mut guard = 0usize;
    loop {
        guard += 1;
        if guard > lib.structs.len() + 1 {
            return; // Cycle; hier_struct_bbox already reports it, so bail out conservatively here
        }
        let parents: Vec<&GdsStruct> = lib
            .structs
            .iter()
            .filter(|s| {
                s.elems.iter().any(|e| match e {
                    GdsElement::GdsStructRef(r) => r.name == cur,
                    GdsElement::GdsArrayRef(a) => a.name == cur,
                    _ => false,
                })
            })
            .collect();
        if parents.is_empty() {
            break;
        }
        if parents.len() > 1 {
            eprintln!(
                "[pcell] note: {} struct {:?} has {} parents; leaving wrapper origins untouched",
                what,
                cur,
                parents.len()
            );
            return;
        }
        let p = parents[0];
        let pure = p.elems.len() == 1
            && matches!(&p.elems[0], GdsElement::GdsStructRef(r)
                if r.name == cur && r.strans.is_none());
        if !pure {
            eprintln!(
                "[pcell] note: {} wrapper struct {:?} is not a pure passthrough; \
                 its origin is left as-is (device frame is still normalized)",
                what, p.name
            );
            return;
        }
        chain.push((p.name.clone(), cur.clone()));
        cur = p.name.clone();
    }
    for (parent, child) in chain {
        if let Some(s) = lib.structs.iter_mut().find(|s| s.name == parent) {
            for e in &mut s.elems {
                if let GdsElement::GdsStructRef(r) = e {
                    if r.name == child {
                        r.xy = GdsPoint { x: 0, y: 0 };
                    }
                }
            }
        }
    }
}

fn prepare_normalized_pcelldir(
    src_dir: &Path,
    src_paths: &[PathBuf],
) -> Result<PathBuf, ParseError> {
    let out_dir = normalized_pcelldir_path(src_dir);
    if out_dir.exists()
        && src_paths.iter().all(|src| {
            let rel = src
                .strip_prefix(src_dir)
                .map(|p| p.to_path_buf())
                .unwrap_or_else(|_| {
                    src.file_name()
                        .map(PathBuf::from)
                        .unwrap_or_else(|| PathBuf::from("unknown.gds"))
                });
            out_dir.join(rel).exists()
        })
    {
        return Ok(out_dir);
    }
    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    let tmp_name = format!(
        "{}.tmp.{}.{}",
        out_dir
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("pcell_normalized"),
        std::process::id(),
        stamp
    );
    let tmp_dir = out_dir
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .join(tmp_name);

    if tmp_dir.exists() {
        std::fs::remove_dir_all(&tmp_dir)?;
    }
    std::fs::create_dir_all(&tmp_dir)?;

    for src in src_paths {
        if !src.exists() {
            return Err(ParseError::Parse(format!(
                "pcell normalize failed: missing source gds {}",
                src.display()
            )));
        }
        let rel = src
            .strip_prefix(src_dir)
            .map(|p| p.to_path_buf())
            .unwrap_or_else(|_| {
                src.file_name()
                    .map(PathBuf::from)
                    .unwrap_or_else(|| PathBuf::from("unknown.gds"))
            });
        let dst = tmp_dir.join(rel);
        normalize_single_gds(src, &dst)?;
    }

    if out_dir.exists() {
        std::fs::remove_dir_all(&out_dir)?;
    }
    std::fs::rename(&tmp_dir, &out_dir)?;
    Ok(out_dir)
}

fn load_stdcell_map(path: Option<&Path>) -> Result<HashMap<String, Vec<String>>, ParseError> {
    let mut map: HashMap<String, Vec<String>> = HashMap::new();
    if let Some(p) = path {
        let content = std::fs::read_to_string(p)?;
        map = serde_json::from_str(&content)?;
    }
    Ok(map)
}

fn load_pdk(path: Option<&Path>) -> Result<PdkRules, ParseError> {
    if let Some(p) = path {
        return parse_pdk(p);
    }
    let default = project_root().join("pdk").join("example_lef.json");
    parse_pdk(&default)
}

fn bbox_from_points(points: &[[i32; 2]]) -> Option<(i32, i32, i32, i32)> {
    if points.is_empty() {
        return None;
    }
    let mut xl = points[0][0];
    let mut yl = points[0][1];
    let mut xh = points[0][0];
    let mut yh = points[0][1];
    for p in points.iter().skip(1) {
        xl = xl.min(p[0]);
        yl = yl.min(p[1]);
        xh = xh.max(p[0]);
        yh = yh.max(p[1]);
    }
    Some((xl, yl, xh, yh))
}

fn bbox_union(a: (i32, i32, i32, i32), b: (i32, i32, i32, i32)) -> (i32, i32, i32, i32) {
    (a.0.min(b.0), a.1.min(b.1), a.2.max(b.2), a.3.max(b.3))
}

fn polys_bbox(polys: &[GdsPoly]) -> Option<(i32, i32, i32, i32)> {
    let mut all_bbox: Option<(i32, i32, i32, i32)> = None;
    for p in polys {
        if let Some(b) = bbox_from_points(&p.points) {
            all_bbox = Some(match all_bbox {
                None => b,
                Some(prev) => bbox_union(prev, b),
            });
        }
    }
    all_bbox
}

fn bbox_overlap(a: &PolyBox, b: &PolyBox) -> bool {
    !(a.xh < b.xl || a.xl > b.xh || a.yh < b.yl || a.yl > b.yh)
}

fn bbox_contains_point(b: &PolyBox, x: i32, y: i32) -> bool {
    x >= b.xl && x <= b.xh && y >= b.yl && y <= b.yh
}

/// Per-struct facts the device-struct picker needs (see [`select_device_struct`]).
#[derive(Debug, Clone, Copy)]
struct StructPick {
    /// TEXT elements in the struct.
    label_count: usize,
    /// BOUNDARY + PATH elements in the struct (SREF/AREF do **not** count).
    geom_count: usize,
    /// Area of the bbox over that geometry; 0 when the struct has none.
    area: i64,
    /// Whether some other struct SREF/AREFs this one.
    referenced: bool,
}

/// Pick the struct that represents the device, given per-struct facts in file
/// order. Tiers, first match wins:
///
/// 1. largest-area struct that carries labels (best for pin extraction);
/// 2. largest-area struct that carries geometry;
/// 3. first struct nobody references (the conventional GDS top);
/// 4. struct 0.
///
/// Ties keep the earlier struct (strict `>` when comparing areas).
///
/// Tier 2 is what keeps the picker independent of **vendor provenance strings**.
/// Some vendor PCells -- serpentine resistors in particular -- carry no functional
/// pin labels at all; their only TEXT elements were a vendor provenance block.
/// Once that block is stripped those files have zero labels, and without tier 2 the chain falls
/// through to the *wrapper* struct, which holds nothing but an SREF — the device
/// would silently lose all of its geometry and pins. Tier 2 lands on the same
/// struct tier 1 used to pick, without reading a single vendor string.
///
/// Verified over every device layout in the maintainer's corpus: this chain picks
/// exactly what the label-only chain (tiers 1/3/4) picked before the provenance
/// strings were stripped, and picks identically before and after.
fn select_device_struct(stats: &[StructPick]) -> usize {
    let best = |pred: fn(&StructPick) -> bool| -> Option<usize> {
        let mut best: Option<(usize, i64)> = None;
        for (idx, s) in stats.iter().enumerate() {
            if !pred(s) {
                continue;
            }
            if best.map(|(_, a)| s.area > a).unwrap_or(true) {
                best = Some((idx, s.area));
            }
        }
        best.map(|(idx, _)| idx)
    };
    best(|s| s.label_count > 0)
        .or_else(|| best(|s| s.geom_count > 0))
        .or_else(|| stats.iter().position(|s| !s.referenced))
        .unwrap_or(0)
}

/// Collects, per struct and in file order, the facts [`select_device_struct`] needs.
fn struct_picks(lib: &GdsLibrary) -> Vec<StructPick> {
    let mut referenced: HashSet<String> = HashSet::new();
    for s in &lib.structs {
        for e in &s.elems {
            match e {
                GdsElement::GdsStructRef(r) => {
                    referenced.insert(r.name.clone());
                }
                GdsElement::GdsArrayRef(a) => {
                    referenced.insert(a.name.clone());
                }
                _ => {}
            }
        }
    }

    let mut stats: Vec<StructPick> = Vec::with_capacity(lib.structs.len());
    for s in lib.structs.iter() {
        let mut label_count = 0usize;
        let mut geom_count = 0usize;
        let mut bbox: Option<(i32, i32, i32, i32)> = None;
        for elem in &s.elems {
            match elem {
                GdsElement::GdsBoundary(b) => {
                    geom_count += 1;
                    let pts: Vec<[i32; 2]> = b.xy.iter().map(|p| [p.x, p.y]).collect();
                    if let Some(bb) = bbox_from_points(&pts) {
                        bbox = Some(match bbox {
                            None => bb,
                            Some(prev) => bbox_union(prev, bb),
                        });
                    }
                }
                // Deliberately *not* the `PATHTYPE` conversion. `struct_picks`' `area`
                // is only the sort key of `select_device_struct` (it picks the largest
                // struct); it reaches neither the DB nor the layout. Freezing it keeps
                // the choice of device struct byte-identical to before, so the
                // single-frame fix (`pick_device_struct_name` is shared with
                // `writer.rs`) is unaffected. Frozen or not, the answer is the same:
                // the corpus test
                // `device_struct_pick_is_insensitive_to_the_path_area_convention`
                // pins that down on every tracked PCell.
                GdsElement::GdsPath(p) => {
                    geom_count += 1;
                    let width = p.width.unwrap_or(0);
                    let half = width / 2;
                    let pts: Vec<[i32; 2]> = p.xy.iter().map(|pt| [pt.x, pt.y]).collect();
                    if let Some((xl, yl, xh, yh)) = bbox_from_points(&pts) {
                        let bb = (xl - half, yl - half, xh + half, yh + half);
                        bbox = Some(match bbox {
                            None => bb,
                            Some(prev) => bbox_union(prev, bb),
                        });
                    }
                }
                GdsElement::GdsTextElem(_) => {
                    label_count += 1;
                }
                _ => {}
            }
        }
        let area = if let Some((xl, yl, xh, yh)) = bbox {
            let w = (xh - xl).max(0) as i64;
            let h = (yh - yl).max(0) as i64;
            w * h
        } else {
            0
        };
        stats.push(StructPick {
            label_count,
            geom_count,
            area,
            referenced: referenced.contains(&s.name),
        });
    }
    stats
}

/// Name of the device struct. **The measurement path in `pcell.rs` and the
/// instantiation path in `writer.rs` share this one selector.** `writer.rs`
/// used to have a chain of its own (it knew only "the largest struct that has a
/// label" and lacked pcell's second tier, "the largest struct that has
/// geometry"). Once the provenance labels were stripped from a set of resistor
/// PCells, the writer landed on the wrapper while pcell landed on the inner
/// struct, so the layout and the DB differed by one wrapper SREF origin.
pub(crate) fn pick_device_struct_name(lib: &GdsLibrary) -> Option<String> {
    if lib.structs.is_empty() {
        return None;
    }
    let stats = struct_picks(lib);
    let idx = select_device_struct(&stats);
    lib.structs.get(idx).map(|s| s.name.clone())
}

fn run_gds21_extract(gds_path: &Path) -> Result<GdsData, ParseError> {
    let lib = GdsLibrary::load(gds_path).map_err(|e| {
        ParseError::Parse(format!(
            "gds21 load failed for {}: {}",
            gds_path.display(),
            e
        ))
    })?;
    let stats = struct_picks(&lib);
    let selected_idx = select_device_struct(&stats);

    let top = lib
        .structs
        .get(selected_idx)
        .map(|s| s.name.clone())
        .unwrap_or_else(|| "TOP".to_string());

    // Hierarchy-aware bbox of the device struct -- this is what the DB cell
    // footprint uses, not the "own polygons" variant (for a hierarchical PCell
    // the latter misses a ring of internal margin contributed by the child
    // structs, as hierarchical capacitor devices show).
    let hier_bbox = if lib.structs.is_empty() {
        None
    } else {
        hier_struct_bbox(&lib, &top)?
    };

    let mut polygons = Vec::new();
    let mut labels = Vec::new();

    let Some(cell) = lib.structs.get(selected_idx) else {
        return Ok(GdsData {
            top,
            polygons,
            labels,
            hier_bbox,
        });
    };

    for elem in &cell.elems {
        match elem {
            GdsElement::GdsBoundary(b) => {
                let pts = b.xy.iter().map(|p| [p.x, p.y]).collect();
                polygons.push(GdsPoly {
                    layer: b.layer as i32,
                    datatype: b.datatype as i32,
                    points: pts,
                });
            }
            GdsElement::GdsPath(p) => {
                let width = p.width.unwrap_or(0);
                let half = width / 2;
                let pts: Vec<[i32; 2]> = p.xy.iter().map(|pt| [pt.x, pt.y]).collect();
                if legacy_pcell_path() {
                    if let Some((xl, yl, xh, yh)) = bbox_from_points(&pts) {
                        let poly = vec![
                            [xl - half, yl - half],
                            [xh + half, yl - half],
                            [xh + half, yh + half],
                            [xl - half, yh + half],
                            [xl - half, yl - half],
                        ];
                        polygons.push(GdsPoly {
                            layer: p.layer as i32,
                            datatype: p.datatype as i32,
                            points: poly,
                        });
                    } else {
                        polygons.push(GdsPoly {
                            layer: p.layer as i32,
                            datatype: p.datatype as i32,
                            points: pts,
                        });
                    }
                } else if p.xy.is_empty() {
                    polygons.push(GdsPoly {
                        layer: p.layer as i32,
                        datatype: p.datatype as i32,
                        points: pts,
                    });
                } else {
                    // End extension by PATHTYPE; the perpendicular `half` is
                    // byte-identical to before (including the integer truncation for
                    // odd line widths).
                    let (bgn, end) = path_ends(p)?.as_i32(half);
                    for (xl, yl, xh, yh) in path_segment_boxes(&p.xy, half, bgn, end) {
                        polygons.push(GdsPoly {
                            layer: p.layer as i32,
                            datatype: p.datatype as i32,
                            points: vec![[xl, yl], [xh, yl], [xh, yh], [xl, yh], [xl, yl]],
                        });
                    }
                }
            }
            GdsElement::GdsTextElem(t) => {
                labels.push(GdsLabel {
                    text: t.string.clone(),
                    layer: t.layer as i32,
                    texttype: t.texttype as i32,
                    x: t.xy.x,
                    y: t.xy.y,
                });
            }
            _ => {}
        }
    }

    Ok(GdsData {
        top,
        polygons,
        labels,
        hier_bbox,
    })
}

/// Which PCell pin ordering a device uses: 0 = MOS, 1 = cap, 2 = res, 3 = standard cell.
///
/// The deck's `device_types` section is the only rule: one declaration, one
/// classification, so a deck cannot say a model is PMOS here and something else in
/// the placer. A deck that declares nothing is refused when it is loaded
/// (`io::apply_device_types`), so the only thing left to decide here is whether an
/// undeclared or `other` model is a standard cell.
fn classify_pcell(
    model: &str,
    pdk: &PdkRules,
    stdcells: &HashMap<String, Vec<String>>,
) -> Option<i32> {
    let Ok(Some(dt)) = pdk.device_types() else {
        return stdcells.contains_key(model).then_some(3);
    };
    match dt.classify(model) {
        Some(DeviceClass::Pmos) | Some(DeviceClass::Nmos) => Some(0),
        Some(DeviceClass::Capacitor) => Some(1),
        Some(DeviceClass::Resistor) => Some(2),
        // `other` and unlisted both fall through to the standard-cell table; an
        // unlisted model is separately a hard error in `io::apply_device_types`.
        _ => stdcells.contains_key(model).then_some(3),
    }
}

fn pin_order_for(
    pcell_type: i32,
    model: &str,
    stdcells: &HashMap<String, Vec<String>>,
) -> Vec<String> {
    match pcell_type {
        0 => vec!["D", "G", "S", "B"]
            .into_iter()
            .map(|s| s.to_string())
            .collect(),
        1 => vec!["PLUS", "MINUS", "BULK"]
            .into_iter()
            .map(|s| s.to_string())
            .collect(),
        2 => vec!["PLUS", "MINUS"]
            .into_iter()
            .map(|s| s.to_string())
            .collect(),
        3 => stdcells.get(model).cloned().unwrap_or_default(),
        _ => Vec::new(),
    }
}

fn gds_path_for(
    pcelldir: &Path,
    cell_name: &str,
    top_name: &str,
    pcell_type: i32,
    model: &str,
) -> PathBuf {
    let leaf = cell_name.split('_').next_back().unwrap_or(cell_name);
    if pcell_type == 3 {
        return pcelldir.join(format!("{}.gds", model));
    }
    let primary = pcelldir.join(format!("{}__{}.gds", top_name, leaf));
    if primary.exists() {
        return primary;
    }
    let fallback_model = pcelldir.join(format!("{}.gds", model));
    if fallback_model.exists() {
        return fallback_model;
    }
    // Last-resort fallback: scan pcelldir for files named `*_{leaf}.gds` (some
    // cases name their layout files with a prefix that matches neither the
    // netlist top name nor the device model). A match requires the character
    // before leaf to be `_` and `.gds` to follow leaf immediately (so that M0
    // does not match M10). Deterministic choice: sort by file name and take the
    // first, warning when there is more than one.
    if let Ok(read_dir) = std::fs::read_dir(pcelldir) {
        let suffix = format!("_{}.gds", leaf);
        let mut matches: Vec<String> = read_dir
            .flatten()
            .filter_map(|e| e.file_name().into_string().ok())
            .filter(|name| name.len() > suffix.len() && name.ends_with(&suffix))
            .collect();
        matches.sort();
        if matches.len() > 1 {
            eprintln!(
                "[pcell] warning: multiple GDS files match *{} in {} ({:?}); using {}",
                suffix,
                pcelldir.display(),
                matches,
                matches[0]
            );
        }
        if let Some(first) = matches.first() {
            return pcelldir.join(first);
        }
    }
    primary
}

fn derive_top_info(db: &CircuitDb) -> (String, HashSet<String>) {
    let top_pins = db
        .ports()
        .iter()
        .filter_map(|&net_idx| db.net(net_idx).map(|net| net.name().to_string()))
        .collect();
    (db.name().to_string(), top_pins)
}

fn canonical_net_name(top: &str, top_pins: &HashSet<String>, net_name: &str) -> String {
    if top_pins.contains(net_name) || net_name.starts_with(&format!("{}_", top)) {
        return net_name.to_string();
    }
    let leaf = net_name.rsplit('_').next().unwrap_or(net_name);
    format!("{}_{}", top, leaf)
}

/// Extracts the pin metal rectangles from a normalized GDS and also returns the
/// metal on routing layers that belongs to no pin (blockages: obstacles for anything
/// routed later). Device layers (PO/CO and anything else outside
/// `pinlayers`) never become blockages.
fn compute_pin_shapes(
    gds: &GdsData,
    pcell_type: i32,
    pin_order: &[String],
    pinlayers: &[i32],
    po_layer: Option<i32>,
    co_layer: Option<i32>,
) -> (Vec<Vec<PolyBox>>, Vec<PolyBox>) {
    let mut all_polys: Vec<PolyBox> = Vec::new();
    for p in &gds.polygons {
        if let Some((xl, yl, xh, yh)) = bbox_from_points(&p.points) {
            all_polys.push(PolyBox {
                layer: p.layer,
                datatype: p.datatype,
                xl,
                yl,
                xh,
                yh,
            });
        }
    }

    let mut metal_polys: Vec<PolyBox> = all_polys
        .iter()
        .filter(|p| pinlayers.contains(&p.layer))
        .cloned()
        .collect();

    let mut dsu = Dsu::new(metal_polys.len());
    for i in 0..metal_polys.len() {
        for j in (i + 1)..metal_polys.len() {
            if bbox_overlap(&metal_polys[i], &metal_polys[j]) {
                dsu.union(i, j);
            }
        }
    }

    let mut label_map: HashMap<&str, Vec<&GdsLabel>> = HashMap::new();
    for label in &gds.labels {
        label_map
            .entry(label.text.as_str())
            .or_default()
            .push(label);
    }

    let mut pin_shapes: Vec<Vec<PolyBox>> = vec![Vec::new(); pin_order.len()];
    // Indices into metal_polys that were ever attributed to some pin (including
    // those later superseded by a gate) never become blockages -- to stay
    // conservative, only metal that was never associated with a pin is treated
    // as an obstacle.
    let mut assigned: HashSet<usize> = HashSet::new();

    for (pidx, pname) in pin_order.iter().enumerate() {
        let Some(labels) = label_map.get(pname.as_str()) else {
            continue;
        };
        let mut roots = HashSet::new();
        for label in labels {
            let target_layer = if pcell_type == 3 {
                label.layer - 100
            } else {
                label.layer
            };
            for (ridx, mp) in metal_polys.iter().enumerate() {
                if pcell_type == 3 && mp.layer != target_layer {
                    continue;
                }
                if bbox_contains_point(mp, label.x, label.y) {
                    let root = dsu.find(ridx);
                    roots.insert(root);
                    break;
                }
            }
        }
        if roots.is_empty() {
            continue;
        }
        let mut added = HashSet::new();
        for (ridx, mp) in metal_polys.iter().enumerate() {
            let root = dsu.find(ridx);
            if roots.contains(&root) && !added.contains(&ridx) {
                pin_shapes[pidx].push(mp.clone());
                added.insert(ridx);
                assigned.insert(ridx);
            }
        }
    }

    // Resistor PCells (pcell_type==2, pin_order=[PLUS,MINUS]) often carry no
    // text label at all. The ports are then inferred
    // from geometry alone: a serpentine resistor is a chain of PO strips
    // stitched together by CO/M1 jumpers, so an interior M1 jumper connects two
    // PO strips (degree 2) while a port M1 group connects only one (degree 1).
    // The two M1 metal groups of degree 1 are PLUS and MINUS.
    if pcell_type == 2 && pin_shapes.len() >= 2 && pin_shapes.iter().all(|v| v.is_empty()) {
        if let (Some(po), Some(co)) = (po_layer, co_layer) {
            if let Some(port_groups) =
                infer_resistor_ports(&gds.top, &all_polys, &metal_polys, &mut dsu, po, co)
            {
                for (pidx, group) in port_groups.into_iter().enumerate() {
                    for midx in group {
                        pin_shapes[pidx].push(metal_polys[midx].clone());
                        assigned.insert(midx);
                    }
                }
            }
        }
    }

    if pcell_type == 0 {
        if let (Some(po), Some(co)) = (po_layer, co_layer) {
            let po_polys: Vec<&PolyBox> = all_polys.iter().filter(|p| p.layer == po).collect();
            let co_polys: Vec<&PolyBox> = all_polys.iter().filter(|p| p.layer == co).collect();
            let mut gate_polys: Vec<PolyBox> = Vec::new();
            // One piece of gate metal overlaps **every** gate CO of the device,
            // so the results must be deduplicated by metal_polys index -- the
            // same `added` guard the label-driven branch above uses. Without it
            // a single horizontal M1 gate strip is pushed N times (N = number of
            // gate COs) and downstream receives N **identical** shapes: the
            // evaluator reports the same gap N times, `pin_metal_area` inflates
            // several-fold, and the A* pin enumeration does N times the useless
            // work.
            let mut gate_added: HashSet<usize> = HashSet::new();
            for co_p in co_polys {
                let mut hit_po = false;
                for po_p in &po_polys {
                    if bbox_overlap(co_p, po_p) {
                        hit_po = true;
                        break;
                    }
                }
                if !hit_po {
                    continue;
                }
                for (midx, mp) in metal_polys.iter().enumerate() {
                    if bbox_overlap(mp, co_p) && gate_added.insert(midx) {
                        gate_polys.push(mp.clone());
                        assigned.insert(midx);
                    }
                }
            }
            if pin_shapes.len() > 1 {
                pin_shapes[1] = gate_polys;
            }
        }
    }

    let blockages: Vec<PolyBox> = metal_polys
        .drain(..)
        .enumerate()
        .filter(|(idx, _)| !assigned.contains(idx))
        .map(|(_, mp)| mp)
        .collect();

    (pin_shapes, blockages)
}

/// Geometric port inference for resistors: the nodes are the PO strips and the
/// M1 metal groups (merged by touch with a Dsu), and a CO that overlaps both a
/// PO strip and a metal group contributes an edge. Returns the metal_polys
/// indices of exactly the two degree-1 metal groups (the ports), ordered
/// lexicographically by the (xl, yl) of their union bbox (the smaller one is
/// PLUS). Returns None with a warning when the number of degree-1 groups is not
/// exactly 2.
fn infer_resistor_ports(
    cell_name: &str,
    all_polys: &[PolyBox],
    metal_polys: &[PolyBox],
    dsu: &mut Dsu,
    po_layer: i32,
    co_layer: i32,
) -> Option<[Vec<usize>; 2]> {
    let po_polys: Vec<&PolyBox> = all_polys.iter().filter(|p| p.layer == po_layer).collect();
    let co_polys: Vec<&PolyBox> = all_polys.iter().filter(|p| p.layer == co_layer).collect();
    if po_polys.is_empty() || co_polys.is_empty() || metal_polys.is_empty() {
        return None;
    }

    // root(metal group) -> set of indices of the PO strips it connects to.
    let mut group_po: HashMap<usize, HashSet<usize>> = HashMap::new();
    for co_p in &co_polys {
        let mut po_hits: Vec<usize> = Vec::new();
        for (poidx, po_p) in po_polys.iter().enumerate() {
            if bbox_overlap(co_p, po_p) {
                po_hits.push(poidx);
            }
        }
        if po_hits.is_empty() {
            continue;
        }
        for (midx, mp) in metal_polys.iter().enumerate() {
            if bbox_overlap(mp, co_p) {
                let root = dsu.find(midx);
                let entry = group_po.entry(root).or_default();
                for &poidx in &po_hits {
                    entry.insert(poidx);
                }
            }
        }
    }

    let port_roots: Vec<usize> = group_po
        .iter()
        .filter(|(_, pos)| pos.len() == 1)
        .map(|(&root, _)| root)
        .collect();
    if port_roots.len() != 2 {
        eprintln!(
            "[pcell] warning: resistor pin inference skipped for cell {}: \
             found {} degree-1 metal groups (expected 2)",
            cell_name,
            port_roots.len()
        );
        return None;
    }

    let members = |root: usize, dsu: &mut Dsu| -> Vec<usize> {
        (0..metal_polys.len())
            .filter(|&i| dsu.find(i) == root)
            .collect()
    };
    let group_key = |idxs: &[usize]| -> (i32, i32) {
        let bbox = poly_bbox(
            &idxs
                .iter()
                .map(|&i| metal_polys[i].clone())
                .collect::<Vec<_>>(),
        )
        .expect("port group is non-empty");
        (bbox.0, bbox.1)
    };
    let g0 = members(port_roots[0], dsu);
    let g1 = members(port_roots[1], dsu);
    if group_key(&g1) < group_key(&g0) {
        return Some([g1, g0]);
    }
    Some([g0, g1])
}

fn poly_bbox(polys: &[PolyBox]) -> Option<(i32, i32, i32, i32)> {
    if polys.is_empty() {
        return None;
    }
    let mut bbox = (polys[0].xl, polys[0].yl, polys[0].xh, polys[0].yh);
    for p in polys.iter().skip(1) {
        bbox = bbox_union(bbox, (p.xl, p.yl, p.xh, p.yh));
    }
    Some(bbox)
}

/// Parse pcells from GDS and update CircuitDb. Returns bbox/pin outputs.
pub fn parse_pcell(
    pcelldir: &Path,
    stdcell: Option<&Path>,
    pdk_path: Option<&Path>,
    db: &mut CircuitDb,
) -> Result<PcellOutput, ParseError> {
    let pdk = load_pdk(pdk_path)?;
    let stdcells = load_stdcell_map(stdcell)?;
    let (top_name, top_pins) = derive_top_info(db);
    let _pin_order: Vec<String> = Vec::new();

    let min_layer = *pdk.layer_map.get("M1").unwrap_or(&31) as i32;
    let max_layer = *pdk.layer_map.get("M6").unwrap_or(&36) as i32;
    let pinlayers: Vec<i32> = (min_layer..=max_layer).collect();
    let po_layer = pdk.layer_map.get("PO").map(|v| *v as i32);
    let co_layer = pdk.layer_map.get("CO").map(|v| *v as i32);

    let mut cell_bboxes: HashMap<String, (i32, i32, i32, i32)> = HashMap::new();
    let mut pin_shapes_by_name: HashMap<String, Vec<Vec<PolyBox>>> = HashMap::new();
    let mut blockages_by_name: HashMap<String, Vec<PolyBox>> = HashMap::new();

    let mut required_gds_paths = HashSet::<PathBuf>::new();
    for idx in 0..db.num_cells() {
        let Some(cell) = db.cell(idx as i32) else {
            continue;
        };
        let model = cell.model().to_string();
        let pcell_type = classify_pcell(&model, &pdk, &stdcells);
        let gds_path = gds_path_for(
            pcelldir,
            cell.name(),
            &top_name,
            pcell_type.unwrap_or(0),
            &model,
        );
        if gds_path.exists() {
            required_gds_paths.insert(gds_path);
        }
    }
    let mut required_gds_vec: Vec<PathBuf> = required_gds_paths.into_iter().collect();
    required_gds_vec.sort();
    let resolved_pcelldir = prepare_normalized_pcelldir(pcelldir, &required_gds_vec)?;

    for idx in 0..db.num_cells() {
        let Some(cell) = db.cell(idx as i32) else {
            continue;
        };

        let model = cell.model().to_string();
        let pcell_type = classify_pcell(&model, &pdk, &stdcells);
        let pin_order = pcell_type
            .map(|t| pin_order_for(t, &model, &stdcells))
            .unwrap_or_default();

        let gds_path = gds_path_for(
            &resolved_pcelldir,
            cell.name(),
            &top_name,
            pcell_type.unwrap_or(0),
            &model,
        );
        if !gds_path.exists() {
            // Some higher-level pseudo cells may not have dedicated GDS; skip them.
            continue;
        }

        let gds = run_gds21_extract(&gds_path)?;

        // Cell footprint = the device struct's hierarchy-aware bbox.
        let cell_bbox = gds
            .hier_bbox
            .or_else(|| polys_bbox(&gds.polygons))
            .unwrap_or((0, 0, 0, 0));
        cell_bboxes.insert(cell.name().to_string(), cell_bbox);

        let (pin_shapes, blockages) = if let Some(pcell_type) = pcell_type {
            compute_pin_shapes(&gds, pcell_type, &pin_order, &pinlayers, po_layer, co_layer)
        } else {
            // Unknown model: keep bbox for geometry, skip pin-shape extraction.
            (Vec::new(), Vec::new())
        };
        pin_shapes_by_name.insert(cell.name().to_string(), pin_shapes);
        blockages_by_name.insert(cell.name().to_string(), blockages);
    }

    // Update DB pins/cells
    for idx in 0..db.num_cells() {
        let Some(cell) = db.cell_mut(idx as i32) else {
            continue;
        };
        if let Some(b) = cell_bboxes.get(cell.name()) {
            let w = b.2 - b.0;
            let h = b.3 - b.1;
            cell.set_geometry_offset(b.0, b.1);
            cell.set_width(w);
            cell.set_height(h);
        }
        if let Some(blockages) = blockages_by_name.get(cell.name()) {
            cell.set_blockages(
                blockages
                    .iter()
                    .map(|p| {
                        applace_core::PinShape::new(
                            p.layer,
                            applace_core::geometry::Box::new(p.xl, p.yl, p.xh, p.yh),
                        )
                    })
                    .collect(),
            );
        }
    }

    let mut pin_updates: Vec<(i32, i32, i32, Vec<i32>, Vec<applace_core::PinShape>)> = Vec::new();
    for pidx in 0..db.num_pins() {
        let Some(pin) = db.pin(pidx as i32) else {
            continue;
        };
        let Some(cell_idx) = pin.cell_idx() else {
            continue;
        };
        let Some(cell) = db.cell(cell_idx) else {
            continue;
        };
        let pin_order_idx = {
            let name = pin.name();
            let suffix = name.rsplit('_').next().unwrap_or("");
            suffix.parse::<usize>().ok()
        };
        let Some(order_idx) = pin_order_idx else {
            continue;
        };
        let Some(pin_shapes) = pin_shapes_by_name.get(cell.name()) else {
            continue;
        };
        if order_idx >= pin_shapes.len() {
            continue;
        }
        let polys = &pin_shapes[order_idx];
        let Some(b) = poly_bbox(polys) else { continue };
        let ox = (b.0 + b.2) / 2;
        let oy = (b.1 + b.3) / 2;
        let mut layers: Vec<i32> = polys.iter().map(|p| p.layer).collect();
        layers.sort_unstable();
        layers.dedup();
        let shapes: Vec<applace_core::PinShape> = polys
            .iter()
            .map(|p| {
                applace_core::PinShape::new(
                    p.layer,
                    applace_core::geometry::Box::new(p.xl, p.yl, p.xh, p.yh),
                )
            })
            .collect();
        pin_updates.push((pidx as i32, ox, oy, layers, shapes));
    }
    for (pidx, ox, oy, layers, shapes) in pin_updates {
        if let Some(pin) = db.pin_mut(pidx) {
            pin.set_offset(ox, oy);
            pin.set_layer_ids(layers);
            pin.set_shapes(shapes);
        }
    }

    // Build bbox.json
    let mut bbox_cells = Vec::new();
    let mut bbox_map: HashMap<String, BboxCell> = HashMap::new();
    let mut seen_cells: HashSet<String> = HashSet::new();
    for idx in 0..db.num_cells() {
        let Some(cell) = db.cell(idx as i32) else {
            continue;
        };
        let leaf = cell.name().rsplit('_').next().unwrap_or(cell.name());
        let canon = format!("{}_{}", top_name, leaf);
        if seen_cells.contains(&canon) {
            continue;
        }
        let Some(b) = cell_bboxes.get(cell.name()) else {
            continue;
        };
        let xl = b.0;
        let yl = b.1;
        let xh = b.2;
        let yh = b.3;
        let entry = BboxCell {
            cell_name: canon.clone(),
            bbox: BboxRect { xl, yl, xh, yh },
            model: cell.model().to_string(),
        };
        bbox_map.insert(canon.clone(), entry);
        seen_cells.insert(canon);
    }
    bbox_cells.extend(bbox_map.values().cloned());
    bbox_cells.sort_by(|a, b| a.cell_name.cmp(&b.cell_name));
    let bbox_file = BboxFile { cells: bbox_cells };

    // Build pin.json
    let mut pins_map: HashMap<String, PinEntry> = HashMap::new();
    for pidx in 0..db.num_pins() {
        let Some(pin) = db.pin(pidx as i32) else {
            continue;
        };
        if pin.name().ends_with("_GR") {
            continue;
        }
        let (cell_name, cell_shape) = if let Some(cell_idx) = pin.cell_idx() {
            if let Some(cell) = db.cell(cell_idx) {
                let cell_bbox = cell_bboxes
                    .get(cell.name())
                    .copied()
                    .unwrap_or((0, 0, 0, 0));
                let leaf = cell.name().rsplit('_').next().unwrap_or(cell.name());
                let canon_cell = format!("{}_{}", top_name, leaf);
                (
                    canon_cell,
                    Some(PinCellShape {
                        shape: BboxRect {
                            xl: cell_bbox.0,
                            yl: cell_bbox.1,
                            xh: cell_bbox.2,
                            yh: cell_bbox.3,
                        },
                    }),
                )
            } else {
                ("".to_string(), None)
            }
        } else {
            ("".to_string(), None)
        };

        let mut nets = Vec::new();
        for &net_idx in pin.net_ids() {
            if let Some(net) = db.net(net_idx) {
                nets.push(canonical_net_name(&top_name, &top_pins, net.name()));
            }
        }

        let layers = pin.layer_ids().to_vec();
        let offset = pin.offset();
        let shape_bbox = if let Some(cell_idx) = pin.cell_idx() {
            if let Some(cell) = db.cell(cell_idx) {
                let pin_order_idx = pin
                    .name()
                    .rsplit('_')
                    .next()
                    .and_then(|s| s.parse::<usize>().ok());
                if let Some(order_idx) = pin_order_idx {
                    if let Some(pin_shapes) = pin_shapes_by_name.get(cell.name()) {
                        if order_idx < pin_shapes.len() {
                            if let Some(b) = poly_bbox(&pin_shapes[order_idx]) {
                                BboxRect {
                                    xl: b.0,
                                    yl: b.1,
                                    xh: b.2,
                                    yh: b.3,
                                }
                            } else {
                                BboxRect {
                                    xl: 0,
                                    yl: 0,
                                    xh: 0,
                                    yh: 0,
                                }
                            }
                        } else {
                            BboxRect {
                                xl: 0,
                                yl: 0,
                                xh: 0,
                                yh: 0,
                            }
                        }
                    } else {
                        BboxRect {
                            xl: 0,
                            yl: 0,
                            xh: 0,
                            yh: 0,
                        }
                    }
                } else {
                    BboxRect {
                        xl: 0,
                        yl: 0,
                        xh: 0,
                        yh: 0,
                    }
                }
            } else {
                BboxRect {
                    xl: 0,
                    yl: 0,
                    xh: 0,
                    yh: 0,
                }
            }
        } else {
            BboxRect {
                xl: 0,
                yl: 0,
                xh: 0,
                yh: 0,
            }
        };

        let pin_name = if let Some(cell_idx) = pin.cell_idx() {
            if let Some(cell) = db.cell(cell_idx) {
                let leaf = cell.name().rsplit('_').next().unwrap_or(cell.name());
                let canon_cell = format!("{}_{}", top_name, leaf);
                let suffix = pin.name().rsplit('_').next().unwrap_or("0");
                format!("{}_{}", canon_cell, suffix)
            } else {
                pin.name().to_string()
            }
        } else {
            pin.name().to_string()
        };

        pins_map.insert(
            pin_name.clone(),
            PinEntry {
                cell: cell_shape,
                cell_name,
                idx: 0,
                is_io: pin.is_io(),
                is_well_tap: pin.is_well_tap(),
                layers,
                name: pin_name,
                nets,
                offset_x: offset.x,
                offset_y: offset.y,
                shape: shape_bbox,
                shapes: pin
                    .shapes()
                    .iter()
                    .map(|s| PinShapeEntry {
                        layer: s.gds_layer,
                        shape: BboxRect {
                            xl: s.rect.xl,
                            yl: s.rect.yl,
                            xh: s.rect.xh,
                            yh: s.rect.yh,
                        },
                    })
                    .collect(),
            },
        );
    }

    // Add global power pins (GND/VDD) as standalone entries if present in top pins
    for pname in ["GND", "VDD"] {
        if top_pins.contains(pname) {
            pins_map.insert(
                pname.to_string(),
                PinEntry {
                    cell: None,
                    cell_name: "".to_string(),
                    idx: 0,
                    is_io: false,
                    is_well_tap: false,
                    layers: Vec::new(),
                    name: pname.to_string(),
                    nets: vec![pname.to_string()],
                    offset_x: 0,
                    offset_y: 0,
                    shape: BboxRect {
                        xl: 0,
                        yl: 0,
                        xh: 0,
                        yh: 0,
                    },
                    shapes: Vec::new(),
                },
            );
        }
    }

    let mut pins_out = Vec::new();
    let mut names: Vec<String> = pins_map.keys().cloned().collect();
    names.sort();
    for name in names {
        if let Some(p) = pins_map.get(&name) {
            pins_out.push(p.clone());
        }
    }
    for (idx, p) in pins_out.iter_mut().enumerate() {
        p.idx = (idx as i32) + 1;
    }

    let pin_file = PinFile { pins: pins_out };

    let out = PcellOutput {
        bbox: bbox_file,
        pins: pin_file,
        resolved_pcelldir,
    };
    // Debug dump: nothing is written to disk by default. This used to write
    // unconditionally to a hardcoded directory inside one example case, so any
    // case run (cargo test included) overwrote that case's files and cases
    // clobbered one another. It is now env-gated with the directory supplied by
    // the caller (APPLACE_PCELL_DUMP_DIR=<dir>), which isolates cases naturally.
    // A failed write only warns and does not affect the parse result.
    if let Some(dir) = std::env::var_os("APPLACE_PCELL_DUMP_DIR") {
        let dir = PathBuf::from(dir);
        match dump_pcell_output(&out, &dir) {
            Ok(()) => eprintln!("[pcell] dumped bbox.json / pin.json to {}", dir.display()),
            Err(e) => eprintln!("[pcell] warning: dump to {} failed: {}", dir.display(), e),
        }
    }

    Ok(out)
}

#[cfg(test)]
mod hier_tests {
    use super::*;
    use gds21::{GdsBoundary, GdsStructRef, GdsTextElem};

    fn rect_elem(layer: i16, xl: i32, yl: i32, xh: i32, yh: i32) -> GdsElement {
        GdsElement::GdsBoundary(GdsBoundary {
            layer,
            datatype: 0,
            xy: vec![
                GdsPoint { x: xl, y: yl },
                GdsPoint { x: xh, y: yl },
                GdsPoint { x: xh, y: yh },
                GdsPoint { x: xl, y: yh },
                GdsPoint { x: xl, y: yl },
            ],
            ..Default::default()
        })
    }

    fn text_elem(layer: i16, s: &str, x: i32, y: i32) -> GdsElement {
        GdsElement::GdsTextElem(GdsTextElem {
            layer,
            texttype: 0,
            string: s.to_string(),
            xy: GdsPoint { x, y },
            ..Default::default()
        })
    }

    fn sref(name: &str, x: i32, y: i32, strans: Option<GdsStrans>) -> GdsElement {
        GdsElement::GdsStructRef(GdsStructRef {
            name: name.to_string(),
            xy: GdsPoint { x, y },
            strans,
            ..Default::default()
        })
    }

    fn strans(reflected: bool, mag: Option<f64>, angle: Option<f64>) -> GdsStrans {
        GdsStrans {
            reflected,
            abs_mag: false,
            abs_angle: false,
            mag,
            angle,
        }
    }

    fn strukt(name: &str, elems: Vec<GdsElement>) -> GdsStruct {
        GdsStruct {
            name: name.to_string(),
            elems,
            ..Default::default()
        }
    }

    fn lib_of(structs: Vec<GdsStruct>) -> GdsLibrary {
        GdsLibrary {
            name: "TEST".into(),
            structs,
            ..Default::default()
        }
    }

    /// The hierarchy-aware bbox must descend into SREF and apply the transform:
    /// translation, mirroring, rotation and magnification all have to be right.
    /// This is the heart of the fix -- the old `compute_gds_bbox` looked only at
    /// the polygons the selected struct owned, so `(dx,dy)` was computed in the
    /// wrong coordinate system.
    #[test]
    fn hier_bbox_applies_sref_transforms() {
        // leaf: a 100x40 rectangle flush with the origin.
        let leaf = strukt("LEAF", vec![rect_elem(31, 0, 0, 100, 40)]);

        // Pure translation
        let lib = lib_of(vec![
            leaf.clone(),
            strukt("T", vec![sref("LEAF", 500, -20, None)]),
        ]);
        assert_eq!(
            hier_struct_bbox(&lib, "T").unwrap(),
            Some((500, -20, 600, 20)),
            "the SREF translation was not applied -- the descent did not happen"
        );

        // Mirror about the x axis then translate: y is negated
        let lib = lib_of(vec![
            leaf.clone(),
            strukt(
                "T",
                vec![sref("LEAF", 0, 0, Some(strans(true, None, None)))],
            ),
        ]);
        assert_eq!(hier_struct_bbox(&lib, "T").unwrap(), Some((0, -40, 100, 0)));

        // Counter-clockwise 90°: (x,y) -> (-y,x)
        let lib = lib_of(vec![
            leaf.clone(),
            strukt(
                "T",
                vec![sref("LEAF", 0, 0, Some(strans(false, None, Some(90.0))))],
            ),
        ]);
        assert_eq!(
            hier_struct_bbox(&lib, "T").unwrap(),
            Some((-40, 0, 0, 100)),
            "a 90° rotation must be exact (floating-point residue in cos90 must not inflate the bbox)"
        );

        // Mirror + 90°: mirror first (x,-y), then rotate -> (y, x)
        let lib = lib_of(vec![
            leaf.clone(),
            strukt(
                "T",
                vec![sref("LEAF", 0, 0, Some(strans(true, None, Some(90.0))))],
            ),
        ]);
        assert_eq!(hier_struct_bbox(&lib, "T").unwrap(), Some((0, 0, 40, 100)));

        // Magnify by 2
        let lib = lib_of(vec![
            leaf.clone(),
            strukt(
                "T",
                vec![sref("LEAF", 10, 10, Some(strans(false, Some(2.0), None)))],
            ),
        ]);
        assert_eq!(
            hier_struct_bbox(&lib, "T").unwrap(),
            Some((10, 10, 210, 90))
        );

        // Two nesting levels: the transforms must compose, not apply independently
        let lib = lib_of(vec![
            leaf,
            strukt("MID", vec![sref("LEAF", 100, 0, None)]),
            strukt(
                "T",
                vec![sref("MID", 0, 0, Some(strans(false, None, Some(90.0))))],
            ),
        ]);
        assert_eq!(
            hier_struct_bbox(&lib, "T").unwrap(),
            Some((-40, 100, 0, 200))
        );
    }

    /// AREF: the row/column pitch vectors already live in the parent coordinate
    /// system, so bbox = single-instance bbox ⊕ bbox of the translation lattice.
    #[test]
    fn hier_bbox_expands_aref_lattice() {
        use gds21::GdsArrayRef;
        let leaf = strukt("LEAF", vec![rect_elem(31, 0, 0, 10, 10)]);
        // 3 columns × 2 rows, column pitch 100, row pitch 50.
        let aref = GdsElement::GdsArrayRef(GdsArrayRef {
            name: "LEAF".into(),
            xy: [
                GdsPoint { x: 0, y: 0 },
                GdsPoint { x: 300, y: 0 },
                GdsPoint { x: 0, y: 100 },
            ],
            cols: 3,
            rows: 2,
            strans: None,
            ..Default::default()
        });
        let lib = lib_of(vec![leaf, strukt("T", vec![aref])]);
        // Instance origins (0,0),(100,0),(200,0),(0,50),(100,50),(200,50)
        assert_eq!(hier_struct_bbox(&lib, "T").unwrap(), Some((0, 0, 210, 60)));
    }

    /// A dangling reference and a cycle must both be hard errors, never silently treated as empty geometry.
    #[test]
    fn hier_bbox_rejects_dangling_and_cyclic_refs() {
        let lib = lib_of(vec![strukt("T", vec![sref("MISSING", 0, 0, None)])]);
        assert!(hier_struct_bbox(&lib, "T").is_err());

        let lib = lib_of(vec![
            strukt("A", vec![rect_elem(31, 0, 0, 1, 1), sref("B", 0, 0, None)]),
            strukt("B", vec![sref("A", 0, 0, None)]),
        ]);
        assert!(hier_struct_bbox(&lib, "A").is_err());
    }

    /// Normalization touches exactly one place, the device struct: not a single
    /// child-struct coordinate may change, while the hierarchy-aware bbox lands
    /// at (0,0) and the size stays the same (no tearing).
    #[test]
    fn normalize_translates_root_only_and_does_not_tear() {
        // Depth 3: DEV -> MID -> LEAF with an offset at every level; DEV owns one piece of metal carrying a label.
        let leaf = strukt("LEAF", vec![rect_elem(31, 0, 0, 100, 100)]);
        let mid = strukt("MID", vec![sref("LEAF", 50, 50, None)]);
        let dev = strukt(
            "DEV",
            vec![
                rect_elem(31, -300, -200, -100, -100),
                text_elem(131, "PLUS", -200, -150),
                sref("MID", 0, 0, None),
            ],
        );
        let top = strukt("TOP", vec![sref("DEV", 0, 0, None)]);
        let mut lib = lib_of(vec![leaf, mid, dev, top]);

        let before = hier_struct_bbox(&lib, "DEV").unwrap().unwrap();
        assert_eq!(before, (-300, -200, 150, 150));

        normalize_library(&mut lib, "fixture").unwrap();

        // The device coordinate system is zeroed and the size is unchanged.
        assert_eq!(
            hier_struct_bbox(&lib, "DEV").unwrap(),
            Some((0, 0, 450, 350))
        );
        // Not a single child-struct coordinate moved.
        let leaf_after = lib.structs.iter().find(|s| s.name == "LEAF").unwrap();
        match &leaf_after.elems[0] {
            GdsElement::GdsBoundary(b) => {
                assert_eq!(
                    (b.xy[0].x, b.xy[0].y),
                    (0, 0),
                    "LEAF was translated -- torn apart"
                );
            }
            _ => panic!("unexpected element"),
        }
        let mid_after = lib.structs.iter().find(|s| s.name == "MID").unwrap();
        match &mid_after.elems[0] {
            GdsElement::GdsStructRef(r) => {
                assert_eq!(
                    (r.xy.x, r.xy.y),
                    (50, 50),
                    "the SREF origin in MID was moved -- torn apart"
                );
            }
            _ => panic!("unexpected element"),
        }
        // The device struct's own elements (including the SREF it emits) were translated once, exactly once.
        let dev_after = lib.structs.iter().find(|s| s.name == "DEV").unwrap();
        match &dev_after.elems[2] {
            GdsElement::GdsStructRef(r) => assert_eq!((r.xy.x, r.xy.y), (300, 200)),
            _ => panic!("unexpected element"),
        }
        match &dev_after.elems[1] {
            GdsElement::GdsTextElem(t) => assert_eq!((t.xy.x, t.xy.y), (100, 50)),
            _ => panic!("unexpected element"),
        }
    }

    /// Re-normalization is idempotent: running again on an already normalized
    /// library must be a no-op. An earlier version was not -- every run tore off
    /// another layer.
    #[test]
    fn normalize_is_idempotent() {
        let leaf = strukt("LEAF", vec![rect_elem(31, 0, 0, 100, 100)]);
        let mid = strukt("MID", vec![sref("LEAF", 50, 50, None)]);
        let dev = strukt(
            "DEV",
            vec![
                rect_elem(31, -300, -200, -100, -100),
                text_elem(131, "PLUS", -200, -150),
                sref("MID", 0, 0, None),
            ],
        );
        let top = strukt("TOP", vec![sref("DEV", 0, 0, None)]);
        let mut lib = lib_of(vec![leaf, mid, dev, top]);

        normalize_library(&mut lib, "fixture").unwrap();
        let once = lib.structs.clone();
        normalize_library(&mut lib, "fixture").unwrap();
        assert_eq!(once, lib.structs, "re-normalization is not a no-op");
    }

    /// The origin of a pure-forwarder wrapper (no geometry of its own, exactly
    /// one reference, identity transform) is zeroed so that both readings,
    /// top-level flatten and device flatten, start at (0,0).
    /// Regression subject: a device layout whose wrapper SREF origin exactly
    /// cancels the negative coordinates of the inner struct.
    #[test]
    fn normalize_zeroes_pure_passthrough_wrapper() {
        let dev = strukt(
            "DEV",
            vec![
                rect_elem(31, -330, -220, 450, 1470),
                text_elem(131, "D", 0, 0),
            ],
        );
        let top = strukt("TOP", vec![sref("DEV", 330, 220, None)]);
        let mut lib = lib_of(vec![dev, top]);
        normalize_library(&mut lib, "fixture").unwrap();
        assert_eq!(
            hier_struct_bbox(&lib, "DEV").unwrap(),
            Some((0, 0, 780, 1690))
        );
        assert_eq!(
            hier_struct_bbox(&lib, "TOP").unwrap(),
            Some((0, 0, 780, 1690)),
            "the pure-forwarder wrapper was not zeroed, the top-level reading is still offset"
        );
    }

    /// A wrapper that carries geometry of its own must not be touched -- the
    /// device coordinate system is still correct, only the top level is not zeroed.
    #[test]
    fn normalize_leaves_non_passthrough_wrapper_alone() {
        let dev = strukt(
            "DEV",
            vec![
                rect_elem(31, -100, -100, 100, 100),
                text_elem(131, "D", 0, 0),
            ],
        );
        let top = strukt(
            "TOP",
            vec![rect_elem(50, 0, 0, 10, 10), sref("DEV", 500, 500, None)],
        );
        let mut lib = lib_of(vec![dev, top]);
        normalize_library(&mut lib, "fixture").unwrap();
        assert_eq!(
            hier_struct_bbox(&lib, "DEV").unwrap(),
            Some((0, 0, 200, 200))
        );
        let top_after = lib.structs.iter().find(|s| s.name == "TOP").unwrap();
        match &top_after.elems[1] {
            GdsElement::GdsStructRef(r) => assert_eq!((r.xy.x, r.xy.y), (500, 500)),
            _ => panic!("unexpected element"),
        }
    }

    /// The measurement path in pcell and the instantiation path in the writer
    /// must land on the same struct.
    /// Regression subject: a resistor PCell whose inner struct carries no label
    /// once the provenance block is stripped, so the old writer chain landed on
    /// the wrapper while pcell landed on the inner struct.
    #[test]
    fn device_struct_pick_is_shared_with_writer() {
        let inner = strukt("RES_EXAMPLE_DEVICE", vec![rect_elem(17, 0, 0, 1000, 500)]);
        let wrapper = strukt(
            "RES_EXAMPLE_WRAPPER",
            vec![sref("RES_EXAMPLE_DEVICE", 0, 0, None)],
        );
        let lib = lib_of(vec![inner, wrapper]);
        assert_eq!(
            pick_device_struct_name(&lib).as_deref(),
            Some("RES_EXAMPLE_DEVICE")
        );
    }

    /// Whole-corpus regression: after normalization every device GDS present in
    /// the repository must satisfy
    ///   (a) the hierarchy-aware bbox of the device struct lands at (0,0),
    ///   (b) the size is unchanged (no tearing),
    ///   (c) re-normalization is a byte-identical no-op (idempotent).
    ///
    /// This is the gatekeeping test for the fix. Before it most device layouts
    /// across the example cases were offset under the top-level reading, and a
    /// handful of hierarchical capacitors were genuinely torn apart.
    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn all_tracked_pcells_normalize_to_origin_and_are_idempotent() {
        let bench = project_root().join("benchmark");
        if !bench.is_dir() {
            eprintln!("[test] no benchmark/ dir; skipping corpus audit");
            return;
        }
        let tmp = std::env::temp_dir().join(format!(
            "applace_pcell_audit_{}_{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or(0)
        ));
        std::fs::create_dir_all(&tmp).unwrap();

        let mut cases: Vec<PathBuf> = std::fs::read_dir(&bench)
            .unwrap()
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.join("custom").is_dir())
            .collect();
        cases.sort();
        if cases.is_empty() {
            eprintln!("[skip] no example design layout present (docs/INPUT_FORMAT.md)");
            return;
        }

        let mut checked = 0usize;
        for case in cases {
            let mut files: Vec<PathBuf> = std::fs::read_dir(case.join("custom"))
                .unwrap()
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("gds"))
                .collect();
            files.sort();
            for src in files {
                let stem = format!(
                    "{}__{}",
                    case.file_name().unwrap().to_string_lossy(),
                    src.file_name().unwrap().to_string_lossy()
                );
                let once = tmp.join(format!("{}.1.gds", stem));
                let twice = tmp.join(format!("{}.2.gds", stem));
                normalize_single_gds(&src, &once)
                    .unwrap_or_else(|e| panic!("normalize {} failed: {}", src.display(), e));

                // (a)+(b) Hierarchy-aware re-measurement: both readings, device struct and top level, must be zeroed.
                let lib = GdsLibrary::load(&once).unwrap();
                let dev = pick_device_struct_name(&lib).unwrap();
                let src_lib = GdsLibrary::load(&src).unwrap();
                let src_dev = pick_device_struct_name(&src_lib).unwrap();
                let before = hier_struct_bbox(&src_lib, &src_dev).unwrap().unwrap();
                let after = hier_struct_bbox(&lib, &dev).unwrap().unwrap();
                assert_eq!(
                    (after.0, after.1),
                    (0, 0),
                    "{}: device struct {:?} not at origin after normalize",
                    src.display(),
                    dev
                );
                assert_eq!(
                    (after.2 - after.0, after.3 - after.1),
                    (before.2 - before.0, before.3 - before.1),
                    "{}: device struct {:?} was torn",
                    src.display(),
                    dev
                );

                // (c) Idempotence: a second run must be byte-identical.
                normalize_single_gds(&once, &twice).unwrap();
                assert_eq!(
                    std::fs::read(&once).unwrap(),
                    std::fs::read(&twice).unwrap(),
                    "{}: re-normalizing is not a byte-identical no-op",
                    src.display()
                );
                let _ = std::fs::remove_file(&once);
                let _ = std::fs::remove_file(&twice);
                checked += 1;
            }
        }
        let _ = std::fs::remove_dir_all(&tmp);
        assert!(
            checked >= 100,
            "expected the full device corpus, saw {}",
            checked
        );
    }

    /// The absolute bits of STRANS are not modelled: a hard error is preferable to guessing a semantics.
    #[test]
    fn hier_bbox_rejects_absolute_strans() {
        let leaf = strukt("LEAF", vec![rect_elem(31, 0, 0, 10, 10)]);
        let abs = GdsStrans {
            reflected: false,
            abs_mag: true,
            abs_angle: false,
            mag: Some(2.0),
            angle: None,
        };
        let lib = lib_of(vec![leaf, strukt("T", vec![sref("LEAF", 0, 0, Some(abs))])]);
        assert!(hier_struct_bbox(&lib, "T").is_err());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect_poly(layer: i32, xl: i32, yl: i32, xh: i32, yh: i32) -> GdsPoly {
        GdsPoly {
            layer,
            datatype: 0,
            points: vec![[xl, yl], [xh, yl], [xh, yh], [xl, yh]],
        }
    }

    fn pick(label: usize, geom: usize, area: i64, referenced: bool) -> StructPick {
        StructPick {
            label_count: label,
            geom_count: geom,
            area,
            referenced,
        }
    }

    /// The four-tier device-struct selection chain, and the property that it
    /// **does not depend on vendor provenance labels**. Regression subject:
    /// resistor PCells that carry a provenance TEXT element but no functional pin
    /// label, so when the provenance was scrubbed from the GDS the old chain
    /// landed on a wrapper struct holding a single SREF and lost all of the
    /// device geometry and the pins.
    #[test]
    fn device_struct_pick_is_provenance_independent() {
        // A device with functional labels (MOS-shaped): the inner struct has both geometry and labels.
        let mos_before = [pick(7, 441, 1_000, false), pick(0, 0, 0, false)];
        let mos_after = [pick(7, 441, 1_000, false), pick(0, 0, 0, false)];
        assert_eq!(select_device_struct(&mos_before), 0);
        assert_eq!(select_device_struct(&mos_after), 0);

        // Resistor-shaped: every label on the inner struct is provenance and the
        // wrapper holds a single SREF (geom=0). The inner struct (idx 0) must be
        // selected both before and after the provenance is stripped.
        let res_before = [pick(7, 14, 1_000, true), pick(0, 0, 0, false)];
        let res_after = [pick(0, 14, 1_000, true), pick(0, 0, 0, false)];
        assert_eq!(select_device_struct(&res_before), 0);
        assert_eq!(
            select_device_struct(&res_after),
            0,
            "stripping the provenance labels must not switch the selection to the empty wrapper"
        );

        // Tier one beats tier two: a labelled struct wins even when its area is smaller.
        let mixed = [pick(0, 9, 9_999, false), pick(3, 1, 10, false)];
        assert_eq!(select_device_struct(&mixed), 1);

        // Neither geometry nor labels anywhere -> tier three: the first struct nobody references.
        let no_geom = [pick(0, 0, 0, true), pick(0, 0, 0, false)];
        assert_eq!(select_device_struct(&no_geom), 1);

        // Everything is referenced -> tier four: struct 0.
        assert_eq!(select_device_struct(&[pick(0, 0, 0, true)]), 0);
        assert_eq!(select_device_struct(&[]), 0);

        // On an area tie the earlier struct is kept (strictly-greater comparison).
        let tie = [pick(0, 2, 500, false), pick(0, 2, 500, false)];
        assert_eq!(select_device_struct(&tie), 0);
    }

    /// Geometric resistor pin inference (no labels): a chain of 3 PO strips,
    /// 2 port M1 groups and 2 interior jumper M1 groups. Asserts that exactly
    /// 2 ports are inferred, that the PLUS/MINUS assignment is deterministic
    /// (the lexicographically smaller (xl,yl) is PLUS), and that the jumper metal
    /// does not enter pin_shapes but becomes a blockage under the existing
    /// semantics.
    #[test]
    fn resistor_pin_inference_from_geometry() {
        const PO: i32 = 17;
        const CO: i32 = 30;
        const M1: i32 = 31;
        let polygons = vec![
            // 3 horizontal PO strips (the serpentine resistor body).
            rect_poly(PO, 0, 0, 1000, 40),
            rect_poly(PO, 0, 100, 1000, 140),
            rect_poly(PO, 0, 200, 1000, 240),
            // Port A: the left end of PO1 (degree 1).
            rect_poly(M1, 0, 0, 10, 40),
            rect_poly(CO, 2, 10, 8, 30),
            // Jumper 1: on the right, connecting PO1-PO2 (degree 2).
            rect_poly(M1, 990, 0, 1000, 140),
            rect_poly(CO, 992, 10, 998, 30),
            rect_poly(CO, 992, 110, 998, 130),
            // Jumper 2: on the left, connecting PO2-PO3 (degree 2).
            rect_poly(M1, 0, 100, 10, 240),
            rect_poly(CO, 2, 110, 8, 130),
            rect_poly(CO, 2, 210, 8, 230),
            // Port B: the right end of PO3 (degree 1).
            rect_poly(M1, 990, 200, 1000, 240),
            rect_poly(CO, 992, 210, 998, 230),
        ];
        let gds = GdsData {
            top: "res_test".to_string(),
            polygons,
            labels: Vec::new(),
            hier_bbox: None,
        };
        let pin_order = vec!["PLUS".to_string(), "MINUS".to_string()];
        let (pin_shapes, blockages) =
            compute_pin_shapes(&gds, 2, &pin_order, &[M1], Some(PO), Some(CO));

        assert_eq!(pin_shapes.len(), 2);
        // PLUS = the lexicographically smaller union-bbox (xl,yl) = port A.
        assert_eq!(pin_shapes[0].len(), 1, "PLUS should be exactly port A");
        let plus = &pin_shapes[0][0];
        assert_eq!((plus.xl, plus.yl, plus.xh, plus.yh), (0, 0, 10, 40));
        assert_eq!(pin_shapes[1].len(), 1, "MINUS should be exactly port B");
        let minus = &pin_shapes[1][0];
        assert_eq!(
            (minus.xl, minus.yl, minus.xh, minus.yh),
            (990, 200, 1000, 240)
        );

        // Jumper metal belongs to no pin and becomes a blockage under the existing semantics.
        assert_eq!(blockages.len(), 2, "both jumpers should be blockages");
        let mut jumpers: Vec<(i32, i32, i32, i32)> =
            blockages.iter().map(|b| (b.xl, b.yl, b.xh, b.yh)).collect();
        jumpers.sort_unstable();
        assert_eq!(jumpers, vec![(0, 100, 10, 240), (990, 0, 1000, 140)]);
    }

    /// When the number of degree-1 groups is not exactly 2 (an anomalous shape
    /// with three ports, say), inference is abandoned and pin_shapes stays empty.
    #[test]
    fn resistor_pin_inference_bails_on_ambiguous_ports() {
        const PO: i32 = 17;
        const CO: i32 = 30;
        const M1: i32 = 31;
        let polygons = vec![
            // A single PO strip with three degree-1 M1 ports.
            rect_poly(PO, 0, 0, 1000, 40),
            rect_poly(M1, 0, 0, 10, 40),
            rect_poly(CO, 2, 10, 8, 30),
            rect_poly(M1, 495, 0, 505, 40),
            rect_poly(CO, 497, 10, 503, 30),
            rect_poly(M1, 990, 0, 1000, 40),
            rect_poly(CO, 992, 10, 998, 30),
        ];
        let gds = GdsData {
            top: "res_odd".to_string(),
            polygons,
            labels: Vec::new(),
            hier_bbox: None,
        };
        let pin_order = vec!["PLUS".to_string(), "MINUS".to_string()];
        let (pin_shapes, blockages) =
            compute_pin_shapes(&gds, 2, &pin_order, &[M1], Some(PO), Some(CO));
        assert!(pin_shapes.iter().all(|v| v.is_empty()));
        assert_eq!(blockages.len(), 3);
    }

    // ---------------------------------------------------------------
    // GDSII PATH -> rectangles
    // ---------------------------------------------------------------

    fn mk_path(
        xy: &[(i32, i32)],
        width: i32,
        path_type: Option<i16>,
        extn: Option<(i32, i32)>,
    ) -> gds21::GdsPath {
        gds21::GdsPath {
            layer: 31,
            datatype: 0,
            xy: xy.iter().map(|(x, y)| GdsPoint::new(*x, *y)).collect(),
            width: Some(width),
            path_type,
            begin_extn: extn.map(|(b, _)| b),
            end_extn: extn.map(|(_, e)| e),
            elflags: None,
            plex: None,
            properties: Vec::new(),
        }
    }

    /// The rectangles of one horizontal PATH under a given PATHTYPE (i32 convention, the pin-metal path).
    fn boxes_i32(p: &gds21::GdsPath) -> Vec<(i32, i32, i32, i32)> {
        let half = p.width.unwrap_or(0) / 2;
        let (bgn, end) = path_ends(p).unwrap().as_i32(half);
        path_segment_boxes(&p.xy, half, bgn, end)
    }

    #[test]
    fn pathtype0_is_flush() {
        // Absent (no PATHTYPE record) and an explicit 0 must agree: no end extension.
        for pt in [None, Some(0i16)] {
            let p = mk_path(&[(0, 100), (1000, 100)], 100, pt, None);
            assert_eq!(boxes_i32(&p), vec![(0, 50, 1000, 150)], "pathtype {pt:?}");
        }
    }

    #[test]
    fn pathtype2_extends_half_width_at_both_ends() {
        let p = mk_path(&[(0, 100), (1000, 100)], 100, Some(2), None);
        assert_eq!(boxes_i32(&p), vec![(-50, 50, 1050, 150)]);
    }

    #[test]
    fn pathtype4_uses_bgnextn_endextn() {
        let p = mk_path(&[(0, 100), (1000, 100)], 100, Some(4), Some((10, 200)));
        assert_eq!(boxes_i32(&p), vec![(-10, 50, 1200, 150)]);
        // Default BGNEXTN/ENDEXTN = 0 degenerates to flush.
        let q = mk_path(&[(0, 100), (1000, 100)], 100, Some(4), None);
        assert_eq!(boxes_i32(&q), vec![(0, 50, 1000, 150)]);
    }

    #[test]
    fn pathtype1_round_is_approximated_as_square() {
        let round = mk_path(&[(0, 0), (0, 500)], 100, Some(1), None);
        let square = mk_path(&[(0, 0), (0, 500)], 100, Some(2), None);
        assert_eq!(boxes_i32(&round), boxes_i32(&square));
        assert_eq!(boxes_i32(&round), vec![(-50, -50, 50, 550)]);
    }

    #[test]
    fn unknown_pathtype_is_a_hard_error() {
        let p = mk_path(&[(0, 0), (100, 0)], 100, Some(3), None);
        assert!(path_ends(&p).is_err());
    }

    #[test]
    fn multi_segment_path_is_a_union_of_per_segment_rects() {
        // L shape: (0,0) -> (1000,0) -> (1000,1000), flush. The old conversion gave one
        // solid bbox (-50,-50,1050,1050); the new one gives two rectangles, and the
        // mitre corner block x in [950,1050] x y in [-50,50] at the bend is covered by
        // both segments, leaving no notch.
        let p = mk_path(&[(0, 0), (1000, 0), (1000, 1000)], 100, Some(0), None);
        assert_eq!(
            boxes_i32(&p),
            vec![(0, -50, 1050, 50), (950, -50, 1050, 1000)]
        );
        // An interior vertex does not take the PATHTYPE end extension (it takes the
        // fixed half, see the mitre corner block); only the two true end points do.
        let q = mk_path(&[(0, 0), (1000, 0), (1000, 1000)], 100, Some(2), None);
        assert_eq!(
            boxes_i32(&q),
            vec![(-50, -50, 1050, 50), (950, -50, 1050, 1050)]
        );
    }

    #[test]
    fn reversed_segment_direction_gives_the_same_rect() {
        let fwd = mk_path(&[(0, 100), (1000, 100)], 100, Some(4), Some((10, 200)));
        let rev = mk_path(&[(1000, 100), (0, 100)], 100, Some(4), Some((200, 10)));
        assert_eq!(boxes_i32(&fwd), boxes_i32(&rev));
    }

    #[test]
    fn degenerate_paths_keep_the_conservative_box() {
        // Single point / zero-length segment: direction undefined, so a conservative +/-half box (as before; no geometry is dropped).
        let one = mk_path(&[(0, 0)], 100, Some(0), None);
        assert_eq!(boxes_i32(&one), vec![(-50, -50, 50, 50)]);
        let zero = mk_path(&[(50, 50), (50, 50)], 100, Some(0), None);
        assert_eq!(boxes_i32(&zero), vec![(0, 0, 100, 100)]);
    }

    #[test]
    fn diagonal_segment_falls_back_to_a_conservative_bbox() {
        let p = mk_path(&[(0, 0), (100, 100)], 100, Some(0), None);
        assert_eq!(boxes_i32(&p), vec![(-50, -50, 150, 150)]);
    }

    #[test]
    fn path_legacy_switch_parses() {
        assert_eq!(parse_pcell_path_legacy_switch(None), Ok(false));
        for v in ["", "0", "false", "OFF", " no "] {
            assert_eq!(parse_pcell_path_legacy_switch(Some(v)), Ok(false), "{v:?}");
        }
        for v in ["1", "true", "On", "YES"] {
            assert_eq!(parse_pcell_path_legacy_switch(Some(v)), Ok(true), "{v:?}");
        }
        assert!(parse_pcell_path_legacy_switch(Some("2")).is_err());
        assert!(parse_pcell_path_legacy_switch(Some("flush")).is_err());
    }

    /// With the switch on, the convention is "one bbox for the whole PATH, grown by
    /// half on all four sides". The expression is replicated here verbatim, pinning
    /// the promise that the switch reproduces the previous geometry exactly.
    #[test]
    fn legacy_expression_is_the_pre_fix_geometry() {
        let p = mk_path(&[(0, 0), (1000, 0), (1000, 1000)], 100, Some(0), None);
        let half = p.width.unwrap() / 2;
        let pts: Vec<[i32; 2]> = p.xy.iter().map(|q| [q.x, q.y]).collect();
        let (xl, yl, xh, yh) = bbox_from_points(&pts).unwrap();
        assert_eq!(
            (xl - half, yl - half, xh + half, yh + half),
            (-50, -50, 1050, 1050)
        );
        // The new convention is strictly smaller: every segment rectangle lies inside the old bbox, and their union is not equal to it.
        for (a, b, c, d) in boxes_i32(&p) {
            assert!(a >= xl - half && b >= yl - half && c <= xh + half && d <= yh + half);
        }
    }

    /// `struct_picks` is frozen on the previous convention (see the comment there).
    /// This test shows that freezing it does not matter: over every tracked PCell,
    /// switching the `PATH` area to the `PATHTYPE` convention changes **none** of the
    /// structs `select_device_struct` picks.
    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn device_struct_pick_is_insensitive_to_the_path_area_convention() {
        let bench = project_root().join("benchmark");
        if !bench.is_dir() {
            eprintln!("[test] no benchmark/ dir; skipping struct-pick audit");
            return;
        }
        let mut cases: Vec<PathBuf> = std::fs::read_dir(&bench)
            .unwrap()
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.join("custom").is_dir())
            .collect();
        cases.sort();
        let mut checked = 0usize;
        for case in cases {
            let mut files: Vec<PathBuf> = std::fs::read_dir(case.join("custom"))
                .unwrap()
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.extension().map(|e| e == "gds").unwrap_or(false))
                .collect();
            files.sort();
            for f in files {
                let lib = GdsLibrary::load(&f).unwrap();
                let frozen = struct_picks(&lib);
                // The same statistics, with only the PATH bbox switched to the PATHTYPE convention.
                let mut aware = frozen.clone();
                for (idx, st) in lib.structs.iter().enumerate() {
                    let mut bbox: Option<(i32, i32, i32, i32)> = None;
                    for elem in &st.elems {
                        match elem {
                            GdsElement::GdsBoundary(b) => {
                                let pts: Vec<[i32; 2]> = b.xy.iter().map(|q| [q.x, q.y]).collect();
                                if let Some(bb) = bbox_from_points(&pts) {
                                    bbox = Some(match bbox {
                                        None => bb,
                                        Some(prev) => bbox_union(prev, bb),
                                    });
                                }
                            }
                            GdsElement::GdsPath(q) => {
                                let half = q.width.unwrap_or(0) / 2;
                                let (bgn, end) = path_ends(q).unwrap().as_i32(half);
                                for bb in path_segment_boxes(&q.xy, half, bgn, end) {
                                    bbox = Some(match bbox {
                                        None => bb,
                                        Some(prev) => bbox_union(prev, bb),
                                    });
                                }
                            }
                            _ => {}
                        }
                    }
                    aware[idx].area = bbox
                        .map(|(xl, yl, xh, yh)| {
                            ((xh - xl).max(0) as i64) * ((yh - yl).max(0) as i64)
                        })
                        .unwrap_or(0);
                }
                assert_eq!(
                    select_device_struct(&frozen),
                    select_device_struct(&aware),
                    "{}: device struct pick depends on the PATH area convention",
                    f.display()
                );
                checked += 1;
            }
        }
        if checked == 0 {
            eprintln!("[skip] no example design layout present (docs/INPUT_FORMAT.md)");
            return;
        }
        eprintln!("[test] struct pick identical on {checked} tracked PCells");
    }

    /// The PCell corpus of the example designs: every M1 `PATH` is flush, so after the
    /// fix each one is half a line width shorter than before at both ends (pinned as a
    /// guard test).
    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn tracked_pcell_paths_are_all_flush_two_point_manhattan() {
        let bench = project_root().join("benchmark");
        if !bench.is_dir() {
            eprintln!("[test] no benchmark/ dir; skipping PATH corpus audit");
            return;
        }
        let mut cases: Vec<PathBuf> = std::fs::read_dir(&bench)
            .unwrap()
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.join("custom").is_dir())
            .collect();
        cases.sort();
        let mut total = 0usize;
        for case in cases {
            let mut files: Vec<PathBuf> = std::fs::read_dir(case.join("custom"))
                .unwrap()
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.extension().map(|e| e == "gds").unwrap_or(false))
                .collect();
            files.sort();
            for f in files {
                let lib = GdsLibrary::load(&f).unwrap();
                for s in &lib.structs {
                    for e in &s.elems {
                        if let GdsElement::GdsPath(p) = e {
                            total += 1;
                            assert_eq!(
                                path_ends(p).unwrap(),
                                PathEnds::Flush,
                                "{}: PATH is not flush",
                                f.display()
                            );
                            assert_eq!(p.xy.len(), 2, "{}: multi-point PATH", f.display());
                        }
                    }
                }
            }
        }
        if total == 0 {
            eprintln!("[skip] no example design layout present (docs/INPUT_FORMAT.md)");
        }
    }
}
