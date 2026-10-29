//! Geometry type definitions

use crate::types::{Index, Loc, Real};

/// 2D rectangle
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Box<T> {
    pub xl: T,
    pub yl: T,
    pub xh: T,
    pub yh: T,
}

impl<T> Box<T> {
    pub fn new(xl: T, yl: T, xh: T, yh: T) -> Self {
        Self { xl, yl, xh, yh }
    }
}

impl Box<Loc> {
    pub fn width(&self) -> Loc {
        self.xh - self.xl
    }

    pub fn height(&self) -> Loc {
        self.yh - self.yl
    }

    pub fn area(&self) -> i64 {
        self.width() as i64 * self.height() as i64
    }
}

impl Box<Real> {
    pub fn width(&self) -> Real {
        self.xh - self.xl
    }

    pub fn height(&self) -> Real {
        self.yh - self.yl
    }

    pub fn area(&self) -> Real {
        self.width() * self.height()
    }
}

/// 3D point
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Point3d<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T> Point3d<T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}

/// 2D point
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point<T> {
    pub x: T,
    pub y: T,
}

impl<T> Point<T> {
    pub fn new(x: T, y: T) -> Self {
        Self { x, y }
    }
}

/// Polygon
#[derive(Debug, Clone, PartialEq)]
pub struct Polygon<T> {
    points: Vec<Point<T>>,
    layer: Index,
    datatype: Index,
}

impl<T> Polygon<T> {
    pub fn new(points: Vec<Point<T>>, layer: Index, datatype: Index) -> Self {
        Self {
            points,
            layer,
            datatype,
        }
    }

    pub fn points(&self) -> &[Point<T>] {
        &self.points
    }

    pub fn layer(&self) -> Index {
        self.layer
    }

    pub fn datatype(&self) -> Index {
        self.datatype
    }
}

impl Polygon<Real> {
    /// Computes the bounding box
    pub fn bounding_box(&self) -> Box<Real> {
        if self.points.is_empty() {
            return Box::new(0.0, 0.0, 0.0, 0.0);
        }

        let mut xl = self.points[0].x;
        let mut yl = self.points[0].y;
        let mut xh = self.points[0].x;
        let mut yh = self.points[0].y;

        for point in &self.points {
            if point.x < xl {
                xl = point.x;
            }
            if point.x > xh {
                xh = point.x;
            }
            if point.y < yl {
                yl = point.y;
            }
            if point.y > yh {
                yh = point.y;
            }
        }

        Box::new(xl, yl, xh, yh)
    }
}

impl Polygon<Loc> {
    /// Computes the bounding box
    pub fn bounding_box(&self) -> Box<Loc> {
        if self.points.is_empty() {
            return Box::new(0, 0, 0, 0);
        }

        let mut xl = self.points[0].x;
        let mut yl = self.points[0].y;
        let mut xh = self.points[0].x;
        let mut yh = self.points[0].y;

        for point in &self.points {
            if point.x < xl {
                xl = point.x;
            }
            if point.x > xh {
                xh = point.x;
            }
            if point.y < yl {
                yl = point.y;
            }
            if point.y > yh {
                yh = point.y;
            }
        }

        Box::new(xl, yl, xh, yh)
    }
}

/// Shape
#[derive(Debug, Clone)]
pub struct Shape<T> {
    polygons: Vec<Polygon<T>>,
    layer_ids: std::collections::HashSet<Index>,
}

impl<T> Shape<T> {
    pub fn new() -> Self {
        Self {
            polygons: Vec::new(),
            layer_ids: std::collections::HashSet::new(),
        }
    }

    pub fn polygons(&self) -> &[Polygon<T>] {
        &self.polygons
    }

    pub fn add_polygon(&mut self, polygon: Polygon<T>) {
        self.layer_ids.insert(polygon.layer());
        self.polygons.push(polygon);
    }

    pub fn num_polys(&self) -> usize {
        self.polygons.len()
    }

    pub fn layer_ids(&self) -> &std::collections::HashSet<Index> {
        &self.layer_ids
    }
}

impl<T> Default for Shape<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl Shape<Real> {
    /// Computes the bounding box
    pub fn bounding_box(&self) -> Box<Real> {
        if self.polygons.is_empty() {
            return Box::new(0.0, 0.0, 0.0, 0.0);
        }

        let first_bbox = self.polygons[0].bounding_box();
        let mut xl = first_bbox.xl;
        let mut yl = first_bbox.yl;
        let mut xh = first_bbox.xh;
        let mut yh = first_bbox.yh;

        for polygon in &self.polygons {
            let bbox = polygon.bounding_box();
            if bbox.xl < xl {
                xl = bbox.xl;
            }
            if bbox.xh > xh {
                xh = bbox.xh;
            }
            if bbox.yl < yl {
                yl = bbox.yl;
            }
            if bbox.yh > yh {
                yh = bbox.yh;
            }
        }

        Box::new(xl, yl, xh, yh)
    }
}

impl Shape<Loc> {
    /// Computes the bounding box
    pub fn bounding_box(&self) -> Box<Loc> {
        if self.polygons.is_empty() {
            return Box::new(0, 0, 0, 0);
        }

        let first_bbox = self.polygons[0].bounding_box();
        let mut xl = first_bbox.xl;
        let mut yl = first_bbox.yl;
        let mut xh = first_bbox.xh;
        let mut yh = first_bbox.yh;

        for polygon in &self.polygons {
            let bbox = polygon.bounding_box();
            if bbox.xl < xl {
                xl = bbox.xl;
            }
            if bbox.xh > xh {
                xh = bbox.xh;
            }
            if bbox.yl < yl {
                yl = bbox.yl;
            }
            if bbox.yh > yh {
                yh = bbox.yh;
            }
        }

        Box::new(xl, yl, xh, yh)
    }
}

/// Pin/blockage metal rectangle (cell-local normalized coordinates; the layer number is the
/// GDS layer number).
///
/// Extracted from the PCell GDS (`io::parse_pcell`) and attached to `Pin::shapes` /
/// `Cell::blockages`, giving downstream consumers the real metal geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PinShape {
    pub gds_layer: Index,
    pub rect: Box<Loc>,
}

impl PinShape {
    pub fn new(gds_layer: Index, rect: Box<Loc>) -> Self {
        Self { gds_layer, rect }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DerivedShapeKind {
    NwellGuardRing,
    LayoutGuardRing,
    PowerStripe,
}

impl DerivedShapeKind {
    pub fn as_str(self) -> &'static str {
        match self {
            DerivedShapeKind::NwellGuardRing => "nwell_guard_ring",
            DerivedShapeKind::LayoutGuardRing => "layout_guard_ring",
            DerivedShapeKind::PowerStripe => "power_stripe",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DerivedPolygon {
    pub layer: Index,
    pub datatype: Index,
    pub points: Vec<Point<Loc>>,
}

impl DerivedPolygon {
    pub fn new(layer: Index, datatype: Index, points: Vec<Point<Loc>>) -> Self {
        Self {
            layer,
            datatype,
            points,
        }
    }

    pub fn rect(layer: Index, datatype: Index, bbox: Box<Loc>) -> Self {
        Self {
            layer,
            datatype,
            points: vec![
                Point::new(bbox.xl, bbox.yl),
                Point::new(bbox.xh, bbox.yl),
                Point::new(bbox.xh, bbox.yh),
                Point::new(bbox.xl, bbox.yh),
                Point::new(bbox.xl, bbox.yl),
            ],
        }
    }

    pub fn bounding_box(&self) -> Box<Loc> {
        if self.points.is_empty() {
            return Box::new(0, 0, 0, 0);
        }
        let mut xl = self.points[0].x;
        let mut yl = self.points[0].y;
        let mut xh = self.points[0].x;
        let mut yh = self.points[0].y;
        for point in &self.points {
            xl = xl.min(point.x);
            yl = yl.min(point.y);
            xh = xh.max(point.x);
            yh = yh.max(point.y);
        }
        Box::new(xl, yl, xh, yh)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DerivedShape {
    pub name: String,
    pub kind: DerivedShapeKind,
    pub owner_idx: Option<Index>,
    pub net_idx: Option<Index>,
    pub pin_idx: Option<Index>,
    pub bbox: Box<Loc>,
    pub polygons: Vec<DerivedPolygon>,
    pub write_as_sref: bool,
}

impl DerivedShape {
    pub fn new(
        name: String,
        kind: DerivedShapeKind,
        owner_idx: Option<Index>,
        net_idx: Option<Index>,
        pin_idx: Option<Index>,
        polygons: Vec<DerivedPolygon>,
    ) -> Self {
        let bbox = polygons
            .iter()
            .map(|p| p.bounding_box())
            .reduce(|a, b| {
                Box::new(
                    a.xl.min(b.xl),
                    a.yl.min(b.yl),
                    a.xh.max(b.xh),
                    a.yh.max(b.yh),
                )
            })
            .unwrap_or_else(|| Box::new(0, 0, 0, 0));
        Self {
            name,
            kind,
            owner_idx,
            net_idx,
            pin_idx,
            bbox,
            polygons,
            write_as_sref: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_box_loc() {
        let b = Box::<Loc>::new(0, 0, 10, 20);
        assert_eq!(b.width(), 10);
        assert_eq!(b.height(), 20);
        assert_eq!(b.area(), 200);
    }

    #[test]
    fn test_box_real() {
        let b = Box::<Real>::new(0.0, 0.0, 10.0, 20.0);
        assert_eq!(b.width(), 10.0);
        assert_eq!(b.height(), 20.0);
        assert_eq!(b.area(), 200.0);
    }

    #[test]
    fn test_point3d() {
        let p = Point3d::new(1, 2, 3);
        assert_eq!(p.x, 1);
        assert_eq!(p.y, 2);
        assert_eq!(p.z, 3);
    }
}
