//! The scalar types the database is written in.
//!
//! One alias per role, so that widening a coordinate or changing the working
//! precision is an edit here and nowhere else.

/// A coordinate snapped to the manufacturing grid, in database units (nanometres).
pub type Loc = i32;

/// A slot in one of the database's arrays. Negative means "no such node".
pub type Index = i32;

/// A continuous quantity: an unsnapped coordinate, a weight, a gradient.
pub type Real = f64;

/// A point on the manufacturing grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Coord {
    pub x: Loc,
    pub y: Loc,
}

impl Coord {
    pub fn new(x: Loc, y: Loc) -> Self {
        Self { x, y }
    }

    pub fn zero() -> Self {
        Self { x: 0, y: 0 }
    }
}

/// A point the optimizer is still free to move: not yet snapped to the grid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CoordF {
    pub x: Real,
    pub y: Real,
}

impl CoordF {
    pub fn new(x: Real, y: Real) -> Self {
        Self { x, y }
    }

    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_coord() {
        let c = Coord::new(10, 20);
        assert_eq!(c.x, 10);
        assert_eq!(c.y, 20);
    }

    #[test]
    fn test_coord_f() {
        let c = CoordF::new(1.5, 2.5);
        assert_eq!(c.x, 1.5);
        assert_eq!(c.y, 2.5);
    }
}
