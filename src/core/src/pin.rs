//! A pin: one terminal of one device, and the metal that realises it.

use crate::geometry::PinShape;
use crate::types::{Coord, Index, Loc};

/// One terminal of one device.
#[derive(Debug, Clone)]
pub struct Pin {
    name: String,
    idx: Index,
    cell_idx: Option<Index>,
    net_ids: Vec<Index>,
    layer_ids: Vec<Index>,
    is_io: bool,
    is_well_tap: bool,
    offset: Coord,
    /// Full metal rectangles (cell-local normalized coordinates, GDS layer numbers).
    /// Empty by default, for backward compatibility.
    shapes: Vec<PinShape>,
}

impl Pin {
    pub fn new(name: String, idx: Index) -> Self {
        Self {
            name,
            idx,
            cell_idx: None,
            net_ids: Vec::new(),
            layer_ids: Vec::new(),
            is_io: false,
            is_well_tap: false,
            offset: Coord::zero(),
            shapes: Vec::new(),
        }
    }

    /// Returns the name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Sets the name
    pub fn set_name(&mut self, name: String) {
        self.name = name;
    }

    /// Returns the index
    pub fn idx(&self) -> Index {
        self.idx
    }

    /// Sets the index
    pub fn set_idx(&mut self, idx: Index) {
        self.idx = idx;
    }

    /// Returns the cell index
    pub fn cell_idx(&self) -> Option<Index> {
        self.cell_idx
    }

    /// Sets the cell index
    pub fn set_cell_idx(&mut self, idx: Option<Index>) {
        self.cell_idx = idx;
    }

    /// Returns the net index (the first one)
    pub fn net_idx(&self) -> Option<Index> {
        self.net_ids.first().copied()
    }

    /// Returns all net indices
    pub fn net_ids(&self) -> &[Index] {
        &self.net_ids
    }

    /// Adds a net index
    pub fn add_net_idx(&mut self, idx: Index) {
        self.net_ids.push(idx);
    }

    /// Returns the list of layer indices
    pub fn layer_ids(&self) -> &[Index] {
        &self.layer_ids
    }

    /// Adds a layer index
    pub fn add_layer_idx(&mut self, idx: Index) {
        self.layer_ids.push(idx);
    }

    /// Overwrites the layer index list
    pub fn set_layer_ids(&mut self, layers: Vec<Index>) {
        self.layer_ids = layers;
    }

    /// Returns the offset
    pub fn offset(&self) -> Coord {
        self.offset
    }

    /// Returns the X offset
    pub fn x_offset(&self) -> Loc {
        self.offset.x
    }

    /// Returns the Y offset
    pub fn y_offset(&self) -> Loc {
        self.offset.y
    }

    /// Sets the offset
    pub fn set_offset(&mut self, x: Loc, y: Loc) {
        self.offset = Coord::new(x, y);
    }

    /// Returns the metal rectangle list (cell-local normalized coordinates, GDS layer numbers)
    pub fn shapes(&self) -> &[PinShape] {
        &self.shapes
    }

    /// Overwrites the metal rectangle list
    pub fn set_shapes(&mut self, shapes: Vec<PinShape>) {
        self.shapes = shapes;
    }

    /// Adds a metal rectangle
    pub fn add_shape(&mut self, shape: PinShape) {
        self.shapes.push(shape);
    }

    /// Whether this is an IO pin
    pub fn is_io(&self) -> bool {
        self.is_io
    }

    /// Sets whether this is an IO pin
    pub fn set_is_io(&mut self, is_io: bool) {
        self.is_io = is_io;
    }

    /// Whether this is an NWGR pin
    pub fn is_well_tap(&self) -> bool {
        self.is_well_tap
    }

    /// Sets whether this is an NWGR pin
    pub fn set_is_well_tap(&mut self, is_well_tap: bool) {
        self.is_well_tap = is_well_tap;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pin_basic() {
        let mut pin = Pin::new("VDD".to_string(), 0);
        pin.set_cell_idx(Some(1));
        pin.add_net_idx(0);
        pin.add_layer_idx(31);
        pin.set_offset(10, 20);
        pin.set_is_io(true);

        assert_eq!(pin.name(), "VDD");
        assert_eq!(pin.idx(), 0);
        assert_eq!(pin.cell_idx(), Some(1));
        assert_eq!(pin.net_idx(), Some(0));
        assert_eq!(pin.x_offset(), 10);
        assert_eq!(pin.y_offset(), 20);
        assert!(pin.is_io());
    }
}
