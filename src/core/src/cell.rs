//! Cell - one device of the circuit

use crate::geometry::{Box, PinShape};
use crate::types::{Coord, CoordF, Index, Loc, Real};

/// Cell - a basic device in the circuit
#[derive(Debug, Clone)]
pub struct Cell {
    /// Instance name, unique in the design.
    name: String,
    /// The cell's own slot in the database.
    idx: Index,
    /// The device model it instantiates; empty when the netlist named none.
    model: String,
    width: Loc,
    height: Loc,
    coord: Coord,
    coord_unsnapped: CoordF,
    geometry_offset: Coord,
    flip_vert: bool,
    flip_hori: bool,
    pin_ids: Vec<Index>,
    net_ids: Vec<Index>,
    layer_ids: Vec<Index>,
    /// Metal on the routing layers (M1..M6) that belongs to no pin (cell-local normalized
    /// coordinates, GDS layer numbers). Device layers (PO, CO, ...) are excluded. Empty by
    /// default, for backward compatibility.
    blockages: Vec<PinShape>,
}

impl Cell {
    pub fn new(name: String) -> Self {
        Self {
            name,
            idx: -1,
            model: String::new(),
            width: 0,
            height: 0,
            coord: Coord::zero(),
            coord_unsnapped: CoordF::zero(),
            geometry_offset: Coord::zero(),
            flip_vert: false,
            flip_hori: false,
            pin_ids: Vec::new(),
            net_ids: Vec::new(),
            layer_ids: Vec::new(),
            blockages: Vec::new(),
        }
    }

    /// Instance name, unique in the design.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Renames the cell.
    pub fn set_name(&mut self, name: String) {
        self.name = name;
    }

    /// The cell's own slot in the database.
    pub fn idx(&self) -> Index {
        self.idx
    }

    /// Records the slot the database put this cell in.
    pub fn set_idx(&mut self, idx: Index) {
        self.idx = idx;
    }

    /// The device model this cell instantiates.
    pub fn model(&self) -> &str {
        &self.model
    }

    /// Records the instantiated model.
    pub fn set_model(&mut self, model: String) {
        self.model = model;
    }

    /// Returns the number of pins
    pub fn num_pins(&self) -> usize {
        self.pin_ids.len()
    }

    /// Returns the list of pin indices
    pub fn pin_ids(&self) -> &[Index] {
        &self.pin_ids
    }

    /// Returns the pin at the given index
    pub fn pin_idx(&self, idx: usize) -> Index {
        self.pin_ids[idx]
    }

    /// Adds a pin index
    pub fn add_pin_idx(&mut self, idx: Index) {
        self.pin_ids.push(idx);
    }

    /// Returns the list of net indices
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

    /// Returns the coordinates
    pub fn coord(&self) -> Coord {
        self.coord
    }

    /// Returns the X coordinate
    pub fn x(&self) -> Loc {
        self.coord.x
    }

    /// Returns the Y coordinate
    pub fn y(&self) -> Loc {
        self.coord.y
    }

    /// Sets the coordinates
    pub fn set_coord(&mut self, x: Loc, y: Loc) {
        self.coord = Coord::new(x, y);
    }

    /// Sets the X coordinate
    pub fn set_x(&mut self, x: Loc) {
        self.coord.x = x;
    }

    /// Sets the Y coordinate
    pub fn set_y(&mut self, y: Loc) {
        self.coord.y = y;
    }

    /// Returns the original coordinates (before alignment)
    pub fn coord_unsnapped(&self) -> CoordF {
        self.coord_unsnapped
    }

    /// Returns the original X coordinate
    pub fn x_unsnapped(&self) -> Real {
        self.coord_unsnapped.x
    }

    /// Returns the original Y coordinate
    pub fn y_unsnapped(&self) -> Real {
        self.coord_unsnapped.y
    }

    /// Sets the original coordinates
    pub fn set_coord_unsnapped(&mut self, x: Real, y: Real) {
        self.coord_unsnapped = CoordF::new(x, y);
    }

    /// Returns the offset of the GDS drawable geometry relative to the SREF coordinates.
    pub fn geometry_offset(&self) -> Coord {
        self.geometry_offset
    }

    /// Sets the offset of the GDS drawable geometry relative to the SREF coordinates.
    pub fn set_geometry_offset(&mut self, x: Loc, y: Loc) {
        self.geometry_offset = Coord::new(x, y);
    }

    /// Returns the width
    pub fn width(&self) -> Loc {
        self.width
    }

    /// Returns the height
    pub fn height(&self) -> Loc {
        self.height
    }

    /// Sets the width
    pub fn set_width(&mut self, w: Loc) {
        self.width = w;
    }

    /// Sets the height
    pub fn set_height(&mut self, h: Loc) {
        self.height = h;
    }

    /// Returns the bounding box
    pub fn bounding_box(&self) -> Box<Loc> {
        Box::new(
            self.coord.x + self.geometry_offset.x,
            self.coord.y + self.geometry_offset.y,
            self.coord.x + self.geometry_offset.x + self.width,
            self.coord.y + self.geometry_offset.y + self.height,
        )
    }

    /// Returns the center point (in original coordinates)
    pub fn center_raw(&self) -> CoordF {
        CoordF::new(
            self.coord_unsnapped.x + self.width as Real / 2.0,
            self.coord_unsnapped.y + self.height as Real / 2.0,
        )
    }

    /// Whether it is flipped vertically
    pub fn flip_vert(&self) -> bool {
        self.flip_vert
    }

    /// Whether it is flipped horizontally
    pub fn flip_hori(&self) -> bool {
        self.flip_hori
    }

    /// Sets the vertical flip
    pub fn set_flip_vert(&mut self, flip: bool) {
        self.flip_vert = flip;
    }

    /// Sets the horizontal flip
    pub fn set_flip_hori(&mut self, flip: bool) {
        self.flip_hori = flip;
    }

    /// Returns the list of blockage metal rectangles (cell-local normalized coordinates, GDS layer numbers)
    pub fn blockages(&self) -> &[PinShape] {
        &self.blockages
    }

    /// Replaces the list of blockage metal rectangles
    pub fn set_blockages(&mut self, blockages: Vec<PinShape>) {
        self.blockages = blockages;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cell_basic() {
        let mut cell = Cell::new("nmos1".to_string());
        cell.set_idx(0);
        cell.set_model("nmos".to_string());
        cell.set_coord(10, 20);
        cell.set_width(100);
        cell.set_height(50);
        cell.add_pin_idx(0);
        cell.add_pin_idx(1);

        assert_eq!(cell.idx(), 0);
        assert_eq!(cell.name(), "nmos1");
        assert_eq!(cell.model(), "nmos");
        assert_eq!(cell.x(), 10);
        assert_eq!(cell.y(), 20);
        assert_eq!(cell.width(), 100);
        assert_eq!(cell.height(), 50);
        assert_eq!(cell.num_pins(), 2);
    }

    #[test]
    fn test_cell_bounding_box() {
        let mut cell = Cell::new("test".to_string());
        cell.set_coord(10, 20);
        cell.set_width(100);
        cell.set_height(50);
        let bbox = cell.bounding_box();
        assert_eq!(bbox.xl, 10);
        assert_eq!(bbox.yl, 20);
        assert_eq!(bbox.xh, 110);
        assert_eq!(bbox.yh, 70);
    }
}
