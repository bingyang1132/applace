//! CircuitDb - the circuit database
//!
//! One design: its devices (cells), nets and pins, the design's ports, and the
//! constraints placement reads. A hierarchical netlist is flattened by the parser
//! before anything lands here.

use crate::cell::Cell;
use crate::constraints::{ArrayGroup, SymGroup};
use crate::device_types::DeviceTypes;
use crate::geometry::DerivedShape;
use crate::net::Net;
use crate::pin::Pin;
use crate::types::Index;
use std::collections::{HashMap, HashSet};

/// CircuitDb - the circuit database
#[derive(Debug, Default)]
pub struct CircuitDb {
    /// The design's name: the netlist's top circuit.
    name: String,
    cells: Vec<Cell>,
    /// The design's ports, in netlist order.
    ports: Vec<Index>,
    port_set: HashSet<Index>,
    /// Symmetry groups, one per `sym.json` block that bound anything, in the order the
    /// blocks were applied. The order is part of the determinism contract.
    sym_groups: Vec<SymGroup>,
    nets: Vec<Net>,
    pins: Vec<Pin>,
    net_hash: HashMap<String, Index>,
    array_groups: Vec<ArrayGroup>,
    well_groups: Vec<Vec<Index>>,
    derived_shapes: Vec<DerivedShape>,
    /// The PDK deck's `device_types` declaration, when it has one. `None` = the deck
    /// declared nothing and every consumer keeps its historical built-in heuristic.
    /// Set once by the stage binary right after the deck is parsed.
    device_types: Option<DeviceTypes>,
}

impl CircuitDb {
    pub fn new() -> Self {
        Self::default()
    }

    /// The deck's device-class declaration, or `None` when the deck declared none.
    ///
    /// Classification helpers read this instead of substring-matching vendor model
    /// spellings; see [`crate::device_types`] for why.
    pub fn device_types(&self) -> Option<&DeviceTypes> {
        self.device_types.as_ref()
    }

    /// Install the deck's declaration. An all-empty declaration is stored as `None`,
    /// so "the section exists but lists nothing" behaves like "no section".
    pub fn set_device_types(&mut self, dt: Option<DeviceTypes>) {
        self.device_types = dt.filter(|d| !d.is_empty());
    }

    /// Every distinct device model string in the database, sorted — the input to
    /// [`crate::device_types::DeviceTypes::unknown_models`].
    pub fn device_model_strings(&self) -> Vec<String> {
        let mut out: Vec<String> = self.cells.iter().map(|c| c.model().to_string()).collect();
        out.sort();
        out.dedup();
        out
    }

    /// Returns the number of cells
    pub fn num_cells(&self) -> usize {
        self.cells.len()
    }

    /// Returns a cell (read-only)
    pub fn cell(&self, idx: Index) -> Option<&Cell> {
        self.cells.get(idx as usize)
    }

    /// Returns a cell (mutable)
    pub fn cell_mut(&mut self, idx: Index) -> Option<&mut Cell> {
        self.cells.get_mut(idx as usize)
    }

    /// Every cell, in index order
    pub fn cells(&self) -> &[Cell] {
        &self.cells
    }

    /// Allocates a new Cell
    pub fn allocate_cell(&mut self, name: String) -> Index {
        let idx = self.cells.len() as Index;
        let mut cell = Cell::new(name);
        cell.set_idx(idx);
        self.cells.push(cell);
        idx
    }

    /// The design's ports, in netlist order
    pub fn ports(&self) -> &[Index] {
        &self.ports
    }

    /// Declares a net a port of the design
    pub fn add_port(&mut self, nidx: Index) {
        if self.port_set.insert(nidx) {
            self.ports.push(nidx);
        }
    }

    /// Whether the net is a port of the design
    pub fn is_port(&self, nidx: Index) -> bool {
        self.port_set.contains(&nidx)
    }

    /// The symmetry groups, in the order they were added
    pub fn sym_groups(&self) -> &[SymGroup] {
        &self.sym_groups
    }

    /// The symmetry groups (mutable)
    pub fn sym_groups_mut(&mut self) -> &mut Vec<SymGroup> {
        &mut self.sym_groups
    }

    /// Appends one symmetry group
    pub fn add_sym_group(&mut self, grp: SymGroup) {
        self.sym_groups.push(grp);
    }

    /// Returns the number of Nets
    pub fn num_nets(&self) -> usize {
        self.nets.len()
    }

    /// Returns a Net (read-only)
    pub fn net(&self, idx: Index) -> Option<&Net> {
        self.nets.get(idx as usize)
    }

    /// Returns a Net (mutable)
    pub fn net_mut(&mut self, idx: Index) -> Option<&mut Net> {
        self.nets.get_mut(idx as usize)
    }

    /// Allocates a new Net
    pub fn allocate_net(&mut self, name: String) -> Index {
        let idx = self.nets.len() as Index;
        let net = Net::new(name, idx);
        self.net_hash.insert(net.name().to_string(), idx);
        self.nets.push(net);
        idx
    }

    /// Looks up a Net index by name
    pub fn find_net(&self, name: &str) -> Option<Index> {
        self.net_hash.get(name).copied()
    }

    /// Marks power nets by name convention and returns how many nets were marked.
    ///
    /// The netlist carries no power annotation, so `Net::is_power` can only be
    /// inferred from the name. Iteration follows index order, which makes the
    /// result deterministic. Nets that are already marked are never cleared.
    pub fn mark_power_nets_by_name(&mut self) -> usize {
        let mut marked = 0usize;
        for net in self.nets.iter_mut() {
            if !net.is_power() && crate::net::is_power_net_name(net.name()) {
                net.set_is_power(true);
                marked += 1;
            }
        }
        marked
    }

    /// Returns the number of Pins
    pub fn num_pins(&self) -> usize {
        self.pins.len()
    }

    /// Returns a Pin (read-only)
    pub fn pin(&self, idx: Index) -> Option<&Pin> {
        self.pins.get(idx as usize)
    }

    /// Returns a Pin (mutable)
    pub fn pin_mut(&mut self, idx: Index) -> Option<&mut Pin> {
        self.pins.get_mut(idx as usize)
    }

    /// Allocates a new Pin
    pub fn allocate_pin(&mut self, name: String) -> Index {
        let idx = self.pins.len() as Index;
        let pin = Pin::new(name, idx);
        self.pins.push(pin);
        idx
    }

    /// Looks up a Pin index by name
    pub fn find_pin(&self, name: &str) -> Option<Index> {
        self.pins
            .iter()
            .position(|p| p.name() == name)
            .map(|i| i as Index)
    }

    /// Returns the name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Sets the name
    pub fn set_name(&mut self, name: String) {
        self.name = name;
    }

    /// Returns the well cluster list
    pub fn well_groups(&self) -> &[Vec<Index>] {
        &self.well_groups
    }

    /// Returns the well cluster list (mutable)
    pub fn well_groups_mut(&mut self) -> &mut Vec<Vec<Index>> {
        &mut self.well_groups
    }

    /// Returns the well cluster at the given index
    pub fn well_group(&self, widx: usize) -> Option<&Vec<Index>> {
        self.well_groups.get(widx)
    }

    /// Returns the well cluster at the given index (mutable)
    pub fn well_group_mut(&mut self, widx: usize) -> Option<&mut Vec<Index>> {
        self.well_groups.get_mut(widx)
    }

    /// Sets the well clusters
    pub fn set_well_groups(&mut self, well_groups: Vec<Vec<Index>>) {
        self.well_groups = well_groups;
    }

    /// Adds a well cluster
    pub fn add_well_group(&mut self, cluster: Vec<Index>) {
        self.well_groups.push(cluster);
    }

    /// Returns the number of well clusters
    pub fn num_well_groups(&self) -> usize {
        self.well_groups.len()
    }

    /// Returns the array group list
    pub fn array_groups(&self) -> &[ArrayGroup] {
        &self.array_groups
    }

    /// Returns the array group list (mutable)
    pub fn array_groups_mut(&mut self) -> &mut Vec<ArrayGroup> {
        &mut self.array_groups
    }

    /// Returns the array group at the given index
    pub fn array_group(&self, gidx: usize) -> Option<&ArrayGroup> {
        self.array_groups.get(gidx)
    }

    /// Returns the array group at the given index (mutable)
    pub fn array_group_mut(&mut self, gidx: usize) -> Option<&mut ArrayGroup> {
        self.array_groups.get_mut(gidx)
    }

    /// Adds an array group
    pub fn add_array_group(&mut self, array_group: ArrayGroup) {
        self.array_groups.push(array_group);
    }

    /// Sets the array group list
    pub fn set_array_groups(&mut self, array_groups: Vec<ArrayGroup>) {
        self.array_groups = array_groups;
    }

    /// Returns the number of array groups
    pub fn num_array_groups(&self) -> usize {
        self.array_groups.len()
    }

    /// Returns the derived layout shapes (produced by a later stage of a full flow)
    pub fn derived_shapes(&self) -> &[DerivedShape] {
        &self.derived_shapes
    }

    /// Returns the derived layout shapes (mutable)
    pub fn derived_shapes_mut(&mut self) -> &mut Vec<DerivedShape> {
        &mut self.derived_shapes
    }

    /// Adds a derived layout shape
    pub fn add_derived_shape(&mut self, shape: DerivedShape) {
        self.derived_shapes.push(shape);
    }

    /// Clears the derived layout shapes
    pub fn clear_derived_shapes(&mut self) {
        self.derived_shapes.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_circuit_db_basic() {
        let mut db = CircuitDb::new();
        db.set_name("top".to_string());

        let m1 = db.allocate_cell("nmos1".to_string());
        let m2 = db.allocate_cell("nmos2".to_string());
        db.cell_mut(m1).unwrap().set_model("nmos".to_string());

        let vdd = db.allocate_net("VDD".to_string());
        db.add_port(vdd);
        db.add_port(vdd);
        let pin_idx = db.allocate_pin("VDD".to_string());

        assert_eq!((m1, m2), (0, 1));
        assert_eq!(db.num_cells(), 2);
        assert_eq!(db.cell(m1).unwrap().model(), "nmos");
        assert_eq!(db.num_nets(), 1);
        assert_eq!(db.num_pins(), 1);
        assert_eq!(pin_idx, 0);
        assert_eq!(db.find_net("VDD"), Some(0));
        assert_eq!(db.ports(), &[vdd]);
        assert!(db.is_port(vdd));
        assert_eq!(
            db.device_model_strings(),
            vec!["".to_string(), "nmos".to_string()]
        );
    }

    #[test]
    fn test_well_groups() {
        let mut db = CircuitDb::new();

        // Create a few cells
        let cell1 = db.allocate_cell("nmos1".to_string());
        let cell2 = db.allocate_cell("nmos2".to_string());
        let cell3 = db.allocate_cell("pmos1".to_string());

        // Add well clusters
        db.add_well_group(vec![cell1, cell2]);
        db.add_well_group(vec![cell3]);

        assert_eq!(db.num_well_groups(), 2);
        assert_eq!(db.well_group(0).unwrap(), &vec![cell1, cell2]);
        assert_eq!(db.well_group(1).unwrap(), &vec![cell3]);

        // Test the setter
        db.set_well_groups(vec![vec![cell1, cell2, cell3]]);
        assert_eq!(db.num_well_groups(), 1);
        assert_eq!(db.well_group(0).unwrap().len(), 3);
    }

    #[test]
    fn test_array_groups() {
        let mut db = CircuitDb::new();

        // Create the array group
        let mut array_group = ArrayGroup::new(0, false);
        array_group.add_array_member(0);
        array_group.add_array_member(1);
        array_group.add_unit_group("cap1".to_string(), vec![0, 1]);

        db.add_array_group(array_group);

        assert_eq!(db.num_array_groups(), 1);
        assert_eq!(db.array_group(0).unwrap().num_units(), 2);
        assert_eq!(
            db.array_group(0).unwrap().unit_group("cap1").unwrap().len(),
            2
        );

        // Test the setter
        let mut array_group2 = ArrayGroup::new(1, true);
        array_group2.add_array_member(2);
        db.set_array_groups(vec![array_group2]);
        assert_eq!(db.num_array_groups(), 1);
        assert_eq!(db.array_group(0).unwrap().num_units(), 1);
    }
}
