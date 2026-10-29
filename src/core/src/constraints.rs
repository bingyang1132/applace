//! Constraints - constraint data structures
//!
//! Provides the data structures for placement constraints such as symmetry and array constraints

use crate::types::{Index, Loc, Real};
use std::collections::HashMap;

/// Symmetry group
///
/// Holds the symmetry constraint information: symmetry pairs, self-symmetric cells and the
/// symmetry axis
#[derive(Debug, Clone, Default)]
pub struct SymGroup {
    /// Which way the mirror runs. Vertical means the partners sit left and right
    /// of the axis.
    is_vert: bool,
    /// Where the optimizer left the axis, before it was snapped to the grid.
    axis_unsnapped: Option<Real>,
    /// The axis the legalizer settled on.
    axis: Option<Loc>,
    /// Cells that mirror onto each other, as `(left, right)` database indices.
    sym_pairs: Vec<(Index, Index)>,
    /// Cells that mirror onto themselves, and so must straddle the axis.
    self_syms: Vec<Index>,
}

impl SymGroup {
    /// Creates a new symmetry group
    pub fn new(is_vert: bool) -> Self {
        Self {
            is_vert,
            axis_unsnapped: None,
            axis: None,
            sym_pairs: Vec::new(),
            self_syms: Vec::new(),
        }
    }

    /// Returns the list of symmetry pairs
    pub fn sym_pairs(&self) -> &[(Index, Index)] {
        &self.sym_pairs
    }

    /// Returns the list of symmetry pairs (mutable)
    pub fn sym_pairs_mut(&mut self) -> &mut Vec<(Index, Index)> {
        &mut self.sym_pairs
    }

    /// Returns the symmetry pair at the given index
    pub fn sym_pair(&self, idx: usize) -> Option<(Index, Index)> {
        self.sym_pairs.get(idx).copied()
    }

    /// Adds a symmetry pair
    pub fn add_sym_pair(&mut self, lidx: Index, ridx: Index) {
        self.sym_pairs.push((lidx, ridx));
    }

    /// Returns the list of self-symmetric cells
    pub fn self_syms(&self) -> &[Index] {
        &self.self_syms
    }

    /// Returns the list of self-symmetric cells (mutable)
    pub fn self_syms_mut(&mut self) -> &mut Vec<Index> {
        &mut self.self_syms
    }

    /// Returns the self-symmetric cell at the given index
    pub fn self_sym(&self, idx: usize) -> Option<Index> {
        self.self_syms.get(idx).copied()
    }

    /// Adds a self-symmetric cell
    pub fn add_self_sym(&mut self, idx: Index) {
        self.self_syms.push(idx);
    }

    /// Returns whether the group is empty (no symmetry pairs and no self-symmetric cells)
    pub fn is_empty(&self) -> bool {
        self.sym_pairs.is_empty() && self.self_syms.is_empty()
    }

    /// Returns whether the symmetry is vertical
    pub fn is_vert(&self) -> bool {
        self.is_vert
    }

    /// Sets whether the symmetry is vertical
    pub fn set_is_vert(&mut self, is_vert: bool) {
        self.is_vert = is_vert;
    }

    /// Returns the raw symmetry axis
    pub fn axis_unsnapped(&self) -> Option<Real> {
        self.axis_unsnapped
    }

    /// Sets the raw symmetry axis
    pub fn set_axis_unsnapped(&mut self, axis_unsnapped: Option<Real>) {
        self.axis_unsnapped = axis_unsnapped;
    }

    /// Returns the aligned symmetry axis
    pub fn axis(&self) -> Option<Loc> {
        self.axis
    }

    /// Sets the aligned symmetry axis
    pub fn set_axis(&mut self, axis: Option<Loc>) {
        self.axis = axis;
    }
}

/// Array group
///
/// Holds the array constraint information, used for array and collinear placement
#[derive(Debug, Clone, Default)]
pub struct ArrayGroup {
    /// How the members are arranged: `0` lays them out along a single line, `1`
    /// alternates them on a checkerboard so that like sits diagonal to like.
    mode: Index,
    /// Which way the arrangement runs. Vertical stacks the members top to bottom.
    is_vert: bool,
    /// The cells in the array, as database indices.
    array_members: Vec<Index>,
    /// The same cells bucketed by cell name, so that interleaving can tell the
    /// repeated units apart.
    unit_groups: HashMap<String, Vec<Index>>,
}

impl ArrayGroup {
    /// Creates a new array group
    pub fn new(mode: Index, is_vert: bool) -> Self {
        Self {
            mode,
            is_vert,
            array_members: Vec::new(),
            unit_groups: HashMap::new(),
        }
    }

    /// Returns the list of array units
    pub fn array_members(&self) -> &[Index] {
        &self.array_members
    }

    /// Returns the list of array units (mutable)
    pub fn array_members_mut(&mut self) -> &mut Vec<Index> {
        &mut self.array_members
    }

    /// Returns the array unit at the given index
    pub fn array_member(&self, idx: usize) -> Option<Index> {
        self.array_members.get(idx).copied()
    }

    /// Adds an array unit
    pub fn add_array_member(&mut self, uidx: Index) {
        self.array_members.push(uidx);
    }

    /// Returns the unit-group map
    pub fn unit_groups(&self) -> &HashMap<String, Vec<Index>> {
        &self.unit_groups
    }

    /// Returns the unit-group map (mutable)
    pub fn unit_groups_mut(&mut self) -> &mut HashMap<String, Vec<Index>> {
        &mut self.unit_groups
    }

    /// Returns the unit group for the given cell name
    pub fn unit_group(&self, cname: &str) -> Option<&Vec<Index>> {
        self.unit_groups.get(cname)
    }

    /// Returns the unit group for the given cell name (mutable)
    pub fn unit_group_mut(&mut self, cname: &str) -> Option<&mut Vec<Index>> {
        self.unit_groups.get_mut(cname)
    }

    /// Adds a unit group
    pub fn add_unit_group(&mut self, cname: String, unit_group: Vec<Index>) {
        self.unit_groups.insert(cname, unit_group);
    }

    /// Returns the mode
    pub fn mode(&self) -> Index {
        self.mode
    }

    /// Sets the mode
    pub fn set_mode(&mut self, mode: Index) {
        self.mode = mode;
    }

    /// Returns whether the orientation is vertical
    pub fn is_vert(&self) -> bool {
        self.is_vert
    }

    /// Sets whether the orientation is vertical
    pub fn set_is_vert(&mut self, is_vert: bool) {
        self.is_vert = is_vert;
    }

    /// Returns whether the group is empty
    pub fn is_empty(&self) -> bool {
        self.array_members.is_empty() && self.unit_groups.is_empty()
    }

    /// Returns the number of units
    pub fn num_units(&self) -> usize {
        self.array_members.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sym_group() {
        let mut sym_group = SymGroup::new(true);

        // Add symmetry pairs
        sym_group.add_sym_pair(0, 1);
        sym_group.add_sym_pair(2, 3);

        // Add a self-symmetric cell
        sym_group.add_self_sym(4);

        // Set the symmetry axis
        sym_group.set_axis_unsnapped(Some(100.5));
        sym_group.set_axis(Some(100));

        assert_eq!(sym_group.sym_pairs().len(), 2);
        assert_eq!(sym_group.self_syms().len(), 1);
        assert!(!sym_group.is_empty());
        assert!(sym_group.is_vert());
        assert_eq!(sym_group.axis_unsnapped(), Some(100.5));
        assert_eq!(sym_group.axis(), Some(100));
    }

    #[test]
    fn test_array_group() {
        let mut array_group = ArrayGroup::new(0, false);

        // Add array units
        array_group.add_array_member(0);
        array_group.add_array_member(1);
        array_group.add_array_member(2);

        // Add a unit group
        array_group.add_unit_group("cap1".to_string(), vec![0, 1]);
        array_group.add_unit_group("cap2".to_string(), vec![2]);

        assert_eq!(array_group.num_units(), 3);
        assert_eq!(array_group.mode(), 0);
        assert!(!array_group.is_vert());
        assert!(!array_group.is_empty());

        assert_eq!(array_group.unit_group("cap1"), Some(&vec![0, 1]));
        assert_eq!(array_group.unit_group("cap2"), Some(&vec![2]));
    }
}
