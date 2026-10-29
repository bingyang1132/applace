//! Net - net data structure

use crate::types::{Index, Real};

/// Decides from the name whether a net is a supply net (both the VDD side and the GND side
/// count as supply).
///
/// The netlist carries no power annotation, so supply identity can only be inferred from the
/// naming convention. Everything that needs to know which nets are supplies asks this
/// one function, so no two callers can disagree.
pub fn is_power_net_name(name: &str) -> bool {
    let n = name.to_ascii_lowercase();
    n == "vdd" || n == "vcc" || n == "vss" || n == "gnd" || n.contains("pwr")
}

/// Decides whether a supply net is on the ground / negative-supply side.
pub fn is_ground_net_name(name: &str) -> bool {
    let n = name.to_ascii_lowercase();
    n == "gnd" || n == "vss" || n.ends_with("_gnd") || n.ends_with("_vss")
}

/// Net
#[derive(Debug, Clone)]
pub struct Net {
    name: String,
    idx: Index,
    pin_ids: Vec<Index>,
    sym_net_idx: Option<Index>,
    is_io: bool,
    is_power: bool,
    is_critical: bool,
    is_backbone: bool,
    is_routed: bool,
    is_updated: bool,
    retry_count: i32,
    routing_priority: Real,
    placement_weight: Real,
    custom_wire_width: Option<i32>,
    history_cost: Real,
}

impl Net {
    pub fn new(name: String, idx: Index) -> Self {
        Self {
            name,
            idx,
            pin_ids: Vec::new(),
            sym_net_idx: None,
            is_io: false,
            is_power: false,
            is_critical: false,
            is_backbone: false,
            is_routed: false,
            is_updated: false,
            retry_count: 0,
            routing_priority: 1.0,
            placement_weight: 1.0,
            custom_wire_width: None,
            history_cost: 0.0,
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

    /// Returns the number of pins
    pub fn num_pins(&self) -> usize {
        self.pin_ids.len()
    }

    /// Returns the list of pin indices
    pub fn pin_ids(&self) -> &[Index] {
        &self.pin_ids
    }

    /// Adds a pin index
    pub fn add_pin_idx(&mut self, idx: Index) {
        self.pin_ids.push(idx);
    }

    /// Removes a pin
    pub fn remove_pin(&mut self, pidx: Index) {
        self.pin_ids.retain(|&x| x != pidx);
    }

    /// Returns whether the net is routed
    pub fn is_routed(&self) -> bool {
        self.is_routed
    }

    /// Sets whether the net is routed
    pub fn set_routed(&mut self, routed: bool) {
        self.is_routed = routed;
    }

    /// Returns whether the net has been updated
    pub fn is_updated(&self) -> bool {
        self.is_updated
    }

    /// Sets whether the net has been updated
    pub fn set_updated(&mut self, updated: bool) {
        self.is_updated = updated;
    }

    /// Returns the failure count
    pub fn retry_count(&self) -> i32 {
        self.retry_count
    }

    /// Increments the failure count
    pub fn add_failed(&mut self) {
        self.retry_count += 1;
    }

    /// Resets the failure count
    pub fn reset_failed(&mut self) {
        self.retry_count = 0;
    }

    /// Sets the failure count
    pub fn set_retry_count(&mut self, count: i32) {
        self.retry_count = count.max(0);
    }

    /// Returns the index of the symmetric net
    pub fn sym_net_idx(&self) -> Option<Index> {
        self.sym_net_idx
    }

    /// Sets the index of the symmetric net
    pub fn set_sym_net(&mut self, idx: Option<Index>) {
        self.sym_net_idx = idx;
    }

    /// Returns whether this is an IO net
    pub fn is_io(&self) -> bool {
        self.is_io
    }

    /// Sets whether this is an IO net
    pub fn set_is_io(&mut self, is_io: bool) {
        self.is_io = is_io;
    }

    /// Returns whether this is a supply net
    pub fn is_power(&self) -> bool {
        self.is_power
    }

    /// Sets whether this is a supply net
    pub fn set_is_power(&mut self, is_power: bool) {
        self.is_power = is_power;
    }

    /// Returns whether this is a critical net
    pub fn is_critical(&self) -> bool {
        self.is_critical
    }

    /// Sets whether this is a critical net
    pub fn set_is_critical(&mut self, is_critical: bool) {
        self.is_critical = is_critical;
    }

    /// Returns whether this is a trunk net
    pub fn is_backbone(&self) -> bool {
        self.is_backbone
    }

    /// Sets whether this is a trunk net
    pub fn set_is_backbone(&mut self, is_backbone: bool) {
        self.is_backbone = is_backbone;
    }

    /// Returns the routing priority
    pub fn routing_priority(&self) -> Real {
        self.routing_priority
    }

    /// Sets the routing priority
    pub fn set_routing_priority(&mut self, priority: Real) {
        self.routing_priority = priority;
    }

    /// Returns the placement weight
    pub fn placement_weight(&self) -> Real {
        self.placement_weight
    }

    /// Sets the placement weight
    pub fn set_placement_weight(&mut self, weight: Real) {
        self.placement_weight = weight;
    }

    /// Returns the custom wire width
    pub fn custom_wire_width(&self) -> Option<i32> {
        self.custom_wire_width
    }

    /// Sets the custom wire width
    pub fn set_custom_wire_width(&mut self, width: Option<i32>) {
        self.custom_wire_width = width;
    }

    /// Returns the history cost
    pub fn history_cost(&self) -> Real {
        self.history_cost
    }

    /// Sets the history cost
    pub fn set_history_cost(&mut self, cost: Real) {
        self.history_cost = cost.max(0.0);
    }

    /// Increases the history cost
    pub fn add_history_cost(&mut self, delta: Real) {
        self.history_cost = (self.history_cost + delta).max(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_net_basic() {
        let mut net = Net::new("VDD".to_string(), 0);
        net.add_pin_idx(0);
        net.add_pin_idx(1);
        net.set_is_power(true);
        net.set_routing_priority(2.0);
        net.set_placement_weight(1.5);

        assert_eq!(net.name(), "VDD");
        assert_eq!(net.idx(), 0);
        assert_eq!(net.num_pins(), 2);
        assert!(net.is_power());
        assert_eq!(net.routing_priority(), 2.0);
        assert_eq!(net.placement_weight(), 1.5);
        assert_eq!(net.history_cost(), 0.0);
    }

    #[test]
    fn test_power_net_name_predicates() {
        for n in ["VDD", "vdd", "GND", "vss", "VCC", "core_pwr"] {
            assert!(is_power_net_name(n), "{n} should be a supply net");
        }
        for n in ["vtail", "net0118", "VIP", "vddx"] {
            assert!(!is_power_net_name(n), "{n} should not be a supply net");
        }
        assert!(is_ground_net_name("GND"));
        assert!(is_ground_net_name("blk_vss"));
        assert!(!is_ground_net_name("VDD"));
    }
}
