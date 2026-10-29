//! Device-class declaration - which model names are PMOS, NMOS, capacitors, resistors.
//!
//! # Why this exists
//!
//! Every stage that needs to know "is this device a PMOS?" has to get the answer out
//! of the device's model string, and a model string is a name some process invented.
//! Deciding it in the source - testing the string against a fixed list of spellings -
//! can only ever be right for the processes whose spellings somebody thought to list.
//! Bring your own PDK, say sky130's `sky130_fd_pr__pfet_01v8`, and every device
//! classifies as *not* PMOS: the guard-ring reservation (`pmosHardExpandForGr`) is then
//! reserved for nothing, the placement is wrong in a way that still looks plausible,
//! and nothing in the flow says a word about it.
//!
//! So the deck declares it, and nothing else does. `lef.json` carries a `device_types`
//! section naming the model families of each class, and that declaration is the only
//! rule this build uses. The section is specified in `docs/INPUT_FORMAT.md`, section
//! 2.4.
//!
//! # Semantics, in full
//!
//! * **A deck that declares nothing is refused.** There is no model-name heuristic in
//!   this build to fall back to, so `io::apply_device_types` rejects a deck with no
//!   `device_types` section, and a section that lists no names at all counts as none.
//!   Validation of the declaration runs on every load.
//! * **A model the declaration does not cover is a hard error**, naming every
//!   uncovered model, rather than a quiet `false`. That error is the whole point: it
//!   is the difference between being told your deck is incomplete and getting a
//!   silently unreserved layout.
//! * **Matching is case-insensitive**, and is either `prefix` (the default: a listed
//!   `pmos_example` covers `pmos_example`, `pmos_example_variant_a`, ...) or `exact`.
//! * **Longest listed token wins.** Listing `pmos_example` under `pmos` and
//!   `pmos_example_dummy` under `other` classifies `pmos_example_dummy_x` as `other`,
//!   which is the only reading that makes a narrow exception expressible. Ties cannot
//!   happen: two categories can only tie on match length by listing the same token,
//!   which [`DeviceTypes::validate`] rejects.
//! * **`other` is a real category**, for devices deliberately outside the four classes
//!   (standard cells, diodes, ESD structures). It suppresses the unknown-model error
//!   without claiming the device is a transistor.
//!
//! The declaration lives in the PDK deck because the model-name vocabulary is the
//! deck's, and it is carried on [`crate::CircuitDb`] because every consumer already has
//! the database in hand.

/// The classes a declared device model can fall into.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum DeviceClass {
    Pmos,
    Nmos,
    Capacitor,
    Resistor,
    /// Declared, but deliberately none of the above (standard cells, diodes, …).
    Other,
}

impl DeviceClass {
    /// The `device_types` key this class is declared under.
    pub fn key(self) -> &'static str {
        match self {
            DeviceClass::Pmos => "pmos",
            DeviceClass::Nmos => "nmos",
            DeviceClass::Capacitor => "capacitor",
            DeviceClass::Resistor => "resistor",
            DeviceClass::Other => "other",
        }
    }
}

/// How a listed model token is compared against a netlist model string.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DeviceMatch {
    /// `model` starts with the token (case-insensitive). The default: one entry per
    /// model *family* covers its whole ladder of flavours.
    #[default]
    Prefix,
    /// `model` equals the token (case-insensitive).
    Exact,
}

impl DeviceMatch {
    /// Parse the deck's `match` field. `None` (absent) is [`DeviceMatch::Prefix`].
    pub fn parse(s: Option<&str>) -> Result<Self, String> {
        match s.map(|v| v.trim().to_ascii_lowercase()).as_deref() {
            None | Some("prefix") => Ok(DeviceMatch::Prefix),
            Some("exact") => Ok(DeviceMatch::Exact),
            Some(other) => Err(format!(
                "device_types.match must be \"prefix\" or \"exact\", got {other:?}"
            )),
        }
    }
}

/// A deck's device-class declaration.
///
/// Construct it from the PDK deck (`io`'s `PdkRules::device_types`), hang it on the
/// [`crate::CircuitDb`], and every stage classifies devices the same way.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DeviceTypes {
    match_mode: DeviceMatch,
    pmos: Vec<String>,
    nmos: Vec<String>,
    capacitor: Vec<String>,
    resistor: Vec<String>,
    other: Vec<String>,
}

impl DeviceTypes {
    pub fn new(
        match_mode: DeviceMatch,
        pmos: Vec<String>,
        nmos: Vec<String>,
        capacitor: Vec<String>,
        resistor: Vec<String>,
        other: Vec<String>,
    ) -> Self {
        Self {
            match_mode,
            pmos,
            nmos,
            capacitor,
            resistor,
            other,
        }
    }

    pub fn match_mode(&self) -> DeviceMatch {
        self.match_mode
    }

    /// The tokens listed under one class, in declaration order.
    pub fn tokens(&self, class: DeviceClass) -> &[String] {
        match class {
            DeviceClass::Pmos => &self.pmos,
            DeviceClass::Nmos => &self.nmos,
            DeviceClass::Capacitor => &self.capacitor,
            DeviceClass::Resistor => &self.resistor,
            DeviceClass::Other => &self.other,
        }
    }

    /// Every class, in a fixed order — so error messages are deterministic.
    pub const ALL_CLASSES: [DeviceClass; 5] = [
        DeviceClass::Pmos,
        DeviceClass::Nmos,
        DeviceClass::Capacitor,
        DeviceClass::Resistor,
        DeviceClass::Other,
    ];

    /// A declaration with no tokens at all is not a declaration: it cannot classify
    /// anything, so treating it as one would turn every device into an error. Callers
    /// drop it and fall back to the built-in heuristic.
    pub fn is_empty(&self) -> bool {
        Self::ALL_CLASSES.iter().all(|c| self.tokens(*c).is_empty())
    }

    /// Reject a declaration that cannot be applied unambiguously.
    ///
    /// Two rules, both about the declaration alone (never about a netlist):
    /// an empty/blank token matches everything and is always a mistake; the same token
    /// under two classes has no defensible winner.
    pub fn validate(&self) -> Result<(), String> {
        let mut seen: Vec<(String, DeviceClass)> = Vec::new();
        for class in Self::ALL_CLASSES {
            for tok in self.tokens(class) {
                if tok.trim().is_empty() {
                    return Err(format!(
                        "device_types.{}: empty model name (it would match every device)",
                        class.key()
                    ));
                }
                let lc = tok.to_ascii_lowercase();
                if let Some((_, prev)) = seen.iter().find(|(t, _)| *t == lc) {
                    return Err(format!(
                        "device_types: model {tok:?} is listed under both {} and {}",
                        prev.key(),
                        class.key()
                    ));
                }
                seen.push((lc, class));
            }
        }
        Ok(())
    }

    fn token_match_len(&self, model_lc: &str, token: &str) -> Option<usize> {
        let tok = token.to_ascii_lowercase();
        let hit = match self.match_mode {
            DeviceMatch::Prefix => model_lc.starts_with(&tok),
            DeviceMatch::Exact => model_lc == tok,
        };
        hit.then_some(tok.len())
    }

    /// Classify one netlist model string. `None` = the declaration does not cover it,
    /// which callers turn into a hard error (see [`DeviceTypes::unknown_models`]).
    pub fn classify(&self, model: &str) -> Option<DeviceClass> {
        let model_lc = model.to_ascii_lowercase();
        let mut best: Option<(usize, DeviceClass)> = None;
        for class in Self::ALL_CLASSES {
            for tok in self.tokens(class) {
                if let Some(len) = self.token_match_len(&model_lc, tok) {
                    if best.is_none_or(|(blen, _)| len > blen) {
                        best = Some((len, class));
                    }
                }
            }
        }
        best.map(|(_, c)| c)
    }

    pub fn is_pmos(&self, model: &str) -> bool {
        self.classify(model) == Some(DeviceClass::Pmos)
    }

    pub fn is_nmos(&self, model: &str) -> bool {
        self.classify(model) == Some(DeviceClass::Nmos)
    }

    /// Whether the class is one of the four the deck can act on (i.e. not `other`).
    pub fn is_class(&self, model: &str, class: DeviceClass) -> bool {
        self.classify(model) == Some(class)
    }

    /// The distinct model strings this declaration does not cover, sorted and
    /// de-duplicated so the error message is identical run to run.
    ///
    /// A deck that declares device types has taken responsibility for the vocabulary;
    /// an unlisted model is a gap in the deck, not a device that happens not to be a
    /// PMOS, and saying so is the whole point of the declaration.
    pub fn unknown_models<'a, I>(&self, models: I) -> Vec<String>
    where
        I: IntoIterator<Item = &'a str>,
    {
        let mut unknown: Vec<String> = models
            .into_iter()
            .filter(|d| self.classify(d).is_none())
            .map(|d| d.to_string())
            .collect();
        unknown.sort();
        unknown.dedup();
        unknown
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn deck() -> DeviceTypes {
        DeviceTypes::new(
            DeviceMatch::Prefix,
            vec!["pmos_example".into()],
            vec!["nmos_example".into()],
            vec!["cap_example".into()],
            vec!["res_example".into()],
            vec![],
        )
    }

    #[test]
    fn declared_prefix_classifies_the_whole_family() {
        let d = deck();
        assert!(d.is_pmos("pmos_example"));
        assert!(d.is_pmos("pmos_example_variant_c"));
        assert!(d.is_pmos("PMOS_EXAMPLE_VARIANT_B")); // case-insensitive
        assert!(!d.is_pmos("nmos_example_variant_a"));
        assert_eq!(
            d.classify("nmos_example_variant_d"),
            Some(DeviceClass::Nmos)
        );
        assert_eq!(
            d.classify("cap_example_variant_a"),
            Some(DeviceClass::Capacitor)
        );
        assert_eq!(
            d.classify("res_example_variant_a"),
            Some(DeviceClass::Resistor)
        );
    }

    #[test]
    fn a_foreign_deck_is_classified_by_its_own_names() {
        // The bug this whole section exists for: the built-in heuristic calls every
        // sky130 device not-PMOS. A declaration fixes it without touching any code.
        let d = DeviceTypes::new(
            DeviceMatch::Prefix,
            vec!["sky130_fd_pr__pfet".into()],
            vec!["sky130_fd_pr__nfet".into()],
            vec![],
            vec![],
            vec![],
        );
        assert!(d.is_pmos("sky130_fd_pr__pfet_01v8"));
        assert!(!d.is_pmos("sky130_fd_pr__nfet_01v8"));
    }

    #[test]
    fn exact_mode_does_not_match_a_flavour() {
        let d = DeviceTypes::new(
            DeviceMatch::Exact,
            vec!["pmos_example".into()],
            vec![],
            vec![],
            vec![],
            vec![],
        );
        assert!(d.is_pmos("pmos_example"));
        assert!(!d.is_pmos("pmos_example_variant_a"));
        assert_eq!(
            d.unknown_models(["pmos_example_variant_a"]),
            vec!["pmos_example_variant_a".to_string()]
        );
    }

    #[test]
    fn longest_token_wins_so_an_exception_is_expressible() {
        let d = DeviceTypes::new(
            DeviceMatch::Prefix,
            vec!["pmos_example".into()],
            vec![],
            vec![],
            vec![],
            vec!["pmos_example_dummy".into()],
        );
        assert_eq!(
            d.classify("pmos_example_variant_a"),
            Some(DeviceClass::Pmos)
        );
        assert_eq!(d.classify("pmos_example_dummy_x"), Some(DeviceClass::Other));
    }

    #[test]
    fn unknown_models_are_reported_sorted_and_deduped() {
        let d = deck();
        let unknown = d.unknown_models([
            "pmos_example",
            "diode_x",
            "nmos_example_variant_a",
            "esd_y",
            "diode_x",
        ]);
        assert_eq!(unknown, vec!["diode_x".to_string(), "esd_y".to_string()]);
    }

    #[test]
    fn validate_rejects_a_model_claimed_by_two_classes() {
        let d = DeviceTypes::new(
            DeviceMatch::Prefix,
            vec!["pmos_example".into()],
            vec!["PMOS_EXAMPLE".into()],
            vec![],
            vec![],
            vec![],
        );
        let err = d.validate().unwrap_err();
        assert!(err.contains("pmos") && err.contains("nmos"), "{err}");
    }

    #[test]
    fn validate_rejects_an_empty_token() {
        let d = DeviceTypes::new(
            DeviceMatch::Prefix,
            vec!["".into()],
            vec![],
            vec![],
            vec![],
            vec![],
        );
        assert!(d.validate().is_err());
    }

    #[test]
    fn an_all_empty_declaration_is_not_a_declaration() {
        assert!(DeviceTypes::default().is_empty());
        assert!(!deck().is_empty());
    }

    #[test]
    fn match_mode_parses_and_rejects_junk() {
        assert_eq!(DeviceMatch::parse(None).unwrap(), DeviceMatch::Prefix);
        assert_eq!(
            DeviceMatch::parse(Some("Exact")).unwrap(),
            DeviceMatch::Exact
        );
        assert!(DeviceMatch::parse(Some("regex")).is_err());
    }
}
