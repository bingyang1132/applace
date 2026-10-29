//! Parser - input parsing module
//!
//! Provides:
//! - reading a hierarchical netlist (SPICE format)
//! - reading a PDK (JSON format, e.g. lef.json)
//! - reading config files (symmetry, electrical, array, ...)

use applace_core::{CircuitDb, DeviceMatch, DeviceTypes, SymGroup};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;
use thiserror::Error;

pub use crate::pcell::parse_pcell;

#[derive(Error, Debug)]
pub enum ParseError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("JSON parse error: {0}")]
    Json(#[from] serde_json::Error),
    #[error("Parse error: {0}")]
    Parse(String),
}

/// PDK (Process Design Kit) data structures
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PdkRules {
    #[serde(rename = "unit_r")]
    pub unit_r: f64,
    #[serde(rename = "unit_n")]
    pub unit_n: f64,
    #[serde(rename = "layer_map")]
    pub layer_map: HashMap<String, u32>,
    #[serde(rename = "grid")]
    pub grid: f64,
    #[serde(rename = "layer_datatype")]
    pub layer_datatype: HashMap<String, u32>,
    #[serde(rename = "min_w_rules")]
    pub min_w_rules: HashMap<String, f64>,
    #[serde(rename = "sp_rules")]
    pub sp_rules: HashMap<String, HashMap<String, f64>>,
    #[serde(rename = "en_rules")]
    pub en_rules: HashMap<String, HashMap<String, f64>>,
    #[serde(rename = "ex_rules")]
    pub ex_rules: HashMap<String, HashMap<String, f64>>,
    #[serde(rename = "nwell_gr")]
    pub nwell_gr: bool,
    #[serde(rename = "sub_gr")]
    pub sub_gr: bool,
    #[serde(rename = "nw_od")]
    pub nw_od: f64,
    #[serde(rename = "np_od")]
    pub np_od: f64,
    #[serde(rename = "dummy_l")]
    pub dummy_l: f64,
    #[serde(rename = "crit_l")]
    pub crit_l: f64,
    #[serde(rename = "crit_sp")]
    pub crit_sp: f64,
    /// Optional: worst-case device-to-device clean spacing (µm) used to derive the
    /// DRC-aware tail inflation floor. Falls back to `crit_sp` when absent so older
    /// `lef.json` files remain valid.
    #[serde(rename = "max_pairwise_clean_spacing", default)]
    pub max_pairwise_clean_spacing: Option<f64>,
    #[serde(rename = "gate_space_25")]
    pub gate_space_25: f64,
    #[serde(rename = "en_od_25")]
    pub en_od_25: f64,
    #[serde(rename = "od_w")]
    pub od_w: f64,
    #[serde(rename = "sp_co_po_25")]
    pub sp_co_po_25: f64,
    #[serde(rename = "w_via")]
    pub w_via: f64,
    #[serde(rename = "sp_via")]
    pub sp_via: f64,
    #[serde(rename = "en_via")]
    pub en_via: f64,
    #[serde(rename = "en_pp_po_wo")]
    pub en_pp_po_wo: f64,
    #[serde(rename = "w_con")]
    pub w_con: f64,
    #[serde(rename = "lvs_sp_co")]
    pub lvs_sp_co: f64,
    #[serde(rename = "sp_sub")]
    pub sp_sub: f64,
    #[serde(rename = "ex_po_od_na")]
    pub ex_po_od_na: f64,
    #[serde(rename = "m_via")]
    pub m_via: f64,
    #[serde(rename = "Rsg")]
    pub rsg: f64,
    #[serde(rename = "Cpsm")]
    pub cpsm: f64,
    #[serde(rename = "Wem")]
    pub wem: f64,
    #[serde(rename = "textlayers")]
    pub textlayers: Vec<u32>,
    #[serde(rename = "pmos")]
    pub pmos: Vec<String>,
    #[serde(rename = "nmos")]
    pub nmos: Vec<String>,
    #[serde(rename = "cap")]
    pub cap: Vec<String>,
    #[serde(rename = "res")]
    pub res: Vec<String>,
    /// Optional: the **device-class declaration**. The deck says which model names
    /// are PMOS, NMOS, capacitors and resistors, instead of the flow guessing from
    /// the spelling of the model string.
    /// Absent means nothing is declared, which this build refuses -- see
    /// `apply_device_types`. Semantics: `core::device_types`.
    #[serde(rename = "device_types", default)]
    pub device_types: Option<DeviceTypesSection>,
    /// Optional: the deck's self-description (provenance, metal stack, and
    /// `physical_grid`, the only one consumed at run time). Absent means nothing is
    /// declared, and every field falls back to its own conservative default.
    #[serde(rename = "_meta", default)]
    pub meta: Option<PdkMeta>,
}

/// `_meta` -- the deck's self-description block. Apart from `physical_grid`,
/// everything in it is human-readable provenance written by whoever produced the
/// deck (`generated_by`, `techfile`, `metal_scheme`, ...); only the fields actually
/// consumed at run time are declared here, and serde ignores the remaining keys.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PdkMeta {
    /// Whether this deck's `grid` is a **real manufacturing grid**.
    ///
    /// The default (field absent / `null`) is `true`, so a real deck and a deck
    /// hand-written by an external user both get manufacturing-grid snapping with no
    /// change. The `grid` in `pdk/example_lef.json` is a placeholder value (a
    /// non-confidential stand-in, coarser than the real one), so those decks declare
    /// `false` and snapping is switched off entirely -- see `core::mfg_grid`.
    #[serde(rename = "physical_grid", default)]
    pub physical_grid: Option<bool>,
}

/// `device_types` -- the deck's device-class declaration (see `core::device_types`).
///
/// ```jsonc
/// "device_types": {
///   "match": "prefix",           // optional: "prefix" (default) | "exact", both
///                                 // case-insensitive
///   "pmos":      ["pmos_example"],
///   "nmos":      ["nmos_example"],
///   "capacitor": ["cap_example"],
///   "resistor":  ["res_example"],
///   "other":     []              // devices deliberately outside the four classes
///                                 // (standard cells, diodes, ...)
/// }
/// ```
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DeviceTypesSection {
    /// `"prefix"` (the default) or `"exact"`.
    #[serde(rename = "match", default)]
    pub match_mode: Option<String>,
    #[serde(default)]
    pub pmos: Vec<String>,
    #[serde(default)]
    pub nmos: Vec<String>,
    #[serde(default)]
    pub capacitor: Vec<String>,
    #[serde(default)]
    pub resistor: Vec<String>,
    #[serde(default)]
    pub other: Vec<String>,
}

impl PdkRules {
    /// The deck's device-class declaration, as core's [`DeviceTypes`].
    ///
    /// Three states: `Ok(None)` = the deck declares nothing (or declares a section
    /// that lists no model name at all, which is the same thing) -- a deck in that
    /// state is refused by `apply_device_types`; `Ok(Some(_))` = classify by the
    /// declaration; `Err` = the declaration contradicts itself (one model name under
    /// two classes, an empty string), which is a **hard error** rather than a guess.
    pub fn device_types(&self) -> Result<Option<DeviceTypes>, String> {
        let Some(sec) = self.device_types.as_ref() else {
            return Ok(None);
        };
        let dt = DeviceTypes::new(
            DeviceMatch::parse(sec.match_mode.as_deref())?,
            sec.pmos.clone(),
            sec.nmos.clone(),
            sec.capacitor.clone(),
            sec.resistor.clone(),
            sec.other.clone(),
        );
        if dt.is_empty() {
            return Ok(None);
        }
        dt.validate()?;
        Ok(Some(dt))
    }

    /// Whether this deck's `grid` is a real manufacturing grid
    /// (`_meta.physical_grid`, default true).
    pub fn physical_grid(&self) -> bool {
        self.meta
            .as_ref()
            .and_then(|m| m.physical_grid)
            .unwrap_or(true)
    }
}

/// Parses a PDK file (JSON format)
pub fn parse_pdk<P: AsRef<Path>>(filename: P) -> Result<PdkRules, ParseError> {
    let file = File::open(filename)?;
    let pdk: PdkRules = serde_json::from_reader(file)?;
    Ok(pdk)
}

/// Install the deck's `device_types` declaration on the database, and check the netlist
/// against it.
///
/// Call this once per stage, after both the netlist and the deck are loaded. It is the
/// single place the declaration enters the flow; every classification downstream reads
/// it back off the `CircuitDb`.
///
/// Three outcomes:
///
/// * the deck declares nothing → **hard error**. There is no model-name
///   heuristic in this build to fall back to, and guessing is what the
///   declaration exists to stop.
/// * the deck declares and covers the netlist → the declaration is installed and is
///   from then on the only classification rule.
/// * the deck declares and the netlist contains a model it does not list → **hard
///   error** naming every uncovered model. A deck that takes responsibility for the
///   vocabulary must cover it; the alternative is the silent `isPmos = false` that
///   loses the guard-ring reservation without a word (`device_types` module docs).
///   Devices that genuinely belong to none of the four classes go under `other`.
pub fn apply_device_types(db: &mut CircuitDb, pdk: &PdkRules) -> Result<(), ParseError> {
    let declared = pdk.device_types().map_err(ParseError::Parse)?;
    let Some(dt) = declared else {
        return Err(ParseError::Parse(
            "the PDK deck does not declare `device_types`, so there is no way to tell which model names are PMOS, NMOS, capacitors or resistors -- and guessing from the model string is how a design silently gets no guard-ring reservation at all. Declare device_types in your deck, see docs/INPUT_FORMAT.md section 2.4."
                .to_string(),
        ));
    };
    let models = db.device_model_strings();
    let unknown = dt.unknown_models(models.iter().map(|s| s.as_str()));
    if !unknown.is_empty() {
        return Err(ParseError::Parse(format!(
            "the PDK deck declares `device_types`, but {} device model(s) in the netlist \
             are listed under none of its categories: {}. Add each to the right category \
             (pmos / nmos / capacitor / resistor), or to `other` if it is deliberately \
             not one of those. See docs/INPUT_FORMAT.md section 2.4.",
            unknown.len(),
            unknown.join(", ")
        )));
    }
    eprintln!(
        "[pdk] device_types declared: {} model families cover {} netlist model(s)",
        DeviceTypes::ALL_CLASSES
            .iter()
            .map(|c| dt.tokens(*c).len())
            .sum::<usize>(),
        models.len()
    );
    db.set_device_types(Some(dt));
    Ok(())
}

/// SPICE SubCkt structure (used during parsing)
#[derive(Debug, Clone)]
struct SpiceSubCkt {
    name: String,
    pins: Vec<String>,
    entries: Vec<SpiceEntry>,
}

/// SPICE Entry structure (used during parsing)
#[derive(Debug, Clone)]
struct SpiceEntry {
    name: String,
    model: String,
    pins: Vec<String>,
}

/// Strips parentheses from a SPICE line: pin-grouping parentheses `(a b c)` become
/// spaces; a parameter-value parenthesis right after `=`, as in `key=(v)`, loses the
/// parentheses and any whitespace inside the value, so `key=v` stays a single token.
fn strip_grouping_parens(line: &str) -> String {
    let mut out = String::with_capacity(line.len());
    let mut prev_non_space: Option<char> = None;
    let mut in_value_paren = false;
    for ch in line.chars() {
        match ch {
            '(' if !in_value_paren => {
                if prev_non_space == Some('=') {
                    in_value_paren = true;
                } else {
                    out.push(' ');
                    prev_non_space = None;
                }
            }
            ')' => {
                if in_value_paren {
                    in_value_paren = false;
                } else {
                    out.push(' ');
                    prev_non_space = None;
                }
            }
            c if in_value_paren && c.is_whitespace() => {}
            c => {
                out.push(c);
                if !c.is_whitespace() {
                    prev_non_space = Some(c);
                }
            }
        }
    }
    out
}

/// Parses a hierarchical netlist (SPICE format)
pub fn parse_hierarchical_netlist<P: AsRef<Path>>(
    filename: P,
    db: &mut CircuitDb,
) -> Result<(), ParseError> {
    let file = File::open(filename)?;
    let reader = BufReader::new(file);

    let comment_prefixes = ["*", "//"];
    let circuit_headers = [".subckt", ".SUBCKT", "subckt", "SUBCKT", "topckt", "TOPCKT"];
    let circuit_tails = [".ends", ".ENDS", "ends", "ENDS"];

    let mut subckts: Vec<SpiceSubCkt> = Vec::new();
    let mut subckt_flag = false;
    let mut current_subckt: Option<SpiceSubCkt> = None;

    // Normalize physical lines into logical lines.
    // Supported continuations:
    // 1) trailing '\' on previous line
    // 2) leading '+' on continuation line
    let mut logical_lines: Vec<String> = Vec::new();
    let mut current_logical = String::new();
    let mut expect_cont = false;
    for raw in reader.lines() {
        let raw = raw?;
        let line = raw.trim();

        // Skip blank lines and comments
        if line.is_empty()
            || comment_prefixes
                .iter()
                .any(|&prefix| line.starts_with(prefix))
        {
            continue;
        }

        let is_plus_cont = line.starts_with('+');
        let mut body = if is_plus_cont {
            line.trim_start_matches('+').trim_start()
        } else {
            line
        };
        let has_backslash_cont = body.ends_with('\\');
        if has_backslash_cont {
            body = body.trim_end_matches('\\').trim_end();
        }

        if expect_cont || is_plus_cont {
            if !current_logical.is_empty() {
                current_logical.push(' ');
            }
            current_logical.push_str(body);
        } else {
            if !current_logical.is_empty() {
                logical_lines.push(current_logical);
                current_logical = String::new();
            }
            current_logical.push_str(body);
        }

        expect_cont = has_backslash_cont;
        if !expect_cont && !current_logical.is_empty() {
            logical_lines.push(current_logical);
            current_logical = String::new();
        }
    }
    if !current_logical.is_empty() {
        logical_lines.push(current_logical);
    }

    for line in logical_lines {
        // Remove the parentheses. In SPICE they serve two purposes: pin grouping
        // `(a b c)`, which becomes spaces, and parameter values `key=(v)`, where the
        // parentheses go but the value must stay glued to `key=`; otherwise the value
        // becomes a token of its own and the backwards model scan mistakes it for the
        // device type (for example `multi=(1)` would turn a resistor's model into "1").
        let line = strip_grouping_parens(&line);
        let tokens: Vec<&str> = line.split_whitespace().collect();

        if tokens.is_empty() {
            continue;
        }

        // Check whether this is the start of a subckt
        if circuit_headers
            .iter()
            .any(|&h| tokens[0].eq_ignore_ascii_case(h))
        {
            if let Some(subckt) = current_subckt.take() {
                subckts.push(subckt);
            }
            subckt_flag = true;
            if tokens.len() < 2 {
                return Err(ParseError::Parse("Invalid subckt header".to_string()));
            }
            let subckt_name = tokens[1].to_string();
            let pins: Vec<String> = tokens[2..].iter().map(|s| s.to_string()).collect();
            current_subckt = Some(SpiceSubCkt {
                name: subckt_name,
                pins,
                entries: Vec::new(),
            });
        }
        // Check whether this is the end of a subckt
        else if circuit_tails
            .iter()
            .any(|&t| tokens[0].eq_ignore_ascii_case(t))
        {
            subckt_flag = false;
        }
        // Inside a subckt: parse an entry
        else if subckt_flag {
            if let Some(ref mut subckt) = current_subckt {
                if tokens.is_empty() {
                    continue;
                }
                if tokens.len() < 2 {
                    // At least entry_name and model are required
                    continue;
                }

                let entry_name = tokens[0].to_string();

                // Special-case: hierarchical instance in "Xinst subckt pins..." format.
                let is_x_inst = entry_name.starts_with('X') || entry_name.starts_with('x');
                let is_known_subckt =
                    tokens.len() >= 2 && subckts.iter().any(|s| s.name == tokens[1]);
                if is_x_inst && is_known_subckt {
                    let model = tokens[1].to_string();
                    let pins: Vec<String> = if tokens.len() > 2 {
                        tokens[2..]
                            .iter()
                            .filter(|&&s| s != "/" && !s.contains('='))
                            .map(|s| s.to_string())
                            .collect()
                    } else {
                        Vec::new()
                    };
                    subckt.entries.push(SpiceEntry {
                        name: entry_name,
                        model,
                        pins,
                    });
                    continue;
                }

                // Scan backwards for the device type (the last non-parameter token)
                let mut model = String::new();
                let mut net_pos = tokens.len();
                for (idx, token) in tokens.iter().enumerate().rev() {
                    if !token.contains('=') {
                        model = token.to_string();
                        net_pos = idx;
                        break;
                    }
                }

                // If no model was found, use the last token
                if model.is_empty() && !tokens.is_empty() {
                    model = tokens.last().unwrap().to_string();
                    net_pos = tokens.len() - 1;
                }

                // Extract the pins (the tokens between entry_name and model)
                let pins: Vec<String> = if net_pos > 1 {
                    tokens[1..net_pos]
                        .iter()
                        .filter(|&&s| s != "/" && !s.contains('='))
                        .map(|s| s.to_string())
                        .collect()
                } else {
                    Vec::new()
                };

                subckt.entries.push(SpiceEntry {
                    name: entry_name,
                    model,
                    pins,
                });
            }
        }
    }

    // Add the last subckt
    if let Some(subckt) = current_subckt {
        subckts.push(subckt);
    }

    if subckts.is_empty() {
        return Err(ParseError::Parse("No subckt found in netlist".to_string()));
    }

    // The last definition is the design; its pins are the design's ports.
    let topckt = subckts.last().unwrap();
    db.set_name(topckt.name.clone());
    let mut netpins = Vec::new();
    for outnet in &topckt.pins {
        let net_idx = db.allocate_net(outnet.clone());
        if let Some(net) = db.net_mut(net_idx) {
            // Set the net name (without prefix)
            net.set_name(outnet.clone());
        }
        db.add_port(net_idx);
        netpins.push(net_idx);
    }

    // The design's guard-ring pin: a placeholder that belongs to no device and that
    // every consumer skips (`*_GR`). It is allocated before any device pin, so device
    // pin numbering is what it has always been.
    db.allocate_pin(format!("{}_GR", topckt.name));

    flatten_subckt(topckt, &subckts, &topckt.name, &netpins, db);

    // Mark the power nets: the netlist carries no such information, so infer it once
    // from the naming convention and let everything downstream share the same
    // definition.
    db.mark_power_nets_by_name();

    Ok(())
}

/// Flattens one circuit definition into the database.
///
/// Every device becomes a cell named `<prefix>_<device>`. An instance of another
/// definition is expanded in place under the prefix `<prefix>_<instance>`, with its
/// pins bound to the nets the instance line connects; a net that is not one of the
/// definition's pins becomes `<prefix>_<net>`.
fn flatten_subckt(
    subckt: &SpiceSubCkt,
    subckts: &[SpiceSubCkt],
    prefix: &str,
    netpins: &[applace_core::Index],
    db: &mut CircuitDb,
) {
    for entry in &subckt.entries {
        if let Some(template) = subckts.iter().find(|s| s.name == entry.model) {
            // An instance of another definition
            let mut new_netpins = Vec::new();
            for entry_pin in &entry.pins {
                if let Some(pos) = subckt.pins.iter().position(|p| p == entry_pin) {
                    new_netpins.push(netpins[pos]);
                } else {
                    new_netpins.push(internal_net(db, prefix, entry_pin));
                }
            }
            let new_prefix = format!("{}_{}", prefix, entry.name);
            flatten_subckt(template, subckts, &new_prefix, &new_netpins, db);
            continue;
        }

        // A device
        let cell_name = format!("{}_{}", prefix, entry.name);
        let cell_idx = db.allocate_cell(cell_name.clone());
        db.cell_mut(cell_idx)
            .unwrap()
            .set_model(entry.model.clone());

        for (pidx, pin_net_name) in entry.pins.iter().enumerate() {
            let pin_idx = db.allocate_pin(format!("{}_{}", cell_name, pidx));
            db.pin_mut(pin_idx).unwrap().set_cell_idx(Some(cell_idx));
            db.cell_mut(cell_idx).unwrap().add_pin_idx(pin_idx);

            let net_idx = match subckt.pins.iter().position(|p| p == pin_net_name) {
                Some(nidx) => netpins[nidx],
                None => internal_net(db, prefix, pin_net_name),
            };
            db.pin_mut(pin_idx).unwrap().add_net_idx(net_idx);
            if let Some(net) = db.net_mut(net_idx) {
                net.add_pin_idx(pin_idx);
            }
        }
    }
}

/// The net `<prefix>_<name>`, allocated on first use. Its display name is `name`.
fn internal_net(db: &mut CircuitDb, prefix: &str, name: &str) -> applace_core::Index {
    let full = format!("{}_{}", prefix, name);
    if let Some(idx) = db.find_net(&full) {
        return idx;
    }
    let idx = db.allocate_net(full);
    if let Some(net) = db.net_mut(idx) {
        net.set_name(name.to_string());
    }
    idx
}

/// A symmetry constraint
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SymmetryConfig {
    #[serde(flatten)]
    pub blocks: HashMap<String, serde_json::Value>,
}

/// The keys one `sym.json` block may carry in the device flow.
const SYM_BLOCK_KEYS: [&str; 5] = [
    "axis",
    "pairs",
    "children",
    "shareSym",
    "shareAxisWithParent",
];

/// Reads a block's symmetry axis, and refuses anything the loader does not understand.
///
/// The axis is spelled out rather than carried as a boolean, and unknown keys are an
/// error rather than a shrug: in the old flat form a mistyped key looked exactly like a
/// device name, and a mistyped flag looked exactly like a pair.
fn resolve_sym_axis(
    block_name: &str,
    obj: &serde_json::Map<String, serde_json::Value>,
) -> Result<bool, ParseError> {
    for key in obj.keys() {
        if !SYM_BLOCK_KEYS.contains(&key.as_str()) {
            return Err(ParseError::Parse(format!(
                "sym.json block {block_name}: unknown key `{key}`. A block is \
                 {{\"axis\": \"vertical\"|\"horizontal\", \"pairs\": {{...}}}}, and may \
                 also carry `children`, `shareSym` and `shareAxisWithParent`."
            )));
        }
    }
    match obj.get("axis").map(|v| v.as_str()) {
        None => Ok(true),
        Some(Some("vertical")) => Ok(true),
        Some(Some("horizontal")) => Ok(false),
        Some(other) => Err(ParseError::Parse(format!(
            "sym.json block {block_name}: axis is `{}`, want `vertical` or `horizontal`",
            other.unwrap_or("not a string")
        ))),
    }
}

/// A block's symmetry pairs, sorted so that the order cannot depend on the map.
fn resolve_sym_pair_map(
    block_name: &str,
    obj: &serde_json::Map<String, serde_json::Value>,
) -> Result<Vec<(String, String)>, ParseError> {
    let Some(pairs_obj) = obj.get("pairs").and_then(|v| v.as_object()) else {
        return Err(ParseError::Parse(format!(
            "sym.json block {block_name} has no `pairs` object"
        )));
    };
    let mut pairs: Vec<(String, String)> = pairs_obj
        .iter()
        .filter_map(|(k, v)| v.as_str().map(|s| (k.clone(), s.to_string())))
        .collect();
    pairs.sort_by(|(lk, lv), (rk, rv)| lk.cmp(rk).then_with(|| lv.cmp(rv)));
    Ok(pairs)
}

/// The word `sym.json` uses for an axis orientation.
fn axis_word(is_vert: bool) -> &'static str {
    if is_vert {
        "vertical"
    } else {
        "horizontal"
    }
}

/// Outcome of matching one `sym.json` token (`M16`) against the design's cells.
///
/// `Ambiguous` used to be folded into "not found" (with a trace-only note), which is how
/// a symmetry constraint could silently bind nothing — or, in the sibling short-name map
/// in `placer/src/data.rs`, bind an arbitrary one of the candidates. Both sides now
/// refuse to guess.
#[derive(Debug)]
enum TokenResolution {
    Found(applace_core::Index),
    NotFound,
    Ambiguous(Vec<String>),
}

/// A cell's short name: the part after its last `_`, the same rule the abstract loader
/// uses (`full.rsplit('_').next()`, `placer/src/data.rs`).
fn short_name(name: &str) -> &str {
    name.rsplit('_').next().unwrap_or(name)
}

fn resolve_device_token(db: &CircuitDb, token: &str) -> TokenResolution {
    let prefix = format!("{}_", db.name());
    let token_lc = token.to_ascii_lowercase();

    // Pass 1: matched either by full name or by "<design>_" + token.
    let mut direct: Vec<applace_core::Index> = Vec::new();
    for cell in db.cells() {
        let cname = cell.name();
        let hit = cname.eq_ignore_ascii_case(token)
            || cname
                .strip_prefix(&prefix)
                .is_some_and(|remain| remain.eq_ignore_ascii_case(&token_lc));
        if hit && !direct.contains(&cell.idx()) {
            direct.push(cell.idx());
        }
    }
    match direct.len() {
        1 => return TokenResolution::Found(direct[0]),
        0 => {}
        _ => return TokenResolution::Ambiguous(cell_names(db, &direct)),
    }

    // Pass 2: matched by short name.
    let candidates = short_name_candidates(db, token);
    match candidates.len() {
        1 => TokenResolution::Found(candidates[0]),
        0 => TokenResolution::NotFound,
        _ => TokenResolution::Ambiguous(cell_names(db, &candidates)),
    }
}

/// Every cell whose full or short name is `token`, in index order.
fn short_name_candidates(db: &CircuitDb, token: &str) -> Vec<applace_core::Index> {
    db.cells()
        .iter()
        .filter(|c| {
            c.name().eq_ignore_ascii_case(token) || short_name(c.name()).eq_ignore_ascii_case(token)
        })
        .map(|c| c.idx())
        .collect()
}

fn cell_names(db: &CircuitDb, idxs: &[applace_core::Index]) -> Vec<String> {
    idxs.iter()
        .map(|&i| {
            db.cell(i)
                .map(|c| c.name().to_string())
                .unwrap_or_else(|| format!("#{i}"))
        })
        .collect()
}

/// Which symmetry group a `sym.json` block is written into.
///
/// The first block of a `parse_symmetry` call replaces whatever groups the database
/// held and becomes the first group; every further block appends a group of its own.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SymSlot {
    First,
    Next,
}

fn apply_sym_block(
    db: &mut CircuitDb,
    is_vert: bool,
    pairs: &[(String, String)],
    block_name: &str,
    slot: SymSlot,
) -> Result<usize, ParseError> {
    if slot == SymSlot::First {
        db.sym_groups_mut().clear();
    } else {
        // Every group ends up on the design's single legalized axis, so they must at
        // least agree on the axis orientation.
        let first_is_vert = db
            .sym_groups()
            .first()
            .map(|g| g.is_vert())
            .unwrap_or(is_vert);
        if first_is_vert != is_vert {
            return Err(ParseError::Parse(format!(
                "sym.json block {block_name} asks for a {} axis but shares the \
                 design {} with a block that asks for a {} axis; the legalizer \
                 resolves one axis per design and cannot honour both.",
                axis_word(is_vert),
                db.name(),
                axis_word(first_is_vert)
            )));
        }
    }
    let mut group = SymGroup::new(is_vert);
    let design = db.name().to_string();

    let mut added = 0usize;
    let mut missing = 0usize;
    let mut missing_tokens: Vec<String> = Vec::new();
    for (lhs, rhs) in pairs {
        let lidx = resolve_device_token(db, lhs);
        let ridx = resolve_device_token(db, rhs);
        for (token, res) in [(lhs, &lidx), (rhs, &ridx)] {
            if let TokenResolution::Ambiguous(names) = res {
                // Picking one of them would bind the symmetry constraint to an arbitrary
                // device. Same ruling as the abstract loader's short-name map (§5.3).
                return Err(ParseError::Parse(format!(
                    "sym.json block {block_name}: token {token} matches {} devices in \
                     design {design}, so the constraint would bind an arbitrary one of \
                     them. Rename the devices or spell the token as a full instance name. \
                     Candidates: {}",
                    names.len(),
                    names.join(", ")
                )));
            }
        }
        match (&lidx, &ridx) {
            (TokenResolution::Found(l), TokenResolution::Found(r)) if l == r => {
                group.add_self_sym(*l);
                added += 1;
            }
            (TokenResolution::Found(l), TokenResolution::Found(r)) => {
                group.add_sym_pair(*l, *r);
                added += 1;
            }
            _ => {
                missing += 1;
                if matches!(
                    std::env::var("HIER_STAGE_TRACE").ok().as_deref(),
                    Some("1" | "true" | "TRUE" | "yes" | "YES")
                ) {
                    let lc = short_name_candidates(db, lhs).len();
                    let rc = short_name_candidates(db, rhs).len();
                    missing_tokens.push(format!("{lhs}->{rhs} (cands {lc}/{rc})"));
                }
            }
        }
    }
    db.add_sym_group(group);
    if matches!(
        std::env::var("HIER_STAGE_TRACE").ok().as_deref(),
        Some("1" | "true" | "TRUE" | "yes" | "YES")
    ) {
        eprintln!(
            "[sym] Applied symmetry block {} to {}: added {} entries (missing {})",
            block_name, design, added, missing
        );
        if !missing_tokens.is_empty() {
            let preview: Vec<String> = missing_tokens.into_iter().take(12).collect();
            eprintln!(
                "[sym] missing examples for {}: {}",
                block_name,
                preview.join(", ")
            );
        }
    }
    Ok(added)
}

/// Parses the symmetry constraint file and applies it to the db
pub fn parse_symmetry<P: AsRef<Path>>(
    filename: P,
    db: &mut CircuitDb,
) -> Result<SymmetryConfig, ParseError> {
    let file = File::open(filename)?;
    let config: SymmetryConfig = serde_json::from_reader(file)?;

    let mut blocks: Vec<(&String, &serde_json::Value)> = config.blocks.iter().collect();
    blocks.sort_by_key(|(ln, _)| *ln);
    // The first block that binds replaces the database's groups; later ones append.
    let mut first = true;
    for (block_name, block_val) in blocks {
        let Some(obj) = block_val.as_object() else {
            continue;
        };
        let is_vert = resolve_sym_axis(block_name, obj)?;
        let pairs = resolve_sym_pair_map(block_name, obj)?;
        if pairs.is_empty() {
            continue;
        }

        // A block named after the design binds its tokens to the design's devices. Any
        // other block names a group of devices that may have no circuit definition of
        // its own (it can exist only inside sym.json), and its tokens are matched by
        // device *short name* -- the same `full.rsplit('_').next()` rule the abstract
        // loader uses (`placer/src/data.rs::build_short_to_idx`). Ambiguity is a hard
        // error on both sides; nobody guesses.
        let names_design = db.name().eq_ignore_ascii_case(block_name);
        if matches!(
            std::env::var("HIER_STAGE_TRACE").ok().as_deref(),
            Some("1" | "true" | "TRUE" | "yes" | "YES")
        ) {
            eprintln!(
                "[sym] block {}: {} pair-entries, names the design: {}",
                block_name,
                pairs.len(),
                names_design
            );
        }
        if !names_design && obj.get("shareAxisWithParent").and_then(|v| v.as_bool()) == Some(false)
        {
            // LCR resolves exactly one axis for the design, which is what
            // `shareAxisWithParent: true` asks for. A block that asks for its own axis
            // cannot be honoured, so say so instead of quietly snapping it onto the
            // design's axis.
            return Err(ParseError::Parse(format!(
                "sym.json block {block_name} is not the design ({}) and declares \
                 shareAxisWithParent=false, but the legalizer resolves a single \
                 symmetry axis for the design. Set shareAxisWithParent=true.",
                db.name()
            )));
        }
        let slot = if first { SymSlot::First } else { SymSlot::Next };
        first = false;
        let added = apply_sym_block(db, is_vert, &pairs, block_name, slot)?;
        if !names_design {
            // Unconditional (not trace-gated): this used to be a silent drop, and the
            // count is the only thing that says whether the block actually bound
            // anything.
            eprintln!(
                "[sym] block {} is not the design; matched {} of {} pair-entries \
                 by device name",
                block_name,
                added,
                pairs.len()
            );
        }
    }

    Ok(config)
}

/// An electrical constraint
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectricalConfig {
    #[serde(flatten)]
    pub constraints: HashMap<String, serde_json::Value>,
}

/// Parses the electrical constraint file
pub fn parse_electrical_constraints<P: AsRef<Path>>(
    filename: P,
    _db: &mut CircuitDb,
) -> Result<ElectricalConfig, ParseError> {
    let file = File::open(filename)?;
    let config: ElectricalConfig = serde_json::from_reader(file)?;
    // TODO: apply the electrical constraints to the db
    Ok(config)
}

/// An array constraint
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArrayConfig {
    pub array: HashMap<String, serde_json::Value>,
    #[serde(rename = "cap_array")]
    pub cap_array: HashMap<String, serde_json::Value>,
}

/// Parses the array constraint file
pub fn parse_array_constraints<P: AsRef<Path>>(
    filename: P,
    _db: &mut CircuitDb,
) -> Result<ArrayConfig, ParseError> {
    let file = File::open(filename)?;
    let config: ArrayConfig = serde_json::from_reader(file)?;
    // TODO: apply the array constraints to the db
    Ok(config)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    /// The example design -- a netlist and one layout file per device -- is not
    /// committed to this repository; see docs/INPUT_FORMAT.md. Tests that need it
    /// report a skip instead of failing on a fresh clone. Where the inputs are
    /// present, every one of these tests runs in full.
    fn example_design_present() -> bool {
        let ok = get_test_file_path("ota2_hier.sp").is_file()
            && get_test_file_path("ota2_raw.sp").is_file()
            && get_test_pcell_dir().is_dir();
        if !ok {
            eprintln!("[skip] example design inputs are not present (docs/INPUT_FORMAT.md)");
        }
        ok
    }

    // Resolve the path of a test file (relative to the project root)
    fn get_test_file_path(filename: &str) -> PathBuf {
        // Take the io directory from CARGO_MANIFEST_DIR, then go up two levels to the
        // project root
        let manifest_dir = env!("CARGO_MANIFEST_DIR");
        let io_dir = PathBuf::from(manifest_dir);
        let project_root = io_dir.parent().unwrap().parent().unwrap();
        let golden = project_root
            .join("src")
            .join("io")
            .join("golden")
            .join("example")
            .join(filename);
        if golden.exists() {
            return golden;
        }
        project_root
            .join("benchmark")
            .join("ota2")
            .join("netlist")
            .join(filename)
    }

    #[test]
    fn test_parse_netlist_handles_backslash_continuation_params() {
        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("time")
            .as_nanos();
        let path = std::env::temp_dir().join(format!("applace_cont_line_{stamp}.sp"));
        let netlist = r#"
topckt T A B VSS
C7 (A B VSS) cap_example c=1.2f l=1u w=100n nf=4 multi=1 \
    tailparam=1.5n endflag=0
ends T
"#;
        fs::write(&path, netlist).expect("write temp netlist");

        let mut db = CircuitDb::new();
        parse_hierarchical_netlist(&path, &mut db).expect("parse should succeed");
        let _ = fs::remove_file(&path);

        let mut has_c7 = false;
        let mut has_bad_name = false;
        let mut has_bad_model = false;
        for idx in 0..db.num_cells() {
            if let Some(c) = db.cell(idx as applace_core::Index) {
                if c.name().contains("C7") {
                    has_c7 = true;
                }
                if c.name().contains("tailparam=") {
                    has_bad_name = true;
                }
                if c.model().contains("endflag=") {
                    has_bad_model = true;
                }
            }
        }
        assert!(has_c7, "expected to parse C7 cell from continued line");
        assert!(
            !has_bad_name,
            "continued parameter should not become cell name"
        );
        assert!(
            !has_bad_model,
            "continued parameter should not become model"
        );
    }

    #[test]
    fn test_strip_grouping_parens_keeps_value_parens_glued() {
        assert_eq!(
            strip_grouping_parens("R4 (N1 N2 VSS ) res_example l=10u multi=(1) m=1"),
            "R4  N1 N2 VSS   res_example l=10u multi=1 m=1"
        );
        // Whitespace inside the value parentheses is stripped too, keeping one token.
        assert_eq!(strip_grouping_parens("k=( 2 )"), "k=2");
        // Plain pin-grouping parentheses are still replaced by spaces.
        assert_eq!(strip_grouping_parens("(a b)"), " a b ");
    }

    #[test]
    fn test_parse_netlist_resistor_model_with_paren_value_param() {
        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("time")
            .as_nanos();
        let path = std::env::temp_dir().join(format!("applace_res_dtype_{stamp}.sp"));
        let netlist = r#"
topckt T A B VSS
R4 (A B VSS ) res_example l=10u w=400n multi=(1) m=1 series=10 spacing=250n
ends T
"#;
        fs::write(&path, netlist).expect("write temp netlist");

        let mut db = CircuitDb::new();
        parse_hierarchical_netlist(&path, &mut db).expect("parse should succeed");
        let _ = fs::remove_file(&path);

        let mut model = None;
        for idx in 0..db.num_cells() {
            if let Some(c) = db.cell(idx as applace_core::Index) {
                if c.name().contains("R4") {
                    model = Some(c.model().to_string());
                }
            }
        }
        assert_eq!(
            model.as_deref(),
            Some("res_example"),
            "resistor model must be device type, not a paren-value fragment"
        );
    }

    /// Classifies a device by the SPICE element letter of its instance name: `M` is a
    /// transistor, `R` a resistor, `C` a capacitor. That letter is part of the netlist
    /// format, so the classification says nothing about any particular process kit --
    /// which matters here because the design these tests read is not distributed with
    /// this repository and its model names are not reproduced.
    fn spice_element(name: &str) -> Option<char> {
        name.rsplit('_')
            .next()
            .and_then(|s| s.chars().next())
            .map(|c| c.to_ascii_uppercase())
    }

    // Dump netlist information for manual inspection
    fn dump_netlist_info(db: &CircuitDb, test_name: &str) {
        println!("\n========== Netlist Dump: {} ==========", test_name);
        println!("Design: {}", db.name());
        println!("Total Cells: {}", db.num_cells());
        println!("Total Nets: {}", db.num_nets());
        println!("Total Pins: {}", db.num_pins());
        println!("Ports: {}", db.ports().len());
        for cell in db.cells().iter().take(20) {
            println!(
                "  [C] {} (type: {}, pins: {})",
                cell.name(),
                cell.model(),
                cell.num_pins()
            );
        }
        if db.num_cells() > 20 {
            println!("  ... and {} more cells", db.num_cells() - 20);
        }
        println!("==========================================\n");
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_ota2_hier() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();
        let file_path = get_test_file_path("ota2_hier.sp");
        let result = parse_hierarchical_netlist(&file_path, &mut db);

        // Parsing should succeed
        assert!(
            result.is_ok(),
            "Failed to parse ota2_hier.sp: {:?}",
            result.err()
        );

        // Dump netlist information for manual inspection (use --nocapture to see it)
        dump_netlist_info(&db, "test_parse_ota2_hier");

        // The last definition is the design
        assert_eq!(db.name(), "OTA_2");

        // Instances of other definitions are expanded in place, under their instance name
        assert!(
            db.cells().iter().any(|c| c.name().starts_with("OTA_2_X")),
            "Expected devices expanded from subcircuit instances (OTA_2_X...)"
        );

        assert!(
            db.num_cells() > 0,
            "Expected cells, found {}",
            db.num_cells()
        );
        assert!(db.num_nets() > 0, "Expected nets, found {}", db.num_nets());
        assert!(db.num_pins() > 0, "Expected pins, found {}", db.num_pins());
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_ota2_raw() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();
        let file_path = get_test_file_path("ota2_raw.sp");
        let result = parse_hierarchical_netlist(&file_path, &mut db);

        // Parsing should succeed
        assert!(
            result.is_ok(),
            "Failed to parse ota2_raw.sp: {:?}",
            result.err()
        );
        assert_eq!(db.name(), "OTA_2");
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_ota2_hier_nets() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        // Check that some key nets exist
        let expected_nets = vec!["GND", "VDD", "VIM", "VIP", "VOM", "VOP"];

        for net_name in expected_nets {
            let found = db.find_net(net_name).is_some();
            assert!(found, "Net {} not found", net_name);
        }
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_ota2_hier_cells() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        // Check that there are MOS transistor cells
        let mut mos_cells = Vec::new();
        for idx in 0..db.num_cells() {
            if let Some(c) = db.cell(idx as applace_core::Index) {
                if spice_element(c.name()) == Some('M') {
                    mos_cells.push(c.name().to_string());
                }
            }
        }

        assert!(
            !mos_cells.is_empty(),
            "Expected MOS cells, found {}",
            mos_cells.len()
        );

        // Check that there are resistor and capacitor cells
        let mut passive_cells = Vec::new();
        for idx in 0..db.num_cells() {
            if let Some(c) = db.cell(idx as applace_core::Index) {
                if matches!(spice_element(c.name()), Some('R') | Some('C')) {
                    passive_cells.push(c.name().to_string());
                }
            }
        }

        assert!(
            !passive_cells.is_empty(),
            "Expected passive cells (R/C), found {}",
            passive_cells.len()
        );
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_ota2_hier_detailed() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        println!("Total Cells: {}", db.num_cells());
        println!("Total Nets: {}", db.num_nets());
        println!("Total Pins: {}", db.num_pins());

        // OTA_2's eight pins are the design's ports
        println!("OTA_2 ports: {}", db.ports().len());
        assert!(
            db.ports().len() >= 8,
            "OTA_2 should have at least 8 ports, found {}",
            db.ports().len()
        );
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_ota2_raw_vs_hier() {
        if !example_design_present() {
            return;
        }

        // Both fixtures describe the same design, one with subcircuits and one without
        let mut db_hier = CircuitDb::new();
        let file_path_hier = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path_hier, &mut db_hier).unwrap();

        let mut db_raw = CircuitDb::new();
        let file_path_raw = get_test_file_path("ota2_raw.sp");
        parse_hierarchical_netlist(&file_path_raw, &mut db_raw).unwrap();

        assert_eq!(db_hier.name(), "OTA_2");
        assert_eq!(db_raw.name(), "OTA_2");
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_pin_net_connections() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        // Check that every pin connects to at least one net (guardring pins aside)
        for idx in 0..db.num_pins() {
            let pin = db.pin(idx as applace_core::Index).unwrap();
            // Guardring pins (GR) may connect to no net, which is normal
            if pin.name().ends_with("_GR") {
                continue;
            }
            assert!(
                !pin.net_ids().is_empty(),
                "Pin {} should be connected to at least one net",
                pin.name()
            );
        }

        // Check that every net contains at least one pin
        for idx in 0..db.num_nets() {
            let net = db.net(idx as applace_core::Index).unwrap();
            assert!(
                net.num_pins() > 0,
                "Net {} should contain at least one pin",
                net.name()
            );
        }

        // Check that the two-way link is consistent: if a pin connects to a net, the net must contain that pin
        for pin_idx in 0..db.num_pins() {
            let pin = db.pin(pin_idx as applace_core::Index).unwrap();
            // Skip guardring pins
            if pin.name().ends_with("_GR") {
                continue;
            }
            for &net_idx in pin.net_ids() {
                let net = db.net(net_idx).unwrap();
                assert!(
                    net.pin_ids().contains(&(pin_idx as applace_core::Index)),
                    "Net {} should contain pin {}, but it doesn't",
                    net.name(),
                    pin.name()
                );
            }
        }

        // Check the reverse direction: if a net contains a pin, the pin must connect to that net
        for net_idx in 0..db.num_nets() {
            let net = db.net(net_idx as applace_core::Index).unwrap();
            for &pin_idx in net.pin_ids() {
                let pin = db.pin(pin_idx).unwrap();
                // Skip guardring pins
                if pin.name().ends_with("_GR") {
                    continue;
                }
                assert!(
                    pin.net_ids().contains(&(net_idx as applace_core::Index)),
                    "Pin {} should be connected to net {}, but it isn't",
                    pin.name(),
                    net.name()
                );
            }
        }
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_cell_pin_connections() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        // Check that every cell has pins
        for idx in 0..db.num_cells() {
            if let Some(cell) = db.cell(idx as applace_core::Index) {
                assert!(
                    cell.num_pins() > 0,
                    "Cell {} should have at least one pin",
                    cell.name()
                );

                // Check that every pin is correctly associated with its cell
                for &pin_idx in cell.pin_ids() {
                    let pin = db.pin(pin_idx).unwrap();
                    assert_eq!(
                        pin.cell_idx(),
                        Some(idx as applace_core::Index),
                        "Pin {} should belong to cell {}",
                        pin.name(),
                        cell.name()
                    );
                }
            }
        }
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_net_name_uniqueness() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        // Check that net names are unique
        let mut net_names = std::collections::HashSet::new();
        for idx in 0..db.num_nets() {
            let net = db.net(idx as applace_core::Index).unwrap();
            let name = net.name().to_string();

            // Note: internal nets may share a name (at different hierarchy levels), but top-level nets should be unique
            // Here we only check that find_net finds the right net
            if let Some(found_idx) = db.find_net(net.name()) {
                // If one is found, it should be the same net, or a net with the same name at a different level
                // For top-level nets it should be unique
                if idx == 0 || idx < 10 {
                    // Check the first few nets (usually the top-level IO nets)
                    assert!(
                        !net_names.contains(&name) || found_idx == idx as applace_core::Index,
                        "Top-level net {} should be unique or find_net should return correct index",
                        name
                    );
                }
            }
            net_names.insert(name);
        }
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_design_ports() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        // The design's ports are the top definition's pins
        let ports = db.ports();
        assert!(ports.len() >= 8, "OTA_2 should have at least 8 ports");

        // Every port exists
        for &net_idx in ports {
            assert!(
                db.net(net_idx).is_some(),
                "Port at index {} should exist",
                net_idx
            );
        }

        // Every expected IO net is a port
        let expected_io_nets = vec!["GND", "VDD", "VIM", "VIP", "VOM", "VOP", "IBIAS", "VCM"];
        for io_net_name in expected_io_nets {
            if let Some(net_idx) = db.find_net(io_net_name) {
                assert!(
                    db.is_port(net_idx),
                    "IO net {} should be a port",
                    io_net_name
                );
            }
        }
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_cell_pin_count_consistency() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        // Check that each cell has as many pins as it has net connections
        for idx in 0..db.num_cells() {
            if let Some(cell) = db.cell(idx as applace_core::Index) {
                let pin_count = cell.num_pins();

                // Check that every pin connects to a net
                let mut connected_pin_count = 0;
                for &pin_idx in cell.pin_ids() {
                    if let Some(pin) = db.pin(pin_idx) {
                        if !pin.net_ids().is_empty() {
                            connected_pin_count += 1;
                        }
                    }
                }

                // Most pins should connect to nets (apart from possibly unconnected ones)
                assert!(
                    connected_pin_count > 0,
                    "Cell {} should have at least one connected pin",
                    cell.name()
                );

                // Check that the pin count is plausible (at least 2, no more than 10; a MOS transistor usually has 4)
                if spice_element(cell.name()) == Some('M') {
                    assert!(
                        pin_count >= 4,
                        "MOS cell {} should have at least 4 pins (G, D, S, B), found {}",
                        cell.name(),
                        pin_count
                    );
                }
            }
        }
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_net_pin_consistency() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        // Check that each net's pin count matches the number of pins connected to it
        for net_idx in 0..db.num_nets() {
            let net = db.net(net_idx as applace_core::Index).unwrap();
            let pin_ids_in_net = net.pin_ids();

            // Check that every pin really connects to this net
            for &pin_idx in pin_ids_in_net {
                let pin = db.pin(pin_idx).unwrap();
                assert!(
                    pin.net_ids().contains(&(net_idx as applace_core::Index)),
                    "Pin {} should be connected to net {}",
                    pin.name(),
                    net.name()
                );
            }

            // Check that a net has at least one pin (barring special cases)
            assert!(
                !pin_ids_in_net.is_empty(),
                "Net {} should have at least one pin",
                net.name()
            );
        }
    }

    // Resolve the path of the test pcell directory
    fn get_test_pcell_dir() -> PathBuf {
        let manifest_dir = env!("CARGO_MANIFEST_DIR");
        let io_dir = PathBuf::from(manifest_dir);
        let project_root = io_dir.parent().unwrap().parent().unwrap();
        let golden = project_root
            .join("src")
            .join("io")
            .join("golden")
            .join("example")
            .join("custom");
        if golden.exists() {
            return golden;
        }
        project_root.join("benchmark").join("ota2").join("custom")
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_pcell_basic() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();

        // Parse the netlist first
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        // Test parse_pcell (it should find some GDS files)
        let pcelldir = get_test_pcell_dir();
        if pcelldir.exists() {
            let result = parse_pcell(&pcelldir, None::<&Path>, None, &mut db);
            // It should succeed: even when no GDS is actually parsed, it must not error
            assert!(
                result.is_ok(),
                "parse_pcell should succeed: {:?}",
                result.err()
            );

            // Check that at least some cells were processed
            let mut cell_count = 0;
            for idx in 0..db.num_cells() {
                if db.cell(idx as applace_core::Index).is_some() {
                    cell_count += 1;
                }
            }
            assert!(cell_count > 0, "Should have at least one cell");
        } else {
            println!(
                "Skipping test: pcell directory not found at {}",
                pcelldir.display()
            );
        }
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_pcell_with_stdcell() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();

        // Parse the netlist first
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        // Test parse_pcell with a stdcell config (an empty one)
        let pcelldir = get_test_pcell_dir();
        if pcelldir.exists() {
            // Create an empty stdcell config
            let stdcell_config = r#"{}"#;
            let stdcell_path = std::env::temp_dir().join("test_stdcell.json");
            std::fs::write(&stdcell_path, stdcell_config).unwrap();

            let result = parse_pcell(&pcelldir, Some(&stdcell_path), None, &mut db);
            assert!(
                result.is_ok(),
                "parse_pcell with stdcell should succeed: {:?}",
                result.err()
            );

            // Clean up the temporary file
            let _ = std::fs::remove_file(&stdcell_path);
        } else {
            println!(
                "Skipping test: pcell directory not found at {}",
                pcelldir.display()
            );
        }
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_pcell_ota2() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();

        // Parse the netlist first
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        // Test parse_pcell against the example design
        let pcelldir = get_test_pcell_dir();
        if pcelldir.exists() {
            println!("Testing parse_pcell against the example design...");
            println!("PCell directory: {}", pcelldir.display());

            // Count the cells before parsing
            let mut cell_count_before = 0;
            for idx in 0..db.num_cells() {
                if let Some(cell) = db.cell(idx as applace_core::Index) {
                    cell_count_before += 1;
                    println!(
                        "  Cell before: {} (type: {}, pins: {})",
                        cell.name(),
                        cell.model(),
                        cell.num_pins()
                    );
                }
            }

            let result = parse_pcell(&pcelldir, None::<&Path>, None, &mut db);
            assert!(
                result.is_ok(),
                "parse_pcell should succeed: {:?}",
                result.err()
            );

            // Check that the cells are still there
            let mut cell_count_after = 0;
            for idx in 0..db.num_cells() {
                if let Some(_cell) = db.cell(idx as applace_core::Index) {
                    cell_count_after += 1;
                }
            }
            assert_eq!(
                cell_count_before, cell_count_after,
                "Cell count should not change after parse_pcell"
            );

            println!("  Processed {} cells", cell_count_after);
        } else {
            println!(
                "Skipping test: pcell directory not found at {}",
                pcelldir.display()
            );
        }
    }

    #[test]
    #[ignore = "requires the example design -- a netlist and one vendor PCell layout per device -- which is not distributed with this repository. Run with `cargo test -- --ignored` where those inputs are present."]
    fn test_parse_pcell_golden() {
        if !example_design_present() {
            return;
        }

        let mut db = CircuitDb::new();

        // Parse the netlist first
        let file_path = get_test_file_path("ota2_hier.sp");
        parse_hierarchical_netlist(&file_path, &mut db).unwrap();

        let pcelldir = get_test_pcell_dir();
        if !pcelldir.exists() {
            println!(
                "Skipping test: pcell directory not found at {}",
                pcelldir.display()
            );
            return;
        }

        let manifest_dir = env!("CARGO_MANIFEST_DIR");
        let io_dir = PathBuf::from(manifest_dir);
        let project_root = io_dir.parent().unwrap().parent().unwrap();
        let pdk_path = project_root.join("pdk").join("example_lef.json");

        let out = parse_pcell(&pcelldir, None::<&Path>, Some(&pdk_path), &mut db).unwrap();

        let golden_bbox_path = project_root
            .join("src")
            .join("io")
            .join("golden")
            .join("bbox.json");
        let golden_pin_path = project_root
            .join("src")
            .join("io")
            .join("golden")
            .join("pin.json");

        if !golden_bbox_path.exists() || !golden_pin_path.exists() {
            println!("Skipping test: golden bbox/pin not found");
            return;
        }

        let mut golden_bbox: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&golden_bbox_path).unwrap()).unwrap();
        let mut golden_pin: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&golden_pin_path).unwrap()).unwrap();

        let mut out_bbox = serde_json::to_value(&out.bbox).unwrap();
        let mut out_pin = serde_json::to_value(&out.pins).unwrap();

        fn sort_bbox(v: &mut serde_json::Value) {
            if let Some(cells) = v.get_mut("cells").and_then(|c| c.as_array_mut()) {
                cells.sort_by(|a, b| {
                    let an = a.get("cell_name").and_then(|x| x.as_str()).unwrap_or("");
                    let bn = b.get("cell_name").and_then(|x| x.as_str()).unwrap_or("");
                    an.cmp(bn)
                });
            }
        }

        fn sort_pins(v: &mut serde_json::Value) {
            if let Some(pins) = v.get_mut("pins").and_then(|p| p.as_array_mut()) {
                pins.sort_by(|a, b| {
                    let an = a.get("name").and_then(|x| x.as_str()).unwrap_or("");
                    let bn = b.get("name").and_then(|x| x.as_str()).unwrap_or("");
                    an.cmp(bn)
                });
            }
        }

        sort_bbox(&mut out_bbox);
        sort_bbox(&mut golden_bbox);
        sort_pins(&mut out_pin);
        sort_pins(&mut golden_pin);

        let out_bbox_cells = out_bbox
            .get("cells")
            .and_then(|v| v.as_array())
            .map(|v| v.len())
            .unwrap_or(0);
        let golden_bbox_cells = golden_bbox
            .get("cells")
            .and_then(|v| v.as_array())
            .map(|v| v.len())
            .unwrap_or(0);
        assert!(
            out_bbox_cells > 0 && golden_bbox_cells > 0,
            "bbox golden coverage should be non-empty"
        );

        let out_pin_count = out_pin
            .get("pins")
            .and_then(|v| v.as_array())
            .map(|v| v.len())
            .unwrap_or(0);
        let golden_pin_count = golden_pin
            .get("pins")
            .and_then(|v| v.as_array())
            .map(|v| v.len())
            .unwrap_or(0);
        assert!(
            out_pin_count > 0 && golden_pin_count > 0,
            "pin golden coverage should be non-empty"
        );
    }

    #[test]
    fn test_parse_symmetry_applies_pairs_to_db() {
        let mut db = CircuitDb::new();
        let _m = design(&mut db, &["A", "B", "A_SELF"]);

        let tmp = std::env::temp_dir().join("sym_apply_test.json");
        std::fs::write(
            &tmp,
            r#"{
  "TOP": {
    "axis": "vertical",
    "pairs": {
      "A": "B",
      "A_SELF": "A_SELF"
    }
  }
}"#,
        )
        .unwrap();

        parse_symmetry(&tmp, &mut db).unwrap();
        let groups = db.sym_groups();
        assert_eq!(groups.len(), 1);
        assert!(groups[0].is_vert());
        assert_eq!(groups[0].sym_pairs().len(), 1);
        assert_eq!(groups[0].self_syms().len(), 1);
        let _ = std::fs::remove_file(&tmp);
    }

    #[test]
    fn test_parse_symmetry_sorts_pair_application_order() {
        let mut db = CircuitDb::new();
        let m = design(&mut db, &["A", "B", "C", "D"]);

        let tmp = std::env::temp_dir().join("sym_apply_order_test.json");
        std::fs::write(
            &tmp,
            r#"{
  "TOP": {
    "axis": "vertical",
    "pairs": {
      "C": "D",
      "A": "B"
    }
  }
}"#,
        )
        .unwrap();

        parse_symmetry(&tmp, &mut db).unwrap();
        // A sym.json with one block makes exactly one group.
        assert_eq!(db.sym_groups().len(), 1);
        assert_eq!(
            db.sym_groups()[0].sym_pairs(),
            &[(m["A"], m["B"]), (m["C"], m["D"])]
        );

        let _ = std::fs::remove_file(&tmp);
    }

    /// A design named TOP whose cells have the given short names.
    /// Returns short name -> cell idx.
    fn design(db: &mut CircuitDb, shorts: &[&str]) -> HashMap<String, applace_core::Index> {
        db.set_name("TOP".to_string());
        let mut map = HashMap::new();
        for s in shorts {
            let idx = db.allocate_cell(format!("TOP_{s}"));
            map.insert((*s).to_string(), idx);
        }
        map
    }

    fn write_tmp(name: &str, body: &str) -> PathBuf {
        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("time")
            .as_nanos();
        let p = std::env::temp_dir().join(format!("{name}_{stamp}.json"));
        std::fs::write(&p, body).unwrap();
        p
    }

    /// Devices that have to be symmetric stay symmetric after flattening.
    /// A sym.json block that names a group of devices with no circuit definition of its
    /// own must still bind its pairs, by device short name, as a group of its own.
    #[test]
    fn test_parse_symmetry_flattened_block_binds_by_device_short_name() {
        let mut db = CircuitDb::new();
        let m = design(&mut db, &["M1", "M2", "M3", "M4", "M9"]);

        let tmp = write_tmp(
            "sym_flat_fallback",
            r#"{
  "TOP":      { "axis": "vertical", "pairs": { "M1": "M2", "M9": "M9" } },
  "SUB_BLOCK":{ "axis": "vertical", "shareAxisWithParent": true, "pairs": { "M3": "M4" } }
}"#,
        );
        parse_symmetry(&tmp, &mut db).unwrap();

        // One group per sym.json block, exactly like the abstract loader -- and in sorted
        // block-name order ("SUB_BLOCK" < "TOP"), which is the determinism contract. Which
        // block comes first carries no extra meaning: the legalizer merges every group
        // onto the design's one axis.
        let groups = db.sym_groups();
        assert_eq!(groups.len(), 2);
        assert_eq!(groups[0].sym_pairs(), &[(m["M3"], m["M4"])]);
        assert!(groups[0].self_syms().is_empty());
        assert_eq!(groups[1].sym_pairs(), &[(m["M1"], m["M2"])]);
        assert_eq!(groups[1].self_syms(), &[m["M9"]]);

        let _ = std::fs::remove_file(&tmp);
    }

    /// Two devices sharing a short name would make the fallback bind an arbitrary one.
    /// Same rule as `placer/src/data.rs::build_short_to_idx`: hard
    /// error naming every conflict, never a silent pick.
    #[test]
    fn test_parse_symmetry_flattened_fallback_rejects_ambiguous_short_name() {
        let mut db = CircuitDb::new();
        let _m = design(&mut db, &["A", "B"]);
        // Two expanded instances each holding a device whose short name is "M9".
        for branch in ["XA", "XB"] {
            db.allocate_cell(format!("TOP_{branch}_M9"));
        }

        let tmp = write_tmp(
            "sym_flat_ambiguous",
            r#"{ "GHOST_BLOCK": { "axis": "vertical", "pairs": { "M9": "A" } } }"#,
        );
        let err = parse_symmetry(&tmp, &mut db).expect_err("ambiguous short name must fail");
        let msg = err.to_string();
        assert!(msg.contains("GHOST_BLOCK"), "{msg}");
        assert!(
            msg.contains("TOP_XA_M9") && msg.contains("TOP_XB_M9"),
            "{msg}"
        );

        let _ = std::fs::remove_file(&tmp);
    }

    /// LCR resolves one axis per design, so every block must agree
    /// on the axis orientation; disagreeing is refused rather than silently coerced.
    #[test]
    fn test_parse_symmetry_flattened_block_rejects_axis_mismatch() {
        let mut db = CircuitDb::new();
        let _m = design(&mut db, &["M1", "M2", "M3", "M4"]);
        let tmp = write_tmp(
            "sym_flat_axis_clash",
            r#"{
  "TOP":      { "axis": "vertical",   "pairs": { "M1": "M2" } },
  "SUB_BLOCK":{ "axis": "horizontal", "pairs": { "M3": "M4" } }
}"#,
        );
        let err = parse_symmetry(&tmp, &mut db).expect_err("axis mismatch must fail");
        assert!(err.to_string().contains("axis"), "{err}");
        let _ = std::fs::remove_file(&tmp);
    }

    /// A block other than the design that explicitly asks for its own axis cannot be
    /// honoured; say so instead of snapping it onto the design's axis.
    #[test]
    fn test_parse_symmetry_flattened_block_rejects_own_axis() {
        let mut db = CircuitDb::new();
        let _m = design(&mut db, &["M1", "M2", "M3", "M4"]);
        let tmp = write_tmp(
            "sym_flat_own_axis",
            r#"{
  "TOP":      { "axis": "vertical", "pairs": { "M1": "M2" } },
  "SUB_BLOCK":{ "axis": "vertical", "shareAxisWithParent": false, "pairs": { "M3": "M4" } }
}"#,
        );
        let err = parse_symmetry(&tmp, &mut db).expect_err("own-axis block must fail");
        assert!(
            err.to_string().contains("shareAxisWithParent"),
            "{}",
            err.to_string()
        );
        let _ = std::fs::remove_file(&tmp);
    }

    /// A block named after the design binds by full name and makes exactly one group.
    #[test]
    fn test_parse_symmetry_design_block_binds_by_full_name() {
        let mut db = CircuitDb::new();
        let m = design(&mut db, &["M3", "M4"]);

        let tmp = write_tmp(
            "sym_design_block",
            r#"{ "TOP": { "axis": "vertical", "pairs": { "TOP_M3": "M4" } } }"#,
        );
        parse_symmetry(&tmp, &mut db).unwrap();

        assert_eq!(db.sym_groups().len(), 1);
        assert_eq!(db.sym_groups()[0].sym_pairs(), &[(m["M3"], m["M4"])]);

        let _ = std::fs::remove_file(&tmp);
    }
}
