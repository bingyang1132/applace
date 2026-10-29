# Contributing to APPlace

Thank you for taking an interest. This is a research tool published alongside a
paper, so the bar for a change is not "it works on my case" but "the four example
cases still satisfy their invariants". Run `scripts/check_reference.sh`; if the
placements change, say in the pull request why they should have. Byte-for-byte
equality (`--strict`) is a same-machine, same-toolchain property and is not the
bar -- see section 5 (Determinism) of `docs/OUTPUT.md`.

## Before you start

- **Open an issue first** for anything larger than a bug fix. A placement
  algorithm has a lot of coupled knobs, and it is much cheaper to agree on the
  shape of a change before it is written.
- **Small, single-purpose pull requests.** One behaviour change per pull request,
  with its test.

## Getting set up

```bash
cargo build --release --manifest-path src/Cargo.toml --workspace
cargo test  --manifest-path src/Cargo.toml --workspace
```

A stock Rust toolchain is the only requirement. The minimum supported version is
in `src/Cargo.toml` (`rust-version`); CI builds on the current stable.

Some tests are marked `#[ignore]`: they need the device-flow example design -- a
SPICE netlist plus one vendor PCell layout per device -- which is not distributed
with this repository. If you hold such a design, put it in `benchmark/<case>/`
(see `docs/INPUT_FORMAT.md`) and run them with:

```bash
cargo test --manifest-path src/Cargo.toml --workspace -- --ignored
```

## What a change has to satisfy

1. **`cargo test --workspace` passes**, and a behaviour change comes with a test
   that fails without it.
2. **`cargo fmt --check` and `cargo clippy --all-targets -- -D warnings` are
   clean.** Both run in CI.
3. **Determinism holds.** The same binary on the same input must produce
   byte-identical output. If your change introduces a `HashMap` iteration, a
   thread, or a tie broken by anything other than a total order, it will break
   this. Check with:

   ```bash
   bash run/10-abstract-placement.sh comp /tmp/a
   bash run/10-abstract-placement.sh comp /tmp/b
   cmp /tmp/a/placement.json /tmp/b/placement.json
   ```

4. **The example cases still satisfy their invariants.**
   `scripts/check_reference.sh` runs all four and checks the summary against
   `benchmark/abstract/<case>/reference/placement_summary.json`: the device,
   PMOS and symmetry-pair counts, exact symmetry, zero overlap, containment, and
   a wide band around the reference wirelength.

   ```bash
   bash scripts/check_reference.sh
   ```

   None of those numbers can be moved by a floating-point difference, so this
   check is meaningful on any machine, and a failure is a defect.

5. **If you moved a device on purpose, say so and regenerate the references.**
   `bash scripts/check_reference.sh --strict` compares `placement.json` byte for
   byte, and is the check that notices a change of that kind -- on the machine
   and toolchain the reference files came from. Across platforms it is expected
   to differ; see section 5 (Determinism) of `docs/OUTPUT.md`.

   A change that is meant to move a device moves
   `benchmark/abstract/<case>/reference/placement.json` and its summary. That is
   fine -- regenerate both in the same commit, and say in the pull request what
   moved and why. What is not fine is regenerating them silently.

   Compare release against release: debug and release builds reduce
   floating-point sums in a different order, so their placements differ in the
   last bits. `run/10-abstract-placement.sh` builds release by default.

## Bugs and the regression set

A bug fix comes with the input that showed the bug, added as a case under
`benchmark/regression/`. `scripts/check_reference.sh` runs every case there along
with the examples, and CI runs that script, so a fixed bug stays fixed.
`benchmark/regression/README.md` says what a case is and how to record its
reference. A design you cannot share can almost always be reduced to a case in the
geometric form, which carries no process data.

When you report a bug, attach that input too; the issue template asks for it.

## Style

- Comments say *why*, not *what*. The code already says what.
- A claim in a comment should be falsifiable: name the case, the count, the
  field. "Measured on the largest example case: 3 rounds" is useful; "much
  faster" is not.
- No unwrap on user input. A bad deck, a bad netlist or a bad parameter file
  should produce an error message that names the file and the field.

## Licence

By contributing you agree that your contribution is licensed under the GNU
General Public License v3.0 (GPL-3.0, `LICENSE`), as described in `README.md`,
with no additional terms.
