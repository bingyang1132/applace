**What this changes, and why**

**If it fixes a bug**

- [ ] The input that showed the bug is added as a case under
      `benchmark/regression/` (see `benchmark/regression/README.md`), and
      `bash scripts/check_reference.sh` fails without the fix and passes with it.

**Checks** (see `CONTRIBUTING.md`)

- [ ] `cargo test --manifest-path src/Cargo.toml --workspace` passes
- [ ] `cargo fmt --check` and `cargo clippy --all-targets -- -D warnings` are clean
- [ ] `bash scripts/check_reference.sh` passes
- [ ] If a reference placement moved on purpose, it is regenerated in this pull
      request and the description says what moved and why
