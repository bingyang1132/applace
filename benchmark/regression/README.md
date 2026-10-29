# Regression cases

Every case here is an input that once showed a bug. Once fixed, it stays, so the
bug cannot come back unnoticed: `scripts/check_reference.sh` runs every case in
this directory together with the four examples under `benchmark/abstract/`, and CI
runs that script on every push.

## What a case is

A directory named after where it came from, for example `issue-42-sym-overlap/`,
holding a case in the geometric form (`docs/INPUT_FORMAT.md`):

```
benchmark/regression/<name>/
  bbox.json  pin.json  sym.json  params.json
  reference/placement.json
  reference/placement_summary.json
```

The geometric form carries no process data, so any design can be reduced to a case
that is safe to publish.

## Adding one

1. Put the four input files in `benchmark/regression/<name>/` and check that the
   case shows the bug before your fix.
2. With the fix, record its reference:

   ```bash
   bash run/10-abstract-placement.sh benchmark/regression/<name> out/<name>
   mkdir -p benchmark/regression/<name>/reference
   cp out/<name>/placement.json out/<name>/placement_summary.json \
      benchmark/regression/<name>/reference/
   ```

3. Look at the placement (`python3 scripts/plot_placement.py out/<name>`) before
   you commit it: a reference records what the tool does, not what it should do.
4. `bash scripts/check_reference.sh` must pass.

The check is the same as for the examples: device, PMOS and symmetry-pair counts
equal to the reference, exact symmetry, no overlap, nothing outside the region, and
wirelength within a wide band of the reference.
