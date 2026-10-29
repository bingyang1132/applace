---
name: Bug report
about: A placement that is wrong, a crash, or an error you did not expect
labels: bug
---

**What happened**

<!-- One or two sentences. What did you expect, and what did you get instead? -->

**The input that shows it**

Please attach the files the run read, so the problem can be reproduced and then
kept in the regression set (`benchmark/regression/`):

- geometric form: `bbox.json`, `pin.json`, `sym.json`, `params.json`
- device form: the netlist, the rule deck and the parameter file, if you can share
  them

If your design cannot be shared, a geometric-form case that still shows the problem
is enough; it carries no process data (`docs/INPUT_FORMAT.md`).

**How you ran it**

```
the exact command, e.g. bash run/10-abstract-placement.sh path/to/case out/
```

**Output**

<!-- The error message, or the placement_summary.json / placement.png that looks
wrong. For a crash, the last 30 lines of the log. -->

**Environment**

- APPlace commit: <!-- git rev-parse --short HEAD -->
- OS and architecture:
- `rustc --version`:
- Does `bash scripts/check_reference.sh` pass on this machine? yes / no
