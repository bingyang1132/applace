# Parameters

APPlace is an analog placement engine. Every tuning knob it exposes is a field of a single
parameter record that is resolved once, at start-up, from three layers: the built-in defaults
compiled into the binary, the case's `params.json`, and any `--set key=value` arguments given on
the command line.

This page is the complete reference for that record. Every default listed here is the value the
binary uses when the key is absent from both `params.json` and the command line.

## Contents

- [Where parameters come from](#where-parameters-come-from)
- [Determinism](#determinism)
- [Optimizer and solver core](#optimizer-and-solver-core)
- [Constraint switches](#constraint-switches)
- [Wirelength and access-point model](#wirelength-and-access-point-model)
- [Symmetry, boundary and region shaping](#symmetry-boundary-and-region-shaping)
- [Overlap, density and mobility](#overlap-density-and-mobility)
- [Initialization](#initialization)
- [Convergence and stopping criteria](#convergence-and-stopping-criteria)
- [Controller keys that live in `placement.global`](#controller-keys-that-live-in-placementglobal)
- [Multi-phase controller: base phase](#multi-phase-controller-base-phase)
- [Tail phase: controller and schedule](#tail-phase-controller-and-schedule)
- [Tail phase: inflated-metric refinement](#tail-phase-inflated-metric-refinement)
- [Discrete de-overlap pass and its retry loop](#discrete-de-overlap-pass-and-its-retry-loop)
- [Clearance and guard-ring reservation](#clearance-and-guard-ring-reservation)
- [Diagnostics](#diagnostics)
- [Derived keys](#derived-keys)
- [Parameters you are most likely to touch](#parameters-you-are-most-likely-to-touch)

## Where parameters come from

Each case directory carries a `params.json`. The file has a fixed two-level shape:

```json
{
  "schemaVersion": 1,
  "placement": {
    "schemaVersion": 1,
    "mode": "multiphase",
    "global": {},
    "base": {},
    "tail": {},
    "deoverlap": {}
  }
}
```

Rules enforced when the file is read:

- Both `schemaVersion` fields are mandatory and must be `1`. Any other value is an error.
- The only allowed top-level keys are `schemaVersion` and `placement`. The only allowed keys
  inside `placement` are `schemaVersion`, `mode`, `global`, `base`, `tail` and `deoverlap`.
- `placement.mode` has one accepted value, `multiphase`. It is still required, and any
  other value is an error rather than a silent substitution.
- Every parameter key belongs to exactly one group. Putting a key in the wrong group is an error,
  and the message names the group it belongs in. An unrecognised key is an error, with a
  spelling suggestion when one is close enough.
- Group membership is fixed by the engine, not by the file. The tables below state, for each
  block of keys, which group they belong to.

Resolution order, later layers winning over earlier ones:

1. Built-in defaults (the `Default` column in every table below).
2. `placement.global`.
3. `placement.base`, then `placement.tail`, then `placement.deoverlap`. Since group membership
   is disjoint, these never overwrite each other; the ordering only matters if a future key
   moves between groups.
4. Command-line `--set` overrides.

### Command-line overrides

`--set key=value` may be repeated. It is applied last, so it beats anything in the file.

- The key is normally the bare camelCase parameter name, exactly as it appears in the tables
  below. The prefixes `common.` and `multiphase.` are also accepted and are simply
  stripped.
- `gpControllerMode` cannot be overridden. Use `placement.mode` in the file.
- Value parsing: `true`, `false`, `null`, a double-quoted string, a JSON object or a JSON array
  are parsed as JSON. Otherwise the value is tried as an integer, then as a floating-point
  number, and finally kept as a bare string.

Concrete example:

```bash
bash run/10-abstract-placement.sh ota2 out/ota2 \
  --set maxIterations=4000 \
  --set symSelfWeight=1.5 \
  --set 'tailDeoverlapRhoMax=0.20'
```

### Accepted key aliases

For backwards compatibility a few snake_case spellings are accepted inside `params.json` and
rewritten to the canonical key. Supplying both an alias and its canonical key with different
values is an error.

In `placement.global`: `pmos_hard_expand_for_gr`, `tail_inflation_safety_margin`.

In `placement.deoverlap`: `deoverlap_search_kappa`, `deoverlap_lambda_expand`,
`deoverlap_lambda_area`, `deoverlap_rho_max`, `deoverlap_infl`, `infl_deoverlap`,
`deoverlap_use_elliptical_search`, `deoverlap_lambda_topo`, `deoverlap_final_phi_iters`,
`lcr_honor_drc_floor`, `lcr_retry_enable`, `lcr_retry_max_rounds`, `lcr_retry_infl_step`.

New files should use the canonical camelCase keys.

## Determinism

Placement is deterministic. The same binary run on the same inputs with the same parameters
produces byte-identical output, run after run and machine after machine. There is no wall-clock
or thread-count dependence in the result.

Determinism holds *within one build profile*. A debug build and a release build perform
floating-point reductions in a different order, so their results can diverge by more than
rounding after a few thousand iterations. Any comparison between two runs -- a parameter sweep, a
before/after check, a regression baseline -- must therefore keep both runs on the same profile.
Release is the profile to use for anything you intend to quote.

Reproducible initialization additionally depends on `randomSeed`. With a seed set (the default),
initialization is fixed; setting it to `null` draws from the system entropy source and gives up
run-to-run reproducibility.

## Optimizer and solver core

The placer minimises a weighted sum of differentiable penalty terms with a first-order optimizer.
These keys control the optimizer itself and the optional backtracking line search that scales its
step. All of them belong in `placement.global`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `maxIterations` | integer | `2000` | Maximum number of optimizer iterations in the main placement loop; must be greater than zero. |
| `optimizer` | string | `"Adam"` | Selects the optimization algorithm, either `"Adam"` or `"SGD"`. |
| `learningRate` | number | `200.0` | Adam learning rate, expressed in placement length units per step; must be positive. |
| `adamBeta1` | number | `0.9` | Adam first-moment decay rate; must lie strictly inside the interval from 0 to 1. |
| `adamBeta2` | number | `0.999` | Adam second-moment decay rate; must lie strictly inside the interval from 0 to 1. |
| `stepStrategy` | string | `"none"` | Step-size strategy: `"linesearch"` enables backtracking line search, `"none"` uses the raw optimizer step. |
| `lsStepInit` | number | `1.0` | Initial step-size multiplier the line search starts from. |
| `lsStepShrink` | number | `0.5` | Factor by which the line search shrinks the step size on each backtracking attempt. |
| `lsStepMin` | number | `1e-6` | Smallest step size the line search will accept before giving up on further shrinking. |
| `armijoC1` | number | `1e-4` | Constant of the Armijo sufficient-decrease condition used by the line search. |

## Constraint switches

Each switch turns one penalty term on or off. A term that is off contributes nothing to the
objective and its associated weights and thresholds are inert. All of these belong in
`placement.global`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `enableWirelength` | boolean | `true` | Enables the log-sum-exp wirelength term, the primary wirelength objective. |
| `useApWl` | boolean | `false` | Blends an access-point-aware wirelength term into the wirelength objective, so nets are pulled toward reachable pin access points rather than cell centres. |
| `enableOverlap` | boolean | `true` | Enables the pairwise overlap penalty that pushes devices apart. |
| `enableSymmetry` | boolean | `true` | Enables the symmetry penalty over the symmetry pairs and self-symmetric devices declared for the case. |
| `enableBoundary` | boolean | `false` | Enables the boundary penalty that keeps devices inside the placement region. |
| `useWellCluster` | boolean | `false` | **Reserved. Currently has no effect on the placement.** The key is accepted and validated so that a parameter file written for it stays loadable; the well-cluster penalty it is meant to switch on is not implemented, and the term contributes exactly zero whichever way the key is set. |
| `enableArray` | boolean | `false` | **Reserved. Currently has no effect on the placement.** As above: accepted and validated, but the array penalty is not implemented and the term contributes exactly zero either way. |
| `useCompaction` | boolean | `false` | Enables the centroid compaction penalty that draws the whole placement toward its centroid; it also requires `compactionWeight` to be greater than zero. |
| `useMassDamping` | boolean | `true` | Requests area-based movement damping, so that larger devices move less per step than smaller ones. |

## Wirelength and access-point model

The wirelength term is a smoothed approximation of half-perimeter wirelength; the smoothing
parameter is annealed downward so that the approximation tightens as the placement settles. The
access-point terms shape pins into a superellipse so that a pin's usable access area, not just its
centre, drives the objective. All keys in this table belong in `placement.global`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `wlSmoothingInit` | number | `1.0` | Initial smoothing parameter of the log-sum-exp wirelength term; larger values give a smoother, less accurate approximation. Must be positive. |
| `wlGammaDecay` | number | `0.95` | Multiplicative decay applied to the wirelength smoothing parameter as optimization proceeds. |
| `apExponentM` | number | `6.0` | Exponent of the superellipse used to model an access point; larger values make the access region more rectangular. Must be at least 2. |
| `apEps` | number | `1e-6` | Numerical guard added inside the access-point superellipse to keep its gradient finite; must be positive. |
| `apStableWeight` | number | `0.0` | Weight of an extra regularization term that stabilises the access-point formulation; zero disables it. |
| `lambdaApMax` | number | `0.7` | Upper bound on the access-point blending weight, as a fraction between 0 and 1 of the total wirelength term. |
| `etaAp` | number | `0.001` | Per-update increment by which the access-point blending weight grows once its gate opens; must be non-negative. |

## Symmetry, boundary and region shaping

These weights scale individual penalty terms against each other. Raising one makes the
corresponding constraint harder relative to wirelength. All belong in `placement.global`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `symSelfWeight` | number | `2.0` | Multiplier applied to the symmetry penalty of self-symmetric devices relative to ordinary symmetry pairs; must be positive. |
| `boundaryWeight` | number | `1.0` | Multiplier on the boundary penalty that keeps devices inside the placement region; must be positive. |
| `aspectInflation` | number | `1.0` | Area multiplier applied when deriving the boundary rectangle from the total device area, used to bias the region's aspect ratio; must be positive. |
| `biasPositionUnit` | number | `1.0` | Length unit used when interpreting per-device bias positions supplied with the case. |
| `compactionWeight` | number | `0.0` | Weight of the centroid compaction penalty; must be non-negative and must be greater than zero for `useCompaction` to have any effect. |

## Overlap, density and mobility

Overlap is handled in two ways: a smooth penalty during optimization, and a per-device mobility
multiplier that scales how far each device is allowed to travel per step. Mobility keeps large
devices from oscillating while letting small ones find gaps. All keys in this table belong in
`placement.global`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `massDampingExp` | number | `0.5` | Exponent of the area-based damping curve; 0 disables damping and larger values slow big devices down more sharply. Must be non-negative. |
| `mobilityMin` | number | `0.6` | Lower clamp on a device's mobility multiplier; must be positive and no greater than `mobilityMax`. |
| `mobilityMax` | number | `3.0` | Upper clamp on a device's mobility multiplier; must be positive and no smaller than `mobilityMin`. |
| `mobilitySymAnchorMult` | number | `1.3` | Mobility multiplier applied to a device that anchors a symmetry group. |
| `mobilityLargeMult` | number | `1.3` | Mobility multiplier applied to devices classified as large. |
| `mobilitySmallMult` | number | `0.8` | Mobility multiplier applied to devices classified as small. |

## Initialization

The default start is a seeded pseudo-random spread inside the placement region. All keys in this
table belong in `placement.global`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `randomSeed` | integer or null | `1` | Fixed seed for every pseudo-random choice made during initialization; `null` draws a seed from the system and makes the run non-reproducible. |
| `randomizeInitialPlacement` | boolean | `false` | Uses the random initial placement instead of starting from a fully overlapped stack at the region centre. |
| `initJitterRatio` | number | `0.1` | Amount of initial positional jitter, as a fraction of each device's own width and height; must be between 0 and 1. |
| `initSpreadRatio` | number | `1.0` | How widely the random initial placement spreads inside the region: 0 keeps everything at the centre, 1 uses the full range. Must be between 0 and 1. |

## Convergence and stopping criteria

The main loop stops at `maxIterations`, or earlier when the quantities below have been small enough for
`convergenceWindow` consecutive iterations. Raising a threshold makes the run stop earlier and
looser; lowering it makes the run go the distance. All keys in this table belong in
`placement.global`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `gradTol` | number | `1e-4` | Gradient-norm threshold below which the placement is considered stationary. |
| `lossRtol` | number | `1e-5` | Threshold on the relative change of the objective between iterations. |
| `moveRtol` | number | `1e-4` | Threshold on the root-mean-square device movement per iteration, expressed as a fraction of the region diagonal. |
| `moveMaxRtol` | number | `5e-4` | Threshold on the largest single-device movement per iteration, expressed as a fraction of the region diagonal. |
| `overlapTarget` | number | `5e-4` | Overlap target, expressed as a ratio of residual overlap area to total device area; must be non-negative. |
| `symmetryTarget` | number | `1e-2` | Symmetry target, normalised by the square of the total device width. |
| `boundaryTarget` | number | `0.0` | Boundary-violation target, normalised by the total device area. |
| `convergenceWindow` | integer | `10` | Number of consecutive iterations that must satisfy all active criteria before the loop stops. |
| `useBestSolution` | boolean | `false` | Restores the lowest-objective position seen during the run instead of keeping the final iterate. |

## Controller keys that live in `placement.global`

Five keys of the multi-phase controller are shared by the base and tail phases, and therefore live
in `placement.global` rather than in a phase group. Putting them anywhere else is rejected.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `lambdaApMax` | number | `0.7` | Upper bound on the access-point blending weight; documented with the other access-point keys above. |
| `etaAp` | number | `0.001` | Growth increment of the access-point blending weight; documented with the other access-point keys above. |
| `mobilityTriggerRatio` | number | `0.3` | Fraction of the overlap target at which the per-device mobility multipliers start to take effect; must be greater than 0 and at most 1. |
| `mobilityRampIters` | integer | `50` | Number of iterations over which the mobility multipliers ramp from inactive to fully applied once triggered; must be greater than zero. |
| `candidateStride` | integer | `5` | Iteration stride at which the base phase records a candidate placement for later selection; must be greater than zero. |

## Multi-phase controller: base phase

The base phase is the first of the controller's phases. It runs a feedback controller on two
quality measures -- residual overlap and residual symmetry error -- and adjusts their penalty
weights in the log domain each iteration so that both are driven toward their targets without
either term overwhelming the wirelength objective. Targets are expressed as ratios of the
corresponding convergence threshold, so tightening `overlapTarget` tightens the controller too.

All keys in this table belong in `placement.base`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `rho` | number | `0.15` | Weight given to the newest sample in the exponential moving average of the overlap and symmetry measures; must be greater than 0 and at most 1. |
| `tauORatio` | number | `0.20` | Base-phase overlap target, as a ratio of the overlap convergence threshold; must be positive. |
| `tauSRatio` | number | `0.50` | Base-phase symmetry target, as a ratio of the symmetry convergence threshold; must be positive. |
| `aO` | number | `1.0` | Proportional coefficient of the overlap controller, acting on the relative gap between the smoothed overlap measure and its target. |
| `bO` | number | `0.25` | Trend coefficient of the overlap controller, acting on the logarithmic change of the overlap measure between updates. |
| `aS` | number | `0.35` | Proportional coefficient of the symmetry controller. |
| `bS` | number | `0.10` | Trend coefficient of the symmetry controller. |
| `lambdaOvlMin` | number | `1e-4` | Lower bound on the overlap penalty weight; must be positive. |
| `lambdaOvlMax` | number | `30.0` | Upper bound on the overlap penalty weight; must be at least the lower bound. |
| `lambdaSymMin` | number | `1e-5` | Lower bound on the symmetry penalty weight; must be positive. |
| `lambdaSymMax` | number | `10.0` | Upper bound on the symmetry penalty weight; must be at least the lower bound. |
| `sLambdaBase` | number | `ln(1.01)`, about `0.00995033` | Maximum magnitude of a single base-phase weight update in the log domain, so one update changes a weight by at most about one percent. |
| `tauGateHiRatio` | number | `0.25` | Upper gate ratio: the overlap level, relative to target, above which the access-point term is held back. Must be greater than the lower ratio. |
| `tauGateLoRatio` | number | `0.10` | Lower gate ratio: the overlap level, relative to target, below which the access-point term is allowed to grow. Must be positive. |
| `anchorBaseRatio` | number | `1e-3` | Base-phase anchor strength, as a ratio that scales the pull of each device toward its reference position; must be positive. |
| `anchorMin` | number | `1e-6` | Lower bound on the anchor weight; must be positive. |
| `anchorMaxBase` | number | `2.0` | Upper bound on the anchor weight during the base phase; must be at least `anchorMin`. |
| `anchorRampIters` | integer | `20` | Number of iterations over which the anchor weight ramps up to its target after a phase change; must be greater than zero. |
| `cDeltaBase` | number | `0.05` | Clipping scale for a single base-phase positional update, limiting how far the controller lets a device jump in one iteration; must be positive. |

## Tail phase: controller and schedule

After the base phase the placement is close to legal but still has residual overlap. The tail
phase re-runs the optimizer on an inflated view of each device so that a placement that merely
touches is treated as overlapping, and pushes devices apart until the inflated view is clean too.

The tail phase also runs an accept/rollback loop: every `tailWindow` iterations it compares the
current state against the state at the start of the window and either keeps it, keeps going, or
rolls back. The `Accept` thresholds decide what counts as progress worth keeping; the `Rollback`
thresholds decide what counts as damage worth undoing; the `Soft`/`Hard` thresholds form a
hysteresis band that governs how aggressively the weights are allowed to move.

All keys in this table belong in `placement.global`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `enableGpTail` | boolean | `true` | Runs the tail refinement phase after the base phase; turning it off ends placement at the base result. |
| `tailIters` | integer | `500` | Maximum number of tail-phase iterations. |
| `tailEmaAlpha` | number | `0.2` | Weight of the newest sample in the exponential moving average used by the adaptive weight controller; must be greater than 0 and at most 1. |
| `tailTauOvlGood` | number | `0.01` | Relative overlap improvement above which the controller considers the tail to be making good progress and keeps pushing. |
| `tailDSoft` | number | `0.06` | Soft hysteresis threshold on positional drift from the reference placement; below it, drift is not penalised. |
| `tailDHard` | number | `0.12` | Hard hysteresis threshold on positional drift; above it, the controller actively pulls devices back toward their reference positions. |
| `tailSSoft` | number | `0.02` | Soft hysteresis threshold on symmetry error. |
| `tailSHard` | number | `0.06` | Hard hysteresis threshold on symmetry error. |
| `tailLambdaGrowthCap` | number | `0.08` | Maximum relative increase a penalty weight may take in a single controller update. |
| `tailLambdaDecayCap` | number | `0.03` | Maximum relative decrease a penalty weight may take in a single controller update. |
| `tailTauAcceptOvl` | number | `0.005` | Overlap improvement over the acceptance window required to accept the window's result. |
| `tailTauMin` | number | `0.001` | Minimum overlap improvement below which the window is rolled back as unproductive. |
| `tailDAccept` | number | `0.10` | Largest positional drift still compatible with accepting a window. |
| `tailDRollback` | number | `0.16` | Positional drift that forces a rollback of the window. |
| `tailSAccept` | number | `0.05` | Largest symmetry error still compatible with accepting a window. |
| `tailSRollback` | number | `0.10` | Symmetry error that forces a rollback of the window. |
| `tailWindow` | integer | `20` | Length in iterations of one accept/rollback window; must be greater than zero. |
| `tailAnchorStrong` | number | `1.0` | Anchor weight for devices that are not involved in any residual conflict, holding them in place; must be non-negative. |
| `tailAnchorMid` | number | `0.35` | Anchor weight for devices involved in a conflict, letting them move while still resisting large excursions; must be non-negative. |
| `tailAnchorWeak` | number | `0.15` | Anchor weight for small conflicting devices, which are the cheapest to relocate; must be non-negative. |
| `tailSelectiveInflation` | boolean | `true` | Inflates only the devices that still conflict, instead of inflating every device uniformly. |
| `tailInflationStep` | number | `0.05` | Increment by which a conflicting device's inflation factor grows per step; must be non-negative. |
| `tailInflationMax` | number | `1.10` | Largest inflation factor a device may reach, as a multiple of its true size; must be at least 1. |

## Tail phase: inflated-metric refinement

These keys shape the inflated geometry the tail phase optimizes against and the cover penalty that
discourages one device from sitting on top of another. All of them belong in `placement.tail`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `sLambdaTail` | number | `ln(1.015)`, about `0.01488861` | Maximum magnitude of a single tail-phase weight update in the log domain, slightly larger than the base-phase step. |
| `anchorTailRatio` | number | `5e-3` | Tail-phase anchor strength ratio; must be positive. |
| `anchorMaxTail` | number | `10.0` | Upper bound on the anchor weight during the tail phase; must be at least `anchorMaxBase`. |
| `cDeltaTail` | number | `0.08` | Clipping scale for a single tail-phase positional update; must be positive. |
| `tailInflRatio` | number | `0.15` | Isotropic inflation applied to every device in the tail metric, as a fraction of the median of each device's smaller dimension; must be non-negative. |
| `tailInflRampIters` | integer | `100` | Number of iterations over which the inflation ramps from none to full; must be greater than zero. |
| `tailTauORatio` | number | `0.15` | Overlap target used during the tail phase, measured under the inflated metric, as a ratio of the overlap threshold; must be positive. |
| `tailCandidateAlphaStart` | number | `0.5` | Inflation ramp progress, between 0 and 1, at which the tail phase begins recording candidate placements. |
| `tailCandidateStride` | integer | `5` | Iteration stride at which the tail phase records a candidate placement; must be greater than zero. |
| `tailRefDilateBeta` | number | `0.20` | Coefficient that dilates the reference placement against which drift is measured, so that legitimate spreading is not mistaken for drift; must be non-negative. |
| `tailCoverTau` | number | `0.70` | Cover ratio, between 0 and 1, above which one device overlapping another is treated as a severe cover rather than an ordinary overlap. |
| `tailCoverSoftness` | number | `0.08` | Softness of the smooth transition into the cover penalty; smaller values make the penalty switch on more abruptly. Must be positive. |
| `tailCoverLambdaRatio` | number | `0.10` | Initial weight of the cover penalty at tail entry, expressed as a ratio of the overlap penalty weight; must be non-negative. |

## Discrete de-overlap pass and its retry loop

The gradient phases leave a placement that is good but not guaranteed overlap-free. The de-overlap
pass is a discrete cleanup: it takes the selected tail candidate and, for each device that still
conflicts, searches a bounded lattice of nearby positions for a legal one, preferring positions
that move the device least and grow the overall bounding area least. It searches on a fine grid
first and falls back to a coarse grid with a wider reach when the fine search finds nothing.

If the pass still cannot resolve every conflict with local moves, the retry loop re-runs the whole
multi-phase placement with the halo of the still-conflicting devices enlarged, so the gradient
phases open the gap themselves rather than asking the discrete pass to do the impossible. The loop
only fires on failure: a case whose de-overlap pass already succeeds never enters it and its
output is unchanged.

All keys in this table belong in `placement.deoverlap`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `tailDeoverlapEnable` | boolean | `true` | Runs the discrete de-overlap pass after the tail candidate has been selected. |
| `tailDeoverlapGridFine` | number | `50.0` | Step of the fine search lattice, in placement length units; must be positive. |
| `tailDeoverlapRingFine` | number | `5000.0` | Largest displacement the fine search will consider, in placement length units; must be positive and at least the fine grid step. |
| `tailDeoverlapGridCoarse` | number | `200.0` | Step of the coarse fallback lattice, in placement length units; must be positive. |
| `tailDeoverlapRingCoarse` | number | `20000.0` | Largest displacement the coarse fallback search will consider, in placement length units; must be positive and at least the coarse grid step. |
| `tailDeoverlapInfl` | number | `1.05` | Fixed inflation factor applied to device geometry when the de-overlap pass tests whether a candidate position is clear; must be at least 1. |
| `tailDeoverlapSearchKappa` | number | `2.0` | Anisotropy clamp for the elliptical search: the width-to-height ratio of the search ellipse is clipped to lie between the reciprocal of this value and this value. Must be at least 1. |
| `tailDeoverlapLambdaExpand` | number | `1.0` | Weight of the penalty on growing the overall placement extent, which keeps the pass from solving conflicts by spreading everything out; must be non-negative. |
| `tailDeoverlapLambdaArea` | number | `1.0` | Weight of the area component inside that expansion penalty; must be non-negative. |
| `tailDeoverlapRhoMax` | number | `0.15` | Hard cap on the relative growth of total placement area the pass is allowed to cause; must be non-negative. |
| `tailDeoverlapUseEllipticalSearch` | boolean | `true` | Shapes the candidate search region as an ellipse matched to the device aspect ratio instead of a square. |
| `tailDeoverlapLambdaTopo` | number | `0.0` | Weight of an optional term that penalises changes to the relative arrangement of devices; reserved, and inert at the default of zero. Must be non-negative. |
| `tailDeoverlapFinalPhiIters` | integer | `300` | Number of fixed-layout access-point wirelength iterations run after the de-overlap pass, which reassign pin access without moving any device; must be at least 1. |
| `lcrHonorDrcFloor` | boolean | `true` | Makes the de-overlap pass respect the same minimum-clearance floor as the tail phase, instead of judging feasibility from its own inflation factor alone. On by default: without it the de-overlap pass can place a pair closer than the floor and undo the clearance the tail phase just established. Turning it off reproduces the pre-2026-09-17 behaviour. |
| `lcrRetryEnable` | boolean | `true` | Enables the retry loop that re-runs placement with enlarged halos when the de-overlap pass cannot resolve every conflict. |
| `lcrRetryMaxRounds` | integer | `6` | Maximum number of extra placement passes the retry loop may run; zero disables the loop. |
| `lcrRetryInflStep` | number | `1.1` | Multiplicative growth applied per retry round to the halo of each still-conflicting device; must be finite and strictly greater than 1, so growth is monotone. |
| `lcrRetryOnCrossing` | boolean | `false` | Adds a second trigger to the retry loop: re-run the expansion phase when the de-overlap pass could only reach a legal position by flying a device over a neighbour, which says the local space is too tight. **Off by default, and treated as an experimental feature**: its convergence cost is not yet established on real designs (the retry loop usually runs to its round cap when this trigger is on), so it ships opt-in. Off is the behaviour of every placement in `benchmark/abstract/*/reference/`. |
| `lcrRetryCrossCriterion` | string | `"either"` | Which predicate the trigger above uses. `"order"`: the mover ended up on the other side of a local neighbour — a device that slipped *past* another one, including out through a gap in a ring. `"cover"`: the swept region buries more than `lcrRetryCrossCoverFrac` of another device — a device that ploughed *over* another one. `"either"`: their union, and the default — across a 20-point perturbation sweep it is the only enabled arm that both leaves no residual overlap and lowers wirelength on average; `"cover"` alone leaves residual overlap at some points and `"order"` alone misses a device that ploughs over a neighbour. Any other value is an error. Inert while `lcrRetryOnCrossing` is off. |
| `lcrRetryCrossCoverFrac` | number | `0.5` | How much of a neighbour a move has to bury before the `"cover"` predicate counts it as a crossing. The region the device's box sweeps between its start and end positions is intersected with each other device's box, and the predicate fires when `covered area / that device's area` exceeds this fraction. Must be in `(0, 1]`; at `1.0` only a move that completely buries a neighbour counts. Read only under `"cover"` and `"either"`. |

## Clearance and guard-ring reservation

Analog devices need more room than their drawn extent: a PMOS device needs space around it for a
guard ring, and any two devices need at least the technology's minimum spacing between them. These
keys reserve that space during placement, by enlarging the geometry the overlap term sees.

All keys in this table belong in `placement.global`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `pmosHardExpandForGr` | number | `650.0` | Half-width, in placement length units, by which every PMOS device is expanded in the overlap term to reserve room for a guard ring around it; must be non-negative. |
| `tailInflationSafetyMargin` | number | `0.01` | Safety margin, in micrometres, added when deriving the minimum-clearance floor from the technology spacing rules; must be non-negative. |
| `tailHardInflationFloor` | number | `0.0` | Minimum clearance, in placement length units, enforced around every overlap-active device during the tail phase; `0.0` disables the floor. When the technology file supplies spacing rules, the placement stage derives this value and overrides whatever is in the file. Must be non-negative. |

## Diagnostics

These keys control what the tail phase writes alongside the placement result. They do not affect
the placement itself. Both belong in `placement.global`.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `tailDiagInterval` | integer | `5` | Writes a diagnostic snapshot every this many tail iterations; must be greater than zero. |
| `tailDiagEnablePlots` | boolean | `true` | Renders plots from the tail diagnostic snapshots in addition to writing the raw data. |


## Derived keys

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `gpControllerMode` | string | `"multiphase"` | Which controller runs. It is derived from `placement.mode` and cannot be written in `params.json` or overridden with `--set`; both are rejected with a message pointing at `placement.mode`. |

## Parameters you are most likely to touch

Everything above is adjustable, but a first pass at a new case almost always comes down to the
following.

**`maxIterations`** (default `2000`). The iteration budget for the base phase. If the placement still
looks like it is moving when the run ends -- devices clustered, wirelength visibly improvable --
raise it; 3000 to 5000 is a reasonable next step. Lowering it is mainly useful for quick
turnaround while you are sweeping other parameters; the run will stop before the convergence
criteria are met and the result will be correspondingly rough. Note that the loop also stops early
once the convergence criteria hold for `convergenceWindow` iterations, so raising `maxIterations` on a
case that already converges costs nothing.

**`tailIters`** (default `500`). The budget for the tail phase, which is where residual overlap
actually gets resolved. If the de-overlap pass reports conflicts it could not fix, more tail
iterations is usually the cheapest remedy, because it lets the gradient phase open gaps that the
discrete pass can only nudge. Cutting it below a couple of hundred tends to hand the discrete pass
more work than it can do locally.

**`learningRate`** (default `200.0`). The learning rate, in placement length units per step. Because it
is a length, the right value scales with the size of your devices: a case with devices an order of
magnitude larger than the examples wants a proportionally larger rate. Too low and the placement
never spreads out within the iteration budget; too high and it oscillates, which shows up as a
loss curve that stops improving while devices keep moving.

**`symSelfWeight`** (default `2.0`). How hard self-symmetric devices are held on the symmetry axis
relative to ordinary symmetry pairs. Raise it when self-symmetric devices drift off-axis in the
result; lower it toward 1.0 when the symmetry constraint is visibly fighting the wirelength
objective and producing long detour nets. The overall strength of the symmetry term is set by the
controller, not by this key -- this is the relative weighting inside the term.

**`tauORatio`** (default `0.20`). The base phase's overlap target, as a fraction
of `overlapTarget`. This is the single most direct handle on how aggressively the base phase separates
devices. Lowering it toward 0.1 makes the base phase demand a cleaner placement before handing off
to the tail, which usually costs wirelength and iterations but gives the tail less to do. Raising
it lets the base phase finish sooner with more residual overlap.

**`lambdaOvlMax`** (default `30.0`). The ceiling on the overlap penalty weight.
If a case ends with overlap that the controller never resolves -- the weight sits pinned at the
ceiling and the overlap measure plateaus -- this is the limit to raise. Raising it too far makes
the overlap term dominate wirelength, and the placement spreads out more than it needs to.

**`tailInflRatio`** (default `0.15`). How much every device is inflated in the
tail metric, as a fraction of the median device dimension. This is effectively the spacing margin
the tail phase aims for. Raise it when the result is legal but too tight to be practical; lower it
when the tail phase is spreading devices further apart than the design needs. It interacts
directly with `tailHardInflationFloor`, which imposes an absolute lower bound on the same
clearance.

**`tailDeoverlapInfl`** (default `1.05`). The clearance the discrete de-overlap
pass insists on, as a multiple of true device size. Raising it makes the pass reject positions
that the tail phase considered acceptable, which is how you tighten the final legality check --
but every increment makes conflicts harder to resolve locally and pushes more cases into the retry
loop. Values much above 1.1 change the character of the pass considerably.

**`tailDeoverlapRhoMax`** (default `0.15`). The hard cap on how much total area
the de-overlap pass may add. The pass will refuse a solution that grows the placement by more than
this fraction, and will fail the conflict instead. Raise it when the pass reports failures on a
case where you would rather accept a larger die; lower it when the de-overlap step is inflating
the result more than you want.

**`lcrRetryEnable` and `lcrRetryMaxRounds`** (defaults `true`
and `6`). The retry loop that re-runs placement with enlarged halos when the discrete pass fails.
Leave it on: it costs nothing on cases that already succeed, because it only fires on failure.
Turn it off when you want to see the raw de-overlap failure for debugging, or when you are timing
a run and want a single pass. Raising the round count past 6 rarely helps -- a case that has not
converged by then is usually one where the geometry cannot be fixed by growing halos, and each
extra round costs a full placement pass.

**`pmosHardExpandForGr`** (default `650.0`). The half-width of space reserved around every PMOS
device for a guard ring, in placement length units. This is a geometric fact about your process
and device set, not a tuning knob in the usual sense: set it from the guard-ring width your
technology requires. Setting it too small produces placements where guard rings would not fit;
setting it too large wastes area on every PMOS in the design, which compounds quickly in a
device-dense analog block.

**`randomSeed`** (default `1`) **and `initSpreadRatio`** (default `1.0`). Together these control
the starting point. Changing the seed gives a different but equally valid starting configuration,
and running a handful of seeds is the standard way to find out how much of your result is the
algorithm and how much is luck. `initSpreadRatio` controls how far from the region centre the
initial spread reaches: values below 1.0 start the devices closer together, which converges faster
on small cases but can trap a larger one in a poor arrangement. Setting `randomSeed` to `null`
gives up reproducibility and should be reserved for deliberate randomised exploration.
