# FastLoop integration contract

Baseline: `sfu-rsl/FastLoop@eb2050fe56d6f6cc7bf5c27122332e2a7aabd9b4`.

## Required refactor, in order

### 1. Split Graphite PGO solve from live-map apply

`OptimizerGPU::OptimizeEssentialGraph4DoF()` currently has two phases in one
function. Everything before `unique_lock<mutex> lock(pMap->mMutexMapUpdate)` is
the solve phase; everything after it writes poses and map points.

Replace it with three functions:

```cpp
EssentialGraphSnapshot CaptureEssentialGraph4DoF(...);
EssentialGraphDelta SolveEssentialGraph4DoF(
    const EssentialGraphSnapshot& snapshot);
bool ApplyEssentialGraphDelta(const EssentialGraphDelta& delta);
```

`EssentialGraphSnapshot` must own copied poses, point positions, reference-KF
IDs, edge measurements and information matrices. The worker may retain opaque
`KeyFrame*`/`MapPoint*` handles for identity only; it must not read mutable
fields through them after capture.

### 2. Split fusion search from replacement

FastLoop already nearly has this split. In `GPUSearchAndFuse()`, stop after
`matcher.GPUFuse(...)` and return a `FusionPlan` of stable map-point IDs. Move
the `pRep->Replace(...)` loop into atomic commit. The CPU path must follow the
same contract.

### 3. Add mutation hooks at semantic write sites

Record mutations after successful writes, not in every getter/setter:

- `LocalMapping::ProcessNewKeyFrame`: keyframe create, related=parent KF;
- map-point creation: map point create, related=reference KF;
- local BA result publication: pose/position mutations;
- keyframe/map-point culling: mark-bad mutations;
- observation add/erase and replacement: observation/topology mutations.

Do not call mutation hooks from the AsyncLoop apply callback: the transaction
engine advances versions for its own write intents after the callback succeeds.
Doing both double-counts versions and can deadlock.

### 4. Replace the loop-only call site

In `LoopClosing::Run()`, replace only the `CorrectLoop()` call for a valid loop
with submission of one `AsyncJob`. Keep map merge and GBA on the serial path in
the first paper implementation. This keeps the contribution focused and avoids
pretending that Atlas multi-map merge is solved.

The job's phases are:

```text
capture -> detached fusion search -> detached Graphite PGO
        -> collect mutation journal -> rebase -> validate -> atomic apply
```

If submission is rejected because another loop is active, or validation fails,
queue one serial fallback. Never run overlapping loop transactions.

### 5. Commit contents

The apply callback may only:

- set optimized poses/velocities;
- set corrected map-point positions;
- apply validated map-point replacements/observations;
- add loop edges;
- update affected connections;
- increment the map change index and notify the Atlas.

It must not run projection search, descriptor matching or PGO. Those operations
inside the commit section would destroy the near-zero-stall claim.

## Invariants to assert in debug builds

1. Every delta target belongs to the captured active map.
2. Every read-set version equals its captured version at commit.
3. Every fusion winner/loser is non-null, distinct and not bad.
4. Every post-snapshot KF has a valid parent chain to a corrected anchor.
5. No map-update lock is held when launching/synchronizing a CUDA kernel.
6. Commit never invokes a callback that records a second mutation.
7. On any failed invariant, discard the delta and execute serial fallback.

## Paper-facing measurements

Instrument capture, solve, rebase, validation and commit independently. Report
P50/P95/P99/max tracking latency, LocalMapping stop time, rejected keyframes,
dropped frames, commit duration, conflict/rollback rate, memory overhead and
ATE/RPE. Mean loop-closing time alone is insufficient evidence for AsyncLoop.
