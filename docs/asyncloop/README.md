# AsyncLoop core (FastLoop integration foundation)

This directory implements the concurrency/consistency contribution required to
turn FastLoop's stop-the-world loop correction into a transactional pipeline.
It is intentionally not another CUDA kernel optimization.

## What is implemented and tested

- versioned map snapshots;
- bounded post-snapshot mutation journal;
- read/write conflict validation that fails closed;
- short atomic commit gate;
- one-at-a-time background loop-correction controller;
- explicit busy rejection, cancellation and fallback accounting;
- SE(3) rebase for post-snapshot keyframes and map points;
- tests for clean commits, conflicts, journal gaps, duplicate writes,
  concurrent LocalMapping mutations, controller serialization and rebase
  invariants.

Run without external SLAM dependencies:

```bash
./run_tests.sh
ASYNCLOOP_SANITIZE=1 ./run_tests.sh
```

The code is C++14 because the pinned FastLoop baseline uses C++14.

## Contribution boundary

The intended runtime is:

1. Capture a lightweight immutable read set from the active map.
2. Run FastLoop fusion search and Graphite PGO on detached values.
3. Keep Tracking and LocalMapping alive; log their mutations.
4. Rebase keyframes/map points created after the snapshot.
5. Validate all snapshot inputs and fusion endpoints.
6. Apply only the prepared pose/point/fusion deltas in a short commit section.
7. Fall back to FastLoop's serial `CorrectLoop()` if the journal is incomplete,
   a read/write conflict is detected, or the detached optimizer fails.

Calling the original `CorrectLoop()` on a worker thread is **not** AsyncLoop:
that function stops LocalMapping, mutates the live map throughout fusion/PGO,
and therefore retains the same pause semantics.

## FastLoop integration status

The core is compiled and tested in isolation. It is pinned to FastLoop commit
`eb2050fe56d6f6cc7bf5c27122332e2a7aabd9b4` for the next integration step.
The full FastLoop tree and its CUDA/Graphite dependencies are required to
compile-check the adapter. Do not claim an end-to-end zero-stall result until
that build and dataset evaluation are complete.

See `docs/FASTLOOP_INTEGRATION.md` for the exact refactor points.
