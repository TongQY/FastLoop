# AsyncLoop

AsyncLoop turns FastLoop's stop-the-world loop correction into a speculative,
transactional correction pipeline for stereo/RGB-D inertial SLAM.

## Implemented path

- immutable capture of keyframe poses, map-point positions and essential-graph
  factors;
- detached Graphite pose-graph optimization with no live-map writes;
- GPU fusion search that returns a delayed replacement plan;
- LocalMapping mutation journal with conservative barriers around local BA,
  culling and neighbor fusion;
- SE(3) rebase for keyframes and map points appended after the snapshot;
- read/write validation and atomic delta publication;
- automatic fallback to FastLoop's original serial `CorrectLoop()` whenever a
  transaction cannot be proven safe;
- capture/solve/commit timing and conflict/fallback counters.

The initial research implementation intentionally enables the transactional path
only for initialized inertial maps with GPU fusion. Map merge, monocular Sim(3)
correction, GBA overlap and CPU fusion retain the original serial behavior.

## Build

```bash
git clone --recursive -b codex/asyncloop https://github.com/TongQY/FastLoop.git
cd FastLoop
chmod +x build.sh scripts/run_asyncloop_core_tests.sh
scripts/run_asyncloop_core_tests.sh
./build.sh
```

Set `ASYNCLOOP_DISABLE=1` to force the unmodified FastLoop correction path.

## Correctness policy

AsyncLoop fails closed. A concurrent local BA, keyframe/map-point culling,
neighbor fusion, incomplete mutation journal, stale entity, invalid fusion
endpoint or detached-solver exception discards the speculative result and calls
FastLoop's original serial correction. A failed transaction never partially
updates the map.

## Required evaluation

Report P50/P95/P99/max tracking latency, LocalMapping stop time, commit time,
rejected keyframes, dropped frames, fallback/conflict rate, memory overhead and
ATE/RPE. Mean loop-closing time alone does not validate the contribution.
