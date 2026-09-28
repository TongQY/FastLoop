#include "AsyncLoop/AsyncController.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>

namespace {

using namespace asyncloop;

int failures = 0;
#define CHECK(condition)                                                     \
  do {                                                                       \
    if (!(condition)) {                                                       \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "         \
                << #condition << '\n';                                        \
      ++failures;                                                             \
    }                                                                         \
  } while (false)

EntityKey kf(std::uint64_t id) {
  return EntityKey{EntityKind::KeyFrame, id};
}

struct PoseCorrection final : CorrectionPayload {
  explicit PoseCorrection(int value_in) : value(value_in) {}
  int value;
};

void backgroundOptimizationDoesNotBlockMutations() {
  TransactionEngine engine;
  engine.recordMutation(kf(1), MutationKind::Create);
  const Snapshot snapshot = engine.capture({kf(1)});
  AsyncController controller(engine);
  std::atomic<bool> optimize_started{false};
  std::atomic<bool> release_optimize{false};
  int committed_value = 0;

  AsyncJob job;
  job.snapshot = snapshot;
  job.optimize = [&] {
    optimize_started.store(true);
    while (!release_optimize.load()) {
      std::this_thread::yield();
    }
    return std::unique_ptr<CorrectionPayload>(new PoseCorrection(42));
  };
  job.rebase = [&](const Snapshot& snap,
                   const CorrectionPayload& payload,
                   const std::vector<Mutation>& mutations) {
    CHECK(mutations.size() == 1);
    const auto& correction = static_cast<const PoseCorrection&>(payload);
    CommitPlan plan;
    plan.snapshot_id = snap.id;
    plan.snapshot_epoch = snap.epoch;
    plan.read_set = snap.read_set;
    plan.writes.push_back(
        WriteIntent{kf(2), engine.versionOf(kf(2)), MutationKind::Pose});
    return PreparedCommit{plan, [&committed_value, &correction] {
      committed_value = correction.value;
      return true;
    }};
  };
  CHECK(controller.submit(std::move(job)));
  while (!optimize_started.load()) {
    std::this_thread::yield();
  }
  // Simulates LocalMapping creating a new keyframe while loop correction runs.
  engine.recordMutation(kf(2), MutationKind::Create, kf(1));
  release_optimize.store(true);
  controller.waitIdle();
  CHECK(controller.stats().last_code == JobCode::Committed);
  CHECK(committed_value == 42);
}

void conflictFailsClosed() {
  TransactionEngine engine;
  engine.recordMutation(kf(1), MutationKind::Create);
  const Snapshot snapshot = engine.capture({kf(1)});
  AsyncController controller(engine);

  AsyncJob job;
  job.snapshot = snapshot;
  job.optimize = [] {
    return std::unique_ptr<CorrectionPayload>(new PoseCorrection(1));
  };
  job.rebase = [&](const Snapshot& snap,
                   const CorrectionPayload&,
                   const std::vector<Mutation>&) {
    CommitPlan plan;
    plan.snapshot_id = snap.id;
    plan.snapshot_epoch = snap.epoch;
    plan.read_set = snap.read_set;
    return PreparedCommit{plan, [] { return true; }};
  };

  engine.recordMutation(kf(1), MutationKind::Pose);
  CHECK(controller.submit(std::move(job)));
  controller.waitIdle();
  CHECK(controller.stats().last_code == JobCode::Conflict);
  CHECK(controller.stats().fallbacks == 1);
}

void busyControllerRejectsSecondLoop() {
  TransactionEngine engine;
  const Snapshot snapshot = engine.capture({});
  AsyncController controller(engine);
  std::atomic<bool> release{false};

  AsyncJob first;
  first.snapshot = snapshot;
  first.optimize = [&] {
    while (!release.load()) {
      std::this_thread::yield();
    }
    return std::unique_ptr<CorrectionPayload>(new PoseCorrection(0));
  };
  first.rebase = [](const Snapshot& snap,
                    const CorrectionPayload&,
                    const std::vector<Mutation>&) {
    CommitPlan plan;
    plan.snapshot_id = snap.id;
    plan.snapshot_epoch = snap.epoch;
    plan.read_set = snap.read_set;
    return PreparedCommit{plan, [] { return true; }};
  };
  CHECK(controller.submit(std::move(first)));

  AsyncJob second;
  second.snapshot = snapshot;
  second.optimize = [] {
    return std::unique_ptr<CorrectionPayload>(new PoseCorrection(0));
  };
  second.rebase = [](const Snapshot&,
                     const CorrectionPayload&,
                     const std::vector<Mutation>&) {
    return PreparedCommit{};
  };
  CHECK(!controller.submit(std::move(second)));
  release.store(true);
  controller.waitIdle();
  CHECK(controller.stats().rejected_busy == 1);
}

}  // namespace

int main() {
  backgroundOptimizationDoesNotBlockMutations();
  conflictFailsClosed();
  busyControllerRejectsSecondLoop();
  if (failures != 0) {
    std::cerr << failures << " test(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "AsyncLoop controller tests passed\n";
  return EXIT_SUCCESS;
}
