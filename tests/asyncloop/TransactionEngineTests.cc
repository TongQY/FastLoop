#include "asyncloop/TransactionEngine.h"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

namespace {

using asyncloop::CommitCode;
using asyncloop::CommitPlan;
using asyncloop::EntityKey;
using asyncloop::EntityKind;
using asyncloop::MutationKind;
using asyncloop::TransactionEngine;
using asyncloop::ValidationCode;
using asyncloop::WriteIntent;

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

EntityKey mp(std::uint64_t id) {
  return EntityKey{EntityKind::MapPoint, id};
}

CommitPlan planFrom(const asyncloop::Snapshot& snapshot) {
  CommitPlan plan;
  plan.snapshot_id = snapshot.id;
  plan.snapshot_epoch = snapshot.epoch;
  plan.read_set = snapshot.read_set;
  return plan;
}

void cleanCommitUpdatesVersion() {
  TransactionEngine engine;
  engine.recordMutation(kf(1), MutationKind::Create);
  const auto snapshot = engine.capture({kf(1), mp(10)});
  CommitPlan plan = planFrom(snapshot);
  plan.writes.push_back(
      WriteIntent{kf(1), engine.versionOf(kf(1)), MutationKind::Pose});
  plan.writes.push_back(
      WriteIntent{mp(10), engine.versionOf(mp(10)), MutationKind::Position});

  int applied = 0;
  const auto result = engine.commit(plan, [&applied] {
    ++applied;
    return true;
  });
  CHECK(result.code == CommitCode::Committed);
  CHECK(applied == 1);
  CHECK(engine.versionOf(kf(1)) == 2);
  CHECK(engine.versionOf(mp(10)) == 1);
}

void changedSnapshotEntityConflicts() {
  TransactionEngine engine;
  engine.recordMutation(kf(3), MutationKind::Create);
  const auto snapshot = engine.capture({kf(3)});
  engine.recordMutation(kf(3), MutationKind::Pose);
  CommitPlan plan = planFrom(snapshot);
  plan.writes.push_back(WriteIntent{kf(3), 1, MutationKind::Pose});

  bool called = false;
  const auto result = engine.commit(plan, [&called] {
    called = true;
    return true;
  });
  CHECK(result.code == CommitCode::ValidationFailed);
  CHECK(result.validation.code == ValidationCode::ReadConflict);
  CHECK(!called);
}

void appendOnlyEntityCanBeRebased() {
  TransactionEngine engine;
  engine.recordMutation(kf(7), MutationKind::Create);
  const auto snapshot = engine.capture({kf(7)});

  // LocalMapping appended KF 8 after the snapshot. It is not part of the
  // optimized read set. The bridge computes its rebased pose and adds a write
  // intent against the current version.
  engine.recordMutation(kf(8), MutationKind::Create, kf(7));
  CommitPlan plan = planFrom(snapshot);
  plan.writes.push_back(
      WriteIntent{kf(8), engine.versionOf(kf(8)), MutationKind::Pose});

  const auto result = engine.commit(plan, [] { return true; });
  CHECK(result.code == CommitCode::Committed);
  CHECK(engine.versionOf(kf(8)) == 2);
}

void boundedJournalFailsClosed() {
  TransactionEngine engine(2);
  const auto snapshot = engine.capture({kf(1)});
  engine.recordMutation(kf(2), MutationKind::Create);
  engine.recordMutation(kf(3), MutationKind::Create);
  engine.recordMutation(kf(4), MutationKind::Create);
  CommitPlan plan = planFrom(snapshot);
  const auto validation = engine.validate(plan);
  CHECK(validation.code == ValidationCode::JournalGap);
}

void duplicateWritesAreRejected() {
  TransactionEngine engine;
  const auto snapshot = engine.capture({kf(1)});
  CommitPlan plan = planFrom(snapshot);
  plan.writes.push_back(WriteIntent{kf(1), 0, MutationKind::Pose});
  plan.writes.push_back(WriteIntent{kf(1), 0, MutationKind::Pose});
  CHECK(engine.validate(plan).code == ValidationCode::DuplicateWrite);
}

void rejectedApplyDoesNotAdvanceVersions() {
  TransactionEngine engine;
  const auto snapshot = engine.capture({mp(2)});
  CommitPlan plan = planFrom(snapshot);
  plan.writes.push_back(WriteIntent{mp(2), 0, MutationKind::Position});
  const auto result = engine.commit(plan, [] { return false; });
  CHECK(result.code == CommitCode::ApplyRejected);
  CHECK(engine.currentEpoch() == snapshot.epoch);
  CHECK(engine.versionOf(mp(2)) == 0);
}

void commitGateSerializesConcurrentWriters() {
  TransactionEngine engine;
  engine.recordMutation(kf(1), MutationKind::Create);
  const auto snapshot = engine.capture({kf(1)});
  CommitPlan plan = planFrom(snapshot);
  plan.writes.push_back(WriteIntent{kf(1), 1, MutationKind::Pose});

  std::atomic<bool> inside{false};
  std::atomic<bool> writer_finished{false};
  std::thread writer;
  const auto result = engine.commit(plan, [&] {
    inside.store(true);
    writer = std::thread([&] {
      engine.recordMutation(kf(9), MutationKind::Create);
      writer_finished.store(true);
    });
    for (int i = 0; i < 100000 && !writer_finished.load(); ++i) {
      std::this_thread::yield();
    }
    CHECK(inside.load());
    CHECK(!writer_finished.load());
    return true;
  });
  CHECK(result.code == CommitCode::Committed);
  writer.join();
  CHECK(writer_finished.load());
}

void mutationJournalIsOrdered() {
  TransactionEngine engine;
  const auto snapshot = engine.capture({});
  engine.recordMutation(kf(1), MutationKind::Create);
  engine.recordMutation(mp(2), MutationKind::Create, kf(1));
  bool complete = false;
  const auto mutations = engine.mutationsSince(snapshot.epoch, &complete);
  CHECK(complete);
  CHECK(mutations.size() == 2);
  CHECK(mutations[0].epoch < mutations[1].epoch);
  CHECK(mutations[1].related == kf(1));
}

}  // namespace

int main() {
  cleanCommitUpdatesVersion();
  changedSnapshotEntityConflicts();
  appendOnlyEntityCanBeRebased();
  boundedJournalFailsClosed();
  duplicateWritesAreRejected();
  rejectedApplyDoesNotAdvanceVersions();
  commitGateSerializesConcurrentWriters();
  mutationJournalIsOrdered();

  if (failures != 0) {
    std::cerr << failures << " test(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "AsyncLoop transaction tests passed\n";
  return EXIT_SUCCESS;
}
