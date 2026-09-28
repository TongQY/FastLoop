#ifndef ASYNCLOOP_TRANSACTION_ENGINE_H
#define ASYNCLOOP_TRANSACTION_ENGINE_H

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace asyncloop {

using EntityId = std::uint64_t;
using Epoch = std::uint64_t;
using SnapshotId = std::uint64_t;

enum class EntityKind : std::uint8_t {
  KeyFrame,
  MapPoint,
  Observation,
  CovisibilityEdge,
  LoopEdge,
  Map
};

struct EntityKey {
  EntityKind kind{EntityKind::KeyFrame};
  EntityId id{0};

  bool operator==(const EntityKey& other) const noexcept {
    return kind == other.kind && id == other.id;
  }
  bool operator!=(const EntityKey& other) const noexcept {
    return !(*this == other);
  }
};

struct EntityKeyHash {
  std::size_t operator()(const EntityKey& key) const noexcept;
};

enum class MutationKind : std::uint8_t {
  Create,
  Pose,
  Position,
  AddObservation,
  EraseObservation,
  Replace,
  MarkBad,
  GraphTopology
};

struct Mutation {
  Epoch epoch{0};
  EntityKey entity;
  MutationKind kind{MutationKind::Create};
  EntityKey related;
};

struct VersionedEntity {
  EntityKey entity;
  std::uint64_t version{0};
};

struct Snapshot {
  SnapshotId id{0};
  Epoch epoch{0};
  std::vector<VersionedEntity> read_set;
};

// A write can only be committed if the entity has not changed since the
// snapshot. New live entities are handled by the bridge's rebase phase and are
// then added here with their current expected version.
struct WriteIntent {
  EntityKey entity;
  std::uint64_t expected_version{0};
  MutationKind kind{MutationKind::Pose};
};

struct CommitPlan {
  SnapshotId snapshot_id{0};
  Epoch snapshot_epoch{0};
  std::vector<VersionedEntity> read_set;
  std::vector<WriteIntent> writes;
};

enum class ValidationCode : std::uint8_t {
  Valid,
  StaleSnapshot,
  JournalGap,
  ReadConflict,
  WriteConflict,
  DuplicateWrite
};

struct ValidationResult {
  ValidationCode code{ValidationCode::Valid};
  EntityKey entity;
  std::uint64_t expected_version{0};
  std::uint64_t actual_version{0};
  std::string message;

  explicit operator bool() const noexcept {
    return code == ValidationCode::Valid;
  }
};

enum class CommitCode : std::uint8_t {
  Committed,
  ValidationFailed,
  ApplyRejected
};

struct CommitResult {
  CommitCode code{CommitCode::ValidationFailed};
  ValidationResult validation;
  Epoch commit_epoch{0};

  explicit operator bool() const noexcept {
    return code == CommitCode::Committed;
  }
};

class PreparedPublication {
 public:
  virtual ~PreparedPublication() = default;
  virtual void publish() noexcept = 0;
};

class TransactionEngine {
 public:
  explicit TransactionEngine(std::size_t journal_capacity = 65536);

  Snapshot capture(const std::vector<EntityKey>& read_set);

  // Called by LocalMapping/Tracking write hooks. The returned epoch is the
  // logical map version after the mutation.
  Epoch recordMutation(const EntityKey& entity,
                       MutationKind kind,
                       const EntityKey& related = EntityKey{});

  std::uint64_t versionOf(const EntityKey& entity) const;
  Epoch currentEpoch() const noexcept;

  // Returns mutations with epoch > snapshot_epoch. complete=false means the
  // bounded journal no longer contains the full interval, so a transaction
  // must fall back instead of guessing.
  std::vector<Mutation> mutationsSince(Epoch snapshot_epoch,
                                       bool* complete) const;

  ValidationResult validate(const CommitPlan& plan) const;

  // apply must be short and must not partially mutate the external map when it
  // returns false. It executes while the transaction gate is held, which is the
  // only blocking section on LocalMapping.
  CommitResult commit(const CommitPlan& plan,
                      const std::function<bool()>& apply);

  // Preferred TxLoop path. Preparation, allocation and all fallible work must
  // finish before this call. A valid publication cannot require rollback.
  CommitResult commitPrepared(const CommitPlan& plan,
                              PreparedPublication& publication);

  std::size_t journalSize() const;

 private:
  using VersionMap =
      std::unordered_map<EntityKey, std::uint64_t, EntityKeyHash>;

  ValidationResult validateLocked(const CommitPlan& plan) const;
  std::uint64_t versionOfLocked(const EntityKey& entity) const;
  void appendMutationLocked(const Mutation& mutation);

  const std::size_t journal_capacity_;
  mutable std::mutex mutex_;
  VersionMap versions_;
  std::deque<Mutation> journal_;
  Epoch epoch_{0};
  SnapshotId next_snapshot_id_{1};
};

const char* toString(ValidationCode code) noexcept;
const char* toString(CommitCode code) noexcept;

}  // namespace asyncloop

#endif  // ASYNCLOOP_TRANSACTION_ENGINE_H
