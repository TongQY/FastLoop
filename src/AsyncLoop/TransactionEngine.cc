#include "AsyncLoop/TransactionEngine.h"

#include <sstream>
#include <unordered_set>

namespace asyncloop {

namespace {

ValidationResult makeResult(ValidationCode code,
                            const EntityKey& entity,
                            std::uint64_t expected,
                            std::uint64_t actual,
                            const char* reason) {
  ValidationResult result;
  result.code = code;
  result.entity = entity;
  result.expected_version = expected;
  result.actual_version = actual;
  std::ostringstream stream;
  stream << reason << " entity(kind=" << static_cast<int>(entity.kind)
         << ", id=" << entity.id << "), expected=" << expected
         << ", actual=" << actual;
  result.message = stream.str();
  return result;
}

}  // namespace

std::size_t EntityKeyHash::operator()(const EntityKey& key) const noexcept {
  const std::size_t a = std::hash<std::uint64_t>{}(key.id);
  const std::size_t b = std::hash<unsigned>{}(
      static_cast<unsigned>(key.kind));
  return a ^ (b + 0x9e3779b9U + (a << 6U) + (a >> 2U));
}

TransactionEngine::TransactionEngine(std::size_t journal_capacity)
    : journal_capacity_(journal_capacity == 0 ? 1 : journal_capacity) {}

Snapshot TransactionEngine::capture(
    const std::vector<EntityKey>& read_set) {
  std::lock_guard<std::mutex> lock(mutex_);
  Snapshot snapshot;
  snapshot.id = next_snapshot_id_++;
  snapshot.epoch = epoch_;
  snapshot.read_set.reserve(read_set.size());

  std::unordered_set<EntityKey, EntityKeyHash> seen;
  for (const EntityKey& entity : read_set) {
    if (seen.insert(entity).second) {
      snapshot.read_set.push_back(
          VersionedEntity{entity, versionOfLocked(entity)});
    }
  }
  return snapshot;
}

Epoch TransactionEngine::recordMutation(const EntityKey& entity,
                                        MutationKind kind,
                                        const EntityKey& related) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++epoch_;
  ++versions_[entity];
  appendMutationLocked(Mutation{epoch_, entity, kind, related});
  return epoch_;
}

std::uint64_t TransactionEngine::versionOf(const EntityKey& entity) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return versionOfLocked(entity);
}

Epoch TransactionEngine::currentEpoch() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return epoch_;
}

std::vector<Mutation> TransactionEngine::mutationsSince(
    Epoch snapshot_epoch,
    bool* complete) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const Epoch oldest_available = journal_.empty() ? epoch_ + 1
                                                   : journal_.front().epoch;
  const bool has_full_interval = snapshot_epoch >= epoch_ ||
      snapshot_epoch + 1 >= oldest_available;
  if (complete != nullptr) {
    *complete = has_full_interval;
  }
  std::vector<Mutation> result;
  if (!has_full_interval) {
    return result;
  }
  for (const Mutation& mutation : journal_) {
    if (mutation.epoch > snapshot_epoch) {
      result.push_back(mutation);
    }
  }
  return result;
}

ValidationResult TransactionEngine::validate(const CommitPlan& plan) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return validateLocked(plan);
}

CommitResult TransactionEngine::commit(const CommitPlan& plan,
                                       const std::function<bool()>& apply) {
  std::lock_guard<std::mutex> lock(mutex_);
  const ValidationResult validation = validateLocked(plan);
  if (!validation) {
    return CommitResult{CommitCode::ValidationFailed, validation, epoch_};
  }

  if (!apply || !apply()) {
    ValidationResult rejected;
    rejected.code = ValidationCode::Valid;
    rejected.message = "external apply callback rejected the commit";
    return CommitResult{CommitCode::ApplyRejected, rejected, epoch_};
  }

  for (const WriteIntent& write : plan.writes) {
    ++epoch_;
    ++versions_[write.entity];
    appendMutationLocked(
        Mutation{epoch_, write.entity, write.kind, EntityKey{}});
  }
  return CommitResult{CommitCode::Committed, ValidationResult{}, epoch_};
}

CommitResult TransactionEngine::commitPrepared(
    const CommitPlan& plan,
    PreparedPublication& publication) {
  std::lock_guard<std::mutex> lock(mutex_);
  const ValidationResult validation = validateLocked(plan);
  if (!validation) {
    return CommitResult{CommitCode::ValidationFailed, validation, epoch_};
  }

  // Build the complete transaction metadata before publishing any external
  // map state. Copies and container growth may throw; swaps and publish are
  // the allocation-free commit point.
  VersionMap prepared_versions = versions_;
  std::deque<Mutation> prepared_journal = journal_;
  Epoch prepared_epoch = epoch_;
  for (const WriteIntent& write : plan.writes) {
    ++prepared_epoch;
    ++prepared_versions[write.entity];
    prepared_journal.push_back(
        Mutation{prepared_epoch, write.entity, write.kind, EntityKey{}});
    while (prepared_journal.size() > journal_capacity_) {
      prepared_journal.pop_front();
    }
  }

  versions_.swap(prepared_versions);
  journal_.swap(prepared_journal);
  epoch_ = prepared_epoch;
  publication.publish();

  return CommitResult{CommitCode::Committed, ValidationResult{}, epoch_};
}

std::size_t TransactionEngine::journalSize() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return journal_.size();
}

ValidationResult TransactionEngine::validateLocked(
    const CommitPlan& plan) const {
  if (plan.snapshot_id == 0 || plan.snapshot_epoch > epoch_) {
    return makeResult(ValidationCode::StaleSnapshot, EntityKey{},
                      plan.snapshot_epoch, epoch_, "invalid snapshot");
  }

  if (!journal_.empty() && plan.snapshot_epoch < epoch_ &&
      plan.snapshot_epoch + 1 < journal_.front().epoch) {
    return makeResult(ValidationCode::JournalGap, EntityKey{},
                      plan.snapshot_epoch + 1, journal_.front().epoch,
                      "mutation journal gap");
  }

  for (const VersionedEntity& read : plan.read_set) {
    const std::uint64_t actual = versionOfLocked(read.entity);
    if (actual != read.version) {
      return makeResult(ValidationCode::ReadConflict, read.entity,
                        read.version, actual, "snapshot read conflict");
    }
  }

  std::unordered_set<EntityKey, EntityKeyHash> write_entities;
  for (const WriteIntent& write : plan.writes) {
    if (!write_entities.insert(write.entity).second) {
      return makeResult(ValidationCode::DuplicateWrite, write.entity,
                        write.expected_version, write.expected_version,
                        "duplicate write intent");
    }
    const std::uint64_t actual = versionOfLocked(write.entity);
    if (actual != write.expected_version) {
      return makeResult(ValidationCode::WriteConflict, write.entity,
                        write.expected_version, actual,
                        "commit write conflict");
    }
  }

  return ValidationResult{};
}

std::uint64_t TransactionEngine::versionOfLocked(
    const EntityKey& entity) const {
  const auto found = versions_.find(entity);
  return found == versions_.end() ? 0 : found->second;
}

void TransactionEngine::appendMutationLocked(const Mutation& mutation) {
  journal_.push_back(mutation);
  while (journal_.size() > journal_capacity_) {
    journal_.pop_front();
  }
}

const char* toString(ValidationCode code) noexcept {
  switch (code) {
    case ValidationCode::Valid: return "valid";
    case ValidationCode::StaleSnapshot: return "stale_snapshot";
    case ValidationCode::JournalGap: return "journal_gap";
    case ValidationCode::ReadConflict: return "read_conflict";
    case ValidationCode::WriteConflict: return "write_conflict";
    case ValidationCode::DuplicateWrite: return "duplicate_write";
  }
  return "unknown";
}

const char* toString(CommitCode code) noexcept {
  switch (code) {
    case CommitCode::Committed: return "committed";
    case CommitCode::ValidationFailed: return "validation_failed";
    case CommitCode::ApplyRejected: return "apply_rejected";
  }
  return "unknown";
}

}  // namespace asyncloop
