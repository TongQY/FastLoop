#ifndef ASYNCLOOP_ASYNC_CONTROLLER_H
#define ASYNCLOOP_ASYNC_CONTROLLER_H

#include "asyncloop/TransactionEngine.h"

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace asyncloop {

class CorrectionPayload {
 public:
  virtual ~CorrectionPayload() = default;
};

struct PreparedCommit {
  CommitPlan plan;
  // Must be a short, no-partial-failure map update. Heavy fusion and PGO work
  // belongs in optimize/rebase, not in this callback.
  std::function<bool()> apply;
};

struct AsyncJob {
  Snapshot snapshot;

  // Runs on the worker without the transaction gate.
  std::function<std::unique_ptr<CorrectionPayload>()> optimize;

  // Converts the immutable correction plus post-snapshot mutations into a
  // rebased commit plan. It also runs without the transaction gate.
  std::function<PreparedCommit(
      const Snapshot&,
      const CorrectionPayload&,
      const std::vector<Mutation>&)> rebase;
};

enum class JobCode : std::uint8_t {
  Idle,
  Queued,
  Optimizing,
  Rebase,
  Committed,
  Conflict,
  JournalGap,
  OptimizeFailed,
  ApplyRejected,
  Cancelled
};

struct ControllerStats {
  std::uint64_t submitted{0};
  std::uint64_t committed{0};
  std::uint64_t conflicts{0};
  std::uint64_t fallbacks{0};
  std::uint64_t rejected_busy{0};
  JobCode last_code{JobCode::Idle};
};

class AsyncController {
 public:
  explicit AsyncController(TransactionEngine& engine);
  ~AsyncController();

  AsyncController(const AsyncController&) = delete;
  AsyncController& operator=(const AsyncController&) = delete;

  // At most one queued/running loop is allowed. This avoids overlapping map
  // corrections and makes the paper's consistency model explicit.
  bool submit(AsyncJob job);
  void cancel();
  void waitIdle();
  bool busy() const;
  ControllerStats stats() const;

 private:
  void workerMain();
  void setLastCode(JobCode code);

  TransactionEngine& engine_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::condition_variable idle_cv_;
  bool stop_{false};
  bool has_job_{false};
  bool running_{false};
  bool cancel_requested_{false};
  AsyncJob job_;
  ControllerStats stats_;
  std::thread worker_;
};

const char* toString(JobCode code) noexcept;

}  // namespace asyncloop

#endif  // ASYNCLOOP_ASYNC_CONTROLLER_H
