#include "asyncloop/AsyncController.h"

#include <exception>
#include <utility>

namespace asyncloop {

AsyncController::AsyncController(TransactionEngine& engine)
    : engine_(engine), worker_(&AsyncController::workerMain, this) {}

AsyncController::~AsyncController() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    cancel_requested_ = true;
  }
  cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool AsyncController::submit(AsyncJob job) {
  if (!job.optimize || !job.rebase || job.snapshot.id == 0) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_ || has_job_ || running_) {
      ++stats_.rejected_busy;
      return false;
    }
    job_ = std::move(job);
    has_job_ = true;
    cancel_requested_ = false;
    ++stats_.submitted;
    stats_.last_code = JobCode::Queued;
  }
  cv_.notify_one();
  return true;
}

void AsyncController::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  cancel_requested_ = true;
}

void AsyncController::waitIdle() {
  std::unique_lock<std::mutex> lock(mutex_);
  idle_cv_.wait(lock, [this] { return !has_job_ && !running_; });
}

bool AsyncController::busy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_job_ || running_;
}

ControllerStats AsyncController::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void AsyncController::workerMain() {
  for (;;) {
    AsyncJob job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return stop_ || has_job_; });
      if (stop_) {
        return;
      }
      job = std::move(job_);
      has_job_ = false;
      running_ = true;
      stats_.last_code = JobCode::Optimizing;
    }

    std::unique_ptr<CorrectionPayload> correction;
    try {
      correction = job.optimize();
    } catch (const std::exception&) {
      correction.reset();
    } catch (...) {
      correction.reset();
    }

    bool cancelled = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cancelled = cancel_requested_;
    }

    JobCode outcome = JobCode::OptimizeFailed;
    if (cancelled) {
      outcome = JobCode::Cancelled;
    } else if (correction) {
      bool complete = false;
      const std::vector<Mutation> mutations =
          engine_.mutationsSince(job.snapshot.epoch, &complete);
      if (!complete) {
        outcome = JobCode::JournalGap;
      } else {
        setLastCode(JobCode::Rebase);
        try {
          PreparedCommit prepared =
              job.rebase(job.snapshot, *correction, mutations);
          const CommitResult result =
              engine_.commit(prepared.plan, prepared.apply);
          if (result.code == CommitCode::Committed) {
            outcome = JobCode::Committed;
          } else if (result.code == CommitCode::ApplyRejected) {
            outcome = JobCode::ApplyRejected;
          } else if (result.validation.code == ValidationCode::JournalGap) {
            outcome = JobCode::JournalGap;
          } else {
            outcome = JobCode::Conflict;
          }
        } catch (const std::exception&) {
          outcome = JobCode::OptimizeFailed;
        } catch (...) {
          outcome = JobCode::OptimizeFailed;
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.last_code = outcome;
      if (outcome == JobCode::Committed) {
        ++stats_.committed;
      } else if (outcome == JobCode::Conflict) {
        ++stats_.conflicts;
        ++stats_.fallbacks;
      } else if (outcome == JobCode::JournalGap ||
                 outcome == JobCode::ApplyRejected ||
                 outcome == JobCode::OptimizeFailed) {
        ++stats_.fallbacks;
      }
      running_ = false;
      cancel_requested_ = false;
    }
    idle_cv_.notify_all();
  }
}

void AsyncController::setLastCode(JobCode code) {
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.last_code = code;
}

const char* toString(JobCode code) noexcept {
  switch (code) {
    case JobCode::Idle: return "idle";
    case JobCode::Queued: return "queued";
    case JobCode::Optimizing: return "optimizing";
    case JobCode::Rebase: return "rebase";
    case JobCode::Committed: return "committed";
    case JobCode::Conflict: return "conflict";
    case JobCode::JournalGap: return "journal_gap";
    case JobCode::OptimizeFailed: return "optimize_failed";
    case JobCode::ApplyRejected: return "apply_rejected";
    case JobCode::Cancelled: return "cancelled";
  }
  return "unknown";
}

}  // namespace asyncloop
