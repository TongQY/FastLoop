#include "AsyncLoop/Pose3.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

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

bool close(double a, double b, double tolerance = 1e-10) {
  return std::fabs(a - b) <= tolerance;
}

void inverseRoundTrip() {
  const double c = std::sqrt(0.5);
  const Pose3 pose{Quaternion{c, 0.0, 0.0, c}, Vec3{1.0, 2.0, 3.0}};
  const Vec3 point{4.0, -2.0, 0.5};
  const Vec3 round_trip = pose.inverse() * (pose * point);
  CHECK(close(round_trip.x, point.x));
  CHECK(close(round_trip.y, point.y));
  CHECK(close(round_trip.z, point.z));
}

void childRebasePreservesRelativePose() {
  const double c = std::sqrt(0.5);
  const Pose3 parent_live{Quaternion{}, Vec3{1.0, 0.0, 0.0}};
  const Pose3 child_live{Quaternion{}, Vec3{1.0, 2.0, 0.0}};
  const Pose3 parent_corrected{
      Quaternion{c, 0.0, 0.0, c}, Vec3{10.0, 0.0, 0.0}};
  const Pose3 child_corrected =
      rebaseChildPose(parent_corrected, parent_live, child_live);

  const Pose3 relative_before = parent_live.inverse() * child_live;
  const Pose3 relative_after = parent_corrected.inverse() * child_corrected;
  CHECK(close(relative_before.t.x, relative_after.t.x));
  CHECK(close(relative_before.t.y, relative_after.t.y));
  CHECK(close(relative_before.t.z, relative_after.t.z));
}

void pointRebasePreservesReferenceCoordinates() {
  const Pose3 reference_live{Quaternion{}, Vec3{3.0, 1.0, -2.0}};
  const Pose3 reference_corrected{Quaternion{}, Vec3{8.0, 4.0, 1.0}};
  const Vec3 point_live{4.0, 3.0, 1.0};
  const Vec3 point_corrected =
      rebasePoint(reference_corrected, reference_live, point_live);
  const Vec3 local_before = reference_live.inverse() * point_live;
  const Vec3 local_after = reference_corrected.inverse() * point_corrected;
  CHECK(close(local_before.x, local_after.x));
  CHECK(close(local_before.y, local_after.y));
  CHECK(close(local_before.z, local_after.z));
}

void invalidQuaternionIsRejected() {
  bool threw = false;
  try {
    const Pose3 bad{Quaternion{0.0, 0.0, 0.0, 0.0}, Vec3{}};
    static_cast<void>(bad.inverse());
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

}  // namespace

int main() {
  inverseRoundTrip();
  childRebasePreservesRelativePose();
  pointRebasePreservesReferenceCoordinates();
  invalidQuaternionIsRejected();
  if (failures != 0) {
    std::cerr << failures << " test(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "AsyncLoop pose/rebase tests passed\n";
  return EXIT_SUCCESS;
}
