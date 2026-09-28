#include "AsyncLoop/Pose3.h"

#include <cmath>
#include <stdexcept>

namespace asyncloop {
namespace {

Quaternion conjugate(const Quaternion& q) noexcept {
  return Quaternion{q.w, -q.x, -q.y, -q.z};
}

Quaternion multiply(const Quaternion& a, const Quaternion& b) noexcept {
  return Quaternion{
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

Vec3 add(const Vec3& a, const Vec3& b) noexcept {
  return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 negate(const Vec3& value) noexcept {
  return Vec3{-value.x, -value.y, -value.z};
}

Vec3 rotate(const Quaternion& q, const Vec3& point) noexcept {
  const Quaternion p{0.0, point.x, point.y, point.z};
  const Quaternion result = multiply(multiply(q, p), conjugate(q));
  return Vec3{result.x, result.y, result.z};
}

}  // namespace

Pose3 Pose3::Identity() noexcept {
  return Pose3{};
}

Pose3 Pose3::normalized() const {
  const double squared = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (!std::isfinite(squared) || squared <= 1e-24) {
    throw std::invalid_argument("Pose3 quaternion has zero or non-finite norm");
  }
  const double inverse_norm = 1.0 / std::sqrt(squared);
  return Pose3{Quaternion{q.w * inverse_norm, q.x * inverse_norm,
                          q.y * inverse_norm, q.z * inverse_norm},
               t};
}

Pose3 Pose3::inverse() const {
  const Pose3 unit = normalized();
  const Quaternion inverse_rotation = conjugate(unit.q);
  return Pose3{inverse_rotation, rotate(inverse_rotation, negate(unit.t))};
}

Pose3 Pose3::operator*(const Pose3& rhs) const {
  const Pose3 lhs_unit = normalized();
  const Pose3 rhs_unit = rhs.normalized();
  Pose3 result;
  result.q = multiply(lhs_unit.q, rhs_unit.q);
  result.t = add(rotate(lhs_unit.q, rhs_unit.t), lhs_unit.t);
  return result.normalized();
}

Vec3 Pose3::operator*(const Vec3& point) const {
  const Pose3 unit = normalized();
  return add(rotate(unit.q, point), unit.t);
}

Pose3 rebaseChildPose(const Pose3& world_from_parent_corrected,
                      const Pose3& world_from_parent_live,
                      const Pose3& world_from_child_live) {
  return world_from_parent_corrected * world_from_parent_live.inverse() *
         world_from_child_live;
}

Vec3 rebasePoint(const Pose3& world_from_reference_corrected,
                 const Pose3& world_from_reference_live,
                 const Vec3& world_point_live) {
  return world_from_reference_corrected *
         (world_from_reference_live.inverse() * world_point_live);
}

bool isFinite(const Pose3& pose) noexcept {
  return std::isfinite(pose.q.w) && std::isfinite(pose.q.x) &&
         std::isfinite(pose.q.y) && std::isfinite(pose.q.z) &&
         std::isfinite(pose.t.x) && std::isfinite(pose.t.y) &&
         std::isfinite(pose.t.z);
}

}  // namespace asyncloop
