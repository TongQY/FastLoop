#ifndef ASYNCLOOP_POSE3_H
#define ASYNCLOOP_POSE3_H

#include <array>

namespace asyncloop {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// Rigid transform with left action: p_a = T_ab * p_b.
struct Pose3 {
  Quaternion q;
  Vec3 t;

  static Pose3 Identity() noexcept;
  Pose3 normalized() const;
  Pose3 inverse() const;
  Pose3 operator*(const Pose3& rhs) const;
  Vec3 operator*(const Vec3& point) const;
};

// Keeps a post-snapshot child at the same child-to-parent transform while
// transferring the parent's loop correction:
// T_w_child_new = T_w_parent_corrected * inv(T_w_parent_live) * T_w_child_live.
Pose3 rebaseChildPose(const Pose3& world_from_parent_corrected,
                      const Pose3& world_from_parent_live,
                      const Pose3& world_from_child_live);

// Re-expresses a live world point through its corrected reference frame.
Vec3 rebasePoint(const Pose3& world_from_reference_corrected,
                 const Pose3& world_from_reference_live,
                 const Vec3& world_point_live);

bool isFinite(const Pose3& pose) noexcept;

}  // namespace asyncloop

#endif  // ASYNCLOOP_POSE3_H
