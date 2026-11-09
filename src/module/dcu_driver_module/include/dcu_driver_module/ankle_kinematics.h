// New F1 geometry kernel, ported from ankle_parallel_validation.
#pragma once
#include <array>
#include <cstddef>

namespace xyber_x1_infer::dcu_driver_module::ankle {
using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;
using Mat2 = std::array<double, 4>;  // Row-major: dq_motor / dq_joint.
struct Branch {
  Vec3 motor, foot;
  double radius, length, zero_angle, assembly_sign, motor_direction;
};
using Geometry = std::array<Branch, 2>;  // Long rod, then short rod.
Geometry unpack(const double* values, std::size_t count);  // 11 values per branch.
Vec2 constraint_residual(const Geometry&, const Vec2& pitch_roll, const Vec2& motor);
Vec2 inverse_position(const Geometry&, const Vec2& pitch_roll);
Mat2 jacobian(const Geometry&, const Vec2& pitch_roll, const Vec2& motor);
Vec2 joint_to_motor_velocity(const Mat2&, const Vec2&);
Vec2 motor_to_joint_velocity(const Mat2&, const Vec2&);
Vec2 joint_to_motor_torque(const Mat2&, const Vec2&);
Vec2 motor_to_joint_torque(const Mat2&, const Vec2&);
}  // namespace ankle

