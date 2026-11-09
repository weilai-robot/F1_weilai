// Geometry/Jacobian kernel port; feedback effort units are not changed here.
#include "dcu_driver_module/ankle_kinematics.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ankle = xyber_x1_infer::dcu_driver_module::ankle;

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
double dot(const ankle::Vec3& a, const ankle::Vec3& b) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
ankle::Vec3 rotate(const ankle::Vec3& p, const ankle::Vec2& q) {
  const double c5=std::cos(q[0]), s5=std::sin(q[0]);
  const double c6=std::cos(q[1]), s6=std::sin(q[1]);
  const double y=c6*p[1]-s6*p[2], z=s6*p[1]+c6*p[2];
  return {c5*p[0]+s5*z, y, -s5*p[0]+c5*z};
}
ankle::Vec2 multiply(const ankle::Mat2& a, const ankle::Vec2& x) {
  return {a[0]*x[0]+a[1]*x[1], a[2]*x[0]+a[3]*x[1]};
}
ankle::Mat2 transpose(const ankle::Mat2& a) {
  return {a[0],a[2],a[1],a[3]};
}
ankle::Mat2 inverse(const ankle::Mat2& a) {
  const double det=a[0]*a[3]-a[1]*a[2];
  const double scale=std::max({std::abs(a[0]),std::abs(a[1]),std::abs(a[2]),std::abs(a[3])});
  require(std::isfinite(det) && scale>0 && std::abs(det)>1e-12*scale*scale,
          "Singular motor/joint Jacobian");
  return {a[3]/det,-a[1]/det,-a[2]/det,a[0]/det};
}
}

namespace xyber_x1_infer::dcu_driver_module::ankle {
Geometry unpack(const double* x, std::size_t count) {
  require(count==22, "Expected 22 geometry values (ABI v2)");
  require(x, "Null geometry pointer");
  Geometry g{};
  for (int i=0;i<2;++i) {
    const double* b=x+11*i;
    for (int j=0;j<11;++j) require(std::isfinite(b[j]), "Non-finite geometry value");
    g[i]={{b[0],b[1],b[2]}, {b[3],b[4],b[5]}, b[6],b[7],b[8],b[9],b[10]};
    require(b[6]>0 && b[7]>0, "Radius and rod length must be positive");
    require(b[9]==1 || b[9]==-1, "Assembly sign must be +1 or -1");
    require(b[10]==1 || b[10]==-1, "Motor direction must be +1 or -1");
  }
  return g;
}

Vec2 constraint_residual(const Geometry& g, const Vec2& q, const Vec2& m) {
  require(std::isfinite(q[0]) && std::isfinite(q[1]) &&
          std::isfinite(m[0]) && std::isfinite(m[1]), "Non-finite state");
  Vec2 result{};
  for (int i=0;i<2;++i) {
    const auto& b=g[i];
    const auto p=rotate(b.foot,q);
    const double theta=b.zero_angle+b.motor_direction*m[i];
    const Vec3 d{p[0]-b.motor[0],p[1]-b.motor[1]+b.radius*std::sin(theta),
                 p[2]-b.motor[2]-b.radius*std::cos(theta)};
    result[i]=std::sqrt(dot(d,d))-b.length;
  }
  return result;
}

Vec2 inverse_position(const Geometry& g, const Vec2& q) {
  require(std::isfinite(q[0]) && std::isfinite(q[1]), "Non-finite joint position");
  Vec2 result{};
  for (int i=0;i<2;++i) {
    const auto& b=g[i];
    const auto p=rotate(b.foot,q);
    const double dx=p[0]-b.motor[0], dy=p[1]-b.motor[1], dz=b.motor[2]-p[2];
    const double a=2*b.radius*dy, c=2*b.radius*dz;
    const double rhs=b.length*b.length-dx*dx-dy*dy-dz*dz-b.radius*b.radius;
    const double rho=std::hypot(a,c);
    require(rho>1e-15, "Degenerate crank constraint");
    const double argument=rhs/rho;
    require(argument>=-1-1e-12 && argument<=1+1e-12, "Unreachable joint position");
    const double theta=std::atan2(a,c)+b.assembly_sign*std::acos(std::clamp(argument,-1.0,1.0));
    result[i]=std::remainder((theta-b.zero_angle)/b.motor_direction,2*kPi);
  }
  return result;
}

Mat2 jacobian(const Geometry& g, const Vec2& q, const Vec2& m) {
  require(std::isfinite(q[0]) && std::isfinite(q[1]) &&
          std::isfinite(m[0]) && std::isfinite(m[1]), "Non-finite state");
  Mat2 j{};
  for (int i=0;i<2;++i) {
    const auto& b=g[i];
    const auto p=rotate(b.foot,q);
    const double theta=b.motor_direction*m[i]+b.zero_angle;
    const double s=std::sin(theta), c=std::cos(theta);
    const Vec3 crank{b.motor[0],b.motor[1]-b.radius*s,b.motor[2]+b.radius*c};
    const Vec3 d{p[0]-crank[0],p[1]-crank[1],p[2]-crank[2]};
    const Vec3 dc{0,-b.motor_direction*b.radius*c,-b.motor_direction*b.radius*s};
    const Vec3 dp5{p[2],0,-p[0]};
    const double yz=b.foot[1]*std::cos(q[1])-b.foot[2]*std::sin(q[1]);
    const double zz=b.foot[1]*std::sin(q[1])+b.foot[2]*std::cos(q[1]);
    const Vec3 dp6{std::sin(q[0])*yz,-zz,std::cos(q[0])*yz};
    const double denominator=dot(d,dc);
    require(std::abs(denominator)>1e-12*b.length*b.radius, "Singular crank constraint");
    // d.F/dq + d.F/dm * dm/dq = 0, F = |foot-crank|^2 - L^2.
    j[2*i]=dot(d,dp5)/denominator;
    j[2*i+1]=dot(d,dp6)/denominator;
  }
  return j;
}
Vec2 joint_to_motor_velocity(const Mat2& j, const Vec2& v) { return multiply(j,v); }
Vec2 motor_to_joint_velocity(const Mat2& j, const Vec2& v) { return multiply(inverse(j),v); }
Vec2 joint_to_motor_torque(const Mat2& j, const Vec2& t) { return multiply(inverse(transpose(j)),t); }
Vec2 motor_to_joint_torque(const Mat2& j, const Vec2& t) { return multiply(transpose(j),t); }
}

