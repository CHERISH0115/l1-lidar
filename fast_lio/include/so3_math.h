#pragma once
#include <Eigen/Eigen>

using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;

// 反对称（hat）算子: v -> [v]×
inline M3D hat(const V3D &v) {
    M3D m;
    m <<      0, -v(2),  v(1),
           v(2),     0, -v(0),
          -v(1),  v(0),     0;
    return m;
}

// SO(3) 指数映射: so(3) -> SO(3)
inline M3D Exp(const V3D &ang) {
    double ang_norm = ang.norm();
    if (ang_norm < 1e-10) return M3D::Identity();
    V3D a = ang / ang_norm;
    double s = std::sin(ang_norm);
    double c = std::cos(ang_norm);
    return c * M3D::Identity() + (1 - c) * a * a.transpose() + s * hat(a);
}

// SO(3) 对数映射: SO(3) -> so(3)
inline V3D Log(const M3D &R) {
    double trace = R.trace();
    double cos_a = (trace - 1.0) * 0.5;
    cos_a = std::clamp(cos_a, -1.0, 1.0);
    double ang = std::acos(cos_a);
    if (std::abs(ang) < 1e-10) return V3D::Zero();
    return ang / (2.0 * std::sin(ang)) * V3D(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));
}

// 轴角旋转一个向量
inline V3D rotate(const M3D &R, const V3D &v) { return R * v; }

// 右雅可比近似 Jr(φ) ≈ I - 0.5 [φ]×
inline M3D Jr(const V3D &phi) {
    return M3D::Identity() - 0.5 * hat(phi);
}
