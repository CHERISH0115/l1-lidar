#pragma once
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <deque>
#include <mutex>

// ── 基础类型别名 ──────────────────────────────────────────────
using V3D  = Eigen::Vector3d;
using M3D  = Eigen::Matrix3d;
using V3F  = Eigen::Vector3f;
using QD   = Eigen::Quaterniond;
using MatXd = Eigen::MatrixXd;

// ── 自定义点类型（含相对时间戳） ─────────────────────────────
struct PointXYZIT {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)(float, time, time))

using PointType  = PointXYZIT;
using CloudType  = pcl::PointCloud<PointType>;

// ── IMU 原始测量 ──────────────────────────────────────────────
struct ImuMeas {
    double   timestamp;
    V3D      acc;   // m/s²
    V3D      gyr;   // rad/s
};

// ── FAST-LIO2 状态向量 ────────────────────────────────────────
struct State {
    V3D  pos   = V3D::Zero();      // 位置
    M3D  rot   = M3D::Identity();  // 旋转
    V3D  vel   = V3D::Zero();      // 速度
    V3D  bg    = V3D::Zero();      // 陀螺零偏
    V3D  ba    = V3D::Zero();      // 加速度零偏
    V3D  grav  = V3D(0, 0, -9.80);// 重力（世界系）
    double time = 0.0;
};

// 状态维度: pos(3)+rot(3)+vel(3)+bg(3)+ba(3) = 15
constexpr int STATE_DIM = 15;
constexpr int MEAS_DIM  = 3;   // 每个点产生1个点到面残差

// ── 常量 ─────────────────────────────────────────────────────
constexpr double G_NORM    = 9.80;
constexpr int    NUM_MATCH = 5;   // KNN 邻居数
constexpr double PLANE_THR = 0.1; // 平面判定阈值 (m)
