#pragma once
// IMU 传播：前向积分 + 状态协方差传递
#include "common_lib.h"
#include "so3_math.h"
#include <deque>

struct ImuPropState {
    State    s;
    MatXd    P;   // 15×15 协方差
    ImuPropState() : P(MatXd::Identity(STATE_DIM, STATE_DIM) * 1e-3) {}
};

class ImuProcessor {
public:
    // 噪声参数
    double acc_n{0.1}, gyr_n{0.01};
    double acc_w{0.001}, gyr_w{0.0001};

    // 外参
    M3D R_IL = M3D::Identity();  // IMU ← LiDAR
    V3D t_IL = V3D::Zero();

    bool   initialized{false};
    State  last_state;

    // 初始化：用静止时一段 IMU 均值估计重力方向
    bool init(const std::deque<ImuMeas> &buf, State &s) {
        if (buf.size() < 20) return false;
        V3D mean_acc = V3D::Zero();
        for (auto &m : buf) mean_acc += m.acc;
        mean_acc /= (double)buf.size();

        // 重力方向对齐
        V3D grav_body = -mean_acc.normalized() * G_NORM;
        V3D grav_world(0, 0, -G_NORM);
        V3D axis = grav_body.cross(grav_world);
        double ang = std::asin(axis.norm() / (grav_body.norm() * grav_world.norm() + 1e-10));
        if (axis.norm() > 1e-10)
            s.rot = Exp(axis.normalized() * ang);
        s.grav = grav_world;
        initialized = true;
        return true;
    }

    // 在两帧 LiDAR 之间向前积分 IMU
    // 同时填充每个 IMU 时刻的中间位姿（用于点云去畸变）
    void propagate(const std::deque<ImuMeas> &imu_buf,
                   double t_start, double t_end,
                   ImuPropState &ps,
                   std::vector<State> &poses_out) {
        poses_out.clear();
        State cur = ps.s;
        double t_prev = t_start;

        // 噪声矩阵 (连续时间)
        // Q: [acc_n², gyr_n², acc_w², gyr_w²] 对应 vel, rot, ba, bg
        auto &P = ps.P;

        for (auto &meas : imu_buf) {
            if (meas.timestamp <= t_start) continue;
            if (meas.timestamp >  t_end)   break;

            double dt = meas.timestamp - t_prev;
            t_prev = meas.timestamp;
            if (dt <= 0) continue;

            V3D acc_unbias = cur.rot * (meas.acc - cur.ba) + cur.grav;
            V3D gyr_unbias = meas.gyr - cur.bg;

            // 姿态传播
            M3D dR = Exp(gyr_unbias * dt);
            // 速度传播
            V3D dv = acc_unbias * dt;
            // 位置传播
            cur.pos += cur.vel * dt + 0.5 * acc_unbias * dt * dt;
            cur.vel += dv;
            cur.rot  = cur.rot * dR;
            cur.rot  = cur.rot * (cur.rot.transpose() * cur.rot).sqrt().inverse(); // 重正交化
            cur.time = meas.timestamp;

            // 协方差传播（简化线性化）
            Eigen::Matrix<double,STATE_DIM,STATE_DIM> F = Eigen::Matrix<double,STATE_DIM,STATE_DIM>::Identity();
            // dp/dv
            F.block<3,3>(0,6)  =  M3D::Identity() * dt;
            // dv/drot
            F.block<3,3>(6,3)  = -cur.rot * hat(meas.acc - cur.ba) * dt;
            // dv/dba
            F.block<3,3>(6,12) = -cur.rot * dt;
            // drot/dbg
            F.block<3,3>(3,9)  = -M3D::Identity() * dt;

            Eigen::Matrix<double,STATE_DIM,12> G = Eigen::Matrix<double,STATE_DIM,12>::Zero();
            G.block<3,3>(6,0)  = cur.rot * dt;     // vel ← acc noise
            G.block<3,3>(3,3)  = M3D::Identity() * dt; // rot ← gyr noise
            G.block<3,3>(12,6) = M3D::Identity() * dt; // ba
            G.block<3,3>(9,9)  = M3D::Identity() * dt; // bg

            Eigen::Matrix<double,12,12> Qc = Eigen::Matrix<double,12,12>::Zero();
            Qc.block<3,3>(0,0)  = M3D::Identity() * acc_n * acc_n;
            Qc.block<3,3>(3,3)  = M3D::Identity() * gyr_n * gyr_n;
            Qc.block<3,3>(6,6)  = M3D::Identity() * acc_w * acc_w;
            Qc.block<3,3>(9,9)  = M3D::Identity() * gyr_w * gyr_w;

            P = F * P * F.transpose() + G * Qc * G.transpose();

            poses_out.push_back(cur);
        }
        ps.s = cur;
    }

    // 用每点时间戳和插值位姿做点云去畸变（变换到帧末时刻）
    void undistort(CloudType::Ptr &cloud,
                   const std::vector<State> &poses,
                   const State &state_end) {
        if (poses.empty()) return;
        for (auto &pt : cloud->points) {
            // 找最近的传播位姿
            const State *sp = &poses.front();
            for (auto &s : poses) {
                if (s.time <= (double)pt.time) sp = &s;
                else break;
            }
            // 将点从 sp 时刻转换到帧末时刻
            M3D dR = state_end.rot.transpose() * sp->rot;
            V3D dp = state_end.rot.transpose() * (sp->pos - state_end.pos);
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f p2 = (dR.cast<float>() * p + dp.cast<float>());
            pt.x = p2(0); pt.y = p2(1); pt.z = p2(2);
        }
    }
};
