// FAST-LIO2 主节点 —— 宇树 L1 适配版
// 算法：IEKF + IKD-tree 点到面配准 + IMU 紧耦合
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/filter.hpp>

#include "common_lib.h"
#include "so3_math.h"
#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "ikd-Tree/ikd_Tree.h"

using std::placeholders::_1;

// ── 判断是否为平面 ─────────────────────────────────────────────
static bool fit_plane(const std::vector<PointType> &pts, V3D &normal, double &d) {
    if ((int)pts.size() < NUM_MATCH) return false;
    // 质心
    V3D cen = V3D::Zero();
    for (auto &p : pts) cen += V3D(p.x, p.y, p.z);
    cen /= (double)pts.size();
    // 协方差
    M3D cov = M3D::Zero();
    for (auto &p : pts) {
        V3D dp = V3D(p.x, p.y, p.z) - cen;
        cov += dp * dp.transpose();
    }
    Eigen::SelfAdjointEigenSolver<M3D> es(cov);
    normal = es.eigenvectors().col(0); // 最小特征向量 = 法向量
    d      = -normal.dot(cen);
    // 平面判定：最小特征值应远小于其余两个
    auto ev = es.eigenvalues();
    return ev(0) < PLANE_THR * ev(1);
}

// ── FAST-LIO2 节点 ─────────────────────────────────────────────
class FastLioNode : public rclcpp::Node {
public:
    FastLioNode() : Node("fast_lio") {
        // 参数读取
        double g_norm    = declare_parameter("common.imu_gravity",  9.80);
        imu_proc_.acc_n  = declare_parameter("common.imu_acc_n",    0.1);
        imu_proc_.gyr_n  = declare_parameter("common.imu_gyr_n",    0.01);
        imu_proc_.acc_w  = declare_parameter("common.imu_acc_w",    0.001);
        imu_proc_.gyr_w  = declare_parameter("common.imu_gyr_w",    0.0001);
        imu_proc_.last_state.grav = V3D(0, 0, -g_norm);

        prep_.blind      = declare_parameter("common.blind",         0.3);
        prep_.det_range  = declare_parameter("common.det_range",    40.0);
        prep_.scan_line  = (int)declare_parameter("common.scan_line", 18);
        prep_.lidar_type = (int)declare_parameter("common.lidar_type", 3);

        filter_corner_   = declare_parameter("common.filter_size_corner", 0.1);
        filter_map_      = declare_parameter("common.filter_size_map",    0.2);
        max_iter_        = (int)declare_parameter("common.max_iteration",  4);

        // 外参
        auto exR = declare_parameter("common.extrinsic_R",
            std::vector<double>{1,0,0, 0,1,0, 0,0,1});
        auto exT = declare_parameter("common.extrinsic_T",
            std::vector<double>{0,0,0});
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                imu_proc_.R_IL(i,j) = exR[i*3+j];
        imu_proc_.t_IL = V3D(exT[0], exT[1], exT[2]);

        // 订阅 / 发布
        std::string lid_topic = declare_parameter("common.lid_topic", std::string("/scan"));
        std::string imu_topic = declare_parameter("common.imu_topic", std::string("/imu/data_raw"));

        sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, 10, std::bind(&FastLioNode::lidar_cb, this, _1));
        sub_imu_   = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 200, std::bind(&FastLioNode::imu_cb, this, _1));

        pub_odom_  = create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10);
        pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10);
        pub_path_  = create_publisher<nav_msgs::msg::Path>("/trajectory", 10);

        tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(get_logger(), "FAST-LIO2 node started (L1 LiDAR)");
    }

private:
    // ── 回调 ────────────────────────────────────────────────────
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
        ImuMeas m;
        m.timestamp = rclcpp::Time(msg->header.stamp).seconds();
        m.acc = V3D(msg->linear_acceleration.x,
                    msg->linear_acceleration.y,
                    msg->linear_acceleration.z);
        m.gyr = V3D(msg->angular_velocity.x,
                    msg->angular_velocity.y,
                    msg->angular_velocity.z);
        std::lock_guard<std::mutex> lk(imu_mtx_);
        imu_buf_.push_back(m);
        // 保留最近 3s
        while (!imu_buf_.empty() &&
               m.timestamp - imu_buf_.front().timestamp > 3.0)
            imu_buf_.pop_front();
    }

    void lidar_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 点云预处理
        CloudType::Ptr raw(new CloudType);
        prep_.process(msg, raw);
        if (raw->empty()) return;

        double t_scan = rclcpp::Time(msg->header.stamp).seconds();

        std::deque<ImuMeas> imu_snap;
        {
            std::lock_guard<std::mutex> lk(imu_mtx_);
            imu_snap = imu_buf_;
        }

        // ── 初始化（静止对齐重力）─────────────────────────────
        if (!initialized_) {
            if (!imu_proc_.init(imu_snap, prop_state_.s)) return;
            last_lidar_time_ = t_scan;
            initialized_ = true;
            RCLCPP_INFO(get_logger(), "IMU initialized");
            return;
        }

        // ── IMU 传播 ──────────────────────────────────────────
        std::vector<State> mid_poses;
        imu_proc_.propagate(imu_snap, last_lidar_time_, t_scan, prop_state_, mid_poses);
        last_lidar_time_ = t_scan;

        // ── 点云去畸变 ────────────────────────────────────────
        CloudType::Ptr cloud_ds(new CloudType);
        imu_proc_.undistort(raw, mid_poses, prop_state_.s);

        // 降采样
        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud(raw);
        vg.setLeafSize(filter_corner_, filter_corner_, filter_corner_);
        vg.filter(*cloud_ds);

        // ── 首帧：初始化地图 ──────────────────────────────────
        if (map_.size() == 0) {
            map_.build(cloud_ds);
            publish_result(t_scan, cloud_ds);
            return;
        }

        // ── IEKF 迭代更新 ─────────────────────────────────────
        State x = prop_state_.s;
        MatXd P = prop_state_.P;

        for (int iter = 0; iter < max_iter_; ++iter) {
            // 构建残差和雅可比
            std::vector<V3D>  residuals;
            std::vector<Eigen::Matrix<double,1,STATE_DIM>> H_list;

            for (const auto &pt : cloud_ds->points) {
                // 将点转到世界系
                V3D p_l(pt.x, pt.y, pt.z);
                V3D p_w = x.rot * p_l + x.pos;

                // KNN 搜索
                PointType q; q.x=p_w(0); q.y=p_w(1); q.z=p_w(2);
                std::vector<PointType> neighbors;
                std::vector<float>     sq_d;
                map_.knn_search(q, NUM_MATCH, neighbors, sq_d);
                if ((int)neighbors.size() < NUM_MATCH) continue;
                if (sq_d.back() > 4.0f) continue;  // 远邻点舍弃

                // 拟合平面
                V3D  nrm; double dist_d;
                if (!fit_plane(neighbors, nrm, dist_d)) continue;

                // 点到面残差
                double res = nrm.dot(p_w) + dist_d;
                if (std::abs(res) > 0.5) continue;  // 外点剔除

                // 雅可比 H = [nrm^T * dR/dx ...]
                // H_pos = nrm^T
                // H_rot = -nrm^T * [p_w]×
                Eigen::Matrix<double,1,STATE_DIM> H = Eigen::Matrix<double,1,STATE_DIM>::Zero();
                H.block<1,3>(0,0) = nrm.transpose();                    // ∂/∂pos
                H.block<1,3>(0,3) = -nrm.transpose() * hat(p_w);        // ∂/∂rot

                residuals.push_back(V3D(res, 0, 0));
                H_list.push_back(H);
            }

            if (residuals.empty()) break;

            int m = (int)residuals.size();
            Eigen::MatrixXd H_mat(m, STATE_DIM);
            Eigen::VectorXd z_vec(m);
            for (int i = 0; i < m; ++i) {
                H_mat.row(i) = H_list[i];
                z_vec(i)     = -residuals[i](0);
            }

            // 测量噪声
            Eigen::MatrixXd R_meas = Eigen::MatrixXd::Identity(m,m) * 0.01;

            // IEKF 增益
            Eigen::MatrixXd S   = H_mat * P * H_mat.transpose() + R_meas;
            Eigen::MatrixXd K   = P * H_mat.transpose() * S.inverse();
            Eigen::VectorXd dx  = K * z_vec;

            // 状态更新（误差状态 -> 名义状态）
            x.pos  += dx.segment<3>(0);
            x.rot   = x.rot * Exp(dx.segment<3>(3));
            x.vel  += dx.segment<3>(6);
            x.bg   += dx.segment<3>(9);
            x.ba   += dx.segment<3>(12);

            // 协方差更新 (Joseph form)
            Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_DIM,STATE_DIM) - K * H_mat;
            P = I_KH * P * I_KH.transpose() + K * R_meas * K.transpose();

            if (dx.norm() < 1e-6) break;
        }

        prop_state_.s = x;
        prop_state_.P = P;

        // ── 将点加入地图（降采样） ────────────────────────────
        CloudType::Ptr cloud_world(new CloudType);
        cloud_world->reserve(cloud_ds->size());
        for (const auto &pt : cloud_ds->points) {
            V3D pw = x.rot * V3D(pt.x, pt.y, pt.z) + x.pos;
            PointType p2 = pt;
            p2.x=pw(0); p2.y=pw(1); p2.z=pw(2);
            cloud_world->push_back(p2);
        }
        CloudType::Ptr cloud_map_ds(new CloudType);
        pcl::VoxelGrid<PointType> vg2;
        vg2.setInputCloud(cloud_world);
        vg2.setLeafSize(filter_map_, filter_map_, filter_map_);
        vg2.filter(*cloud_map_ds);
        map_.insert(cloud_map_ds);

        publish_result(t_scan, cloud_world);
    }

    // ── 发布里程计 / 注册点云 / TF ───────────────────────────
    void publish_result(double t_sec, const CloudType::Ptr &cloud_w) {
        auto stamp = rclcpp::Time(static_cast<uint64_t>(t_sec * 1e9));
        const State &s = prop_state_.s;

        // 里程计
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = stamp;
        odom.header.frame_id = "map";
        odom.child_frame_id  = "lidar_link";
        odom.pose.pose.position.x = s.pos(0);
        odom.pose.pose.position.y = s.pos(1);
        odom.pose.pose.position.z = s.pos(2);
        Eigen::Quaterniond q(s.rot);
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        pub_odom_->publish(odom);

        // 轨迹
        geometry_msgs::msg::PoseStamped ps;
        ps.header = odom.header;
        ps.pose   = odom.pose.pose;
        path_.header = odom.header;
        path_.poses.push_back(ps);
        pub_path_->publish(path_);

        // 注册点云
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_w, cloud_msg);
        cloud_msg.header.stamp    = stamp;
        cloud_msg.header.frame_id = "map";
        pub_cloud_->publish(cloud_msg);

        // TF: map -> lidar_link
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id  = "lidar_link";
        tf.transform.translation.x = s.pos(0);
        tf.transform.translation.y = s.pos(1);
        tf.transform.translation.z = s.pos(2);
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf_br_->sendTransform(tf);
    }

    // ── 成员 ───────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         sub_imu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_cloud_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              pub_path_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                 tf_br_;

    std::mutex            imu_mtx_;
    std::deque<ImuMeas>   imu_buf_;

    ImuProcessor   imu_proc_;
    Preprocess     prep_;
    IKDTree        map_;
    ImuPropState   prop_state_;
    nav_msgs::msg::Path path_;

    double last_lidar_time_{0.0};
    bool   initialized_{false};
    double filter_corner_{0.1}, filter_map_{0.2};
    int    max_iter_{4};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastLioNode>());
    rclcpp::shutdown();
}
