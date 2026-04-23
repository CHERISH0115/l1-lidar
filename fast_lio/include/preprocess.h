#pragma once
#include "common_lib.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class Preprocess {
public:
    int    lidar_type{3};   // 3 = 标准 PointCloud2
    double blind{0.3};      // 近距离盲区 (m)
    double det_range{40.0}; // 最大距离
    int    scan_line{18};

    void process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                 CloudType::Ptr &out);

private:
    void process_standard(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                          CloudType::Ptr &out);
};
