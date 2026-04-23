#include "preprocess.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>

void Preprocess::process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                         CloudType::Ptr &out) {
    out->clear();
    process_standard(msg, out);
}

void Preprocess::process_standard(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                                  CloudType::Ptr &out) {
    // 检查是否有 time 字段
    bool has_time = false;
    for (auto &f : msg->fields)
        if (f.name == "time") { has_time = true; break; }

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_i(*msg, "intensity");

    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_t;
    if (has_time)
        it_t = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "time");

    size_t n = msg->width * msg->height;
    out->reserve(n);

    for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z, ++it_i) {
        float x = *it_x, y = *it_y, z = *it_z;
        float r = std::sqrt(x*x + y*y + z*z);
        if (r < (float)blind || r > (float)det_range) {
            if (it_t) ++(*it_t);
            continue;
        }
        PointType pt;
        pt.x = x; pt.y = y; pt.z = z;
        pt.intensity = *it_i;
        pt.time = it_t ? *(*it_t) : 0.0f;
        out->push_back(pt);
        if (it_t) ++(*it_t);
    }
    out->width  = out->size();
    out->height = 1;
    out->is_dense = true;
}
