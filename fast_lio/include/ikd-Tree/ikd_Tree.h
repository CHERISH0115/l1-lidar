#pragma once
// IKD-Tree: 增量式 KD-tree（简化版，支持批量插入和 KNN 搜索）
// 完整版参见 hku-mars/ikd-Tree；此处实现满足 FAST-LIO2 基本需求。
#include <vector>
#include <array>
#include <memory>
#include <mutex>
#include <Eigen/Eigen>
#include "common_lib.h"

struct KdNode {
    PointType          point;
    std::unique_ptr<KdNode> left, right;
    int                axis{0};
    bool               deleted{false};
};

class IKDTree {
public:
    void build(const CloudType::Ptr &cloud);
    void insert(const CloudType::Ptr &cloud);
    // 返回最近 k 个点（point-to-plane 需要 k=5）
    void knn_search(const PointType &query, int k,
                    std::vector<PointType> &results,
                    std::vector<float>     &sq_dists) const;
    size_t size() const { return size_; }

private:
    std::unique_ptr<KdNode>  root_;
    size_t                   size_{0};
    mutable std::mutex       mtx_;

    using PointVec = std::vector<PointType>;

    std::unique_ptr<KdNode> build_recursive(PointVec &pts, int l, int r, int depth);
    void insert_recursive(std::unique_ptr<KdNode> &node, const PointType &pt, int depth);
    void knn_recursive(const KdNode *node, const PointType &query, int k,
                       std::vector<std::pair<float,const PointType*>> &heap) const;

    static float sq_dist(const PointType &a, const PointType &b) {
        float dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z;
        return dx*dx + dy*dy + dz*dz;
    }
    static float coord(const PointType &p, int axis) {
        return (axis==0) ? p.x : (axis==1) ? p.y : p.z;
    }
};
