#include "ikd-Tree/ikd_Tree.h"
#include <algorithm>
#include <numeric>

// ── 构建 ──────────────────────────────────────────────────────
void IKDTree::build(const CloudType::Ptr &cloud) {
    std::lock_guard<std::mutex> lk(mtx_);
    PointVec pts(cloud->points.begin(), cloud->points.end());
    size_ = pts.size();
    root_ = build_recursive(pts, 0, (int)pts.size(), 0);
}

std::unique_ptr<KdNode> IKDTree::build_recursive(PointVec &pts, int l, int r, int depth) {
    if (l >= r) return nullptr;
    int axis = depth % 3;
    int mid  = (l + r) / 2;
    std::nth_element(pts.begin()+l, pts.begin()+mid, pts.begin()+r,
        [axis](const PointType &a, const PointType &b){
            return coord(a,axis) < coord(b,axis);
        });
    auto node = std::make_unique<KdNode>();
    node->point = pts[mid];
    node->axis  = axis;
    node->left  = build_recursive(pts, l,   mid,   depth+1);
    node->right = build_recursive(pts, mid+1, r,   depth+1);
    return node;
}

// ── 批量插入 ──────────────────────────────────────────────────
void IKDTree::insert(const CloudType::Ptr &cloud) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto &pt : cloud->points) {
        insert_recursive(root_, pt, 0);
        ++size_;
    }
}

void IKDTree::insert_recursive(std::unique_ptr<KdNode> &node, const PointType &pt, int depth) {
    if (!node) {
        node = std::make_unique<KdNode>();
        node->point = pt;
        node->axis  = depth % 3;
        return;
    }
    if (coord(pt, node->axis) < coord(node->point, node->axis))
        insert_recursive(node->left,  pt, depth+1);
    else
        insert_recursive(node->right, pt, depth+1);
}

// ── KNN 搜索 ──────────────────────────────────────────────────
void IKDTree::knn_search(const PointType &query, int k,
                         std::vector<PointType> &results,
                         std::vector<float>     &sq_dists) const {
    std::lock_guard<std::mutex> lk(mtx_);
    // 最大堆: pair<dist², *point>
    std::vector<std::pair<float,const PointType*>> heap;
    heap.reserve(k+1);
    knn_recursive(root_.get(), query, k, heap);
    std::sort(heap.begin(), heap.end(),
              [](auto &a, auto &b){ return a.first < b.first; });
    results.clear(); sq_dists.clear();
    for (auto &[d, p] : heap) {
        results.push_back(*p);
        sq_dists.push_back(d);
    }
}

void IKDTree::knn_recursive(const KdNode *node, const PointType &query, int k,
                             std::vector<std::pair<float,const PointType*>> &heap) const {
    if (!node || node->deleted) return;

    float d = sq_dist(query, node->point);
    if ((int)heap.size() < k) {
        heap.push_back({d, &node->point});
        std::push_heap(heap.begin(), heap.end(),
                       [](auto &a, auto &b){ return a.first < b.first; });
    } else if (d < heap.front().first) {
        std::pop_heap(heap.begin(), heap.end(),
                      [](auto &a, auto &b){ return a.first < b.first; });
        heap.back() = {d, &node->point};
        std::push_heap(heap.begin(), heap.end(),
                       [](auto &a, auto &b){ return a.first < b.first; });
    }

    float diff = coord(query, node->axis) - coord(node->point, node->axis);
    auto *first  = diff < 0 ? node->left.get()  : node->right.get();
    auto *second = diff < 0 ? node->right.get() : node->left.get();

    knn_recursive(first, query, k, heap);
    float worst = heap.empty() ? 1e30f : heap.front().first;
    if (diff*diff < worst || (int)heap.size() < k)
        knn_recursive(second, query, k, heap);
}
