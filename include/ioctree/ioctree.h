#ifndef IOCTREE_H_
#define IOCTREE_H_

// Copyright (c) 2023 Jun Zhu, Tsinghua University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include <stdint.h>

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <utility>
#include <vector>

namespace iOctree {

inline constexpr int kDim = 4;
inline constexpr int kMaxBucket = 6;

struct DistanceIndex {
  float dist_;
  float* index_;

  DistanceIndex() { dist_ = std::numeric_limits<float>::max(); }

  DistanceIndex(float dist, float* index) : dist_(dist), index_(index) {}

  bool operator<(const DistanceIndex& dist_index) const {
    return dist_ < dist_index.dist_;
  }
};

class KNNSimpleResultSet {
 private:
  size_t capacity_;
  size_t count_;
  float worst_distance_;
  std::vector<DistanceIndex> dist_index_;

 public:
  KNNSimpleResultSet(size_t capacity_) : capacity_(capacity_) {
    dist_index_.resize(capacity_, DistanceIndex());
    clear();
  }
  const std::vector<DistanceIndex>& GetData() { return dist_index_; }

  ~KNNSimpleResultSet() {}

  void clear() {
    worst_distance_ = std::numeric_limits<float>::max();
    dist_index_[capacity_ - 1].dist_ = worst_distance_;
    count_ = 0;
  }

  size_t size() const { return count_; }

  bool full() const { return count_ == capacity_; }

  void AddPoint(float dist, float* index) {
    if (dist >= worst_distance_) return;

    if (count_ < capacity_) ++count_;
    size_t i;

    for (i = count_ - 1; i > 0; --i) {
      if (dist_index_[i - 1].dist_ > dist)
        dist_index_[i] = dist_index_[i - 1];
      else
        break;
    }
    dist_index_[i].dist_ = dist;
    dist_index_[i].index_ = index;
    worst_distance_ = dist_index_[capacity_ - 1].dist_;
  }

  float WorstDist() const { return worst_distance_; }
};

struct BoxDeleteType {
  float min[3];
  float max[3];
  void Show() {
    printf("min:(%f, %f, %f), max:(%f, %f, %f)\n", min[0], min[1], min[2],
           max[0], max[1], max[2]);
  }
};

class Octant {
 public:
  int idx = -1;
  bool is_active_;
  float x, y, z;
  float extent;
  std::vector<float*> points;
  Octant** child;

  Octant() : x(0.0f), y(0.0f), z(0.0f), extent(0.0f) {
    child = nullptr;
    is_active_ = true;
    points.reserve(kMaxBucket);
  }

  ~Octant() {
    if (child != nullptr) {
      for (size_t i = 0; i < 8; ++i) {
        if (child[i] != nullptr) delete child[i];
      }
      delete[] child;
      child = nullptr;
    } else {
      points.clear();
    }
  }

  size_t size() {
    size_t pts_num = 0;
    GetOctantSize(this, pts_num);
    return pts_num;
  }

  void GetOctantSize(Octant* octant, size_t& size_) {
    if (octant->child == nullptr) {
      size_ += octant->points.size();
      return;
    }

    for (size_t c = 0; c < 8; ++c) {
      if (octant->child[c] == nullptr) continue;
      GetOctantSize(octant->child[c], size_);
    }
  }

  void InitChild() {
    child = new Octant*[8];
    memset(child, 0, 8 * sizeof(Octant*));
  }
};

class Octree {
 public:
  bool print_debug_ = false;
  size_t bucket_size_;
  float min_extent_;
  bool down_size_;
  int dim = kDim;
  size_t ordered_indices_[8][7] = {
      {1, 2, 4, 3, 5, 6, 7}, {0, 3, 5, 2, 4, 7, 6}, {0, 3, 6, 1, 4, 7, 5},
      {1, 2, 7, 0, 5, 6, 4}, {0, 5, 6, 1, 2, 7, 3}, {1, 4, 7, 0, 3, 6, 2},
      {2, 4, 7, 0, 3, 5, 1}, {3, 5, 6, 1, 2, 4, 0}};
  bool ordered_;

  Octree()
      : bucket_size_(32),
        min_extent_(0.01f),
        root_(nullptr),
        down_size_(false) {
    ordered_ = true;
    pts_num_deleted = last_pts_num = octant_num = 0;
    dim = kDim;
    bucket_size_ = fmin(bucket_size_, kMaxBucket);
  }

  Octree(size_t bucketSize_, bool copyPoints_, float minExtent_)
      : bucket_size_(bucketSize_),
        min_extent_(minExtent_),
        root_(nullptr),
        down_size_(false) {
    ordered_ = true;
    pts_num_deleted = last_pts_num = octant_num = 0;
    dim = kDim;
    bucket_size_ = fmin(bucket_size_, kMaxBucket);
  }

  Octree(size_t bucketSize_, bool copyPoints_, float minExtent_, int dim_)
      : bucket_size_(bucketSize_),
        min_extent_(minExtent_),
        root_(nullptr),
        down_size_(false) {
    ordered_ = true;
    pts_num_deleted = last_pts_num = octant_num = 0;
    dim = kDim;
    bucket_size_ = fmin(bucket_size_, kMaxBucket);
  }

  ~Octree() { clear(); }

  void SetOrder(bool ordered = false) { ordered_ = ordered; }

  void SetMaxNewPoints(int max_new_points) {
    new_points = new float[max_new_points * kDim];
  }

  void SetMaxOctants(int max_octants) {
    octant_max = max_octants;
    all_points = new float[max_octants * kMaxBucket * kDim];
  }

  void SetMinExtent(float extent) { min_extent_ = extent; }

  void SetBucketSize(size_t bucket_size) {
    bucket_size_ = fmin(bucket_size, kMaxBucket);
  }

  void SetDownSize(bool down_size) { down_size_ = down_size; }

  template <typename ContainerT>
  void Initialize(ContainerT& pts_, int filter_size,
                  std::vector<int>& filter_idxs, std::vector<int>& added_idxs,
                  std::vector<int>& new_idxs, bool down_size = true) {
    added_idxs.clear();
    new_idxs.clear();
    added_idxs.reserve(filter_size);
    new_idxs.reserve(filter_size);
    down_size_ = down_size;
    clear();
    const size_t pts_num = filter_size;
    std::vector<float*> points;
    int dim_ = 3;
    points.resize(pts_num, nullptr);
    size_t cloud_index = 0;
    float max_extent = 0.01f;
    float min[3], max[3], ctr[3], extent[3];

    for (size_t i = 0; i < pts_num; ++i) {
      const float& x = pts_[filter_idxs[i]].x;
      const float& y = pts_[filter_idxs[i]].y;
      const float& z = pts_[filter_idxs[i]].z;
      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
      float* cloud_ptr = new_points + cloud_index * kDim;
      cloud_ptr[0] = x;
      cloud_ptr[1] = y;
      cloud_ptr[2] = z;
      cloud_ptr[3] = -(filter_idxs[i] + 1);
      points[cloud_index] = cloud_ptr;
      if (cloud_index == 0) {
        min[0] = max[0] = x;
        min[1] = max[1] = y;
        min[2] = max[2] = z;
      } else {
        min[0] = x < min[0] ? x : min[0];
        min[1] = y < min[1] ? y : min[1];
        min[2] = z < min[2] ? z : min[2];
        max[0] = x > max[0] ? x : max[0];
        max[1] = y > max[1] ? y : max[1];
        max[2] = z > max[2] ? z : max[2];
      }
      cloud_index++;
    }
    if (cloud_index == 0) return;
    points.resize(cloud_index);

    if (print_debug_) {
      std::cerr << "orig min: " << min[0] << " " << min[1] << " " << min[2]
                << std::endl;
      std::cerr << "orig max: " << max[0] << " " << max[1] << " " << max[2]
                << std::endl;
    }

    for (size_t i = 0; i < 3; ++i) {
      min[i] = floor(min[i] / min_extent_) * min_extent_;
      max[i] = ceil(max[i] / min_extent_) * min_extent_;
      if (static_cast<int>(min[i] / min_extent_) % 2 != 0)
        min[i] += min_extent_;
      if (static_cast<int>(max[i] / min_extent_) % 2 != 0)
        max[i] -= min_extent_;
      extent[i] = 0.5f * (max[i] - min[i]);
      max_extent = fmax(max_extent, extent[i]);
      ctr[i] = min[i] + extent[i];
    }

    if (print_debug_) {
      std::cerr << "proc min: " << min[0] << " " << min[1] << " " << min[2]
                << std::endl;
      std::cerr << "proc max: " << max[0] << " " << max[1] << " " << max[2]
                << std::endl;
      std::cerr << "ctr: " << ctr[0] << " " << ctr[1] << " " << ctr[2]
                << std::endl;
      std::cerr << "extent: " << extent[0] << " " << extent[1] << " "
                << extent[2] << std::endl;
      std::cerr << "maxextent: " << max_extent << std::endl;
    }

    root_ = CreateOctant(ctr[0], ctr[1], ctr[2], max_extent, points, added_idxs,
                         new_idxs);
  }

  template <typename ContainerT>
  void Initialize(ContainerT& pts_, std::vector<int>& added_idxs,
                  std::vector<int>& new_idxs, bool down_size = true,
                  int start_idx = 0, int end_idx = 0) {
    added_idxs.clear();
    new_idxs.clear();

    if (end_idx == 0) end_idx = pts_.size();
    const size_t pts_num = end_idx - start_idx;

    added_idxs.reserve(pts_num);
    new_idxs.reserve(pts_num);
    down_size_ = down_size;
    clear();
    std::vector<float*> points;
    int dim_ = 3;
    points.resize(pts_num, 0);
    size_t cloud_index = 0;
    float max_extent = 0.01f;
    float min[3], max[3], ctr[3], extent[3];

    for (size_t i = start_idx; i < end_idx; ++i) {
      const float& x = pts_[i].x;
      const float& y = pts_[i].y;
      const float& z = pts_[i].z;
      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
      float* cloud_ptr = new_points + cloud_index * kDim;
      cloud_ptr[0] = x;
      cloud_ptr[1] = y;
      cloud_ptr[2] = z;
      cloud_ptr[3] = -(static_cast<int>(i) + 1);
      points[cloud_index] = cloud_ptr;
      if (cloud_index == 0) {
        min[0] = max[0] = x;
        min[1] = max[1] = y;
        min[2] = max[2] = z;
      } else {
        min[0] = x < min[0] ? x : min[0];
        min[1] = y < min[1] ? y : min[1];
        min[2] = z < min[2] ? z : min[2];
        max[0] = x > max[0] ? x : max[0];
        max[1] = y > max[1] ? y : max[1];
        max[2] = z > max[2] ? z : max[2];
      }
      cloud_index++;
    }
    points.resize(cloud_index);

    if (print_debug_) {
      std::cerr << "orig min: " << min[0] << " " << min[1] << " " << min[2]
                << std::endl;
      std::cerr << "orig max: " << max[0] << " " << max[1] << " " << max[2]
                << std::endl;
    }

    for (size_t i = 0; i < 3; ++i) {
      min[i] = floor(min[i] / min_extent_) * min_extent_;
      max[i] = ceil(max[i] / min_extent_) * min_extent_;
      if (static_cast<int>(min[i] / min_extent_) % 2 != 0)
        min[i] += min_extent_;
      if (static_cast<int>(max[i] / min_extent_) % 2 != 0)
        max[i] -= min_extent_;
      extent[i] = 0.5f * (max[i] - min[i]);
      max_extent = fmax(max_extent, extent[i]);
      ctr[i] = min[i] + extent[i];
    }

    if (print_debug_) {
      std::cerr << "proc min: " << min[0] << " " << min[1] << " " << min[2]
                << std::endl;
      std::cerr << "proc max: " << max[0] << " " << max[1] << " " << max[2]
                << std::endl;
      std::cerr << "ctr: " << ctr[0] << " " << ctr[1] << " " << ctr[2]
                << std::endl;
      std::cerr << "extent: " << extent[0] << " " << extent[1] << " "
                << extent[2] << std::endl;
      std::cerr << "maxextent: " << max_extent << std::endl;
    }

    root_ = CreateOctant(ctr[0], ctr[1], ctr[2], max_extent, points, added_idxs,
                         new_idxs);
  }

  template <typename ContainerT>
  void Update(ContainerT& pts_, int filter_size, std::vector<int>& filter_idxs,
              std::vector<int>& added_idxs, std::vector<int>& new_idxs,
              bool down_size = true) {
    if (root_ == nullptr) {
      Initialize(pts_, filter_size, filter_idxs, added_idxs, new_idxs,
                 down_size);
      return;
    }
    added_idxs.clear();
    new_idxs.clear();

    const size_t pts_num = filter_size;

    added_idxs.reserve(pts_num);
    new_idxs.reserve(pts_num);
    down_size_ = down_size;
    std::vector<float*> points;
    int dim_ = 3;
    points.resize(pts_num, 0);
    size_t cloud_index = 0;
    float max_extent = 0.01f;
    float min[3], max[3], ctr[3], extent[3];

    for (size_t i = 0; i < pts_num; ++i) {
      const float& x = pts_[filter_idxs[i]].x;
      const float& y = pts_[filter_idxs[i]].y;
      const float& z = pts_[filter_idxs[i]].z;
      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
      float* cloud_ptr = new_points + cloud_index * kDim;
      cloud_ptr[0] = x;
      cloud_ptr[1] = y;
      cloud_ptr[2] = z;
      cloud_ptr[3] = -(filter_idxs[i] + 1);
      points[cloud_index] = cloud_ptr;
      if (cloud_index == 0) {
        min[0] = max[0] = x;
        min[1] = max[1] = y;
        min[2] = max[2] = z;
      } else {
        min[0] = x < min[0] ? x : min[0];
        min[1] = y < min[1] ? y : min[1];
        min[2] = z < min[2] ? z : min[2];
        max[0] = x > max[0] ? x : max[0];
        max[1] = y > max[1] ? y : max[1];
        max[2] = z > max[2] ? z : max[2];
      }
      cloud_index++;
    }
    if (cloud_index == 0) return;
    points.resize(cloud_index);

    if (print_debug_) {
      std::cerr << "orig min: " << min[0] << " " << min[1] << " " << min[2]
                << std::endl;
      std::cerr << "orig max: " << max[0] << " " << max[1] << " " << max[2]
                << std::endl;
    }
    for (size_t i = 0; i < 3; ++i) {
      min[i] = floor(min[i] / min_extent_) * min_extent_;
      max[i] = ceil(max[i] / min_extent_) * min_extent_;
      if (static_cast<int>(min[i] / min_extent_) % 2 != 0)
        min[i] += min_extent_;
      if (static_cast<int>(max[i] / min_extent_) % 2 != 0)
        max[i] -= min_extent_;
      extent[i] = 0.5f * (max[i] - min[i]);
      max_extent = fmax(max_extent, extent[i]);
      ctr[i] = min[i] + extent[i];
    }

    if (print_debug_) {
      std::cerr << "proc min: " << min[0] << " " << min[1] << " " << min[2]
                << std::endl;
      std::cerr << "proc max: " << max[0] << " " << max[1] << " " << max[2]
                << std::endl;
      std::cerr << "ctr: " << ctr[0] << " " << ctr[1] << " " << ctr[2]
                << std::endl;
      std::cerr << "extent: " << extent[0] << " " << extent[1] << " "
                << extent[2] << std::endl;
      std::cerr << "maxextent: " << max_extent << std::endl;
    }

    static const float factor[] = {-0.5f, 0.5f};
    while (std::abs(max[0] - root_->x) > root_->extent ||
           std::abs(max[1] - root_->y) > root_->extent ||
           std::abs(max[2] - root_->z) > root_->extent) {
      float parentExtent = 2 * root_->extent;
      float parentX = root_->x + factor[max[0] > root_->x] * parentExtent;
      float parentY = root_->y + factor[max[1] > root_->y] * parentExtent;
      float parentZ = root_->z + factor[max[2] > root_->z] * parentExtent;

      Octant* octant = new Octant;
      octant->x = parentX;
      octant->y = parentY;
      octant->z = parentZ;
      octant->extent = parentExtent;
      octant->InitChild();
      size_t mortonCode = 0;
      if (root_->x > parentX) mortonCode |= 1;
      if (root_->y > parentY) mortonCode |= 2;
      if (root_->z > parentZ) mortonCode |= 4;
      octant->child[mortonCode] = root_;
      root_ = octant;
    }
    while (std::abs(min[0] - root_->x) > root_->extent ||
           std::abs(min[1] - root_->y) > root_->extent ||
           std::abs(min[2] - root_->z) > root_->extent) {
      float parentExtent = 2 * root_->extent;
      float parentX = root_->x + factor[min[0] > root_->x] * parentExtent;
      float parentY = root_->y + factor[min[1] > root_->y] * parentExtent;
      float parentZ = root_->z + factor[min[2] > root_->z] * parentExtent;

      Octant* octant = new Octant;

      octant->x = parentX;
      octant->y = parentY;
      octant->z = parentZ;
      octant->extent = parentExtent;
      octant->InitChild();
      size_t mortonCode = 0;
      if (root_->x > parentX) mortonCode |= 1;
      if (root_->y > parentY) mortonCode |= 2;
      if (root_->z > parentZ) mortonCode |= 4;
      octant->child[mortonCode] = root_;
      root_ = octant;
    }

    UpdateOctant(root_, points, added_idxs, new_idxs);
  }

  template <typename ContainerT>
  void Update(ContainerT& pts_, std::vector<int>& added_idxs,
              std::vector<int>& new_idxs, int start_idx, int end_idx, float res,
              bool down_size = true) {
    if (root_ == nullptr) {
      Initialize(pts_, added_idxs, new_idxs, down_size, start_idx, end_idx);
      return;
    }
    added_idxs.clear();
    new_idxs.clear();

    if (end_idx == 0) end_idx = pts_.size();
    const size_t pts_num = end_idx - start_idx;

    added_idxs.reserve(pts_num);
    new_idxs.reserve(pts_num);
    down_size_ = down_size;
    std::vector<float*> points;
    int dim_ = 3;
    points.resize(pts_num, 0);
    size_t cloud_index = 0;
    float max_extent = 0.01f;
    float tmp_res = min_extent_;
    float min[3], max[3], ctr[3], extent[3];

    for (size_t i = start_idx; i < end_idx; ++i) {
      const float& x = pts_[i].x;
      const float& y = pts_[i].y;
      const float& z = pts_[i].z;
      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
      float* cloud_ptr = new_points + cloud_index * kDim;
      cloud_ptr[0] = x;
      cloud_ptr[1] = y;
      cloud_ptr[2] = z;
      cloud_ptr[3] = -(static_cast<int>(i) + 1);
      points[cloud_index] = cloud_ptr;
      if (cloud_index == 0) {
        min[0] = max[0] = x;
        min[1] = max[1] = y;
        min[2] = max[2] = z;
      } else {
        min[0] = x < min[0] ? x : min[0];
        min[1] = y < min[1] ? y : min[1];
        min[2] = z < min[2] ? z : min[2];
        max[0] = x > max[0] ? x : max[0];
        max[1] = y > max[1] ? y : max[1];
        max[2] = z > max[2] ? z : max[2];
      }
      cloud_index++;
    }
    if (cloud_index == 0) return;
    points.resize(cloud_index);

    if (print_debug_) {
      std::cerr << "orig min: " << min[0] << " " << min[1] << " " << min[2]
                << std::endl;
      std::cerr << "orig max: " << max[0] << " " << max[1] << " " << max[2]
                << std::endl;
    }

    for (size_t i = 0; i < 3; ++i) {
      min[i] = floor(min[i] / min_extent_) * min_extent_;
      max[i] = ceil(max[i] / min_extent_) * min_extent_;
      if (static_cast<int>(min[i] / min_extent_) % 2 != 0)
        min[i] += min_extent_;
      if (static_cast<int>(max[i] / min_extent_) % 2 != 0)
        max[i] -= min_extent_;
      extent[i] = 0.5f * (max[i] - min[i]);
      max_extent = fmax(max_extent, extent[i]);
      ctr[i] = min[i] + extent[i];
    }

    if (print_debug_) {
      std::cerr << "proc min: " << min[0] << " " << min[1] << " " << min[2]
                << std::endl;
      std::cerr << "proc max: " << max[0] << " " << max[1] << " " << max[2]
                << std::endl;
      std::cerr << "ctr: " << ctr[0] << " " << ctr[1] << " " << ctr[2]
                << std::endl;
      std::cerr << "extent: " << extent[0] << " " << extent[1] << " "
                << extent[2] << std::endl;
      std::cerr << "maxextent: " << max_extent << std::endl;
    }

    static const float factor[] = {-0.5f, 0.5f};
    while (std::abs(max[0] - root_->x) > root_->extent ||
           std::abs(max[1] - root_->y) > root_->extent ||
           std::abs(max[2] - root_->z) > root_->extent) {
      float parentExtent = 2 * root_->extent;
      float parentX = root_->x + factor[max[0] > root_->x] * parentExtent;
      float parentY = root_->y + factor[max[1] > root_->y] * parentExtent;
      float parentZ = root_->z + factor[max[2] > root_->z] * parentExtent;

      Octant* octant = new Octant;
      octant->x = parentX;
      octant->y = parentY;
      octant->z = parentZ;
      octant->extent = parentExtent;
      octant->InitChild();
      size_t mortonCode = 0;
      if (root_->x > parentX) mortonCode |= 1;
      if (root_->y > parentY) mortonCode |= 2;
      if (root_->z > parentZ) mortonCode |= 4;
      octant->child[mortonCode] = root_;
      root_ = octant;
    }
    while (std::abs(min[0] - root_->x) > root_->extent ||
           std::abs(min[1] - root_->y) > root_->extent ||
           std::abs(min[2] - root_->z) > root_->extent) {
      float parentExtent = 2 * root_->extent;
      float parentX = root_->x + factor[min[0] > root_->x] * parentExtent;
      float parentY = root_->y + factor[min[1] > root_->y] * parentExtent;
      float parentZ = root_->z + factor[min[2] > root_->z] * parentExtent;

      Octant* octant = new Octant;

      octant->x = parentX;
      octant->y = parentY;
      octant->z = parentZ;
      octant->extent = parentExtent;
      octant->InitChild();
      size_t mortonCode = 0;
      if (root_->x > parentX) mortonCode |= 1;
      if (root_->y > parentY) mortonCode |= 2;
      if (root_->z > parentZ) mortonCode |= 4;
      octant->child[mortonCode] = root_;
      root_ = octant;
    }

    min_extent_ = res;
    UpdateOctant(root_, points, added_idxs, new_idxs);
    min_extent_ = tmp_res;
  }

  void clear() {
    delete root_;
    root_ = nullptr;
    pts_num_deleted = last_pts_num = octant_num = 0;
  }

  template <typename PointT>
  void RadiusNeighbors(const PointT& query, float radius,
                       std::vector<int>& resultIndices,
                       size_t bucket_size = 0) {
    resultIndices.clear();
    if (root_ == nullptr) return;
    float sqrRadius = radius * radius;
    float query_[3] = {query.x, query.y, query.z};
    std::vector<float*> points_ptr;
    RadiusNeighbors(root_, query_, radius, sqrRadius, points_ptr, bucket_size);
    resultIndices.resize(points_ptr.size());

    for (size_t i = 0; i < points_ptr.size(); i++) {
      resultIndices[i] = static_cast<int>(points_ptr[i][3]);
    }
  }

  template <typename PointT>
  void RadiusNeighbors(const PointT& query, float radius,
                       std::vector<size_t>& resultIndices,
                       std::vector<float>& distances, size_t bucket_size = 0) {
    resultIndices.clear();
    distances.clear();
    if (root_ == nullptr) return;
    float sqrRadius = radius * radius;
    float query_[3] = {query.x, query.y, query.z};
    std::vector<float*> points_ptr;
    RadiusNeighbors(root_, query_, radius, sqrRadius, points_ptr, distances,
                    bucket_size);
    resultIndices.resize(points_ptr.size());

    for (size_t i = 0; i < points_ptr.size(); i++) {
      resultIndices[i] = size_t(points_ptr[i][3]);
    }
  }

  template <typename PointT>
  void RadiusNeighbors(
      const PointT& query, float radius,
      std::vector<PointT, Eigen::aligned_allocator<PointT>>& resultPoints,
      std::vector<float>& distances, size_t bucket_size = 0) {
    resultPoints.clear();
    distances.clear();
    if (root_ == nullptr) return;
    float sqrRadius = radius * radius;
    float query_[3] = {query.x, query.y, query.z};
    std::vector<float*> points_ptr;
    RadiusNeighbors(root_, query_, radius, sqrRadius, points_ptr, distances,
                    bucket_size);
    resultPoints.resize(points_ptr.size());

    for (size_t i = 0; i < resultPoints.size(); i++) {
      PointT pt;
      pt.x = points_ptr[i][0];
      pt.y = points_ptr[i][1];
      pt.z = points_ptr[i][2];
      resultPoints[i] = pt;
    }
  }

  template <typename PointT>
  void RadiusNeighbors(const PointT& query, float radius,
                       Eigen::MatrixXf& resultMatrix,
                       std::vector<float>& distances, size_t bucket_size = 0) {
    distances.clear();
    if (root_ == nullptr) return;
    float sqrRadius = radius * radius;  // "squared" radius
    float query_[3] = {query.x, query.y, query.z};
    std::vector<float*> points_ptr;
    RadiusNeighbors(root_, query_, radius, sqrRadius, points_ptr, distances,
                    bucket_size);
    resultMatrix.resize(points_ptr.size(), 3);

    for (size_t i = 0; i < resultMatrix.rows(); i++) {
      resultMatrix(i, 0) = points_ptr[i][0];
      resultMatrix(i, 1) = points_ptr[i][1];
      resultMatrix(i, 2) = points_ptr[i][2];
    }
  }

  template <typename PointT>
  void RadiusNeighbors(const PointT& query, float radius,
                       Eigen::MatrixXf& resultMatrix,
                       std::vector<int>& resultIndices,
                       size_t bucket_size = 0) {
    resultIndices.clear();
    if (root_ == nullptr) return;
    float sqrRadius = radius * radius;
    float query_[3] = {query.x, query.y, query.z};
    std::vector<float*> points_ptr;
    RadiusNeighbors(root_, query_, radius, sqrRadius, points_ptr, bucket_size);
    resultMatrix.resize(points_ptr.size(), 3);
    resultIndices.resize(points_ptr.size(), 3);

    for (size_t i = 0; i < resultMatrix.rows(); i++) {
      resultMatrix(i, 0) = points_ptr[i][0];
      resultMatrix(i, 1) = points_ptr[i][1];
      resultMatrix(i, 2) = points_ptr[i][2];
      resultIndices[i] = size_t(points_ptr[i][3]);
    }
  }

  template <typename PointT>
  int32_t KnnNeighbors(
      const PointT& query, int k,
      std::vector<PointT, Eigen::aligned_allocator<PointT>>& resultIndices,
      std::vector<float>& distances) {
    if (root_ == nullptr) return 0;

    float query_[3] = {query.x, query.y, query.z};

    KNNSimpleResultSet heap(k);
    KnnNeighbors(root_, query_, heap);

    std::vector<DistanceIndex> data = heap.GetData();
    resultIndices.resize(heap.size());
    distances.resize(heap.size());

    for (size_t i = 0; i < heap.size(); i++) {
      PointT pt;
      pt.x = data[i].index_[0];
      pt.y = data[i].index_[1];
      pt.z = data[i].index_[2];
      resultIndices[i] = pt;
      distances[i] = data[i].dist_;
    }

    return data.size();
  }

  template <typename PointT>
  int32_t KnnNeighbors(const PointT& query, int k,
                       std::vector<size_t>& resultIndices,
                       std::vector<float>& distances) {
    if (root_ == nullptr) return 0;

    float query_[3] = {query.x, query.y, query.z};

    KNNSimpleResultSet heap(k);
    KnnNeighbors(root_, query_, heap);

    std::vector<DistanceIndex> data = heap.GetData();
    resultIndices.resize(heap.size());
    distances.resize(heap.size());

    for (int i = 0; i < heap.size(); i++) {
      resultIndices[i] = size_t(data[i].index_[3]);
      distances[i] = data[i].dist_;
    }

    return data.size();
  }

  template <typename PointT>
  int32_t KnnNeighbors(const PointT& query, int k,
                       std::vector<int>& resultIndices,
                       std::vector<float>& distances) {
    if (root_ == nullptr) return 0;

    float query_[3] = {query.x, query.y, query.z};

    KNNSimpleResultSet heap(k);
    KnnNeighbors(root_, query_, heap);

    std::vector<DistanceIndex> data = heap.GetData();
    resultIndices.resize(heap.size());
    distances.resize(heap.size());

    for (int i = 0; i < heap.size(); i++) {
      resultIndices[i] = static_cast<int>(data[i].index_[3]);
      distances[i] = data[i].dist_;
    }

    return data.size();
  }

  int32_t KnnNeighbors(const Eigen::Vector3f& query, int k,
                       std::vector<int>& resultIndices,
                       std::vector<float>& distances) {
    if (root_ == nullptr) return 0;

    float query_[3] = {query(0), query(1), query(2)};

    KNNSimpleResultSet heap(k);
    KnnNeighbors(root_, query_, heap);

    std::vector<DistanceIndex> data = heap.GetData();
    resultIndices.resize(heap.size());
    distances.resize(heap.size());

    for (int i = 0; i < heap.size(); i++) {
      resultIndices[i] = static_cast<int>(data[i].index_[3]);
      distances[i] = data[i].dist_;
    }

    return data.size();
  }

  int32_t KnnNeighbors(const Eigen::Vector3f& query, int k,
                       std::vector<int>& resultIndices,
                       std::vector<float>& distances, const float& search_rad) {
    if (root_ == nullptr) return 0;

    float sqrRadius = search_rad * search_rad;
    float query_[3] = {query(0), query(1), query(2)};

    KNNSimpleResultSet heap(k);
    KnnNeighbors(root_, query_, heap, sqrRadius);

    std::vector<DistanceIndex> data = heap.GetData();
    resultIndices.resize(heap.size());
    distances.resize(heap.size());

    for (int i = 0; i < heap.size(); i++) {
      resultIndices[i] = static_cast<int>(data[i].index_[3]);
      distances[i] = data[i].dist_;
    }

    return data.size();
  }

  void BoxWiseDelete(const BoxDeleteType& box_range, bool clear_data) {
    if (root_ == nullptr) return;
    bool deleted = false;
    BoxWiseDelete(root_, box_range, deleted, clear_data);
    if (deleted) root_ = nullptr;
  }

  void BoxWiseDelete(const float* min, const float* max, bool clear_data) {
    if (root_ == nullptr) return;
    BoxDeleteType box_range;
    box_range.min[0] = min[0];
    box_range.min[1] = min[1];
    box_range.min[2] = min[2];
    box_range.max[0] = max[0];
    box_range.max[1] = max[1];
    box_range.max[2] = max[2];

    bool deleted = false;
    BoxWiseDelete(root_, box_range, deleted, clear_data);
    if (deleted) root_ = nullptr;
  }

  size_t OctantSize() { return octant_num; }

  size_t size() { return last_pts_num - pts_num_deleted; }

  void GetNodes(Octant* octant, std::vector<Octant*>& nodes,
                float min_extent = 0) {
    if (octant == nullptr) return;
    if (min_extent > 0) {
      if (octant->extent <= min_extent) {
        nodes.push_back(octant);
        return;
      }
    }
    nodes.push_back(octant);
    if (octant->child == nullptr) {
      return;
    }

    for (int i = 0; i < 8; i++) {
      GetNodes(octant->child[i], nodes, min_extent);
    }
  }

  template <typename PointT, typename ContainerT>
  ContainerT GetData() {
    std::vector<Octant*> nodes;
    ContainerT pts;
    GetNodes(root_, nodes);

    for (auto octant : nodes) {
      for (auto p : octant->points) {
        PointT pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];
        pts.push_back(pt);
      }
    }
    return pts;
  }

  void GetLeafNodes(const Octant* octant, std::vector<const Octant*>& nodes) {
    if (octant == nullptr) return;
    if (octant->child == nullptr) {
      nodes.push_back(octant);
      return;
    }
    nodes.reserve(nodes.size() + 8);

    for (int i = 0; i < 8; i++) {
      GetLeafNodes(octant->child[i], nodes);
    }
  }

 protected:
  Octant* root_;
  size_t last_pts_num, pts_num_deleted, octant_num, octant_max;
  float* new_points;
  float* all_points;
  Octant* all_octants;

  Octree(const Octree&) = delete;
  Octree& operator=(const Octree&) = delete;

  Octant* CreateOctant(float x, float y, float z, float extent,
                       std::vector<float*>& points,
                       std::vector<int>& added_idxs,
                       std::vector<int>& new_idxs) {
    Octant* octant = new Octant;
    const size_t size = points.size();
    octant->x = x;
    octant->y = y;
    octant->z = z;
    octant->extent = extent;
    static const float factor[] = {-0.5f, 0.5f};

    if (size > bucket_size_ && extent >= 2 * min_extent_) {
      std::vector<std::vector<float*>> child_points(8, std::vector<float*>());

      for (size_t i = 0; i < size; ++i) {
        float* p = points[i];
        size_t mortonCode = 0;
        if (p[0] > x) mortonCode |= 1;
        if (p[1] > y) mortonCode |= 2;
        if (p[2] > z) mortonCode |= 4;
        child_points[mortonCode].push_back(p);
      }
      float childExtent = 0.5f * extent;
      octant->InitChild();

      for (size_t i = 0; i < 8; ++i) {
        if (child_points[i].empty()) continue;
        float childX = x + factor[(i & 1) > 0] * extent;
        float childY = y + factor[(i & 2) > 0] * extent;
        float childZ = z + factor[(i & 4) > 0] * extent;
        octant->child[i] = CreateOctant(childX, childY, childZ, childExtent,
                                        child_points[i], added_idxs, new_idxs);
      }
    } else {
      const size_t size = std::min(points.size(), bucket_size_);
      octant->points.resize(size);

      if (octant->idx < 0 && size > 0) octant->idx = octant_num++;
      if (octant_num >= octant_max) {
        std::cerr << "Octant overflow max: " << octant_max
                  << " num: " << octant_num << std::endl;
        exit(1);
      }
      const size_t oct_idx = octant->idx * kMaxBucket * kDim;
      for (size_t i = 0; i < size; ++i) {
        std::copy(points[i], points[i] + dim,
                  all_points + oct_idx + (i * kDim));
        octant->points[i] = all_points + oct_idx + (i * kDim);
        if (octant->points[i][3] < 0) {
          added_idxs.push_back(-(octant->points[i][3] + 1));
          octant->points[i][3] = last_pts_num++;
          new_idxs.push_back(octant->points[i][3]);
        }
      }
    }
    return octant;
  }

  void UpdateOctant(Octant* octant, const std::vector<float*>& points,
                    std::vector<int>& added_idxs, std::vector<int>& new_idxs) {
    static const float factor[] = {-0.5f, 0.5f};
    const float x = octant->x, y = octant->y, z = octant->z,
                extent = octant->extent;
    octant->is_active_ = true;
    if (octant->child == nullptr) {
      if (octant->points.size() + points.size() > bucket_size_ &&
          extent >= 2 * min_extent_) {
        octant->points.insert(octant->points.end(), points.begin(),
                              points.end());
        const size_t size = octant->points.size();
        std::vector<std::vector<float*>> child_points(8, std::vector<float*>());

        for (size_t i = 0; i < size; ++i) {
          size_t mortonCode = 0;
          if (octant->points[i][0] > x) mortonCode |= 1;
          if (octant->points[i][1] > y) mortonCode |= 2;
          if (octant->points[i][2] > z) mortonCode |= 4;
          child_points[mortonCode].push_back(octant->points[i]);
        }
        float childExtent = 0.5f * extent;
        octant->InitChild();

        for (size_t i = 0; i < 8; ++i) {
          if (child_points[i].empty()) continue;
          float childX = x + factor[(i & 1) > 0] * extent;
          float childY = y + factor[(i & 2) > 0] * extent;
          float childZ = z + factor[(i & 4) > 0] * extent;
          octant->child[i] =
              CreateOctant(childX, childY, childZ, childExtent, child_points[i],
                           added_idxs, new_idxs);
        }
        octant->points.clear();
      } else {
        if (down_size_ && octant->points.size() >= bucket_size_) return;
        const size_t old_size = octant->points.size();
        const size_t dif_size =
            std::min(points.size(), bucket_size_ - old_size);
        octant->points.insert(octant->points.end(), points.begin(),
                              points.begin() + dif_size);
        const size_t new_size = octant->points.size();

        if (octant->idx < 0 && new_size > 0) octant->idx = octant_num++;
        if (octant_num >= octant_max) {
          std::cerr << "Octant overflow max: " << octant_max
                    << " num: " << octant_num << std::endl;
          exit(1);
        }
        const size_t oct_idx = octant->idx * kMaxBucket * kDim;
        for (size_t i = 0; i < new_size; ++i) {
          std::copy(octant->points[i], octant->points[i] + dim,
                    all_points + oct_idx + (i * kDim));
          octant->points[i] = all_points + oct_idx + (i * kDim);
          if (octant->points[i][3] < 0) {
            added_idxs.push_back(-(octant->points[i][3] + 1));
            octant->points[i][3] = last_pts_num++;
            new_idxs.push_back(octant->points[i][3]);
          }
        }
      }
    } else {
      const size_t size = points.size();
      std::vector<std::vector<float*>> child_points(8, std::vector<float*>());

      for (size_t i = 0; i < size; ++i) {
        size_t mortonCode = 0;
        if (points[i][0] > x) mortonCode |= 1;
        if (points[i][1] > y) mortonCode |= 2;
        if (points[i][2] > z) mortonCode |= 4;
        child_points[mortonCode].push_back(points[i]);
      }
      float childExtent = 0.5f * extent;

      for (size_t i = 0; i < 8; ++i) {
        if (child_points[i].size() > 0) {
          if (octant->child[i] == nullptr) {
            float childX = x + factor[(i & 1) > 0] * extent;
            float childY = y + factor[(i & 2) > 0] * extent;
            float childZ = z + factor[(i & 4) > 0] * extent;
            octant->child[i] =
                CreateOctant(childX, childY, childZ, childExtent,
                             child_points[i], added_idxs, new_idxs);
          } else
            UpdateOctant(octant->child[i], child_points[i], added_idxs,
                         new_idxs);
        }
      }
    }
  }

  void RadiusNeighbors(const Octant* octant, const float* query, float radius,
                       float sqrRadius, std::vector<float*>& resultIndices,
                       size_t bucket_size) {
    if (!octant->is_active_) return;
    if (3 * octant->extent * octant->extent < sqrRadius &&
        contains(query, sqrRadius, octant)) {
      std::vector<const Octant*> candidate_octants;
      candidate_octants.reserve(8);
      GetLeafNodes(octant, candidate_octants);

      for (size_t k = 0; k < candidate_octants.size(); k++) {
        size_t size = candidate_octants[k]->points.size();
        if (bucket_size) size = std::min(size, bucket_size);

        const size_t result_size = resultIndices.size();
        resultIndices.resize(result_size + size);

        size_t m = 0;
        for (size_t i = 0; i < size; ++i) {
          const float* p = candidate_octants[k]->points[i];
          float dist = 0, diff = 0;

          for (size_t j = 0; j < 3; ++j) {
            diff = p[j] - query[j];
            dist += diff * diff;
          }
          if (dist > 0) {
            resultIndices[result_size + m] = candidate_octants[k]->points[i];
            ++m;
          }
        }
        resultIndices.resize(result_size + m);
      }
      return;
    }
    if (octant->child == nullptr) {
      size_t size = octant->points.size();
      if (bucket_size) size = std::min(size, bucket_size);

      for (size_t i = 0; i < size; ++i) {
        const float* p = octant->points[i];
        float dist = 0, diff = 0;

        for (size_t j = 0; j < 3; ++j) {
          diff = p[j] - query[j];
          dist += diff * diff;
        }
        if (dist > 0 && dist < sqrRadius)
          resultIndices.push_back(octant->points[i]);
      }
      return;
    }

    for (size_t c = 0; c < 8; ++c) {
      if (octant->child[c] == nullptr) continue;
      if (!overlaps(query, sqrRadius, octant->child[c])) continue;
      RadiusNeighbors(octant->child[c], query, radius, sqrRadius, resultIndices,
                      bucket_size);
    }
  }

  void RadiusNeighbors(const Octant* octant, const float* query, float radius,
                       float sqrRadius, std::vector<float*>& resultIndices,
                       std::vector<float>& distances, size_t bucket_size) {
    if (!octant->is_active_) return;
    if (3 * octant->extent * octant->extent < sqrRadius &&
        contains(query, sqrRadius, octant)) {
      std::vector<const Octant*> candidate_octants;
      GetLeafNodes(octant, candidate_octants);

      for (size_t k = 0; k < candidate_octants.size(); k++) {
        size_t size = candidate_octants[k]->points.size();
        if (bucket_size) size = std::min(size, bucket_size);

        const size_t result_size = resultIndices.size();
        resultIndices.resize(result_size + size);
        size_t m = 0;
        for (size_t i = 0; i < size; ++i) {
          const float* p = candidate_octants[k]->points[i];
          float dist = 0, diff = 0;

          for (size_t j = 0; j < 3; ++j) {
            diff = p[j] - query[j];
            dist += diff * diff;
          }
          if (dist > 0) {
            distances.push_back(dist);
            resultIndices[result_size + m] = candidate_octants[k]->points[i];
            ++m;
          }
        }
        resultIndices.resize(result_size + m);
      }
      return;
    }
    if (octant->child == nullptr) {
      size_t size = octant->points.size();
      if (bucket_size) size = std::min(size, bucket_size);

      for (size_t i = 0; i < size; ++i) {
        const float* p = octant->points[i];
        float dist = 0, diff = 0;

        for (size_t j = 0; j < 3; ++j) {
          diff = p[j] - query[j];
          dist += diff * diff;
        }
        if (dist > 0 && dist < sqrRadius) {
          resultIndices.push_back(octant->points[i]);
          distances.push_back(dist);
        }
      }
      return;
    }

    for (size_t c = 0; c < 8; ++c) {
      if (octant->child[c] == nullptr) continue;
      if (!overlaps(query, sqrRadius, octant->child[c])) continue;
      RadiusNeighbors(octant->child[c], query, radius, sqrRadius, resultIndices,
                      distances, bucket_size);
    }
  }

  bool KnnNeighbors(const Octant* octant, const float* query,
                    KNNSimpleResultSet& heap) {
    if (!octant->is_active_) return false;
    if (octant->child == nullptr) {
      const size_t size = octant->points.size();

      for (int i = 0; i < size; ++i) {
        const float* p = octant->points[i];
        float dist = 0, diff = 0;

        for (int j = 0; j < 3; ++j) {
          diff = p[j] - query[j];
          dist += diff * diff;
        }
        if (dist > 0 && dist < heap.WorstDist())
          heap.AddPoint(dist, octant->points[i]);
      }

      return heap.full() && inside(query, heap.WorstDist(), octant);
    }
    size_t mortonCode = 0;
    if (query[0] > octant->x) mortonCode |= 1;
    if (query[1] > octant->y) mortonCode |= 2;
    if (query[2] > octant->z) mortonCode |= 4;
    if (octant->child[mortonCode] != nullptr) {
      if (KnnNeighbors(octant->child[mortonCode], query, heap)) return true;
    }

    for (int i = 0; i < 7; ++i) {
      int c = ordered_indices_[mortonCode][i];
      if (octant->child[c] == nullptr) continue;
      if (heap.full() && !overlaps(query, heap.WorstDist(), octant->child[c]))
        continue;
      if (KnnNeighbors(octant->child[c], query, heap)) return true;
    }
    return heap.full() && inside(query, heap.WorstDist(), octant);
  }

  bool KnnNeighbors(const Octant* octant, const float* query,
                    KNNSimpleResultSet& heap, float& sqrRadius) {
    if (!octant->is_active_) return false;
    if (octant->child == nullptr) {
      const size_t size = octant->points.size();

      for (int i = 0; i < size; ++i) {
        const float* p = octant->points[i];
        float dist = 0, diff = 0;

        for (int j = 0; j < 3; ++j) {
          diff = p[j] - query[j];
          dist += diff * diff;
        }
        if (dist > sqrRadius) continue;
        if (dist > 0 && dist < heap.WorstDist())
          heap.AddPoint(dist, octant->points[i]);
      }

      return heap.full() && inside(query, heap.WorstDist(), octant);
    }
    size_t mortonCode = 0;
    if (query[0] > octant->x) mortonCode |= 1;
    if (query[1] > octant->y) mortonCode |= 2;
    if (query[2] > octant->z) mortonCode |= 4;
    if (octant->child[mortonCode] != nullptr) {
      if (KnnNeighbors(octant->child[mortonCode], query, heap, sqrRadius))
        return true;
    }

    for (int i = 0; i < 7; ++i) {
      int c = ordered_indices_[mortonCode][i];
      if (octant->child[c] == nullptr) continue;
      if (heap.full() && !overlaps(query, heap.WorstDist(), octant->child[c]))
        continue;
      if (!overlaps(query, sqrRadius, octant->child[c])) continue;
      if (KnnNeighbors(octant->child[c], query, heap, sqrRadius)) return true;
    }
    return heap.full() && inside(query, heap.WorstDist(), octant);
  }

  void BoxWiseDelete(Octant* octant, const BoxDeleteType& box_range,
                     bool& deleted, bool clear_data) {
    float cur_min[3];
    float cur_max[3];
    cur_min[0] = octant->x - octant->extent;
    cur_min[1] = octant->y - octant->extent;
    cur_min[2] = octant->z - octant->extent;
    cur_max[0] = octant->x + octant->extent;
    cur_max[1] = octant->y + octant->extent;
    cur_max[2] = octant->z + octant->extent;
    if (cur_min[0] > box_range.max[0] || box_range.min[0] > cur_max[0]) return;
    if (cur_min[1] > box_range.max[1] || box_range.min[1] > cur_max[1]) return;
    if (cur_min[2] > box_range.max[2] || box_range.min[2] > cur_max[2]) return;

    if (cur_min[0] >= box_range.min[0] && cur_min[1] >= box_range.min[1] &&
        cur_min[2] >= box_range.min[2] && cur_max[0] <= box_range.max[0] &&
        cur_max[1] <= box_range.max[1] && cur_max[2] <= box_range.max[2]) {
      if (!clear_data) {
        octant->is_active_ = false;
        return;
      } else {
        pts_num_deleted += octant->size();
        delete octant;
        deleted = true;
        return;
      }
    }
    if (octant->child == nullptr) {
      if (!clear_data) {
        octant->is_active_ = false;
        return;
      } else {
        const size_t size = octant->points.size();
        std::vector<float*> remainder_points;
        remainder_points.resize(size, 0);
        size_t valid_num = 0;

        for (int i = 0; i < size; ++i) {
          const float* p = octant->points[i];
          if (p[0] > box_range.max[0] || box_range.min[0] > p[0]) {
            remainder_points[valid_num] = octant->points[i];
            valid_num++;
            continue;
          }
          if (p[1] > box_range.max[1] || box_range.min[1] > p[1]) {
            remainder_points[valid_num] = octant->points[i];
            valid_num++;
            continue;
          }
          if (p[2] > box_range.max[2] || box_range.min[2] > p[2]) {
            remainder_points[valid_num] = octant->points[i];
            valid_num++;
            continue;
          }
        }
        pts_num_deleted += size - valid_num;
        if (valid_num == 0) {
          delete octant;
          deleted = true;
          return;
        }

        octant->points.resize(valid_num);
        if (octant->idx < 0 && valid_num > 0) octant->idx = octant_num++;
        if (octant_num >= octant_max) {
          std::cerr << "Octant overflow max: " << octant_max
                    << " num: " << octant_num << std::endl;
          exit(1);
        }
        const size_t oct_idx = octant->idx * kMaxBucket * kDim;
        for (size_t i = 0; i < valid_num; ++i) {
          std::copy(remainder_points[i], remainder_points[i] + dim,
                    all_points + oct_idx + (i * kDim));
          octant->points[i] = all_points + oct_idx + (i * kDim);
        }
        return;
      }
    }

    for (size_t c = 0; c < 8; ++c) {
      if (octant->child[c] == nullptr) continue;
      bool deleted1 = false;
      BoxWiseDelete(octant->child[c], box_range, deleted1, clear_data);
      if (deleted1) octant->child[c] = nullptr;
    }
    int valid_child = 0;

    for (size_t c = 0; c < 8; ++c) {
      if (octant->child[c] == nullptr) continue;
      valid_child++;
    }
    if (valid_child == 0) {
      delete octant;
      deleted = true;
      return;
    }
  }

  bool overlaps(const float* query, float sqRadius, const Octant* o) {
    /** \brief test if search ball S(q,r) overlaps with octant
     * @param query   query point
     * @param radius  "squared" radius
     * @param o       pointer to octant
     * @return true, if search ball overlaps with octant, false otherwise.
     */
    // we exploit the symmetry to reduce the test to testing if its inside the
    // Minkowski sum around the positive quadrant.
    float x = std::abs(query[0] - o->x) - o->extent;
    float y = std::abs(query[1] - o->y) - o->extent;
    float z = std::abs(query[2] - o->z) - o->extent;
    // float maxdist = radius + o->extent;
    // Completely outside, since q' is outside the relevant area.
    // std::abs(query[0] - o->x) - o->extent > radius
    if ((x > 0 && x * x > sqRadius) || (y > 0 && y * y > sqRadius) ||
        (z > 0 && z * z > sqRadius))
      return false;
    int32_t num_less_extent = (x < 0) + (y < 0) + (z < 0);
    // Checking different cases:
    // a. inside the surface region of the octant.
    if (num_less_extent > 1) return true;
    // b. checking the corner region && edge region.
    x = std::max(x, 0.0f);
    y = std::max(y, 0.0f);
    z = std::max(z, 0.0f);

    return (x * x + y * y + z * z < sqRadius);
  }

  float SqrDistPointToOctant(const float* query, const Octant* o) {
    float x, y, z;
    x = std::max(std::abs(query[0] - o->x) - o->extent, 0.0f);
    y = std::max(std::abs(query[1] - o->y) - o->extent, 0.0f);
    z = std::max(std::abs(query[2] - o->z) - o->extent, 0.0f);
    return x * x + y * y + z * z;
  }

  bool contains(const float* query, float sqRadius, const Octant* o) {
    /** \brief test if search ball S(q,r) contains octant
     * @param query    query point
     * @param sqRadius "squared" radius
     * @param octant   pointer to octant
     * @return true, if search ball overlaps with octant, false otherwise.
     */
    // we exploit the symmetry to reduce the test to test
    // whether the farthest corner is inside the search ball.
    float x = std::abs(query[0] - o->x) + o->extent;
    float y = std::abs(query[1] - o->y) + o->extent;
    float z = std::abs(query[2] - o->z) + o->extent;

    return (x * x + y * y + z * z < sqRadius);
  }

  bool inside(const float* query, float radius2, const Octant* octant) {
    /** \brief test if search ball S(q,r) is completely inside octant.
     * @param query   query point
     * @param radius2  radius r*r
     * @param octant  point to octant.
     * @return true, if search ball is completely inside the octant, false
     * otherwise.
     */
    // we exploit the symmetry to reduce the test to test
    // whether the farthest corner is inside the search ball.
    float x = octant->extent - std::abs(query[0] - octant->x);
    float y = octant->extent - std::abs(query[1] - octant->y);
    float z = octant->extent - std::abs(query[2] - octant->z);
    // octant->extent < radius + std::abs(query[0] - octant->x)
    if (x < 0 || x * x < radius2) return false;
    if (y < 0 || y * y < radius2) return false;
    if (z < 0 || z * z < radius2) return false;
    return true;
  }
};

}  // namespace iOctree

#endif  // IOCTREE_H_
