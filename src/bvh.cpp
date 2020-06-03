#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CMU462 {
namespace StaticScene {

int computeBucket(int axis, int num_buckets, double coordinate,
                  const BBox &bb) {
  const double delta = bb.extent[axis] / num_buckets;

  int idx_bucket;
  for(idx_bucket = 0; idx_bucket < num_buckets; idx_bucket++) {
    double coordinate_division = delta * (idx_bucket + 1) + bb.min[axis];
    if(coordinate_division > coordinate) {
      break;
    }
  }
  return idx_bucket;
}

template <class T>
void printStlVector(const vector<T>& v) {
  for(const auto& e : v) {
    cout << e << " ";
  }
  cout << endl;
}

struct Bucket {
  BBox bbox;
  vector<size_t> primitives_indices;
};

void computePartitionBySah(const std::vector<Primitive*>& primitives,
    const std::vector<Bucket>& buckets, const BBox& bb_parent,
    size_t* min_idx, double* min_cost,
    vector<size_t>* l_primitives, vector<size_t>* r_primitives) {
  const double Sn = bb_parent.surface_area();
  *min_cost = INF_D;
  *min_idx = 0;
  for(size_t i = 0; i < buckets.size() - 1; i++) {
    BBox left;
    BBox right;
    size_t N_left = 0;
    size_t N_right = 0;
    for(size_t j = 0; j <= i; j++) {
      left.expand(buckets[j].bbox);
      N_left += buckets[j].primitives_indices.size();
    }
    for(size_t j = i+1; j < buckets.size(); j++) {
      right.expand(buckets[j].bbox);
      N_right += buckets[j].primitives_indices.size();
    }
    double cost = 1 + left.surface_area() / Sn * N_left +
        right.surface_area() / Sn * N_right;
    if(cost < *min_cost) {
      *min_idx = i;
      *min_cost = cost;
    }

//    // debug
//    cout << "Partition " << i << ", cost: " << cost << endl;
//    l_primitives->clear();
//    for(size_t j = 0; j <= i; j++) {
//      l_primitives->insert(
//          l_primitives->end(),
//          buckets[j].primitives_indices.begin(),
//          buckets[j].primitives_indices.end());
//    }
//
//    r_primitives->clear();
//    for(size_t j = i + 1; j < buckets.size(); j++) {
//      r_primitives->insert(
//          r_primitives->end(),
//          buckets[j].primitives_indices.begin(),
//          buckets[j].primitives_indices.end());
//    }
//    cout << "Left:";
//    printStlVector(*l_primitives);
//    cout << left << endl;
//    cout << "Right:" << endl;
//    printStlVector(*r_primitives);
//    cout << right << endl;
//    cout << endl;
  }

  l_primitives->clear();
  for(size_t j = 0; j <= *min_idx; j++) {
    l_primitives->insert(
        l_primitives->end(),
        buckets[j].primitives_indices.begin(),
        buckets[j].primitives_indices.end());
  }

  r_primitives->clear();
  for(size_t j = *min_idx + 1; j < buckets.size(); j++) {
    r_primitives->insert(
        r_primitives->end(),
        buckets[j].primitives_indices.begin(),
        buckets[j].primitives_indices.end());
  }

}



BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {
  this->primitives = _primitives;

  // TODO (PathTracer):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bb;
  for (const auto &p : primitives) {
    bb.expand(p->get_bbox());
  }

  root = new BVHNode(bb, 0, primitives.size());

  // x buckets.
  const BVHNode &node = *root;
  const int num_buckets = 8;
  int num_axes = 2;
  vector<size_t> optimal_partition_idx(num_axes);
  vector<double> sah_cost(num_axes);
  vector<vector<size_t> > l_primitives_list(num_axes);
  vector<vector<size_t> > r_primitives_list(num_axes);

//  cout << endl;
//  for(const auto& p : primitives) {
//    cout << p->get_bbox() << endl;
//  }

  for(int axis = 0; axis < num_axes; axis++) {
    vector<Bucket> buckets(num_buckets);
    cout << "xyz: " << axis << endl;
    for (int p = node.start; p < node.start + node.range; p++) {
      int idx_bucket = computeBucket(
          axis, num_buckets,primitives[p]->get_centroid()[axis], bb);
      buckets[idx_bucket].primitives_indices.push_back(p);
      buckets[idx_bucket].bbox.expand(primitives[p]->get_bbox());
    }

//    for(const auto& bucket : buckets) {
//      cout << "Bucket:" << bucket.bbox << endl;
//      printStlVector(bucket.primitives_indices);
//      cout << endl;
//    }

    computePartitionBySah(primitives, buckets, bb,
                          &optimal_partition_idx[axis], &sah_cost[axis],
                          &l_primitives_list[axis],
                          &r_primitives_list[axis]);
    cout << "Left:";
    printStlVector(l_primitives_list[axis]);
    cout << "Right:" << endl;
    printStlVector(r_primitives_list[axis]);
    cout << "Cost: " << sah_cost[axis] << endl;
    cout << endl;

    

  }
}

BVHAccel::~BVHAccel() {
  // TODO (PathTracer):
  // Implement a proper destructor for your BVH accelerator aggregate
}

BBox BVHAccel::get_bbox() const { return root->bb; }

bool BVHAccel::intersect(const Ray &ray) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate.

  bool hit = false;
  for (size_t p = 0; p < primitives.size(); ++p) {
    if (primitives[p]->intersect(ray))
      hit = true;
  }

  return hit;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *isect) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate. When an intersection does happen.
  // You should store the non-aggregate primitive in the intersection data
  // and not the BVH aggregate itself.

  bool hit = false;
  for (size_t p = 0; p < primitives.size(); ++p) {
    if (primitives[p]->intersect(ray, isect))
      hit = true;
  }

  return hit;
}

} // namespace StaticScene
} // namespace CMU462
