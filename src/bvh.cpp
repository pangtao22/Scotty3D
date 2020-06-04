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
  if (num_buckets == 1) {
    return 0;
  }

  const double delta = bb.extent[axis] / num_buckets;

  int idx_bucket;
  for (idx_bucket = 0; idx_bucket < num_buckets; idx_bucket++) {
    double coordinate_division = delta * (idx_bucket + 1) + bb.min[axis];
    if (coordinate_division > coordinate) {
      break;
    }
  }
  return idx_bucket;
}

template <class T> void printStlVector(const vector<T> &v) {
  for (const auto &e : v) {
    cout << e << " ";
  }
  cout << endl;
}

void computePartitionBySah(const std::vector<Primitive *> &primitives,
                           const std::vector<Bucket> &buckets,
                           const BBox &bb_parent, size_t *min_idx,
                           double *min_cost, vector<size_t> *l_primitives,
                           vector<size_t> *r_primitives, BBox *left_bb_best,
                           BBox *right_bb_best) {
  const double Sn = bb_parent.surface_area();
  *min_cost = INF_D;
  *min_idx = 0;
  for (size_t i = 0; i < buckets.size() - 1; i++) {
    BBox left_bb;
    BBox right_bb;
    size_t N_left = 0;
    size_t N_right = 0;
    for (size_t j = 0; j <= i; j++) {
      left_bb.expand(buckets[j].bbox);
      N_left += buckets[j].primitives_indices.size();
    }
    for (size_t j = i + 1; j < buckets.size(); j++) {
      right_bb.expand(buckets[j].bbox);
      N_right += buckets[j].primitives_indices.size();
    }
    double cost = 1 + left_bb.surface_area() / Sn * N_left +
                  right_bb.surface_area() / Sn * N_right;
    if (cost < *min_cost) {
      *min_idx = i;
      *min_cost = cost;
      *left_bb_best = left_bb;
      *right_bb_best = right_bb;
    }
  }

  l_primitives->clear();
  for (size_t j = 0; j <= *min_idx; j++) {
    l_primitives->insert(l_primitives->end(),
                         buckets[j].primitives_indices.begin(),
                         buckets[j].primitives_indices.end());
  }

  r_primitives->clear();
  for (size_t j = *min_idx + 1; j < buckets.size(); j++) {
    r_primitives->insert(r_primitives->end(),
                         buckets[j].primitives_indices.begin(),
                         buckets[j].primitives_indices.end());
  }
}

void divideBVHnode(std::vector<Primitive *> *primitives_ptr,
                   size_t max_leaf_size, const BBox &bb, BVHNode *node_p_ptr) {
  if (node_p_ptr->range <= max_leaf_size) {
    return;
  }

  // Find optimal left/right assignment along each axis.
  BVHNode &node_p = *node_p_ptr;
  auto &primitives = *primitives_ptr;

  const int num_buckets = 8;
  const int num_axes = 3;
  vector<size_t> optimal_partition_idx(num_axes);
  vector<double> sah_cost(num_axes);
  vector<vector<size_t>> l_primitives_list(num_axes);
  vector<vector<size_t>> r_primitives_list(num_axes);
  vector<BBox> l_bbox_best(num_axes);
  vector<BBox> r_bbox_best(num_axes);

//  cout << bb << endl;

  for (int axis = 0; axis < num_axes; axis++) {
    if(bb.extent[axis] < std::numeric_limits<double>::epsilon()) {
      sah_cost[axis] = INF_D;
      continue;
    }
    vector<Bucket> buckets(num_buckets);
    //    cout << "xyz: " << axis << endl;
    for (int p = node_p.start; p < node_p.start + node_p.range; p++) {
      int idx_bucket = computeBucket(axis, num_buckets,
                                     primitives[p]->get_centroid()[axis], bb);
      buckets[idx_bucket].primitives_indices.push_back(p);
      buckets[idx_bucket].bbox.expand(primitives[p]->get_bbox());
    }

    computePartitionBySah(primitives, buckets, bb, &optimal_partition_idx[axis],
                          &sah_cost[axis], &l_primitives_list[axis],
                          &r_primitives_list[axis], &l_bbox_best[axis],
                          &r_bbox_best[axis]);
  }

  // Get best primitives assignment.
  const size_t best_axis = std::distance(
      sah_cost.begin(), min_element(sah_cost.begin(), sah_cost.end()));
  const auto &l_primitives = l_primitives_list[best_axis];
  const auto &r_primitives = r_primitives_list[best_axis];
  const auto &l_bbox = l_bbox_best[best_axis];
  const auto &r_bbox = r_bbox_best[best_axis];

  // Rearrange primitives array.
  vector<Primitive *> new_node_primitives(node_p.range);
  int idx = 0;
  for (const auto i : l_primitives) {
    new_node_primitives[idx] = primitives[i];
    idx++;
  }
  for (const auto i : r_primitives) {
    new_node_primitives[idx] = primitives[i];
    idx++;
  }

  for (size_t i = 0; i < node_p.range; i++) {
    primitives[i + node_p.start] = new_node_primitives[i];
  }

  // Create new nodes
  node_p.l = new BVHNode(l_bbox, node_p.start, l_primitives.size());
  node_p.r = new BVHNode(r_bbox, node_p.start + l_primitives.size(),
                         r_primitives.size());

  // Continue to divide.
  divideBVHnode(primitives_ptr, max_leaf_size, l_bbox, node_p.l);
  divideBVHnode(primitives_ptr, max_leaf_size, r_bbox, node_p.r);
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

  divideBVHnode(&primitives, max_leaf_size, bb, root);
}

BVHAccel::~BVHAccel() {
  // TODO (PathTracer):
  // Implement a proper destructor for your BVH accelerator aggregate
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::findClosetHit(const Ray &ray, BVHNode *node,
                             Intersection *closest) const {
  Intersection buffer;
  if (node->isLeaf()) {
    for (auto &p : primitives) {
      bool hit = p->intersect(ray, &buffer);
      if (hit && buffer.t < closest->t) {
        closest->primitive = p;
        closest->t = buffer.t;
        closest->n = buffer.n;
        closest->bsdf = buffer.bsdf;
      }
    }
  } else {
    double t0_l, t1_l, t0_r, t1_r;
    bool l_intersect = node->l->bb.intersect(ray, t0_l, t1_l);
    bool r_intersect = node->r->bb.intersect(ray, t0_r, t1_r);

    if(l_intersect && !r_intersect) {
      findClosetHit(ray, node->l, closest);
      return;
    }

    if (!l_intersect && r_intersect) {
      findClosetHit(ray, node->r, closest);
      return;
    }

    if (l_intersect && r_intersect) {
      BVHNode *first = t0_l <= t0_r ? node->l : node->r;
      BVHNode *second = t0_l <= t0_r ? node->r : node->l;
//    double t_first = t0_l <= t0_r ? t0_l : t0_r;
      double t_second = t0_l <= t0_r ? t0_r : t0_l;

      findClosetHit(ray, first, closest);
      if(t_second < closest->t) {
        findClosetHit(ray, second, closest);
      }
    }

  }
}

bool BVHAccel::intersect(const Ray &ray, Intersection *isect) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate. When an intersection does happen.
  // You should store the non-aggregate primitive in the intersection data
  // and not the BVH aggregate itself.

  findClosetHit(ray, root, isect);
  return isect->primitive != nullptr;
}

bool BVHAccel::intersect(const Ray &ray) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate.
  Intersection isect;
  return intersect(ray, &isect);
}

} // namespace StaticScene
} // namespace CMU462
