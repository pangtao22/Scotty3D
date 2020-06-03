#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 {
namespace StaticScene {

Triangle::Triangle(const Mesh *mesh, vector<size_t> &v) : mesh(mesh), v(v) {}
Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3)
    : mesh(mesh), v1(v1), v2(v2), v3(v3) {}

BBox Triangle::get_bbox() const {
  // TODO (PathTracer):
  // compute the bounding box of the triangle
  const auto &p1 = mesh->positions[v1];
  const auto &p2 = mesh->positions[v2];
  const auto &p3 = mesh->positions[v3];

  auto bbox = BBox();

  for (int i = 0; i < 3; i++) {
    bbox.max[i] = std::max(p1[i], std::max(p2[i], p3[i]));
    bbox.min[i] = std::min(p1[i], std::min(p2[i], p3[i]));
  }
  return bbox;
}

bool Triangle::intersect(const Ray &r) const {
  // TODO (PathTracer): implement ray-triangle intersection
  return intersect(r, nullptr);
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // TODO (PathTracer):
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  const auto &p0 = mesh->positions[v1];
  const auto &p1 = mesh->positions[v2];
  const auto &p2 = mesh->positions[v3];
  const auto &d = r.d;

  Vector3D e1 = p1 - p0;
  Vector3D e2 = p2 - p0;
  Vector3D s = r.o - p0;
  Vector3D a1 = cross(e1, d);
  Vector3D a2 = cross(s, e2);

  double denominator = dot(a1, e2);
  double u = -dot(a2, d) / denominator;
  double v = dot(a1, s) / denominator;
  double t = - dot(a2, e1) / denominator;

//  Matrix3x3 M;
//  M.column(0) = e1;
//  M.column(1) = e2;
//  M.column(2) = -r.d;
//  auto a = M.inv() * s;
//  double u = a[0];
//  double v = a[1];
//  double t = a[2];

  const bool result = (u >= 0 && u <= 1) && (v >= 0 && v <= 1) &&
      (u + v <= 1) && (t >= r.min_t && t <= r.max_t);
  if(result) {
    r.max_t = t;
  }

  if(result && isect != nullptr) {
    const auto &n0 = mesh->normals[v1];
    const auto &n1 = mesh->normals[v2];
    const auto &n2 = mesh->normals[v3];
    const auto n = v * n2 + u * n1 + (1 - u - v) * n0;

    isect->t = t;
    isect->n = dot(n, r.d) > 0 ? -n : n;
    isect->primitive = this;
    isect->bsdf = mesh->get_bsdf();
  }

  return result;
}

void Triangle::draw(const Color &c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color &c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

} // namespace StaticScene
} // namespace CMU462
