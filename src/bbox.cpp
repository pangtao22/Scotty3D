#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>
#include <vector>

namespace CMU462 {

using std::cout;
using std::endl;

bool BBox::intersect(const Ray &r, double &t0, double &t1) const {
  // TODO (PathTracer):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bounding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  t0 = -INF_D;
  t1 = INF_D;

  for(int i = 0; i < 3; i++) {

    double ta = (min[i] - r.o[i]) / r.d[i];
    double tb = (max[i] - r.o[i]) / r.d[i];
    double t_min = ta < tb ? ta : tb;
    double t_max = ta < tb ? tb : ta;
    if (t_min > t0) {
      t0 = t_min;
    }
    if(t_max < t1) {
      t1 = t_max;
    }
  }
  bool result = (t0 - std::numeric_limits<double>::epsilon() <= t1)
      && (t0 >= r.min_t) && (t0 <= r.max_t);
  return result;
}

void BBox::draw(Color c) const {
  glColor4f(c.r, c.g, c.b, c.a);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();
}

std::ostream &operator<<(std::ostream &os, const BBox &b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

}  // namespace CMU462
