#include "sphere.h"

#include <cmath>

#include "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 {
namespace StaticScene {

bool Sphere::test(const Ray& r, double& t0, double& t1) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t0 and the larger in t1.
  const double a = dot(r.d, r.d);
  const double b = 2 * dot(r.o, r.d);
  const double c =  dot(r.o, r.o) - this->r;
  const double delta = b * b - 4 * a * c;

  if (delta >= 0) {
    const double ta = (-b + sqrt(delta)) / 2 / a;
    const double tb = (-b - sqrt(delta)) / 2 / a;
    if (ta < tb) {
      t0 = ta;
      t1 = tb;
    } else {
      t0 = tb;
      t1 = ta;
    }
    bool result = (t0 > r.min_t) && (t0 < r.max_t);
    if(result) {
      r.max_t = t0;
    }
  }

  return false;
}

bool Sphere::intersect(const Ray& r) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection.
  // Note that you might want to use the Sphere::test helper here.
  double t1, t2;
  return test(r, t1, t2);
}

bool Sphere::intersect(const Ray& r, Intersection* isect) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1, t2;
  if(test(r, t1, t2)) {
    auto n = r.o + r.d * t1 - o;
    n.normalize();
    isect->t = t1;
    isect->n = n;
    isect->primitive = this;
    isect->bsdf = get_bsdf();
    return true;
  }


  return false;
}

void Sphere::draw(const Color& c) const { Misc::draw_sphere_opengl(o, r, c); }

void Sphere::drawOutline(const Color& c) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

}  // namespace StaticScene
}  // namespace CMU462
