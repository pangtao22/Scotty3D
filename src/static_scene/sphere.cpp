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
  const Vector3D origin = r.o - o;
  const double a = dot(r.d, r.d);
  const double b = 2 * dot(origin, r.d);
  const double c =  dot(origin, origin) - this->r;
  const double delta = b * b - 4 * a * c;
  bool result = false;

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
    result = (t0 > r.min_t) && (t0 < r.max_t);
    if(result) {
      r.max_t = t0;
    }
//    cout << r.o << endl;
//    cout << r.d << endl;
//    cout << result << endl;
//    cout << r.min_t << " " << r.max_t << endl;
//    cout << t0 << " " << t1 << endl << endl;
  }

  return result;
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
    isect->t = t1;
    isect->n = normal(r.o + r.d * t1);
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
