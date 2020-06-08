#include <random>
#include <math.h>
#include "sampler.h"

namespace CMU462 {

// Uniform Sampler2D Implementation //

Vector2D UniformGridSampler2D::get_sample() const {
  // TODO (PathTracer):
  // Implement uniform 2D grid sampler
  return Vector2D(double(std::rand()) / RAND_MAX,
                  double(std::rand()) / RAND_MAX);
}

// Uniform Hemisphere Sampler3D Implementation //

Vector3D UniformHemisphereSampler3D::get_sample() const {
  double Xi1 = (double)(std::rand()) / RAND_MAX;
  double Xi2 = (double)(std::rand()) / RAND_MAX;

  double a = sqrt(1 - Xi1 * Xi1);
  double b = 2.0 * PI * Xi2;

  double xs = a * cos(b);
  double ys = a * sin(b);
  double zs = Xi1;

  return Vector3D(xs, ys, zs);
}

Vector3D CosineWeightedHemisphereSampler3D::get_sample() const {
  float f;
  return get_sample(&f);
}

Vector3D CosineWeightedHemisphereSampler3D::get_sample(float *pdf) const {
  // You may implement this, but don't have to.
  double Xi1 = (double)(std::rand()) / RAND_MAX;
  double Xi2 = (double)(std::rand()) / RAND_MAX;

  double theta = acos(1 - 2 * Xi2) / 2;
  double phi = 2.0 * PI * Xi1;
  double s_theta = sin(theta);
  double c_theta = cos(theta);
  double s_phi = sin(phi);
  double c_phi = cos(phi);

  double x = s_theta * c_phi;
  double y = s_theta * s_phi;
  double z = c_theta;
  *pdf = s_theta * c_theta / PI;

  return Vector3D(x, y, z);
}

}  // namespace CMU462
