#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

#include "environment_light.h"

namespace CMU462 {
namespace StaticScene {

using std::cos;
using std::cout;
using std::endl;
using std::floor;
using std::sin;

Vector3D sampleSphereUniform(double *theta_ptr, double *phi_ptr, float *pdf) {
  *pdf = 1.f / 4 / PI;
  const double xi1 = (double)(std::rand()) / RAND_MAX;
  const double xi2 = (double)(std::rand()) / RAND_MAX;
  const double phi = 2 * PI * xi1;
  const double theta = std::acos(1 - 2 * xi2);
  *theta_ptr = theta;
  *phi_ptr = phi;

  return {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
}

Spectrum interpolateBilnear(double theta, double phi,
                            const HDRImageBuffer &envMap) {
  float x = phi / 2 / PI * envMap.w;
  float y = theta / PI * envMap.h;
  auto x0 = static_cast<size_t>(floor(x));
  auto y0 = static_cast<size_t>(floor(y));
  auto x1 = std::min(x0 + 1, envMap.w);
  auto y1 = std::min(y0 + 1, envMap.h);

  auto c00 = envMap.data[x0 + y0 * envMap.w];
  auto c01 = envMap.data[x0 + y1 * envMap.w];
  auto c10 = envMap.data[x1 + y0 * envMap.w];
  auto c11 = envMap.data[x1 + y1 * envMap.w];

  auto cxy0 = (x1 - x) * c00 + (x - x0) * c10;
  auto cxy1 = (x1 - x) * c01 + (x - x0) * c11;
  auto cxy = (y1 - y) * cxy0 + (y - y0) * cxy1;

  return cxy;
}

Vector3D EnvironmentLight::sampleSphereImportance(double *theta_ptr,
                                                  double *phi_ptr,
                                                  float *pdf) const {
  const double xi1 = (double)(std::rand()) / RAND_MAX;
  const double xi2 = (double)(std::rand()) / RAND_MAX;

  size_t theta_idx = std::upper_bound(cdf_theta.begin(), cdf_theta.end(), xi1) -
                   cdf_theta.begin();
//  cout << xi1 << " theta idx " << theta_idx << endl;
  theta_idx = std::min(theta_idx, envMap->h - 1);
  const auto& cdf_phi_theta = cdf_phi_theta_conditional[theta_idx];
  size_t phi_idx = std::upper_bound(cdf_phi_theta.begin(), cdf_phi_theta.end(),
      xi2) - cdf_phi_theta.begin();
  phi_idx = std::min(phi_idx, envMap->w - 1);
//  cout << xi2 <<  " phi idx " << phi_idx << endl;

  auto theta = (0.5 + theta_idx) / envMap->h * PI;
  auto phi = (0.5 + phi_idx) / envMap->w * 2 * PI;
  *theta_ptr = theta;
  *phi_ptr = phi;

  *pdf = p_theta_phi[phi_idx + theta_idx * envMap->w] / sin(theta) /
      dtheta / dphi;

//  cout << theta << " " << phi << " " << *pdf << endl;

  return {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
}

EnvironmentLight::EnvironmentLight(const HDRImageBuffer *envMap)
    : envMap(envMap), dtheta(PI / envMap->h), dphi(2 * PI / envMap->w) {
  // TODO: (PathTracer) initialize things here as needed

  // Joint distribution.
  p_theta_phi.resize(envMap->w * envMap->h);
  double p_theta_phi_sum = 0;
  for (size_t j = 0; j < envMap->h; j++) {
    double theta = (j + 0.5) / envMap->h * PI;
//    cout << j << " " << theta << endl;
    for (size_t i = 0; i < envMap->w; i++) {
      size_t idx = i + j * envMap->w;
      p_theta_phi[idx] = sin(theta) * envMap->data[idx].illum();
      p_theta_phi_sum += p_theta_phi[idx];
//      cout << i << " " << sin(theta) << " " << p_theta_phi[i] << endl;
    }
  }
//  cout << "SUM! " << p_theta_phi_sum << endl;

  for (auto &p : p_theta_phi) {
    p /= p_theta_phi_sum;
  }

  double abc = 0;
  for (auto &p : p_theta_phi) {
    abc += p;
  }
//  cout << "SUM! " << p_theta_phi_sum << abc << endl;

  // Theta marginal.
  std::vector<double> p_theta;
  p_theta.resize(envMap->h);
  cdf_theta.resize(envMap->h);
  std::fill(p_theta.begin(), p_theta.end(), 0);
  for (size_t j = 0; j < envMap->h; j++) {
    for (size_t i = 0; i < envMap->w; i++) {
      size_t idx = i + j * envMap->w;
      p_theta[j] += p_theta_phi[idx];
    }
    if (j == 0) {
      cdf_theta[j] = p_theta[j];
    } else {
      cdf_theta[j] = p_theta[j] + cdf_theta[j - 1];
    }
//    cout << j << " " << cdf_theta[j] << p_theta[j] << endl;
  }

  // p(phi | theta)
  std::vector<std::vector<double>> p_phi_theta_conditional;
  p_phi_theta_conditional.resize(envMap->h);
  cdf_phi_theta_conditional.resize(envMap->h);
  for (size_t j = 0; j < envMap->h; j++) {
    p_phi_theta_conditional[j].resize(envMap->w);
    cdf_phi_theta_conditional[j].resize(envMap->w);
    if (p_theta[j] > 0) {
      for (size_t i = 0; i < envMap->w; i++) {
        double a = p_theta_phi[i + j * envMap->w] / p_theta[j];
        p_phi_theta_conditional[j][i] = a;
        if (i == 0) {
          cdf_phi_theta_conditional[j][i] = a;
        } else {
          cdf_phi_theta_conditional[j][i] =
              cdf_phi_theta_conditional[j][i - 1] + a;
        }
      }
    } else {
      std::fill(p_phi_theta_conditional[j].begin(),
                p_phi_theta_conditional[j].end(), 0);
      std::fill(cdf_phi_theta_conditional[j].begin(),
                cdf_phi_theta_conditional[j].end(), 0);
    }
    //    cout << p_theta[j] << " " <<
    //      *(cdf_phi_theta_conditional[j].end() - 1)<< endl;
  }
}

Spectrum EnvironmentLight::sample_L(const Vector3D &p, Vector3D *wi,
                                    float *distToLight, float *pdf) const {
  // TODO: (PathTracer) Implement
  double theta, phi;
  *wi = sampleSphereImportance(&theta, &phi, pdf);
  *distToLight = INF_F;

  return interpolateBilnear(theta, phi, *envMap);
}

Spectrum EnvironmentLight::sample_dir(const Ray &r) const {
  // TODO: (PathTracer) Implement
  Vector3D v = r.d.unit();
  double theta = acos(v.z);
  double phi = atan2(v.y, v.x) + PI;

  return interpolateBilnear(theta, phi, *envMap);
}

} // namespace StaticScene
} // namespace CMU462
