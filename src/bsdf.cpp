#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::min;
using std::max;
using std::swap;
using std::cout;
using std::endl;

namespace CMU462 {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {
  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
    h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
    h.y = 1.0;
  else
    h.z = 1.0;

  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

// Diffuse BSDF //

Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return albedo * (1.0 / PI);
}

Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO (PathTracer):
  // Implement DiffuseBSDF
//  *wi = cos_sampler.get_sample(pdf);

  *pdf = 1.f / 2 / PI;
  *wi = uniform_sampler.get_sample();
  return albedo * (1.0 / PI);
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO (PathTracer):
  // Implement MirrorBSDF
  reflect(wo, wi);
  *pdf = 1;
  return 1.f / fabs(wo.z) * reflectance;
}

// Glossy BSDF //

/*
Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0f;
  return reflect(wo, wi, reflectance);
}
*/

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return 1 / cos_theta(wo);
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi,
                                  float* pdf) {
  // TODO (PathTracer):
  // Implement RefractionBSDF
  *wi = wo;
  *pdf = 1;
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO (PathTracer):
  // Compute Fresnel coefficient and either reflect or refract based on it.
  ior = 2.5;

  if(!refract(wo, wi, ior)) {
    reflect(wo, wi);
    *pdf = 1;
    return 1.f / wo.z * reflectance;
  }

  const double R0 = pow((1 - ior) / (1 + ior), 2);
  const double Fr = R0 + (1 - R0) * pow(1 - fabs(wi->z), 5);

  double random_number = (double)(std::rand()) / RAND_MAX;
  if(random_number < Fr) {
    // reflect
    *pdf = Fr;
    reflect(wo, wi);
    return *pdf / wi->z * reflectance;
  } else {
    // refract
    *pdf = 1 - Fr;
    return *pdf / wi->z * transmittance;
  }
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {
  // TODO (PathTracer):
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  *wi = Vector3D(-wo[0], -wo[1], wo[2]);
}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi_ptr, float ior) {
  // TODO (PathTracer):
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo, n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
  double b;
  if(wo.z >= 0) {
    b = 1 / ior;
  } else {
    b = ior;
  }

  Vector3D& wi = *wi_ptr;
  double a = 1 - b * b * (1 - wo.z * wo.z);

  if(a >= 0) {
    wi[0] = -wo[0] * b;
    wi[1] = -wo[1] * b;
    wi[2] = (wo.z >= 0 ? -1. : 1.) * sqrt(a);
    return true;
  } else {
    return false;
  }

}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *wi = sampler.get_sample(pdf);
  return Spectrum();
}

}  // namespace CMU462
