#ifndef BODY_H
#define BODY_H
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>

struct Body
{
  PCL_ADD_POINT4D;
  double m;
  double f[3];
  double v[3];
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Body(double x, double y, double z, double m) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->m = m;
    force().setZero();
    velocity().setZero();
  }

  inline Eigen::Map<Eigen::Vector3d> force() {
    return Eigen::Map<Eigen::Vector3d>(f);
  }

  inline Eigen::Map<Eigen::Vector3d> velocity() {
    return Eigen::Map<Eigen::Vector3d>(v);
  }

} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(Body,
                                  (double, x, x)
                                  (double, y, y)
                                  (double, z, z)
                                  (double, m, m)
                                  (double[3], f, f)
                                  (double[3], v, v))
#endif /* BODY_H */
