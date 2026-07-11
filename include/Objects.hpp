#pragma once

#include <Eigen/Dense>

#include "Config.hpp"

struct Imu {
  double stamp;
  Eigen::Vector3d ang_vel;
  Eigen::Vector3d lin_accel;
  Eigen::Quaterniond q;

  Imu() : stamp(0.),
          ang_vel(Eigen::Vector3d::Zero()),
          lin_accel(Eigen::Vector3d::Zero()),
          q(Eigen::Quaterniond::Identity()) {}
};


struct Cone {
    double x;
    double y;
    double z;
    int idx;
    float confidence = 0.;

    Cone() : x(0.0), y(0.0), z(0.0), idx(0) {}
    Cone(double a, double b, double c) : x(a), y(b), z(c), idx(0) {}
    Cone(double a, double b, double c, int i) : x(a), y(b), z(c), idx(i) {}
    Cone(double a, double b, double c, float conf) : x(a), y(b), z(c), confidence(conf) {}
    Cone(double a, double b, double c, int i, float conf) : x(a), y(b), z(c), idx(i), confidence(conf) {}

    Eigen::Vector3d toEigen() const {
      return Eigen::Vector3d(x, y, z);
    }
};

inline Cone operator*(const Eigen::Isometry3d& A, const Cone& c) {

    Eigen::Vector3d v = A * c.toEigen();
    return Cone(v.x(), v.y(), v.z(), c.idx, c.confidence);
}

using Cones = std::vector<Cone>;