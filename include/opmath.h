#ifndef __OPMATH_HPP
#define __OPMATH_HPP

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>

// void rotToRpy(Eigen::Matrix3d &rot_matrix,Eigen::Vector3d &rpy);
// void quatToRpy(Eigen::Vector4d &qua,Eigen::Vector3d &rpy);
// void quattToRpy(Eigen::Vector4d &qua,Eigen::Vector3d &rpy);
// void quatToEulerVec(const double* quat, double* eulerVec);
Eigen::Vector4d RpyToqua(double yaw, double pitch, double roll);
void quaToRpy(Eigen::Vector4d &qua,Eigen::Vector3d &rpy);
void toEulerAngle(Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);
Eigen::Matrix3d rpy2romatrix(double roll,double pitch,double yaw);

#endif