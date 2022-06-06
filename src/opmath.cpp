#include "opmath.h"

Eigen::Vector4d RpyToqua(double yaw, double pitch, double roll){
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Vector4d qua;
    qua[0] = cy * cp * cr + sy * sp * sr;
    qua[1] = cy * cp * sr - sy * sp * cr;
    qua[2] = sy * cp * sr + cy * sp * cr;
    qua[3] = sy * cp * cr - cy * sp * sr;

    return qua;

}
void quaToRpy(Eigen::Vector4d &qua,Eigen::Vector3d &rpy){
    Eigen::Quaterniond quaternion(qua[0],qua[1],qua[2],qua[3]);
    toEulerAngle(quaternion, rpy[0], rpy[1], rpy[2]);
}

void toEulerAngle(Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
// roll (x-axis rotation)
double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
if (fabs(sinp) >= 1)
pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
else
pitch = asin(sinp);

// yaw (z-axis rotation)
double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
yaw = atan2(siny_cosp, cosy_cosp);
}

Eigen::Matrix3d rpy2romatrix(double roll,double pitch,double yaw){
    Eigen::Vector3d ea(roll, pitch, yaw);
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX());
    return rotation_matrix3;
}



// void quaToRpy(Eigen::Vector4d &qua,Eigen::Vector3d &rpy){
//     Eigen::Quaterniond quaternion(qua[0],qua[1],qua[2],qua[3]);
//     rpy = quaternion.matrix().eulerAngles(2,1,0);
// }
// void quatToRpy(Eigen::Vector4d &qua,Eigen::Vector3d &rpy){
//     double quat[4];
//     double eulerVec[3];
//     for(int i(0);i<4;i++){
//         quat[i] = qua[i];
//     }
//     quatToEulerVec(quat, eulerVec);
//     for(int i(0);i<3;i++){
//         rpy[i] = eulerVec[i];
//     }
// }
// void quatToEulerVec(const double* quat, double* eulerVec) {
//   const double norm = (std::sqrt(quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]));
//   if(fabs(norm) < 1e-12) {
//     eulerVec[0] = 0;
//     eulerVec[1] = 0;
//     eulerVec[2] = 0;
//     return;
//   }

//   const double normInv = 1.0/norm;
//   const double angleNormInv = std::acos(std::min(quat[0],1.0)) * 2.0 * normInv;
//   eulerVec[0] = quat[1] * angleNormInv;
//   eulerVec[1] = quat[2] * angleNormInv;
//   eulerVec[2] = quat[3] * angleNormInv;
// }

// void rotToRpy(Eigen::Matrix3d &rot_matrix,Eigen::Vector3d &rpy){
//     rpy = rot_matrix.eulerAngles(2,1,0);
// }
