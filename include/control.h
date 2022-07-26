#ifndef __CONTROL_H
#define __CONTROL_H
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>


#define fl 0
#define fr 1

#define PI 3.1415926535

#define Len0 0.0755f
#define Len1 0.2115f
#define Len2 0.22f

#define width 0.26f
#define detx 0.02f
#define detz 0.035f
// #define detx 0.02f
// #define detz 0.065f
#define hG 0.338f

typedef int Leg;

typedef struct{
	float q[3];
} Angle;

typedef struct{
	float x;
	float y;
	float z;
} Position;

//general
void Kinematics(Angle *angle,Position *position,Leg leg);
void Inv_kinematics(Angle *angle,Position *position,Leg leg);
void Kinematics_ref(Angle *angle,Position *position,Leg leg);
void Inv_kinematics_ref(Angle *angle,Position *position,Leg leg);
Eigen::MatrixXd calcu_Jaco(Eigen::Vector3d angle,Leg leg);
Position getFootPositionInBaswFrame(Eigen::VectorXd angle, Leg leg);



#endif