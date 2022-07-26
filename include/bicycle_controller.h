#ifndef __BICYCLE_CONTROL_H
#define __BICYCLE_CONTROL_H

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "model.h"
#include "main.h"

#define PI 3.1415926535

typedef struct{
/*自行车常量*/
	double mb, hb, Jb, R;//质量、重心高度、转动惯量、车轮半径
	double g;
	double l, lb, lt;//前后轮距、重心至后轮距离、前轮尾迹
	double psi_offset;//偏航角的偏差 (rad)
	double epsilon;
	double Jt;	
/*位姿*/
	double x, y;
	double xe, ye;
	double vr;		//前轮速度(m/s)
	double varphi, dvarphi;	//倾斜角、倾斜角速度
	double psi, dpsi;//偏航角、偏航角速度
	double phi, dphi;//转向角、转向角速度	
/*计算过程*/
	double varphie;	
/*控制参数*/
	double b2, b1, b0;
	double k, a1, a0;	
	double dx,dy,accx,accy,accz,dvarphie,dpitch;
/*控制输出*/
	double new_phi;	//算出到转向角
	double last_target, target;//上一次目标转向角、当前目标转向角
}Balance_data;

class bicycle_controller{

public:
	bicycle_controller(robot *bicycle, char *ch);
	void set_PDGain();
	void initial_param();
	Eigen::VectorXd tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT);
	Eigen::VectorXd get_action(int bike_mode, float velocity);
	Eigen::VectorXd get_action1(int bike_mode, float velocity, int kbflag);
	void balanceCalc();
	void balanceCalc1(int traj, int kbflag);
	void balanceCalc2(int traj, int kbflag);
	void balanceCalc3(int traj);
	void balance();
	void stateUpdate();
	double get_xdistance();
	double get_tau();
	double get_h_varphi();
	double get_varphie();
private:
	robot *bike;
	Eigen::VectorXd pGain, dGain;
	Balance_data bikebot;
	double phi_cmd;
};

double solution(double vr, double dpsi, double u_psi);
double rad2deg(double rad);
double signd(double x);
#endif