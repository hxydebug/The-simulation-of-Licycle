#ifndef __MODEL_H
#define __MODEL_H

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "main.h"
#include<iostream>
#include<queue>

class robot{

public:
	robot(raisim::ArticulatedSystem* client);
	raisim::ArticulatedSystem* bicycle;

	int getGeneralizedCoordinateDim();
	int getDOF();
	void initial_bicycle();
	Eigen::VectorXd get_bicycle_vel();
	Eigen::VectorXd get_leg_vel();
	Eigen::VectorXd get_bicycle_pos();
	Eigen::VectorXd get_leg_pos();
	Eigen::VectorXd get_base_orientation();
	Eigen::VectorXd get_base_rpy();
	Eigen::VectorXd get_base_angle_v();
	double get_base_v();
	Eigen::Vector3d get_base_vec();
	double get_psi();
	double get_varphi();
	double get_phi();
	double get_dpsi();
	double get_dvarphi();
	double get_dphi();
	void update_state();
	void update_angle();
	std::vector<int> GetFootContact();
	void step(Eigen::VectorXd bicycle_tau,Eigen::VectorXd leg_tau);
private:
	Eigen::VectorXd pos;
	Eigen::VectorXd vel;
	Eigen::Matrix3d rot_matrix;
	double dpsi,last_dpsi = 0,llast_dpsi = 0;
	double dvarphi,last_dvarphi = 0,llast_dvarphi = 0;
	std::vector<int> contact_leg={0,0};
	// std::queue<double> dpsi_q;
	// std::queue<double> dvarphi_q;

};

#endif