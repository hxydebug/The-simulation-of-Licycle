#include "bicycle_controller.h"

using namespace std;
ofstream dataFile;

///定义常数
double body_m = 25;
double body_h = 0.365;
double body_Jb = 0.8;
double body_l = 0.87;
double body_lb = 0.42;

double body_epsilon = 17.0 * PI / 180.0;
double body_R = 0.21;
double body_lt = body_R * tan(body_epsilon);
double body_g = 9.8;

double body_Jt = body_m*body_h*body_h + body_Jb;

double body_b2 = 10, body_b1 = 6, body_b0 =3;
// double body_b2 = 3, body_b1 = 13, body_b0 = 10;
double body_a1 = 25, body_a0 = 180;
// double body_a1 = 5, body_a0 = 11;
double body_k = 1.0;

double body_vr = 0;
double tire_omega = body_vr/body_R;
double LIMIT_STEER = 30.0 * PI / 180.0;
// double LIMIT_STEER = 5.0 * PI / 180.0;
double new_Wt = -PI/2;
double dt = 0.02;
double Wt = 3.14;

///定义变量
double xe = 0; 			double ye = 0;
double dxe = 0;   	 	double dye = 0;
double ddxe = 0;   	 	double ddye = 0;
double dddxe = 0;   	double dddye = 0;
double ddddxe = 0;  	double ddddye = 0;
double dddddxe = 0; 	double dddddye = 0;

double tau_S= 0;
double tau_Smax = 0;
double e_tau = 0;
double h_varphi = 0;
double bike_varphie = 0;


bicycle_controller::bicycle_controller(robot *bicycle, char *ch){
	bike = bicycle;
	bicycle_controller::initial_param();
	bicycle_controller::set_PDGain();
	char result[100] = {0};
	sprintf(result, "/home/hxy/sim_data/bike_data%s.txt", ch);
	dataFile.open(result, ofstream::app);
}

void bicycle_controller::initial_param(){
	/*自行车常量*/
	bikebot.mb = body_m;	bikebot.hb = body_h;
	bikebot.Jb = body_Jb;	bikebot.R  = body_R;
	bikebot.g  = body_g;	bikebot.l  = body_l;
	bikebot.lb = body_lb;	bikebot.lt = body_lt;
	bikebot.Jt = body_Jt;
	bikebot.epsilon = body_epsilon;
	bikebot.psi_offset = bike->get_psi();
	
	/*控制参数*/
	bikebot.b2 = body_b2;	bikebot.b1 = body_b1;	bikebot.b0 = body_b0;	
	bikebot.a1 = body_a1;	bikebot.a0 = body_a0;
	bikebot.k  = body_k;
	
	/*其他初值*/
	bikebot.vr = body_vr;
	bikebot.psi = 0;
	bikebot.x = 0;
	bikebot.y = 0;
	bikebot.last_target = 0;
	phi_cmd = 0;
}
void bicycle_controller::set_PDGain(){
	pGain.resize(3);
	dGain.resize(3);
	// pGain<< 100, 0, 0; 
	// dGain<< 10.0, 10.0, 0.0;
	pGain<< 100, 0, 0; 
	dGain<< 20, 4.5, 0.0;
}
double bicycle_controller::get_xdistance(){
	return bikebot.x;
}
Eigen::VectorXd bicycle_controller::get_action(int bike_mode, float velocity){
	
	Eigen::VectorXd p(3),v(3);
	// bikebot.vr = velocity;
	tire_omega = velocity/body_R;
	dGain << 3, 4.5, 0.0;

	bicycle_controller::stateUpdate();

	if(bike_mode == 0){

		///get the control input
		p<< 0,0,0;
		v<< 0,-tire_omega,0;
	}
	else if(bike_mode == 1){

		///no torque
		p<< 0,0,0;
		v<< 0,-tire_omega,0;
		dGain<< 3, 0, 0.0;
	}
	else if(bike_mode == 2){

		///run the controller
		bicycle_controller::balanceCalc1(1);
		///get the control input
		p<< -phi_cmd,0,0;
		v<< 0,-tire_omega,0;
	}
	else if(bike_mode == 3){

		///run the controller circle
		bicycle_controller::balanceCalc1(0);
		///get the control input
		p<< -phi_cmd,0,0;
		v<< 0,-tire_omega,0;
	}
	else if(bike_mode == 4){

		///run the controller without eic
		bicycle_controller::balanceCalc2(0);
		///get the control input
		p<< -phi_cmd,0,0;
		v<< 0,-tire_omega,0;
	}
	else{
		///run the controller with eic
		bicycle_controller::balanceCalc3(1);
		///get the control input
		p<< -phi_cmd,0,0;
		v<< 0,-tire_omega,0;
	}

	return bicycle_controller::tau(bike->get_bicycle_pos(),bike->get_bicycle_vel(),p,v);
	
}
Eigen::VectorXd bicycle_controller::tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT){
//   Eigen::VectorXd vTarget = vT;
//   Eigen::VectorXd vActual = vA;
//   vTarget[1] = -body_vr;
//   vActual[1] = bike->get_base_v();
  return dGain.cwiseProduct(vT-vA) + pGain.cwiseProduct(pT-pA);

}
void bicycle_controller::stateUpdate(){
	bikebot.varphi = bike->get_varphi();
	bikebot.dvarphi = bike->get_dvarphi();

	bikebot.phi = bike->get_phi();
	bikebot.dphi = bike->get_dphi();

	bikebot.psi = bike->get_psi() - bikebot.psi_offset;
	bikebot.dpsi = bike->get_dpsi();

	bikebot.vr = bike->get_base_v();


	bikebot.dx = bikebot.vr * cos(bikebot.psi);
	bikebot.dy = bikebot.vr * sin(bikebot.psi);

	//更新下一时刻参数
	bikebot.x += bikebot.dx * dt;
	bikebot.y += bikebot.dy * dt;

	// std::cout<<"varphi: "<<rad2deg(bikebot.varphi)<<std::endl;
	// std::cout<<"dvarphi: "<<rad2deg(bikebot.dvarphi)<<std::endl;
	// std::cout<<"phi: "<<rad2deg(bikebot.phi)<<std::endl;
	// std::cout<<"dphi: "<<rad2deg(bikebot.dphi)<<std::endl;
	// std::cout<<"psi: "<<rad2deg(bikebot.psi)<<std::endl;
	// std::cout<<"dpsi: "<<rad2deg(bikebot.dpsi)<<std::endl;
	// std::cout<<"x: "<<bikebot.x<<std::endl;
	// std::cout<<"y: "<<bikebot.y<<std::endl;
	// std::cout<<"vr: "<<bikebot.vr<<std::endl;

	/*** record the data ***/
	// 朝TXT文档中写入数据
	dataFile << bikebot.varphi << ", " << bikebot.dvarphi << ", " << bikebot.phi << ", " 
			<< bikebot.dphi<< ", " << bikebot.psi << ", " << bikebot.dpsi << ", " 
			<< bikebot.vr << ", " << bikebot.x << ", " << bikebot.y << ", " 
			<< bikebot.accx << ", " << bikebot.accy << ", " << bikebot.accz << ", " 
			<< bikebot.varphie << ", " << bikebot.dvarphie << ", " << bikebot.dpitch << ", " 
			<< bikebot.new_phi << ", " << bikebot.dx << ", " << bikebot.dy << ", " 
			<< bikebot.xe << ", " << bikebot.ye << ", " <<phi_cmd << ", " << tau_Smax << ", " << tau_S << ", " << e_tau
			<< std::endl;
}

void bicycle_controller::balance(){
	double varphie = 0.0*PI/180.0;
	double a1 = 20;
	double a0 = 70;
	double m = bikebot.mb;
	double h = bikebot.hb;
	double l = bikebot.l;
	double lb = bikebot.lb;
	double lt = bikebot.lt;
	double epsilon = bikebot.epsilon;
	double g = bikebot.g;
	double Jt = bikebot.Jt;
	double Jb = bikebot.Jb;

	double vr = bikebot.vr;
	double psi = bikebot.psi;
	double dpsi = bikebot.dpsi;
	double varphi = bikebot.varphi;
	double dvarphi = bikebot.dvarphi;
	double sinpsi = sin(psi);		double cospsi = cos(psi);

	double v_psi_int = -a1*dvarphi - a0*(varphi-varphie);
	double f_varphi = m*h*cos(varphi)*vr*dpsi+m*h*h*cos(varphi)*sin(varphi)*dpsi*dpsi
			+m*h*g*sin(varphi)+m*g*lt*lb*dpsi*cos(varphi)/vr*cos(epsilon);
	double g_varphi = m*h*lb*cos(varphi);
	double u_psi_int = (-f_varphi+Jt*v_psi_int)/g_varphi;

	/*计算转向角*/
	bikebot.new_phi = atan((dpsi+u_psi_int*dt)*l*cos(varphi)/vr/cos(epsilon));
	double target = bikebot.new_phi;
	if(target > LIMIT_STEER) target = LIMIT_STEER;
	else if(target < -LIMIT_STEER) target = -LIMIT_STEER;
	bikebot.target = target;
	phi_cmd = bikebot.target;
	
	// //控制限制
	double limit = 0.02;
	double last = bikebot.last_target;
	target = bikebot.target;
	if(target > last) phi_cmd = (target - last) < limit ? target : last + limit;
	else phi_cmd = (last - target) < limit ? target : last - limit;
	bikebot.last_target = phi_cmd;
	

}

void bicycle_controller::balanceCalc(){
	/*变量读取*/
	double vr = bikebot.vr;		double dvr = 0;
	double psi = bikebot.psi;
	double dpsi = bikebot.dpsi;
	double dpsi3 = dpsi*dpsi*dpsi;//dpsi的三次方
	double sinpsi = sin(psi);		double cospsi = cos(psi);
	double x = bikebot.x;	double dx = vr * cospsi;	double ddx = -vr * sinpsi * dpsi;
	double y = bikebot.y;	double dy = vr * sinpsi;	double ddy = vr * cospsi * dpsi;
	double ve = vr;
	/*trajectory*/
	// 走圆	
	double cr = 1.5; double w = ve/cr;
	double x_offset = 0; double y_offset = cr;
	new_Wt += w*dt; 
	xe = x_offset + cr * cos(new_Wt); 	ye =  y_offset + cr*sin(new_Wt);
	dxe = -cr * sin(new_Wt) * w;		dye = cr * cos(new_Wt) * w;	
	ddxe = -dye * w; 		 ddye = dxe * w;
	dddxe = -ddye * w; 		 dddye = ddxe * w;
	ddddxe = -dddye * w; 	 ddddye = dddxe * w;
	dddddxe = -ddddye * w; 	 dddddye = ddddxe * w;
	// //走直线
	// dxe = ve; dye = 0; 
	// xe += ve*dt; ye = 0;

	bikebot.xe = xe;
	bikebot.ye = ye;
	// std::cout<<"v: "<<bike->get_base_v()<<std::endl;
	// std::cout<<"x: "<<bikebot.x<<"xe: "<<bikebot.xe<<std::endl;
	// std::cout<<"y: "<<bikebot.y<<"ye: "<<bikebot.ye<<std::endl;
	/*External*/
	//利用控制律计算出前轮轨迹r(3)及其一次导和二次导
	bikebot.b2 = 1;	bikebot.b1 = 1;	bikebot.b0 = 5;	
	double u_w_ext1 = dddxe + bikebot.b2 * (ddxe - ddx) + bikebot.b1 * (dxe - dx) + bikebot.b0 * (xe - x);
	double u_w_ext2 = dddye + bikebot.b2 * (ddye - ddy) + bikebot.b1 * (dye - dy) + bikebot.b0 * (ye - y);
	double du_w_ext1 = ddddxe + bikebot.b2 * (dddxe - u_w_ext1) + bikebot.b1 * (ddxe - ddx) + bikebot.b0 * (dxe - dx);
	double du_w_ext2 = ddddye + bikebot.b2 * (dddye - u_w_ext2) + bikebot.b1 * (ddye - ddy) + bikebot.b0 * (dye - dy);
	double ddu_w_ext1 = dddddxe + bikebot.b2 * (ddddxe - du_w_ext1) + bikebot.b1 * (dddxe - u_w_ext1) + bikebot.b0 * (ddxe - ddx);
	double ddu_w_ext2 = dddddye + bikebot.b2 * (ddddye - du_w_ext2) + bikebot.b1 * (dddye - u_w_ext2) + bikebot.b0 * (ddye - ddy);
	// double ur = vr*dpsi*dpsi+cospsi*u_w_ext1+sinpsi*u_w_ext2;
	//这里速度到二次导我们设为0
	double ur = 0;
	//计算出满足external条件的u_psi及其一阶和二阶导
	// double u_psi = -2*dvr*dpsi/vr-sinpsi/vr*u_w_ext1+cospsi/vr*u_w_ext2;
	// double du_psi = -2*((ur*vr-dvr*dvr)/vr/vr*dpsi+dvr/vr*u_psi) - (cospsi*dpsi*vr-sinpsi*dvr)/vr/vr*u_w_ext1 - sinpsi/vr*du_w_ext1
    //         + (-sinpsi*dpsi*vr-cospsi*dvr)/vr/vr*u_w_ext2 + cospsi/vr*du_w_ext2;
    // double ddu_psi = (sinpsi*dpsi*dpsi-cospsi*u_psi)/vr*u_w_ext1 - 2*cospsi*dpsi/vr*du_w_ext1 - sinpsi/vr*ddu_w_ext1
    //         - (cospsi*dpsi*dpsi+sinpsi*u_psi)/vr*u_w_ext2 - 2*sinpsi*dpsi/vr*du_w_ext2 + cospsi/vr*ddu_w_ext2;
	
	// /*Internal*/
	// //将满足external条件的u_psi代入求解器，求解期望倾斜角
	// double varphie = solution(vr, dpsi, u_psi);//牛顿迭代法求方程根
	// bikebot.varphie = varphie;
	// //求解期望倾斜角的一阶和二阶导
	// double cosvare = cos(varphie);
	// double sinvare = sin(varphie);
	// double M1 = bikebot.hb*cosvare*dpsi*dpsi+bikebot.g/cosvare/cosvare;
	// double hhh = bikebot.g*bikebot.lt*bikebot.lb*cos(bikebot.epsilon)/bikebot.hb;
    // double M2 = (u_psi*vr+dpsi*dvr+2*bikebot.hb*dpsi*u_psi*sinvare+hhh*(u_psi*vr-dpsi*dvr)/vr/vr+bikebot.lb*du_psi);	
	// double dvarphie = -M2/M1;	
	// double dM1 = (2*bikebot.hb*dpsi*u_psi*cosvare-bikebot.hb*dpsi*dpsi*sinvare+2*bikebot.g/cosvare/cosvare*sinvare/cosvare)*dvarphie;
	// double dM2 = du_psi*vr+2*u_psi*dvr+dpsi*ur+2*bikebot.hb*(u_psi*u_psi*sinvare+dpsi*du_psi*sinvare+dpsi*u_psi*cosvare*dvarphie)
    //         + hhh * (du_psi*vr-2*u_psi*dvr-dpsi*ur+2*dpsi*dvr*dvr/vr)/vr/vr + bikebot.lb*ddu_psi;    
	// double ddvarphie = (dM1*M2/M1 - dM2)/M1;
	// //根据控制律求解合适到控制输入:varphi的二阶导
	// double v_psi_int = ddvarphie + bikebot.a1*(dvarphie-bikebot.dvarphi)+ bikebot.a0*(varphie-bikebot.varphi);
	// //将控制输入代入动力学模型方程，计算得到控制量u_psi
	double cosvar = cos(bikebot.varphi);
	double sinvar = sin(bikebot.varphi);
	// double xxx = bikebot.mb*bikebot.g*bikebot.lt*bikebot.lb*cos(bikebot.epsilon);
	// double f_varphi = (bikebot.mb*bikebot.hb*vr + bikebot.mb*bikebot.hb*bikebot.hb*sinvar*dpsi)*cosvar*dpsi
	// 			+bikebot.mb*bikebot.hb*bikebot.g*sinvar + xxx*dpsi*cosvar/vr;
	// double g_psi = bikebot.mb*bikebot.hb*bikebot.lb*cosvar;
	// double u_psi_int = (-f_varphi + bikebot.Jt*v_psi_int)/g_psi;
	//自此，以外部得到的u_r和内部得到的u_psi得到最终控制输入

	/*计算转向角*/
	bikebot.new_phi = u_w_ext2*2;
	double target = bikebot.new_phi;
	if(target > LIMIT_STEER) target = LIMIT_STEER;
	else if(target < -LIMIT_STEER) target = -LIMIT_STEER;
	bikebot.target = target;
	phi_cmd = bikebot.target;
	
	//控制限制
	double limit = 0.8;
	double last = bikebot.last_target;
	target = bikebot.target;
	if(target > last) phi_cmd = (target - last) < limit ? target : last + limit;
	else phi_cmd = (last - target) < limit ? target : last - limit;
	bikebot.last_target = phi_cmd;

}

void bicycle_controller::balanceCalc1(int traj){
	/*变量读取*/
	double vr = bikebot.vr;		double dvr = 0;
	double psi = bikebot.psi;
	double dpsi = bikebot.dpsi;
	double dpsi3 = dpsi*dpsi*dpsi;//dpsi的三次方
	double sinpsi = sin(psi);		double cospsi = cos(psi);
	double x = bikebot.x;	double dx = vr * cospsi;	double ddx = -vr * sinpsi * dpsi;
	double y = bikebot.y;	double dy = vr * sinpsi;	double ddy = vr * cospsi * dpsi;
	double ve = vr;
	/*trajectory*/
	// // 走圆	
	if(traj==0){
		double cr = 1.5; double w = ve/cr;
		double x_offset = 0; double y_offset = cr;
		new_Wt += w*dt; 
		xe = x_offset + cr * cos(new_Wt); 	ye =  y_offset + cr*sin(new_Wt);
		dxe = -cr * sin(new_Wt) * w;		dye = cr * cos(new_Wt) * w;	
		ddxe = -dye * w; 		 ddye = dxe * w;
		dddxe = -ddye * w; 		 dddye = ddxe * w;
		ddddxe = -dddye * w; 	 ddddye = dddxe * w;
		dddddxe = -ddddye * w; 	 dddddye = ddddxe * w;
	}
	//走直线
	else{
		dxe = ve; dye = 0; 
		xe += ve*dt; ye = 0;
	}
	

	bikebot.xe = xe;
	bikebot.ye = ye;
	// std::cout<<"v: "<<bike->get_base_v()<<std::endl;
	// std::cout<<"x: "<<bikebot.x<<"xe: "<<bikebot.xe<<std::endl;
	// std::cout<<"y: "<<bikebot.y<<"ye: "<<bikebot.ye<<std::endl;
	/*External*/
	//利用控制律计算出前轮轨迹r(3)及其一次导和二次导
	double u_w_ext1 = dddxe + bikebot.b2 * (ddxe - ddx) + bikebot.b1 * (dxe - dx) + bikebot.b0 * (xe - x);
	double u_w_ext2 = dddye + bikebot.b2 * (ddye - ddy) + bikebot.b1 * (dye - dy) + bikebot.b0 * (ye - y);
	double du_w_ext1 = ddddxe + bikebot.b2 * (dddxe - u_w_ext1) + bikebot.b1 * (ddxe - ddx) + bikebot.b0 * (dxe - dx);
	double du_w_ext2 = ddddye + bikebot.b2 * (dddye - u_w_ext2) + bikebot.b1 * (ddye - ddy) + bikebot.b0 * (dye - dy);
	double ddu_w_ext1 = dddddxe + bikebot.b2 * (ddddxe - du_w_ext1) + bikebot.b1 * (dddxe - u_w_ext1) + bikebot.b0 * (ddxe - ddx);
	double ddu_w_ext2 = dddddye + bikebot.b2 * (ddddye - du_w_ext2) + bikebot.b1 * (dddye - u_w_ext2) + bikebot.b0 * (ddye - ddy);
	// double ur = vr*dpsi*dpsi+cospsi*u_w_ext1+sinpsi*u_w_ext2;
	//这里速度到二次导我们设为0
	double ur = 0;
	//计算出满足external条件的u_psi及其一阶和二阶导
	double u_psi = -2*dvr*dpsi/vr-sinpsi/vr*u_w_ext1+cospsi/vr*u_w_ext2;
	double du_psi = -2*((ur*vr-dvr*dvr)/vr/vr*dpsi+dvr/vr*u_psi) - (cospsi*dpsi*vr-sinpsi*dvr)/vr/vr*u_w_ext1 - sinpsi/vr*du_w_ext1
            + (-sinpsi*dpsi*vr-cospsi*dvr)/vr/vr*u_w_ext2 + cospsi/vr*du_w_ext2;
    double ddu_psi = (sinpsi*dpsi*dpsi-cospsi*u_psi)/vr*u_w_ext1 - 2*cospsi*dpsi/vr*du_w_ext1 - sinpsi/vr*ddu_w_ext1
            - (cospsi*dpsi*dpsi+sinpsi*u_psi)/vr*u_w_ext2 - 2*sinpsi*dpsi/vr*du_w_ext2 + cospsi/vr*ddu_w_ext2;
	
	/*Internal*/
	//将满足external条件的u_psi代入求解器，求解期望倾斜角
	double varphie = solution(vr, dpsi, u_psi);//牛顿迭代法求方程根
	bikebot.varphie = varphie;
	//求解期望倾斜角的一阶和二阶导
	double cosvare = cos(varphie);
	double sinvare = sin(varphie);
	double M1 = bikebot.hb*cosvare*dpsi*dpsi+bikebot.g/cosvare/cosvare;
	double hhh = bikebot.g*bikebot.lt*bikebot.lb*cos(bikebot.epsilon)/bikebot.hb;
    double M2 = (u_psi*vr+dpsi*dvr+2*bikebot.hb*dpsi*u_psi*sinvare+hhh*(u_psi*vr-dpsi*dvr)/vr/vr+bikebot.lb*du_psi);	
	double dvarphie = -M2/M1;	
	bikebot.dvarphie = dvarphie;
	double dM1 = (2*bikebot.hb*dpsi*u_psi*cosvare-bikebot.hb*dpsi*dpsi*sinvare+2*bikebot.g/cosvare/cosvare*sinvare/cosvare)*dvarphie;
	double dM2 = du_psi*vr+2*u_psi*dvr+dpsi*ur+2*bikebot.hb*(u_psi*u_psi*sinvare+dpsi*du_psi*sinvare+dpsi*u_psi*cosvare*dvarphie)
            + hhh * (du_psi*vr-2*u_psi*dvr-dpsi*ur+2*dpsi*dvr*dvr/vr)/vr/vr + bikebot.lb*ddu_psi;    
	double ddvarphie = (dM1*M2/M1 - dM2)/M1;
	//根据控制律求解合适到控制输入:varphi的二阶导
	double v_psi_int = ddvarphie + bikebot.a1*(dvarphie-bikebot.dvarphi)+ bikebot.a0*(varphie-bikebot.varphi);
	//将控制输入代入动力学模型方程，计算得到控制量u_psi
	double cosvar = cos(bikebot.varphi);
	double sinvar = sin(bikebot.varphi);
	double xxx = bikebot.mb*bikebot.g*bikebot.lt*bikebot.lb*cos(bikebot.epsilon);
	double f_varphi = (bikebot.mb*bikebot.hb*vr + bikebot.mb*bikebot.hb*bikebot.hb*sinvar*dpsi)*cosvar*dpsi
				+bikebot.mb*bikebot.hb*bikebot.g*sinvar + xxx*dpsi*cosvar/vr;
	double g_psi = bikebot.mb*bikebot.hb*bikebot.lb*cosvar;
	double u_psi_int = (-f_varphi + bikebot.Jt*v_psi_int)/g_psi;
	tau_S = g_psi*u_psi_int;
	tau_Smax = signd(tau_S)*bikebot.mb*bikebot.hb*cos(bikebot.epsilon)*bikebot.vr*bikebot.vr*tan(LIMIT_STEER)/bikebot.l;
	e_tau = tau_S-tau_Smax;
	//自此，以外部得到的u_r和内部得到的u_psi得到最终控制输入

	/*计算转向角*/
	bikebot.new_phi = atan((dpsi+u_psi_int*dt)*bikebot.l/cos(bikebot.epsilon)*cosvar/vr)*bikebot.k;
	double target = bikebot.new_phi;
	if(target > LIMIT_STEER) target = LIMIT_STEER;
	else if(target < -LIMIT_STEER) target = -LIMIT_STEER;
	bikebot.target = target;
	phi_cmd = bikebot.target;
	
	//控制限制
	double limit = 0.02;
	double last = bikebot.last_target;
	target = bikebot.target;
	if(target > last) phi_cmd = (target - last) < limit ? target : last + limit;
	else phi_cmd = (last - target) < limit ? target : last - limit;
	bikebot.last_target = phi_cmd;
}
// external control
void bicycle_controller::balanceCalc2(int traj){
	/*变量读取*/
	double vr = bikebot.vr;		double dvr = 0;
	double psi = bikebot.psi;
	double dpsi = bikebot.dpsi;
	double dpsi3 = dpsi*dpsi*dpsi;//dpsi的三次方
	double sinpsi = sin(psi);		double cospsi = cos(psi);
	double x = bikebot.x;	double dx = vr * cospsi;	double ddx = -vr * sinpsi * dpsi;
	double y = bikebot.y;	double dy = vr * sinpsi;	double ddy = vr * cospsi * dpsi;
	double ve = vr;
	/*trajectory*/
	// // 走圆	
	if(traj==0){
		double cr = 2; double w = ve/4;
		double x_offset = 0; double y_offset = cr;
		new_Wt += w*dt; 
		xe = x_offset + 6 * cos(new_Wt); 	ye =  y_offset + cr*sin(new_Wt);
		dxe = -6 * sin(new_Wt) * w;		dye = cr * cos(new_Wt) * w;	
		// ddxe = -dye * w; 		 ddye = dxe * w;
		// dddxe = -ddye * w; 		 dddye = ddxe * w;
		// ddddxe = -dddye * w; 	 ddddye = dddxe * w;
		// dddddxe = -ddddye * w; 	 dddddye = ddddxe * w;
		// Wt -= 0.1*dt;
		// xe = 3*cos(Wt)-3;
		// ye = 8*sin(Wt);
		// dxe = -3*sin(Wt);
		// dye = 8*cos(Wt);

	}
	//走直线
	else{
		dxe = ve; dye = 0; 
		xe += ve*dt; ye = 0;
	}
	

	bikebot.xe = xe;
	bikebot.ye = ye;
	// std::cout<<"v: "<<bike->get_base_v()<<std::endl;
	// std::cout<<"x: "<<bikebot.x<<"xe: "<<bikebot.xe<<std::endl;
	// std::cout<<"y: "<<bikebot.y<<"ye: "<<bikebot.ye<<std::endl;
	/*External*/
	//利用控制律计算出前轮轨迹r(3)及其一次导和二次导
	double u_w_ext1 = dddxe + bikebot.b2 * (ddxe - ddx) + bikebot.b1 * (dxe - dx) + bikebot.b0 * (xe - x);
	double u_w_ext2 = dddye + bikebot.b2 * (ddye - ddy) + bikebot.b1 * (dye - dy) + bikebot.b0 * (ye - y);
	double du_w_ext1 = ddddxe + bikebot.b2 * (dddxe - u_w_ext1) + bikebot.b1 * (ddxe - ddx) + bikebot.b0 * (dxe - dx);
	double du_w_ext2 = ddddye + bikebot.b2 * (dddye - u_w_ext2) + bikebot.b1 * (ddye - ddy) + bikebot.b0 * (dye - dy);
	double ddu_w_ext1 = dddddxe + bikebot.b2 * (ddddxe - du_w_ext1) + bikebot.b1 * (dddxe - u_w_ext1) + bikebot.b0 * (ddxe - ddx);
	double ddu_w_ext2 = dddddye + bikebot.b2 * (ddddye - du_w_ext2) + bikebot.b1 * (dddye - u_w_ext2) + bikebot.b0 * (ddye - ddy);
	// double ur = vr*dpsi*dpsi+cospsi*u_w_ext1+sinpsi*u_w_ext2;
	//这里速度到二次导我们设为0
	double ur = 0;
	//计算出满足external条件的u_psi及其一阶和二阶导
	double u_psi = -2*dvr*dpsi/vr-sinpsi/vr*u_w_ext1+cospsi/vr*u_w_ext2;
	double du_psi = -2*((ur*vr-dvr*dvr)/vr/vr*dpsi+dvr/vr*u_psi) - (cospsi*dpsi*vr-sinpsi*dvr)/vr/vr*u_w_ext1 - sinpsi/vr*du_w_ext1
            + (-sinpsi*dpsi*vr-cospsi*dvr)/vr/vr*u_w_ext2 + cospsi/vr*du_w_ext2;
    double ddu_psi = (sinpsi*dpsi*dpsi-cospsi*u_psi)/vr*u_w_ext1 - 2*cospsi*dpsi/vr*du_w_ext1 - sinpsi/vr*ddu_w_ext1
            - (cospsi*dpsi*dpsi+sinpsi*u_psi)/vr*u_w_ext2 - 2*sinpsi*dpsi/vr*du_w_ext2 + cospsi/vr*ddu_w_ext2;
	
	/*Internal*/
	//将满足external条件的u_psi代入求解器，求解期望倾斜角
	double varphie = solution(vr, dpsi, u_psi);//牛顿迭代法求方程根
	bikebot.varphie = varphie;
	//求解期望倾斜角的一阶和二阶导
	double cosvare = cos(varphie);
	double sinvare = sin(varphie);
	double M1 = bikebot.hb*cosvare*dpsi*dpsi+bikebot.g/cosvare/cosvare;
	double hhh = bikebot.g*bikebot.lt*bikebot.lb*cos(bikebot.epsilon)/bikebot.hb;
    double M2 = (u_psi*vr+dpsi*dvr+2*bikebot.hb*dpsi*u_psi*sinvare+hhh*(u_psi*vr-dpsi*dvr)/vr/vr+bikebot.lb*du_psi);	
	double dvarphie = -M2/M1;	
	bikebot.dvarphie = dvarphie;
	double dM1 = (2*bikebot.hb*dpsi*u_psi*cosvare-bikebot.hb*dpsi*dpsi*sinvare+2*bikebot.g/cosvare/cosvare*sinvare/cosvare)*dvarphie;
	double dM2 = du_psi*vr+2*u_psi*dvr+dpsi*ur+2*bikebot.hb*(u_psi*u_psi*sinvare+dpsi*du_psi*sinvare+dpsi*u_psi*cosvare*dvarphie)
            + hhh * (du_psi*vr-2*u_psi*dvr-dpsi*ur+2*dpsi*dvr*dvr/vr)/vr/vr + bikebot.lb*ddu_psi;    
	double ddvarphie = (dM1*M2/M1 - dM2)/M1;
	//根据控制律求解合适到控制输入:varphi的二阶导
	double v_psi_int = ddvarphie + bikebot.a1*(dvarphie-bikebot.dvarphi)+ bikebot.a0*(varphie-bikebot.varphi);
	//将控制输入代入动力学模型方程，计算得到控制量u_psi
	double cosvar = cos(bikebot.varphi);
	double sinvar = sin(bikebot.varphi);
	double xxx = bikebot.mb*bikebot.g*bikebot.lt*bikebot.lb*cos(bikebot.epsilon);
	double f_varphi = (bikebot.mb*bikebot.hb*vr + bikebot.mb*bikebot.hb*bikebot.hb*sinvar*dpsi)*cosvar*dpsi
				+bikebot.mb*bikebot.hb*bikebot.g*sinvar + xxx*dpsi*cosvar/vr;
	double g_psi = bikebot.mb*bikebot.hb*bikebot.lb*cosvar;
	double u_psi_int = (-f_varphi + bikebot.Jt*v_psi_int)/g_psi;
	tau_S = g_psi*u_psi;
	h_varphi = (bikebot.mb*bikebot.hb*vr + bikebot.mb*bikebot.hb*bikebot.hb*sinvar*dpsi)*cosvar*dpsi + xxx*dpsi*cosvar/vr + tau_S;
	tau_Smax = signd(tau_S)*bikebot.mb*bikebot.hb*cos(bikebot.epsilon)*bikebot.vr*bikebot.vr*tan(LIMIT_STEER)/bikebot.l;
	bike_varphie = varphie;
	e_tau = tau_S-tau_Smax;
	//自此，以外部得到的u_r和内部得到的u_psi得到最终控制输入

	/*计算转向角*/
	bikebot.new_phi = atan((dpsi+u_psi*dt)*bikebot.l/cos(bikebot.epsilon)*cosvar/vr)*bikebot.k;
	double target = bikebot.new_phi;
	if(target > LIMIT_STEER) target = LIMIT_STEER;
	else if(target < -LIMIT_STEER) target = -LIMIT_STEER;
	bikebot.target = target;
	phi_cmd = bikebot.target;
	
	//控制限制
	double limit = 0.01;
	double last = bikebot.last_target;
	target = bikebot.target;
	if(target > last) phi_cmd = (target - last) < limit ? target : last + limit;
	else phi_cmd = (last - target) < limit ? target : last - limit;
	bikebot.last_target = phi_cmd;
}
// EIC control
void bicycle_controller::balanceCalc3(int traj){
	/*变量读取*/
	double vr = bikebot.vr;		double dvr = 0;
	double psi = bikebot.psi;
	double dpsi = bikebot.dpsi;
	double dpsi3 = dpsi*dpsi*dpsi;//dpsi的三次方
	double sinpsi = sin(psi);		double cospsi = cos(psi);
	double x = bikebot.x;	double dx = vr * cospsi;	double ddx = -vr * sinpsi * dpsi;
	double y = bikebot.y;	double dy = vr * sinpsi;	double ddy = vr * cospsi * dpsi;
	double ve = vr;
	/*trajectory*/
	// // 走圆	
	if(traj==0){
		double cr = 1.5; double w = ve/cr;
		double x_offset = 0; double y_offset = cr;
		new_Wt += w*dt; 
		xe = x_offset + cr * cos(new_Wt); 	ye =  y_offset + cr*sin(new_Wt);
		dxe = -cr * sin(new_Wt) * w;		dye = cr * cos(new_Wt) * w;	
		ddxe = -dye * w; 		 ddye = dxe * w;
		dddxe = -ddye * w; 		 dddye = ddxe * w;
		ddddxe = -dddye * w; 	 ddddye = dddxe * w;
		dddddxe = -ddddye * w; 	 dddddye = ddddxe * w;
	}
	//走直线
	else{
		dxe = ve; dye = 0; 
		xe += ve*dt; ye = 0;
	}
	

	bikebot.xe = xe;
	bikebot.ye = ye;
	// std::cout<<"v: "<<bike->get_base_v()<<std::endl;
	// std::cout<<"x: "<<bikebot.x<<"xe: "<<bikebot.xe<<std::endl;
	// std::cout<<"y: "<<bikebot.y<<"ye: "<<bikebot.ye<<std::endl;
	/*External*/
	//利用控制律计算出前轮轨迹r(3)及其一次导和二次导
	double u_w_ext1 = dddxe + bikebot.b2 * (ddxe - ddx) + bikebot.b1 * (dxe - dx) + bikebot.b0 * (xe - x);
	double u_w_ext2 = dddye + bikebot.b2 * (ddye - ddy) + bikebot.b1 * (dye - dy) + bikebot.b0 * (ye - y);
	double du_w_ext1 = ddddxe + bikebot.b2 * (dddxe - u_w_ext1) + bikebot.b1 * (ddxe - ddx) + bikebot.b0 * (dxe - dx);
	double du_w_ext2 = ddddye + bikebot.b2 * (dddye - u_w_ext2) + bikebot.b1 * (ddye - ddy) + bikebot.b0 * (dye - dy);
	double ddu_w_ext1 = dddddxe + bikebot.b2 * (ddddxe - du_w_ext1) + bikebot.b1 * (dddxe - u_w_ext1) + bikebot.b0 * (ddxe - ddx);
	double ddu_w_ext2 = dddddye + bikebot.b2 * (ddddye - du_w_ext2) + bikebot.b1 * (dddye - u_w_ext2) + bikebot.b0 * (ddye - ddy);
	// double ur = vr*dpsi*dpsi+cospsi*u_w_ext1+sinpsi*u_w_ext2;
	//这里速度到二次导我们设为0
	double ur = 0;
	//计算出满足external条件的u_psi及其一阶和二阶导
	double u_psi = -2*dvr*dpsi/vr-sinpsi/vr*u_w_ext1+cospsi/vr*u_w_ext2;
	double du_psi = -2*((ur*vr-dvr*dvr)/vr/vr*dpsi+dvr/vr*u_psi) - (cospsi*dpsi*vr-sinpsi*dvr)/vr/vr*u_w_ext1 - sinpsi/vr*du_w_ext1
            + (-sinpsi*dpsi*vr-cospsi*dvr)/vr/vr*u_w_ext2 + cospsi/vr*du_w_ext2;
    double ddu_psi = (sinpsi*dpsi*dpsi-cospsi*u_psi)/vr*u_w_ext1 - 2*cospsi*dpsi/vr*du_w_ext1 - sinpsi/vr*ddu_w_ext1
            - (cospsi*dpsi*dpsi+sinpsi*u_psi)/vr*u_w_ext2 - 2*sinpsi*dpsi/vr*du_w_ext2 + cospsi/vr*ddu_w_ext2;
	
	/*Internal*/
	//将满足external条件的u_psi代入求解器，求解期望倾斜角
	double varphie = solution(vr, dpsi, u_psi);//牛顿迭代法求方程根
	bikebot.varphie = varphie;
	//求解期望倾斜角的一阶和二阶导
	double cosvare = cos(varphie);
	double sinvare = sin(varphie);
	double M1 = bikebot.hb*cosvare*dpsi*dpsi+bikebot.g/cosvare/cosvare;
	double hhh = bikebot.g*bikebot.lt*bikebot.lb*cos(bikebot.epsilon)/bikebot.hb;
    double M2 = (u_psi*vr+dpsi*dvr+2*bikebot.hb*dpsi*u_psi*sinvare+hhh*(u_psi*vr-dpsi*dvr)/vr/vr+bikebot.lb*du_psi);	
	double dvarphie = -M2/M1;	
	bikebot.dvarphie = dvarphie;
	double dM1 = (2*bikebot.hb*dpsi*u_psi*cosvare-bikebot.hb*dpsi*dpsi*sinvare+2*bikebot.g/cosvare/cosvare*sinvare/cosvare)*dvarphie;
	double dM2 = du_psi*vr+2*u_psi*dvr+dpsi*ur+2*bikebot.hb*(u_psi*u_psi*sinvare+dpsi*du_psi*sinvare+dpsi*u_psi*cosvare*dvarphie)
            + hhh * (du_psi*vr-2*u_psi*dvr-dpsi*ur+2*dpsi*dvr*dvr/vr)/vr/vr + bikebot.lb*ddu_psi;    
	double ddvarphie = (dM1*M2/M1 - dM2)/M1;
	//根据控制律求解合适到控制输入:varphi的二阶导
	double v_psi_int = ddvarphie + bikebot.a1*(dvarphie-bikebot.dvarphi)+ bikebot.a0*(varphie-bikebot.varphi);
	//将控制输入代入动力学模型方程，计算得到控制量u_psi
	double cosvar = cos(bikebot.varphi);
	double sinvar = sin(bikebot.varphi);
	double xxx = bikebot.mb*bikebot.g*bikebot.lt*bikebot.lb*cos(bikebot.epsilon);
	double f_varphi = (bikebot.mb*bikebot.hb*vr + bikebot.mb*bikebot.hb*bikebot.hb*sinvar*dpsi)*cosvar*dpsi
				+bikebot.mb*bikebot.hb*bikebot.g*sinvar + xxx*dpsi*cosvar/vr;
	double g_psi = bikebot.mb*bikebot.hb*bikebot.lb*cosvar;
	double u_psi_int = (-f_varphi + bikebot.Jt*v_psi_int)/g_psi;
	tau_S = g_psi*u_psi;
	tau_Smax = signd(tau_S)*bikebot.mb*bikebot.hb*cos(bikebot.epsilon)*bikebot.vr*bikebot.vr*tan(LIMIT_STEER)/bikebot.l;
	h_varphi = (bikebot.mb*bikebot.hb*vr + bikebot.mb*bikebot.hb*bikebot.hb*sinvar*dpsi)*cosvar*dpsi + xxx*dpsi*cosvar/vr + tau_Smax;
	
	bike_varphie = varphie;
	e_tau = tau_S-tau_Smax;
	//自此，以外部得到的u_r和内部得到的u_psi得到最终控制输入

	/*计算转向角*/
	bikebot.new_phi = atan((dpsi+u_psi_int*dt)*bikebot.l/cos(bikebot.epsilon)*cosvar/vr)*bikebot.k;
	double target = bikebot.new_phi;
	if(target > LIMIT_STEER) target = LIMIT_STEER;
	else if(target < -LIMIT_STEER) target = -LIMIT_STEER;
	bikebot.target = target;
	phi_cmd = bikebot.target;
	
	//控制限制
	double limit = 0.1;
	double last = bikebot.last_target;
	target = bikebot.target;
	if(target > last) phi_cmd = (target - last) < limit ? target : last + limit;
	else phi_cmd = (last - target) < limit ? target : last - limit;
	bikebot.last_target = phi_cmd;
}
double bicycle_controller::get_varphie(){
	return bike_varphie;
}
double bicycle_controller::get_h_varphi(){
	return h_varphi;
}
double bicycle_controller::get_tau(){
	return e_tau;
}
double solution(double vr, double dpsi, double u_psi){
	int cnt = 0;
	double init = 0;
	double res = 0;
	double fun, dfun;
	double tmp;
	double second = (vr + body_g*body_lt*body_lb*cos(body_epsilon)/body_h/vr)*dpsi + body_lb * u_psi;
	do{
		init = res;
		fun = body_h*dpsi*dpsi*sin(init)+body_g*tan(init)+ second;
		tmp = cos(init);
		dfun = body_h*dpsi*dpsi*tmp+body_g/tmp/tmp;
		res = init - fun/dfun;
		if(++cnt > 10){//至多迭代10次
			break;
		}
	}while(fabs(res-init)>1e-3);
	return res;
}

double rad2deg(double rad){
	return rad/PI*180.0;
}

double signd(double x){
	double y;
	if(x<0) y = -1;
	else if(x>0) y = 1;
	else y = 0;
	return y;
}