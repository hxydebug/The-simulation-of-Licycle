#ifndef __LEG_CONTROL_H
#define __LEG_CONTROL_H


#include "model.h"
#include "control.h"
#include "swing_leg_controller.h"
#include "stance_leg_controller.h"

#define period 300

// #define initial 0
// #define impulse 1
// #define cosgait 2
// #define pgait 3

// #define Run_mode 0

#define force_period 60

#define dL 0.2f

#define INITIAL 0
#define SWING_DOWN 1
#define SWING_UP 2
#define STANCE 3
#define TWO_LEGS 4
#define SWING 5

typedef int Leg_StateName;


class swing_control{

public:
	swing_control();
	void get_cmd(Angle *angle,Leg leg);
	void check_Transition(void);
	void get_start(Eigen::VectorXd angle, Position posi, Leg leg);
private:
	int count;
};

class swingdown_control{

public:
	swingdown_control();
	void get_cmd(Angle *angle,Leg leg);
	void check_Transition(void);
	void get_start(Eigen::VectorXd angle, Position posi, Leg leg);
private:
	int count;
};

class swingup_control{

public:
	swingup_control();
	void get_cmd(Angle *angle,Leg leg);
	void check_Transition(void);
	void get_start(Eigen::VectorXd angle, Position posi, Leg leg);
private:
	int count;
};

class stance_control{

public:
	stance_control();
	Eigen::Vector3d get_cmd(Leg leg,double roll,Eigen::VectorXd angles);
	void check_Transition(void);
	void get_start(Leg leg);
private:
	int count;
	Eigen::VectorXd F;
};

class leg_controller{

public:
	leg_controller(robot *bicycle,gait_generator *gait_gen,swing_leg_controller *swc, stance_leg_controller *stc);
	void set_PDGain();
	Eigen::VectorXd tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT);
	Eigen::VectorXd get_action(int Run_mode);
	void create_gait(void);
	void goto_xyz(float xx,float yy,float zz,Leg leg);
private:
	robot *bike;
	Eigen::VectorXd pGain,dGain;
	double T = 0.2;
	float dt = 0.001;
	float Hf = 0.1;
	float Hb = 0.305;
	float swing_time = 0.1;

	float tf = 0;
	float p0 = 0;
	float pf = 0;
	float vf = 0;
	float v0 = 0;

	Angle angle[2];
	Angle angleV[2];
	Position position[2];
	Position velocity[2];

	int force_flag = 0;

	Eigen::VectorXd posT,angT;
	Eigen::VectorXd Tau_e,Tau_l,Tau_r;

	std::vector<float> x_position;
	std::vector<float> z_position;

	swing_control swing_contr;
	swingdown_control swdown_contr;
	swingup_control swup_contr;
	stance_control stance_contr;

	gait_generator *gait_generate;
 	swing_leg_controller *swctr;
	stance_leg_controller *stctr;

};


void init_chabu(Position *pdes,Position *vdes,Position *pini,Position *vini,float step,Leg leg);
void chabu(Position *pos,float step,Leg leg);
float linear(float ini,float des,float tf,float step);

void set_xyz(Leg leg,Angle *angle,float xx,float yy,float zz);
//control
Position pos_control(Leg leg, double roll);
void force_control();


#endif