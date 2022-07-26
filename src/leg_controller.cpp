#include "leg_controller.h"
#include "main.h"
#include <algorithm>


float a0[2][3] = {0};
float a1[2][3] = {0};
float a2[2][3] = {0};
float a3[2][3] = {0};

double rollMax = 2*PI/180;

float timer = 0;

Leg_StateName last_state[2];
Leg_StateName current_state[2];
Leg_StateName pre_state[2];

Position init_pos[2];
Angle init_angle[2];

leg_controller::leg_controller(robot *bicycle,gait_generator *gait_gen,swing_leg_controller *swc,stance_leg_controller *stc){
  
	bike = bicycle;
	leg_controller::set_PDGain();
	leg_controller::create_gait();
  posT.resize(6);
  angT.resize(6);
  Tau_e.resize(6);
  // set_xyz(fl,&init_angle[0],0.16,0.08,-0.12);
  // set_xyz(fr,&init_angle[1],0.16,-0.08,-0.12);
  set_xyz(fl,&init_angle[0],0.04,0.13,-0.32);
  set_xyz(fr,&init_angle[1],0.04,-0.13,-0.32);
  posT << init_angle[0].q[0],init_angle[0].q[1],init_angle[0].q[2],
          init_angle[1].q[0],init_angle[1].q[1],init_angle[1].q[2];
  angT << 0,0,0,0,0,0;

  Kinematics(&init_angle[0],&init_pos[0],0);
  Kinematics(&init_angle[1],&init_pos[1],1);

  std::cout<<init_pos[0].x<<init_pos[0].y<<init_pos[0].z<<std::endl;
  std::cout<<init_pos[1].x<<init_pos[1].y<<init_pos[0].z<<std::endl;

  current_state[fl] = INITIAL;
  current_state[fr] = INITIAL;


  gait_generate = gait_gen;
  swctr = swc;
  stctr = stc;

}
void leg_controller::set_PDGain(){
	pGain.resize(6);
	dGain.resize(6);
	pGain.setConstant(20.0);
	dGain.setConstant(0);

}
Eigen::VectorXd leg_controller::get_action(int Run_mode){
		
  if(Run_mode==0){ 
    // posT<< 0.5,0.3,-0.3,-0.5,-0.3,-0.3;
    // angT<< 0,0,0,0,0,0;
    // set_xyz(fl,&init_angle[0],0.16,0.08,-0.12);
    // set_xyz(fr,&init_angle[1],0.16,-0.08,-0.12);
    // set_xyz(fl,&init_angle[0],0,0.08,-0.32);
    // set_xyz(fr,&init_angle[1],0,-0.08,-0.32);
    posT << init_angle[0].q[0],init_angle[0].q[1],init_angle[0].q[2],
            init_angle[1].q[0],init_angle[1].q[1],init_angle[1].q[2];
    pGain.setConstant(50.0);
	  dGain.setConstant(0);

    Tau_e = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,angT);
  }
  else if(Run_mode==1){
      static int stance_flag[2] = {1,1};
      static int swdown_flag[2] = {1,1};
      //update state
      Eigen::VectorXd angles = bike->get_leg_pos();
      double roll = bike->get_varphi();
      //位控
      Leg leg;
      if(roll <= -rollMax){
        leg = fl;
        Position pos = pos_control(leg,-rollMax);
        // leg_controller::goto_xyz(pos.x,pos.y,pos.z,leg);
        // last_state[leg] = current_state[leg];
        if(swdown_flag[leg]==1){
          pre_state[leg] = SWING_DOWN;
          swdown_contr.get_start(angles,pos,leg);
          swdown_flag[leg] = 0;
        }
        
      } 
      else if(roll >= rollMax){
        leg = fr;
        Position pos = pos_control(leg,rollMax);
        // leg_controller::goto_xyz(pos.x,pos.y,pos.z,leg);
        // last_state[leg] = current_state[leg];
        if(swdown_flag[leg]==1){
          pre_state[leg] = SWING_DOWN;
          swdown_contr.get_start(angles,pos,leg);
          swdown_flag[leg] = 0;
        }
        
      }
      else{
        stance_flag[0] = 1;
        swdown_flag[0] = 1;
        stance_flag[1] = 1;
        swdown_flag[1] = 1;
      }
      //Force control
      auto l_foot = bike->bicycle->getBodyIdx("left_shank_Link");
      auto r_foot = bike->bicycle->getBodyIdx("right_shank_Link");
      for(auto& contact: bike->bicycle->getContacts()) {
        if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
        if ( l_foot == contact.getlocalBodyIndex() ) {
          // last_state[leg] = current_state[leg];
          if(stance_flag[0]==1){
            pre_state[0] = STANCE;
            stance_contr.get_start(0);
            stance_flag[0] = 0;
          }
          break;
        }
        if ( r_foot == contact.getlocalBodyIndex() ) {
          // last_state[leg] = current_state[leg];
          if(stance_flag[1]==1){
            pre_state[1] = STANCE;
            stance_contr.get_start(1);
            stance_flag[1] = 0;
          };
          break;
        }
      }
      for(int i(0);i<2;i++){
        if(pre_state[i]==SWING_UP){
          swup_contr.get_start(angles,init_pos[i],i);
        }
      }
      
      //判断state
      Angle pos_angle;
      switch(current_state[0]){
        case INITIAL:
          std::cout<<"left  INITIAL"<<std::endl;
          // init_contr.get_cmd(&pos_angle,0);
          posT[0] = init_angle[0].q[0];
          posT[1] = init_angle[0].q[1];
          posT[2] = init_angle[0].q[2];
          Tau_l = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,angT).head(3);
          
          break;
        case SWING_DOWN:
          std::cout<<"left  SWING_DOWN"<<std::endl;
          swdown_contr.get_cmd(&pos_angle,0);
          posT[0] = pos_angle.q[0];
          posT[1] = pos_angle.q[1];
          posT[2] = pos_angle.q[2];
          Tau_l = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,angT).head(3);
          
          break;
        case SWING_UP:
          std::cout<<"left  SWING_UP"<<std::endl;
          swup_contr.get_cmd(&pos_angle,0);
          posT[0] = pos_angle.q[0];
          posT[1] = pos_angle.q[1];
          posT[2] = pos_angle.q[2];
          Tau_l = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,angT).head(3);
          
          break;
        case STANCE:
          std::cout<<"left  STANCE"<<std::endl;
          Tau_l = stance_contr.get_cmd(0,roll,angles);
          
          break;
      }
      switch(current_state[1]){
        case INITIAL:
          // init_contr.get_cmd(&pos_angle,1);
          std::cout<<"right  INITIAL"<<std::endl;
          posT[3] = init_angle[1].q[0];
          posT[4] = init_angle[1].q[1];
          posT[5] = init_angle[1].q[2];
          Tau_r = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,angT).tail(3);
          
          break;
        case SWING_DOWN:
          std::cout<<"right  SWING_DOWN"<<std::endl;
          swdown_contr.get_cmd(&pos_angle,1);
          posT[3] = pos_angle.q[0];
          posT[4] = pos_angle.q[1];
          posT[5] = pos_angle.q[2];
          Tau_r = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,angT).tail(3);
          
          break;
        case SWING_UP:
          std::cout<<"right  SWING_UP"<<std::endl;
          swup_contr.get_cmd(&pos_angle,1);
          posT[3] = pos_angle.q[0];
          posT[4] = pos_angle.q[1];
          posT[5] = pos_angle.q[2];
          Tau_r = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,angT).tail(3);
          
          break;
        case STANCE:
          std::cout<<"right  STANCE"<<std::endl;
          Tau_r = stance_contr.get_cmd(1,roll,angles);
          
          break;
      }
      // for(int i(0);i<3;i++){
      //   Tau_e[i] = Tau_l[i];
      //   Tau_e[i+3] = Tau_r[i];
      // }
      Tau_e << Tau_l,Tau_r;
      


  }
	else if(Run_mode==2){
    float wid = 0.25;
    static int count = 0;
    static int count1 = T/dt/2.0;    
    int j = 0;
    position[j].x = detx+x_position[count];
    position[j].y = wid;
    position[j].z = detz+z_position[count];
    Inv_kinematics(&angle[j],&position[j],j);
    j = 1;
    position[j].x = detx+x_position[count1];
    position[j].y = -wid;
    position[j].z = detz+z_position[count1];
    Inv_kinematics(&angle[j],&position[j],j);

    posT<< angle[0].q[0],angle[0].q[1],angle[0].q[2],angle[1].q[0],angle[1].q[1],angle[1].q[2];
    angT<< 0,0,0,0,0,0;

    Tau_e = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,angT);

    count ++;
    count1 ++;

    if(count>=x_position.size()){
      count = 0;
    }
    if(count1>=x_position.size()){
      count1 = 0;
    }
  }
  else if(Run_mode==5){

    set_xyz(fl,&init_angle[0],0.16,0.08,-0.12);
    set_xyz(fr,&init_angle[1],0.16,-0.08,-0.12);
    posT << init_angle[0].q[0],init_angle[0].q[1],init_angle[0].q[2],
            init_angle[1].q[0],init_angle[1].q[1],init_angle[1].q[2];
    pGain.setConstant(10.0);
	  dGain.setConstant(0.1);

    Tau_e = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,angT);
  }
  else if(Run_mode==6 || Run_mode==7 ){
    gait_generate->update(timer);
    swctr->update(timer);
    // Eigen::VectorXd stc_tau(6);
    // stc_tau.setConstant(0);
    // Eigen::VectorXd swc_tau(6);
    // swc_tau.setConstant(0);

    Eigen::VectorXd stc_tau = stctr->get_act();
    Eigen::VectorXd swc_tau = swctr->get_action();

    Eigen::VectorXd anglesV(6);
    anglesV.setConstant(0);
    float error = 10+fabs(80*bike->get_varphi());
    std::cout<<"error:"<<error<<std::endl;

    if(Run_mode==6){
      pGain.setConstant(error);
    }
    if(Run_mode==7){
      pGain.setConstant(0);
    }

    auto Tau_t = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,anglesV);

    Eigen::VectorXd ltau(3),rtau(3);
    if(gait_generate->leg_state[0]==stance_leg || gait_generate->leg_state[0]==Early_Contact){
      ltau = stc_tau.head(3)+Tau_t.head(3);
    }
    else{
      ltau = swc_tau.head(3);
    }
    if(gait_generate->leg_state[1]==stance_leg || gait_generate->leg_state[1]==Early_Contact){
      rtau = stc_tau.tail(3)+Tau_t.tail(3);
    }
    else{
      rtau = swc_tau.tail(3);
    }

    Tau_e << ltau,rtau;
    // Tau_e << 0,0,0,0,0,0;

    timer += 0.001;
  }
  else{
    
    gait_generate->update(timer);
    swctr->update(timer);
    // Eigen::VectorXd stc_tau(6);
    // stc_tau.setConstant(0);
    // Eigen::VectorXd swc_tau(6);
    // swc_tau.setConstant(0);

    Eigen::VectorXd stc_tau = stctr->get_action();
    Eigen::VectorXd swc_tau = swctr->get_action();

    Eigen::VectorXd anglesV(6);
    anglesV.setConstant(0);
    float error = fabs(80*bike->get_varphi());
    // std::cout<<"error:"<<error<<std::endl;

    if(Run_mode==8){
      pGain.setConstant(error);
    }
    if(Run_mode==3){
      pGain.setConstant(30+error);
    }
    if(Run_mode==4){ //without pd
      pGain.setConstant(0);
    }

    auto Tau_t = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,anglesV);

    Eigen::VectorXd ltau(3),rtau(3);
    if(gait_generate->leg_state[0]==stance_leg || gait_generate->leg_state[0]==Early_Contact){
      ltau = stc_tau.head(3)+Tau_t.head(3);
    }
    else{
      ltau = swc_tau.head(3);
    }
    if(gait_generate->leg_state[1]==stance_leg || gait_generate->leg_state[1]==Early_Contact){
      rtau = stc_tau.tail(3)+Tau_t.tail(3);
    }
    else{
      rtau = swc_tau.tail(3);
    }

    Tau_e << ltau,rtau;
    // Tau_e << 0,0,0,0,0,0;

    timer += 0.001;
    // Tau_e = leg_controller::tau(bike->get_leg_pos(),bike->get_leg_vel(),posT,angT);
    // std::cout<<"timer:"<<timer<<std::endl;

  }
  // std::cout<<Tau_e<<std::endl;
  for(int i(0);i<6;i++){
		if(Tau_e[i] < -18.0) Tau_e[i] = -18.0;
		if(Tau_e[i] > 18.0) Tau_e[i] = 18.0;
	}
  // std::cout<<Tau_e<<std::endl;
  return Tau_e;

}
Eigen::VectorXd leg_controller::tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT){
  
  return dGain.cwiseProduct(vT-vA) + pGain.cwiseProduct(pT-pA);

}

void leg_controller::create_gait(void){
  float t;
  float x_sw,z_sw,x_st,z_st;

  //赋值
  tf = T*swing_time;
  p0 = -0.1;
  pf = 0.1;
  vf = -1;
  v0 = -1;
  //计算三次样条
  float a0,a1,a2,a3;
  a0 = p0;
  a1 = v0;
  a2 = 3.0/(tf*tf)*(pf-p0)-2.0/tf*v0-1.0/tf*vf;
  a3 = -2.0/(tf*tf*tf)*(pf-p0)+1.0/(tf*tf)*(vf+v0);

  for(int i(1);i<tf/dt+1;i++){
    t = i*dt;
    x_sw = a0+a1*t+a2*t*t+a3*t*t*t;
    z_sw = -Hb + Hf*0.5*(1+cos(2*PI*t/(tf)+PI));
    x_position.push_back(x_sw);
    z_position.push_back(z_sw);
  }

  //赋值
  tf = T*(1-swing_time);
  p0 = 0.15;
  pf = -0.05;
  vf = -1;
  v0 = -1;
  //计算三次样条
  a0 = p0;
  a1 = v0;
  a2 = 3.0/(tf*tf)*(pf-p0)-2.0/tf*v0-1.0/tf*vf;
  a3 = -2.0/(tf*tf*tf)*(pf-p0)+1.0/(tf*tf)*(vf+v0);

  for(int i(1);i<tf/dt+1;i++){
    t = i*dt;
    x_sw = a0+a1*t+a2*t*t+a3*t*t*t;
    z_sw = -Hb;
    x_position.push_back(x_sw);
    z_position.push_back(z_sw);
  }

}

void leg_controller::goto_xyz(float xx,float yy,float zz,Leg leg){

  //define the variables
  Angle now_angle;
  Angle now_angleV;
  Position now_position;
  Position now_velocity;

  //get the joint angle position and velocity
  auto pos = bike->get_leg_pos();
  if(leg == fl){
    for(int i(0);i<3;i++){
        now_angle.q[i] = pos[i];
    }
  }
  else{
    for(int i(0);i<3;i++){
        now_angle.q[i] = pos[i+3];
    }
  }

  auto vel = bike->get_leg_vel();
  if(leg == fl){
    for(int i(0);i<3;i++){
        now_angleV.q[i] = vel[i];
    }
  }
  else{
    for(int i(0);i<3;i++){
        now_angleV.q[i] = vel[i+3];
    }
  }

  //get the end point position and velocity using forward kinematics and jacobian
  Kinematics(&now_angle,&now_position,leg);

  now_velocity.x = 0;
  now_velocity.y = 0;
  now_velocity.z = 0;

  //input the desire position and velocity
	Position pdes;
  Position vdes;

  pdes.x = xx;
  pdes.y = yy;
  pdes.z = zz;

  vdes.x = 0;
  vdes.y = 0;
  vdes.z = 0;

  //init chabu
  init_chabu(&pdes,&vdes,&now_position,&now_velocity,period,leg);

  
}

swing_control::swing_control(){
  this->count = 0;
}
void swing_control::get_cmd(Angle *angle,Leg leg){
  this->count ++;
  if(this->count > period) this->count = period;
  std::cout<<this->count<<std::endl;
  Position pos;
  chabu(&pos,this->count,leg);
  Inv_kinematics_ref(angle,&pos,leg);
  
}
void swing_control::check_Transition(void){

}
void swing_control::get_start(Eigen::VectorXd angle, Position posi, Leg leg){
  if(current_state[leg] == INITIAL){
    //define the variables
    Angle now_angle;
    Position now_position;
    Position now_velocity;

    //get the joint angle position and velocity
    auto pos = angle;
    if(leg == fl){
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i];
      }
    }
    else{
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i+3];
      }
    }

    //get the end point position and velocity using forward kinematics and jacobian
    Kinematics_ref(&now_angle,&now_position,leg);

    now_velocity.x = 0;
    now_velocity.y = 0;
    now_velocity.z = 0;

    //input the desire position and velocity
    Position pdes;
    Position vdes;

    pdes.x = posi.x;
    pdes.y = posi.y;
    pdes.z = posi.z;

    vdes.x = 0;
    vdes.y = 0;
    vdes.z = 0;

    //init chabu
    init_chabu(&pdes,&vdes,&now_position,&now_velocity,period,leg);
    last_state[leg] = current_state[leg];
    current_state[leg] = SWING;
    this->count = 0;
    // std::cout<<"no"<<std::endl;
  }
}


swingdown_control::swingdown_control(){
  this->count = 0;
}
void swingdown_control::get_cmd(Angle *angle,Leg leg){
  this->count ++;
  if(this->count > period) this->count = period;
  std::cout<<this->count<<std::endl;
  Position pos;
  chabu(&pos,this->count,leg);
  Inv_kinematics(angle,&pos,leg);
  
}
void swingdown_control::check_Transition(void){

}
void swingdown_control::get_start(Eigen::VectorXd angle, Position posi, Leg leg){
  if((current_state[leg] != pre_state[leg]) && (current_state[leg] == INITIAL)){
    //define the variables
    Angle now_angle;
    Position now_position;
    Position now_velocity;

    //get the joint angle position and velocity
    auto pos = angle;
    if(leg == fl){
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i];
      }
    }
    else{
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i+3];
      }
    }

    //get the end point position and velocity using forward kinematics and jacobian
    Kinematics(&now_angle,&now_position,leg);

    now_velocity.x = 0;
    now_velocity.y = 0;
    now_velocity.z = 0;

    //input the desire position and velocity
    Position pdes;
    Position vdes;

    pdes.x = posi.x;
    pdes.y = posi.y;
    pdes.z = posi.z;

    vdes.x = 0;
    vdes.y = 0;
    vdes.z = 0;

    //init chabu
    init_chabu(&pdes,&vdes,&now_position,&now_velocity,period,leg);
    last_state[leg] = current_state[leg];
    current_state[leg] = SWING_DOWN;
    this->count = 0;
    // std::cout<<"no"<<std::endl;
  }
}

swingup_control::swingup_control(){
  this->count = 0;
}
void swingup_control::get_cmd(Angle *angle,Leg leg){
  this->count ++;
  if(this->count > period){
     this->count = period;
     current_state[leg] = INITIAL;
  }

  Position pos;
  chabu(&pos,this->count,leg);
  Inv_kinematics(angle,&pos,leg);
  
}
void swingup_control::check_Transition(void){

}
void swingup_control::get_start(Eigen::VectorXd angle, Position posi, Leg leg){
  if((pre_state[leg] != current_state[leg]) && (current_state[leg] == STANCE)){
    //define the variables
    Angle now_angle;
    Position now_position;
    Position now_velocity;

    //get the joint angle position and velocity
    auto pos = angle;
    if(leg == fl){
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i];
      }
    }
    else{
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i+3];
      }
    }

    //get the end point position and velocity using forward kinematics and jacobian
    Kinematics(&now_angle,&now_position,leg);

    now_velocity.x = 0;
    now_velocity.y = 0;
    now_velocity.z = 0;

    //input the desire position and velocity
    Position pdes;
    Position vdes;

    pdes.x = posi.x;
    pdes.y = posi.y;
    pdes.z = posi.z;

    vdes.x = 0;
    vdes.y = 0;
    vdes.z = 0;

    //init chabu
    init_chabu(&pdes,&vdes,&now_position,&now_velocity,period,leg);
    last_state[leg] = current_state[leg];
    current_state[leg] = SWING_UP;
    this->count = 0;
  }
}

stance_control::stance_control(){
  F.resize(3);
  this->count = 0;
}
Eigen::Vector3d stance_control::get_cmd(Leg leg,double roll,Eigen::VectorXd angles){
  this->count ++;
  if(this->count >= force_period){
    pre_state[leg] = SWING_UP;
  }
  std::cout<<this->count<<std::endl;

  auto rot_matrix = rpy2romatrix(roll,0,0);
  Eigen::Vector3d tau;
  if(leg==fl)  tau = calcu_Jaco(angles.head(3),leg).transpose() * rot_matrix.transpose() * F;
  else tau = calcu_Jaco(angles.tail(3),leg).transpose() * rot_matrix.transpose() * F;
  return tau;
}
void stance_control::check_Transition(void){

}
void stance_control::get_start(Leg leg){
  if((current_state[leg] != pre_state[leg]) && (current_state[leg] == SWING_DOWN)){
    this->F << 0,0,-200;
    this->count = 0;
    last_state[leg] = current_state[leg];
    current_state[leg] = STANCE;
  }
}

Position pos_control(Leg leg, double roll){
  Eigen::Vector3d pos_B,pos_H;
  if(leg == fl){
    pos_H[0] = 0.15;
    pos_H[1] = dL;
    pos_H[2] = -hG*cos(roll)+width/2*sin(fabs(roll)); 
  }
  else{
    pos_H[0] = 0.15;
    pos_H[1] = -dL;
    pos_H[2] = -hG*cos(roll)+width/2*sin(fabs(roll));
  }
  auto rot_matrix = rpy2romatrix(roll,0,0);
  pos_B = rot_matrix.transpose() * pos_H;

  Position position_B;
  position_B.x = pos_B[0];
  position_B.y = pos_B[1];
  position_B.z = pos_B[2];

  return position_B;
}

void force_control(){

}

void set_xyz(Leg leg,Angle *angle,float xx,float yy,float zz){
  Position pos;
  pos.x = xx;
  pos.y = yy;
  pos.z = zz;

  Inv_kinematics_ref(angle,&pos,leg);
}

void init_chabu(Position *pdes,Position *vdes,Position *pini,Position *vini,float step,Leg leg){
	
	float p0[3];
	float pf[3];
	float v0[3];
	float vf[3];
	float tf;
	
	p0[0] = pini->x;
	p0[1] = pini->y;
	p0[2] = pini->z;
	
	pf[0] = pdes->x;
	pf[1] = pdes->y;
	pf[2] = pdes->z;
	
	v0[0] = vini->x;
	v0[1] = vini->y;
	v0[2] = vini->z;
	
	vf[0] = vdes->x;
	vf[1] = vdes->y;
	vf[2] = vdes->z;

	tf = step;
	
	for(int i=0;i<3;i++){
		a0[leg][i] = p0[i];
		a1[leg][i] = v0[i];
		a2[leg][i] = 3.0/(tf*tf)*(pf[i]-p0[i])-2.0/tf*v0[i]-1.0/tf*vf[i];
		a3[leg][i] = -2.0/(tf*tf*tf)*(pf[i]-p0[i])+1.0/(tf*tf)*(vf[i]+v0[i]);
	}
}

void chabu(Position *pos,float step,Leg leg){
	float p[3];
	for(int i=0;i<3;i++){
		p[i] = a0[leg][i] + a1[leg][i]*step + a2[leg][i]*step*step +a3[leg][i]*step*step*step;
	}
	pos->x = p[0];
	pos->y = p[1];
	pos->z = p[2];
	
}

float linear(float ini,float des,float tf,float step){
	float p0 = ini;
	float pf = des;
	float b0,b1;
	b0 = p0;
	b1 = (pf-p0)/tf;
	float p = b0+b1*step;
	return p;
}

