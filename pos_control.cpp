#include "main.h"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  /// create raisim world
  raisim::World::setActivationKey("/home/hxy/.raisim/activate.raisim");
  raisim::World world;

  /// create objects
  auto mini_cheetah = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\mini_cheetah\\urdf\\mini_cheetah.urdf");
  auto ground = world.addGround();
  world.setTimeStep(0.001);

  /// calculate init position

  simple_set(0.3,0.2,-0.3);

  /// mini_cheetah init
  Eigen::VectorXd jointNominalConfig(mini_cheetah->getGeneralizedCoordinateDim()), jointVelocityTarget(mini_cheetah->getDOF());
  select_p(angle, jointNominalConfig);
  jointVelocityTarget.setZero();

  mini_cheetah->setGeneralizedCoordinate(jointNominalConfig);
  mini_cheetah->setGeneralizedForce(Eigen::VectorXd::Zero(mini_cheetah->getDOF()));
  mini_cheetah->setName("mini_cheetah");

  /// mini_cheetah joint PD controller
  Eigen::VectorXd pGain(mini_cheetah->getDOF()), dGain(mini_cheetah->getDOF());
  pGain<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10; 
  dGain<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  pGain.tail(12).setConstant(100.0);
  dGain.tail(12).setConstant(1.0);
  mini_cheetah->setPdGains(pGain, dGain);

  Eigen::VectorXd pTarget(mini_cheetah->getGeneralizedCoordinateDim()), vTarget(mini_cheetah->getDOF());
  select_p(angle, pTarget);
  select_v(angleV, vTarget);
  mini_cheetah->setPdTarget(pTarget, vTarget);

  /// launch raisim server for visualization. Can be visualized using raisimUnity
  raisim::RaisimServer server(&world);
  server.launchServer();

  /// printf info
  std::cout<<mini_cheetah->getGeneralizedCoordinate()<<std::endl;
  std::cout<<mini_cheetah->getGeneralizedVelocity()<<std::endl;


  //生成步态
  create_gait();
  std::cout<<x_position.size()<<std::endl;
  //初始化，使腿末端运行到希望到位置
  // setpoint(mini_cheetah->getGeneralizedCoordinate(),mini_cheetah->getGeneralizedVelocity(),0,0.062,-0.1);
  // for(int i(0);i<period;i++){

  //   raisim::MSLEEP(1);

  //   for(int j(0);j<4;j++){
  //     chabu(&position[j],i+1,j);
  //     Inv_kinematics_ref(&angle[j],&position[j],j);
  //   }
  //   select_p(angle, pTarget);
  //   mini_cheetah->setPdTarget(pTarget, vTarget);

  //   server.integrateWorldThreadSafe();
  // }

  // //jump
  // raisim::MSLEEP(1);

  // simple_set2(0,0.062,-0.3);
  // select_p(angle, pTarget);
  // mini_cheetah->setPdTarget(pTarget, vTarget);

  // server.integrateWorldThreadSafe();

  setpoint(mini_cheetah->getGeneralizedCoordinate(),mini_cheetah->getGeneralizedVelocity(),0,0.062,-0.3);
  for(int i(0);i<period;i++){

    raisim::MSLEEP(1);

    for(int j(0);j<4;j++){
      chabu(&position[j],i+1,j);
      Inv_kinematics_ref(&angle[j],&position[j],j);
    }
    select_p(angle, pTarget);
    mini_cheetah->setPdTarget(pTarget, vTarget);

    server.integrateWorldThreadSafe();
  }

  
  int count = 0;
  int count1 = T/dt/2.0;
  while(1){
    raisim::MSLEEP(1);

    // ///获取数据
    // mini_cheetah->getState(co,vel);
    // R = mini_cheetah->getBaseOrientation().e();
    // auto R_inv = R.transpose();
    // raisim::Vec<3> footVelocity, footPosition;
    // mini_cheetah->getFrameVelocity("toe_fr_joint",footVelocity);
    // mini_cheetah->getFramePosition("toe_fr_joint",footPosition);
    // mini_cheetah->getBasePosition(bodyPosition);
    // raisim::Vec<3> footPosition_w,footPosition_b;
    // footPosition_b = {0,0,-0.18};
    // mini_cheetah->getPosition(mini_cheetah->getBodyIdx("shank_fr"),footPosition_b,footPosition_w);
    // mini_cheetah->getVelocity(0,bodyVelocity);
    // //获取接触力
    // // auto force = mini_cheetah->getContacts()[0].impulse() / dt;
    // // std::cout<<force<<std::endl;
    // ///计算数据
    // //计算关节角度
    // Angle now_angle[4];
    // int cou = 7;
    // for(int i=0;i<4;i++){
    //   for(int j=0;j<3;j++){
    //     now_angle[i].q[j] = co[cou];
    //     cou ++;
    //   }
    // }
    // //计算关节角速度
    // Angle now_angleV[4];
    // cou = 6;
    // for(int i=0;i<4;i++){
    //   for(int j=0;j<3;j++){
    //     now_angleV[i].q[j] = vel[cou];
    //     cou ++;
    //   }
    // }
    // Eigen::VectorXd angleV(3);
    // angleV << now_angleV[0].q[0],now_angleV[0].q[1],now_angleV[0].q[2];
    // //计算雅可比
    // auto Jaco1 = calcu_Jaco(&now_angle[0],0);

    // raisim::SparseJacobian Jaco;
    // mini_cheetah->getSparseJacobian(mini_cheetah->getBodyIdx("shank_fr"),footPosition,Jaco);
    // //计算末端速度
    // auto footVel = R*Jaco1*angleV + bodyVelocity.e();
    // auto footVel2 = Jaco.e() * vel.block(0,0,9,1);

    // std::cout<<bodyVelocity.e()<<std::endl;
    // std::cout<<"begin1"<<std::endl;
    // std::cout<<calcu_endpoint_pos(0)<<std::endl;
    // std::cout<<footPosition.e()<<std::endl;
    // std::cout<<"begin2"<<std::endl;
    // std::cout<<Jaco.e()<<std::endl;
    // std::cout<<R*Jaco1<<std::endl;
    // std::cout<<vel<<std::endl;
    // std::cout<<"begin3"<<std::endl;
    // std::cout<<footVel2<<std::endl;
    // std::cout<<footVelocity.e()<<std::endl;
    // std::cout<<" "<<std::endl;


    int j = 1;
    position[j].x = x_position[count];
    position[j].z = z_position[count];
    Inv_kinematics_ref(&angle[j],&position[j],j);
    j = 2;
    position[j].x = x_position[count];
    position[j].z = z_position[count];
    Inv_kinematics_ref(&angle[j],&position[j],j);

    j = 0;
    position[j].x = x_position[count1];
    position[j].z = z_position[count1];
    Inv_kinematics_ref(&angle[j],&position[j],j);
    j = 3;
    position[j].x = x_position[count1];
    position[j].z = z_position[count1];
    Inv_kinematics_ref(&angle[j],&position[j],j);

    select_p(angle, pTarget);
    mini_cheetah->setPdTarget(pTarget, vTarget);

    count ++;
    count1 ++;

    if(count>=x_position.size()){
        count = 0;
    }
    if(count1>=x_position.size()){
        count1 = 0;
    }

    server.integrateWorldThreadSafe();

  }

  server.killServer();
  std::cout<<"end of simulation"<<std::endl;
}
void simple_set(float xx,float yy,float zz){

  for(int i=0;i<4;i++){
    position[i].z = zz;
  }
  position[fr].x = xx;
  position[fr].y = -yy;
  position[fl].x = xx;
  position[fl].y = yy;
  position[hr].x = -xx;
  position[hr].y = -yy;
  position[hl].x = -xx;
  position[hl].y = yy;
  

  Inv_kinematics(&angle[fr],&position[fr],fr);
  Inv_kinematics(&angle[fl],&position[fl],fl);
  Inv_kinematics(&angle[hr],&position[hr],hr);
  Inv_kinematics(&angle[hl],&position[hl],hl);

}
void simple_set2(float xx,float yy,float zz){

  for(int i=0;i<4;i++){
    position[i].x = xx;
		position[i].z = zz;
  }

  position[fl].y = yy;
  position[hl].y = yy;
  position[fr].y = -yy;
  position[hr].y = -yy;
  

  Inv_kinematics_ref(&angle[fr],&position[fr],fr);
  Inv_kinematics_ref(&angle[fl],&position[fl],fl);
  Inv_kinematics_ref(&angle[hr],&position[hr],hr);
  Inv_kinematics_ref(&angle[hl],&position[hl],hl);

}
void set_position(Leg leg,float xx,float yy,float zz){
  position[leg].x = xx;
  position[leg].y = yy;
  position[leg].z = zz;

  Inv_kinematics(&angle[leg],&position[leg],leg);
}

void setpoint(const raisim::VecDyn & pos,const raisim::VecDyn & vel,float xx,float yy,float zz){

  Angle now_angle[4];
  Angle now_angleV[4];
  Position now_position[4];
  Position now_velocity[4];

  //get the joint angle position and velocity
  int count = 7;
  for(int i=0;i<4;i++){
    for(int j=0;j<3;j++){
      now_angle[i].q[j] = pos[count];
      count ++;
    }
  }

  count = 6;
  for(int i=0;i<4;i++){
    for(int j=0;j<3;j++){
      now_angleV[i].q[j] = vel[count];
      count ++;
    }
  }

  //get the end point position and velocity using forward kinematics and jacobian
  for(int i=0;i<4;i++){
    Kinematics_ref(&now_angle[i],&now_position[i],i);
  }

  for(int i=0;i<4;i++){
    now_velocity[i].x = 0;
		now_velocity[i].y = 0;
		now_velocity[i].z = 0;
  }
	
  //input the desire position and velocity
	Position pdes[4];
  Position vdes[4];

  for(int i=0;i<4;i++){
    vdes[i].x = 0;
		vdes[i].y = 0;
		vdes[i].z = 0;
  }

  for(int i=0;i<4;i++){
    pdes[i].x = xx;
		pdes[i].z = zz;
  }

  pdes[fl].y = yy;
  pdes[hl].y = yy;
  pdes[fr].y = -yy;
  pdes[hr].y = -yy;

  //init chabu
  for(int i(0);i<4;i++){
    init_chabu(&pdes[i],&vdes[i],&now_position[i],&now_velocity[i],period,i);
  }
  
}

void create_gait(void){
  float t;
  float x_sw,z_sw,x_st,z_st;

  //赋值
  tf = T*swing_time;
  std::cout<<tf<<std::endl;
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
  p0 = 0.1;
  pf = -0.1;
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