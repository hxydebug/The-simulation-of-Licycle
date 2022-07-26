#include "main.h"
#include "leg_controller.h"
#include "bicycle_controller.h"
#include "model.h"
#include "swing_leg_controller.h"
#include "stance_leg_controller.h"
#include <iostream>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
// #include <term.h>
// #include <curses.h>
#include <unistd.h>
static struct termios initial_settings, new_settings;
static int peek_character = -1;
void init_keyboard(); 
void close_keyboard() ;
int kbhit() ;
int readch();

int counttt=20;
char ch[64] = {0};
int kbflag = 0;
int kbmode = 1;

void init_keyboard (){
	tcgetattr (0,&initial_settings) ;
	new_settings = initial_settings ;
	new_settings.c_lflag &= ~ ICANON;
	new_settings.c_lflag &= ~ ECHO;
	new_settings.c_lflag &= ~ ISIG;
	new_settings.c_cc [VMIN] = 1;
	new_settings.c_cc [VTIME] = 0;
	tcsetattr(0, TCSANOW, &new_settings) ;
}
void close_keyboard()
{
tcsetattr(0, TCSANOW, &initial_settings) ;
}
//下面就是检测是否有击键动作的kbhit函数:
int kbhit ()
{
	char ch;
	int nread;
	if (peek_character != -1)
		return 1;
	new_settings.c_cc[VMIN]=0;
	tcsetattr(0,TCSANOW, &new_settings) ;
	nread = read(0,&ch,1) ;
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0, TCSANOW, &new_settings) ;
	if (nread == 1) {
		peek_character = ch;
		return 1;
	}
	return 0;
}
 
//按键对应的字符由下-一个函数readch读取，它会将变量peek_ character重置为-1以进入下一次循环。
 
int readch()
{
	char ch;
	if(peek_character != -1) {
		ch = peek_character;
		peek_character = -1;
		return ch;
	}
	read(0,&ch,1) ;
	return ch;
}

int main(int argc, char* argv[]) {

  init_keyboard() ;
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  /// create raisim world
  raisim::World::setActivationKey("/home/hxy/.raisim/activate.raisim");
  raisim::World world;
  auto smart_bycicle = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\licycle_urdfnew\\urdf\\licycle_urdf.urdf");
 
  auto ground = world.addGround(0,"steel");

  // raisim::TerrainProperties terrainProperties;
  // terrainProperties.frequency = 0.4;
  // terrainProperties.zScale = 0.5;
  // terrainProperties.xSize = 40.0;
  // terrainProperties.ySize = 20.0;
  // terrainProperties.xSamples = 50;
  // terrainProperties.ySamples = 50;
  // terrainProperties.fractalOctaves = 3;
  // terrainProperties.fractalLacunarity = 2.0;
  // terrainProperties.fractalGain = 0.25;
  // auto hm = world.addHeightMap(0.0, 0.0, terrainProperties,"steel"); 
  // std::vector<double> height = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0, 
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0, 
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,
  //                               0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0};
  // auto heightMap = world.addHeightMap(20, 20, 5, 5, 0, 0, height);

  // auto hm = world.addHeightMap("/home/hxy/raisim_workspace/legs/src/rugged_terrain.txt", 4, 0);
  // auto hm = world.addHeightMap("/home/hxy/raisim_workspace/legs/src/rugged_terrain.txt", 6, 1);

  // auto hm = world.addHeightMap("/home/hxy/raisim_workspace/legs/src/rugged_terrain.txt", 4, 0);
  // auto hm = world.addHeightMap("/home/hxy/raisim_workspace/legs/src/door.txt", 4, 4);
  // auto hm = world.addHeightMap("/home/hxy/raisim_workspace/legs/src/afile.txt", 9, 0,"steel");
  // auto hm = world.addHeightMap("/home/hxy/raisim_workspace/legs/src/bfile10.txt", 2.5, 2,"steel");


  world.setMaterialPairProp("steel", "rubber", 0.45, 0.15, 0.001);

  world.setTimeStep(0.001);

  /// smart_bycicle init
  robot robot(smart_bycicle);

  //生成数据编号
  time_t tt = time(NULL);
  strftime(ch, sizeof(ch) - 1, "%H%M", localtime(&tt));
  char result[100] = {0};
  sprintf(result, "/home/hxy/sim_data/dataFile%s.txt", ch);
  std::ofstream dataFile;
  dataFile.open(result, std::ofstream::app);

  ///controller init
  gait_generator gait_gen(&robot);
 	swing_leg_controller swc(&robot,&gait_gen,0.8);
 	stance_leg_controller stc(&robot,&gait_gen,0.8);
  bicycle_controller b_control(&robot,ch);
  leg_controller l_control(&robot,&gait_gen,&swc,&stc);

  /// launch raisim server for visualization. Can be visualized using raisimUnity
  raisim::RaisimServer server(&world);
  server.launchServer();

  Eigen::VectorXd bicycle_tau;
  Eigen::VectorXd leg_tau;
  float global_timer = 0;

  float desire_v = 0;
  
  while(1){
    raisim::MSLEEP(1);
    robot.update_state();
    
    if(global_timer<1){
      //initial
      if(counttt==20){
        bicycle_tau = b_control.get_action(1,0);
        counttt=1;
      }
      counttt ++;
      leg_tau = l_control.get_action(0);
    }


    // //back
    //   float desire_v = -0.8;
    //   swc.desired_xspeed = desire_v;
    //   stc.desired_xspeed = desire_v;
    //   if(counttt==20){
    //     bicycle_tau = b_control.get_action(1,desire_v);
    //     counttt=1;
    //   }
    //   counttt ++;
    //   auto leg_tau = l_control.get_action(3);

    // //ready
    //   float desire_v = 0;
    //   swc.desired_xspeed = desire_v;
    //   stc.desired_xspeed = desire_v;
    //   if(counttt==20){
    //     bicycle_tau = b_control.get_action(1,desire_v);
    //     counttt=1;
    //   }
    //   counttt ++;
    //   leg_tau = l_control.get_action(3);

    // //speed up
    //   float desire_v = 0.5;
    //   swc.desired_xspeed = desire_v;
    //   stc.desired_xspeed = desire_v;
    //   // stc.desired_roll = -5*PI/180;
    //   if(counttt==20){
    //     bicycle_tau = b_control.get_action(0,desire_v);
    //     counttt=1;
    //   }
    //   counttt ++;
    //   leg_tau = l_control.get_action(3);

    // //without eic
    //   float desire_v = 0.5;
    //   swc.desired_xspeed = desire_v;
    //   stc.desired_xspeed = desire_v;
    //   // stc.desired_roll = -5*PI/180;
    //   if(counttt==20){
    //     bicycle_tau = b_control.get_action(4,desire_v);
    //     counttt=1;
    //   }
    //   stc.h_varphi = b_control.get_h_varphi();
    //   stc.desired_roll = b_control.get_varphie();
    //   counttt ++;
    //   leg_tau = l_control.get_action(3);

    // //rugged road
    // float desire_v = 1.2;
    // swc.desired_xspeed = desire_v;
    // stc.desired_xspeed = desire_v;
    // // stc.desired_roll = -5*PI/180;
    // if(counttt==20){
    //   bicycle_tau = b_control.get_action(5,desire_v);
    //   counttt=1;
    // }
    // stc.h_varphi = b_control.get_h_varphi();
    // // stc.desired_roll = b_control.get_varphie();
    // counttt ++;
    // leg_tau = l_control.get_action(4);

    // else if(global_timer<2){
    //   // //back
    //   // float desire_v = -0.8;
    //   // swc.desired_xspeed = desire_v;
    //   // stc.desired_xspeed = desire_v;
    //   // if(counttt==20){
    //   //   bicycle_tau = b_control.get_action(1,desire_v);
    //   //   counttt=1;
    //   // }
    //   // counttt ++;
    //   // auto leg_tau = l_control.get_action(3);
    //   //ready
    //   float desire_v = 0;
    //   swc.desired_xspeed = desire_v;
    //   stc.desired_xspeed = desire_v;
    //   if(counttt==20){
    //     bicycle_tau = b_control.get_action(1,desire_v);
    //     counttt=1;
    //   }
    //   counttt ++;
    //   leg_tau = l_control.get_action(3);
    // }
    // else if(global_timer<15){
    //   //speed up
    //   float desire_v = 0.5;
    //   swc.desired_xspeed = desire_v;
    //   stc.desired_xspeed = desire_v;
    //   // stc.desired_roll = -5*PI/180;
    //   if(counttt==20){
    //     bicycle_tau = b_control.get_action(0,desire_v);
    //     counttt=1;
    //   }
    //   counttt ++;
    //   leg_tau = l_control.get_action(3);
    // }
    // door
    // else if(global_timer<180){
    //   float desire_v = 0.5;
    //   swc.desired_xspeed = desire_v;
    //   stc.desired_xspeed = desire_v;
    //   // stc.desired_roll = -5*PI/180;
    //   if(counttt==20){
    //     bicycle_tau = b_control.get_action(4,desire_v);
    //     counttt=1;
    //   }
    //   stc.h_varphi = b_control.get_h_varphi();
    //   stc.desired_roll = b_control.get_varphie();
    //   counttt ++;
    //   leg_tau = l_control.get_action(3);
    // }
    // else if(global_timer<180){
    //   float desire_v = 0.5;
    //   swc.desired_xspeed = desire_v;
    //   stc.desired_xspeed = desire_v;
    //   // stc.desired_roll = -5*PI/180;
    //   if(counttt==20){
    //     bicycle_tau = b_control.get_action(3,desire_v);
    //     counttt=1;
    //   }
    //   counttt ++;
    //   leg_tau = l_control.get_action(5);
    // }
    // rugged terrain
    // else if(global_timer<180){
    //   if(b_control.get_xdistance()<3.5){
    //     float desire_v = 1.2;
    //     swc.desired_xspeed = desire_v;
    //     stc.desired_xspeed = desire_v;
    //     // stc.desired_roll = -5*PI/180;
    //     if(counttt==20){
    //       bicycle_tau = b_control.get_action(2,desire_v);
    //       counttt=1;
    //     }
    //     counttt ++;
    //     leg_tau = l_control.get_action(5);
    //   }
    //   else if(b_control.get_xdistance()<14.8){
    //     float desire_v = 1.2;
    //     swc.desired_xspeed = desire_v;
    //     stc.desired_xspeed = desire_v;
    //     // stc.desired_roll = -5*PI/180;
    //     if(counttt==20){
    //       bicycle_tau = b_control.get_action(5,desire_v);
    //       counttt=1;
    //     }
    //     stc.h_varphi = b_control.get_h_varphi();
    //     // stc.desired_roll = b_control.get_varphie();
    //     counttt ++;
    //     leg_tau = l_control.get_action(4);
    //   }
    //   // else if(b_control.get_xdistance()<14.8){
    //   //   float desire_v = 0.9;
    //   //   swc.desired_xspeed = desire_v;
    //   //   stc.desired_xspeed = desire_v;
    //   //   // stc.desired_roll = -5*PI/180;
    //   //   if(counttt==20){
    //   //     bicycle_tau = b_control.get_action(2,desire_v);
    //   //     counttt=1;
    //   //   }
    //   //   stc.bike_tau = b_control.get_tau();
    //   //   counttt ++;
    //   //   leg_tau = l_control.get_action(7);
    //   // }
    //   else{
    //     float desire_v =1.2;
    //     swc.desired_xspeed = desire_v;
    //     stc.desired_xspeed = desire_v;
    //     // stc.desired_roll = -5*PI/180;
    //     if(counttt==20){
    //       bicycle_tau = b_control.get_action(2,desire_v);
    //       counttt=1;
    //     }
    //     counttt ++;
    //     leg_tau = l_control.get_action(5);
    //   }
    // }
    // else if(global_timer<18){
    //   float desire_v = 1.2;
    //   swc.desired_xspeed = desire_v;
    //   stc.desired_xspeed = desire_v;
    //   // stc.desired_roll = -5*PI/180;
    //   if(counttt==20){
    //     bicycle_tau = b_control.get_action(2,desire_v);
    //     counttt=1;
    //   }
    //   counttt ++;
    //   leg_tau = l_control.get_action(5);
    // }
    else{

      if(kbhit()){
        int k = readch();
        std::cout<<k<<"hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh"<<std::endl;
        // direction
        if(k == 119) kbflag = 0;//w forward
        if(k == 100) kbflag = 1;//d right
        if(k == 97) kbflag = 2;//a left
        // mode
        if(k == 103) kbmode = 0;//g (without leg)
        if(k == 115) kbmode = 1;//s stop
        if(k == 98) kbmode = 2;//b back     2
        if(k == 114) kbmode = 3;//r ready
        if(k == 108) kbmode = 4;//l slow(without eic)     2
        if(k == 111) kbmode = 5;//o rugged road     2
        if(k == 107) kbmode = 6;//k slow(without eic)  decrease the pd
      }

      // std::cout<<kbmode<<std::endl;
      close_keyboard();

      switch(kbmode){
        case 0:
          desire_v = 1.2;
          swc.desired_xspeed = desire_v;
          stc.desired_xspeed = desire_v;
          // stc.desired_roll = -5*PI/180;
          if(counttt==20){
            bicycle_tau = b_control.get_action1(2,desire_v,kbflag);
            counttt=1;
          }
          counttt ++;
          leg_tau = l_control.get_action(5);
          break;

        case 1:
          if(counttt==20){
            bicycle_tau = b_control.get_action(1,0);
            counttt=1;
          }
          counttt ++;
          leg_tau = l_control.get_action(0);
          break;

        case 2:
          desire_v = -0.8;
          swc.desired_xspeed = desire_v;
          stc.desired_xspeed = desire_v;
          if(counttt==20){
            bicycle_tau = b_control.get_action1(1,desire_v,kbflag);
            counttt=1;
          }
          counttt ++;
          leg_tau = l_control.get_action(3);
          break;

        case 3:
          desire_v = 0;
          swc.desired_xspeed = desire_v;
          stc.desired_xspeed = desire_v;
          if(counttt==20){
            bicycle_tau = b_control.get_action(1,desire_v);
            counttt=1;
          }
          counttt ++;
          leg_tau = l_control.get_action(3);
          break;

        case 4:
          desire_v = 0.5;
          swc.desired_xspeed = desire_v;
          stc.desired_xspeed = desire_v;
          // stc.desired_roll = -5*PI/180;
          if(counttt==20){
            bicycle_tau = b_control.get_action1(4,desire_v,kbflag);
            counttt=1;
          }
          // stc.h_varphi = b_control.get_h_varphi();
          // stc.desired_roll = b_control.get_varphie();
          counttt ++;
          leg_tau = l_control.get_action(3);
          break;

        case 5:
          desire_v = 1.2;
          swc.desired_xspeed = desire_v;
          stc.desired_xspeed = desire_v;
          // stc.desired_roll = -5*PI/180;
          if(counttt==20){
            bicycle_tau = b_control.get_action1(2,desire_v,kbflag);
            counttt=1;
          }
          // stc.h_varphi = b_control.get_h_varphi();
          // stc.desired_roll = b_control.get_varphie();
          counttt ++;
          leg_tau = l_control.get_action(4);
          break;

        case 6:
          desire_v = 0.5;
          swc.desired_xspeed = desire_v;
          stc.desired_xspeed = desire_v;
          // stc.desired_roll = -5*PI/180;
          if(counttt==20){
            bicycle_tau = b_control.get_action(4,desire_v);
            counttt=1;
          }
          // stc.h_varphi = b_control.get_h_varphi();
          // stc.desired_roll = b_control.get_varphie();
          counttt ++;
          leg_tau = l_control.get_action(8);
          break;

      }
    }



    int stance = gait_gen.leg_state[0];

    Position l_leg_p = swc.postarget[0];
    Position r_leg_p = swc.postarget[1];

    // state
    Angle l_angle;
    Angle r_angle;
    Position l_position;
    Position r_position;
    auto pos = robot.get_leg_pos();
    for(int i(0);i<3;i++){
        l_angle.q[i] = pos[i];
        r_angle.q[i] = pos[i+3];
    }
    Kinematics(&l_angle,&l_position,0);
    Kinematics(&r_angle,&r_position,1);
    /// record the data
    // 朝TXT文档中写入数据
    dataFile << leg_tau[0] << ", " << leg_tau[1] << ", " << leg_tau[2] << ", " 
              << leg_tau[3] << ", " << leg_tau[4] << ", " << leg_tau[5] << ", " 
              << l_leg_p.x << ", " << l_leg_p.y << ", " << l_leg_p.z << ", " 
              << r_leg_p.x << ", " << r_leg_p.y << ", " << r_leg_p.z << ", " << stance<< ", "
              << l_position.x << ", " << l_position.y << ", " << l_position.z << ", " 
              << r_position.x << ", " << r_position.y << ", " << r_position.z << ", " 
              << l_angle.q[0] << ", " << l_angle.q[1] << ", " << l_angle.q[2] << ", " 
              << r_angle.q[0] << ", " << r_angle.q[1] << ", " << r_angle.q[2] << ", " << robot.get_dvarphi() 
              << std::endl;

    robot.step(bicycle_tau,leg_tau);
    
    global_timer += 0.001;
    // ///获取数据
    // mini_cheetah->getState(co,vel);
    // raisim::Vec<3> footVelocity, footPosition;
    // mini_cheetah->getFrameVelocity("toe_fr_joint",footVelocity);
    // mini_cheetah->getFramePosition("toe_fr_joint",footPosition);
    // mini_cheetah->getBasePosition(bodyPosition);
    // mini_cheetah->getVelocity(0,bodyVelocity);
    // R = mini_cheetah->getBaseOrientation().e();
    // raisim::SparseJacobian Jac;
    // mini_cheetah->getSparseJacobian(mini_cheetah->getBodyIdx("shank_fr"),footPosition,Jac);
    // auto tau1 = mini_cheetah->getGeneralizedForce();
    
    // ///计算数据
    // //计算雅可比
    // auto Jaco = Jac.e().block(0,6,3,3);
    // //计算旋转矩阵的逆
    // auto R_inv = R.transpose();
    // //计算接触力
    // auto footIndex = mini_cheetah->getBodyIdx("shank_fr");
    // for(auto& contact: mini_cheetah->getContacts()) {
    //   if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
    //   if ( footIndex == contact.getlocalBodyIndex() ) {
    //     force = contact.getContactFrame().e().transpose() * contact.getImpulse()->e()/dt;
    //   }
    // }
    // //计算关节力矩
    // Eigen::VectorXd tau3d(3);
    // int cou = 6;
    // for(int j=0;j<3;j++){
    //   tau3d[j] = tau1[cou];
    //   cou ++;
    // }
    // //计算关节力矩2
    // auto tau2 = Jaco.transpose()*R_inv*force;


    

    // ///打印信息
    // std::cout<<"begin1"<<std::endl;
    // std::cout<<tau2<<std::endl;
    // std::cout<<"begin2"<<std::endl;
    // std::cout<<tau3d<<std::endl;
    // std::cout<<"begin3"<<std::endl;
    // std::cout<<tau1.e().tail(12).transpose()<<std::endl;
    // std::cout<<"begin4"<<std::endl;
    // std::cout<<co.tail(12).transpose()<<std::endl;
    // std::cout<<" "<<std::endl;

    


    server.integrateWorldThreadSafe();

  }

  dataFile.close();
  server.killServer();
}