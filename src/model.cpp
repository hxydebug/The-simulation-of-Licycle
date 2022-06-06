#include "model.h"
#include "leg_controller.h"

#define PI 3.1415926535

robot::robot(raisim::ArticulatedSystem* client){
	bicycle = client;
    robot::initial_bicycle(); 
    robot::update_state();   
}
void robot::initial_bicycle(){
    float high = 0;
    Angle angle_l,angle_r;
    // set_xyz(fl,&angle_l,0.16,0.08,-0.12);
    // set_xyz(fr,&angle_r,0.16,-0.08,-0.12);
    set_xyz(fl,&angle_l,0,0.08,-0.32);
    set_xyz(fr,&angle_r,0,-0.08,-0.32);
    Eigen::VectorXd jointNominalConfig(bicycle->getGeneralizedCoordinateDim()), jointVelocityTarget(bicycle->getDOF());
    jointNominalConfig<< 0, 0, 0.38+high, RpyToqua(0.0*PI/180.0,0.0*PI/180.0,0.0*PI/180.0),  
                         0,0,0, 
                         angle_l.q[0],angle_l.q[1],angle_l.q[2],
                         angle_r.q[0],angle_r.q[1],angle_r.q[2];
                        //  0.0757548,  0.309034,  -1.23186,
                        //  -0.075739,  0.882229,  -1.26519;
    jointVelocityTarget.setZero();
    jointVelocityTarget[0] = 0.0;//velocity
    bicycle->setGeneralizedCoordinate(jointNominalConfig);
    bicycle->setGeneralizedVelocity(jointVelocityTarget);
    bicycle->setGeneralizedForce(Eigen::VectorXd::Zero(bicycle->getDOF()));
    bicycle->setName("smart bicycle");
}
int robot::getDOF(){
    return bicycle->getDOF();
}
int robot::getGeneralizedCoordinateDim(){
    return bicycle->getGeneralizedCoordinateDim();
}
void robot::step(Eigen::VectorXd bicycle_tau,Eigen::VectorXd leg_tau){
    Eigen::VectorXd base_tau(6),tau(15);
    base_tau.setZero();
    tau<<base_tau,bicycle_tau,leg_tau;
    bicycle->setGeneralizedForce(tau);
}
void robot::update_state(){
    bicycle->getState(pos,vel);
    rot_matrix = bicycle->getBaseOrientation().e();
    robot::update_angle();
}
void robot::update_angle(){
    Eigen::Vector3d angle_v = robot::get_base_angle_v();
    dpsi = (angle_v[2]+last_dpsi)/2;
    dvarphi = (angle_v[0]+last_dvarphi)/2;
    last_dpsi = angle_v[2];
    last_dvarphi = angle_v[0];
    // dpsi = angle_v[2];
    // dvarphi = angle_v[0];
}
// void robot::update_angle(){
//     Eigen::Vector3d angle_v = robot::get_base_angle_v();
//     dpsi = (angle_v[2]+last_dpsi+llast_dpsi)/3;
//     dvarphi = (angle_v[0]+last_dvarphi+llast_dvarphi)/3;
//     llast_dpsi = last_dpsi;
//     llast_dvarphi = last_dvarphi;
//     last_dpsi = angle_v[2];
//     last_dvarphi = angle_v[0];
// }
Eigen::VectorXd robot::get_leg_vel(){
    return vel.tail(6);
}
Eigen::VectorXd robot::get_bicycle_vel(){
    return vel.segment(6,3);
}
Eigen::VectorXd robot::get_leg_pos(){
    return pos.tail(6);
}
Eigen::VectorXd robot::get_bicycle_pos(){
    return pos.segment(7,3);
}
Eigen::VectorXd robot::get_base_orientation(){
    return pos.segment(3,4);
}
Eigen::VectorXd robot::get_base_rpy(){
    Eigen::Vector3d rpy;
    Eigen::Vector4d quat = pos.segment(3,4);
    quaToRpy(quat,rpy);
    return rpy;
}
Eigen::VectorXd robot::get_base_angle_v(){
    Eigen::Vector3d angle_v;
    Eigen::Vector3d angle_v_world = vel.segment(3,3);
    angle_v = rot_matrix.transpose()*angle_v_world;
    return angle_v;
}
double robot::get_psi(){
    Eigen::Vector3d rpy = robot::get_base_rpy();
    return rpy[2];
}
double robot::get_varphi(){
    Eigen::Vector3d rpy = robot::get_base_rpy();
    return rpy[0];
}
double robot::get_phi(){
    return pos[7];
}
double robot::get_dphi(){
    return vel[6];
}
double robot::get_dpsi(){
    return dpsi;
}
double robot::get_dvarphi(){
    return dvarphi;
}
double robot::get_base_v(){
    Eigen::Vector3d v;
    Eigen::Vector3d v_world = vel.head(3);
    v = rot_matrix.transpose()*v_world;
    return v[0];
}
Eigen::Vector3d robot::get_base_vec(){
    Eigen::Vector3d v_world = vel.head(3);
    return rot_matrix.transpose()*v_world;
}

std::vector<int> robot::GetFootContact(){
    auto l_foot = bicycle->getBodyIdx("left_shank_Link");
    auto r_foot = bicycle->getBodyIdx("right_shank_Link");
    contact_leg[0] = 0;
    contact_leg[1] = 0;
    for(auto& contact: bicycle->getContacts()) {
        if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
        if ( l_foot == contact.getlocalBodyIndex() ) {
            contact_leg[0] = 1;
        }
        if ( r_foot == contact.getlocalBodyIndex() ) {
            contact_leg[1] = 1;
        }
    }
    return contact_leg;
}