#include "Domi.h"

// For std::cout
#include <iostream>

// For strtok_r
#include <string.h>

// For atof
#include <cstdlib>

// For timer
#include <sutil/CSystemClock.hpp>

// For popen
#include <fcntl.h>
#include <QThread>

// Gripper
#include <chrono>
#include <thread>
using namespace std;

Domi::Domi(scl::SGcModel &_rgcm,                // Robot data structure with dynamic quantities 
           scl::SRobotIO &_rio,                 // I/O data structure
           scl::CDynamicsScl &_dyn_scl,         // Robot Kinematics and dynamics computation object
           scl::SRigidBodyDyn *_rhand,          // Orientation (last link)
           scl::SRigidBodyDyn *_rwrist,         // Position     (second to last link)
           const Eigen::Vector3d &_hpos,
           double _dt)
           :
           state(Domi::MOVE_TO_REST),
           rgcm(_rgcm),
           rio(_rio),
           dyn_scl(_dyn_scl),
           rhand(_rhand),
           rwrist(_rwrist),
           hpos(_hpos),
           dt(_dt),
           iter(0),
           py_fp(NULL),
           x_rest(-0.47),
           y_rest(-0.57),
           z_above(0.4),
           z_depth(0.31) {
           
    is_simulator = (dt > 0);
    
    q_lim = Eigen::VectorXd::Zero(7);
    dq_lim = Eigen::VectorXd::Zero(7);
    torque_lim = Eigen::VectorXd::Zero(7);

    // Initializing the Gripper
    schunkGripper = new SchunkGripper();
    schunkGripper->start(QThread::TimeCriticalPriority);
           
    if (is_simulator) {
        kpO = 100;      // Orientation Proportional Gain
        kvO = 10;       // Orientation Differential Gain
        kp = 150;       // Position Proportional Gain
        kv = 30;        // Position Differential Gain
        kq = 10;        // Joint Porportional Gain
        kdq = 15;       // Jonint differential q_limit_gain
        kq_lim = 10;    // Joint limit proportional Gain
        kdq_lim = 10;   // Joint limit differential Gain
        torque_lim << 1, 1, 1, 1, 1, 1, 1; //<< 175.0, 175.0, 109.0, 109.0, 109.0, 39.0, 39.0;
    }
    else {
        kpO = 225;
        kvO = 3;
        kp = 360;   // Worked well with 360
        kv = 28;    // Worked well with 28
        kq = 0;
        kdq = 15; // Worked well with 15 on the robot. Position control should account for the rest.
        kq_lim = 10;
        kdq_lim = 10;
        torque_lim = Eigen::VectorXd::Ones(7) * 20;
        g = Eigen::Vector3d::Zero(3);
        g(2) = 1.9*9.81;
    }
    
    q_lim << 160.0*M_PI/180.0, 110.0*M_PI/180.0, 160.0*M_PI/180.0, 110.0*M_PI/180.0, 160.0*M_PI/180.0, 110.0*M_PI/180.0, 165.0*M_PI/180.0;
    dq_lim << 98.0*M_PI/180.0, 98.0*M_PI/180.0, 100.0*M_PI/180.0, 130.0*M_PI/180.0, 140.0*M_PI/180.0, 180.0*M_PI/180.0, 180.0*M_PI/180.0;
}

void Domi::mainloop() {

    switch (state) {
    case Domi::MOVE_TO_REST:
        move_to_rest();
        break;
    case Domi::REST:
        rest();
        break;
    case Domi::OBSERVATION:
        observation();
        break;
    case Domi::GET_ABOVE:
        get_above();
        break;
    case Domi::GET_DEPTH:
        get_depth();
        break;
    case Domi::GET_GRASP:
        get_grasp();
        break;
    case Domi::GET_REVERSE_DEPTH:
        get_reverse_depth();
        break;
    case Domi::PUT_ABOVE:
        put_above();
        break;
    case Domi::PUT_DEPTH:
        put_depth();
        break;
    case Domi::PUT_GRASP:
        put_grasp();
        break;
    case Domi::PUT_REVERSE_DEPTH:
        put_reverse_depth();
        break;
    }
    
    ++iter;
}

void Domi::move_to_rest() {

    if (iter == 0) {
        set_des_pos_ori_duration(x_rest, y_rest, z_above, 0, 5);
    }

    control_pos_ori();
    
   if (!has_error()) {
        state = Domi::REST;
        rest_end = get_time() + 10;
        std::cout<<"\nMOVE_TO_REST -> REST\n";
   }
}

void Domi::rest() {
    // Hold current position and orientation
    control_pos_ori();

    if (get_time() >= rest_end) {
        state = Domi::OBSERVATION;
        std::cout<<"\nREST -> OBSERVATION\n";
    }
}

void Domi::observation() {
    std::cout<<".";
    if (!py_fp) {
        py_fp = popen("python DomiBot/cv/cv.py", "r");
        if (!py_fp) {
            std::cout<<"ERROR with popen\n";
            return;
        }
        
        // Set fgets to not block
        int fd = fileno(py_fp);
        int flags = fcntl(fd, F_GETFL, 0);
        flags |= O_NONBLOCK;
        fcntl(fd, F_SETFL, flags);
    }
    
    // Hold current position and orientation
    control_pos_ori();
    
    // Check for python output
    // Assume all output is on one line
    if (fgets(py_out, sizeof(py_out), py_fp)) {
    
        char *saveptr;
        char *domino_end_1 = strtok_r(py_out, ",", &saveptr);
        char *domino_end_2 = strtok_r(NULL, ",", &saveptr);
        x_get = atof(strtok_r(NULL, ",", &saveptr));
        y_get = atof(strtok_r(NULL, ",", &saveptr));
        ori_get = atof(strtok_r(NULL, ",", &saveptr));
        x_put = atof(strtok_r(NULL, ",", &saveptr));
        y_put = atof(strtok_r(NULL, ",", &saveptr));
        ori_put = atof(strtok_r(NULL, ",", &saveptr));

        std::cout<<"\ndomino ["<<domino_end_1<<"|"<<domino_end_2<<"] from ("<<x_get<<","<<y_get<<","<<ori_get<<") to ("<<x_put<<","<<y_put<<","<<ori_put<<")\n";
        
        // x_get = 0.27;
        // y_get = -0.516;
        // ori_get = -1.97;
        // x_put = 0.15;
        // y_put = -0.608;
        // ori_put = 0;

        pclose(py_fp);
        py_fp = NULL;

        if (x_get == -1) {
            // No more moves. Go back to rest.
            state = Domi::MOVE_TO_REST;
            set_des_pos_ori_duration(x_rest, y_rest, z_above, 0, 5);
            std::cout<<"\nOBSERVATION -> MOVE_TO_REST\n";
        } else {
            set_des_pos_ori_duration(x_get, y_get, z_above, ori_get, 5);
            state = Domi::GET_ABOVE;
            std::cout<<"\nOBSERVATION -> GET_ABOVE\n";
        }
    }
}

void Domi::get_above() {

    control_pos_ori();

    if (!has_error(0.01, 0.01) && !has_ori_error(0.02)) {
        state = Domi::GET_DEPTH;
        set_des_pos_ori_duration(x_get, y_get, z_depth, ori_get, 5);
        std::cout<<"\nGET_ABOVE -> GET_DEPTH\n";
    }
}

void Domi::get_depth() {

    control_pos_ori();

    if (!has_error(0.007, 0.01)) {
        state = Domi::GET_GRASP;
        set_des_pos_ori_duration(x_get, y_get, z_depth, ori_get, 5);
	    rest_end = get_time() + 2;
        std::cout<<"\nGET_DEPTH -> GET_GRASP\n";
    }
}

void Domi::get_grasp() {

    control_pos_ori();
    schunkGripper->SetDesiredPosition(35, 0, 100);

    if (get_time() >= rest_end) {
        state = Domi::GET_REVERSE_DEPTH;
        set_des_pos_ori_duration(x_get, y_get, z_above, ori_get, 5);
        std::cout<<"\nGET_GRASP -> GET_REVERSE_DEPTH\n";
    }
}

void Domi::get_reverse_depth() {

    control_pos_ori();

    if (!has_error(0.01, 0.01)) {
        state = Domi::PUT_ABOVE;
        set_des_pos_ori_duration(x_put, y_put, z_above, ori_put, 5);
        rest_end = get_time() + 2;
        std::cout<<"\nGET_REVERSE_DEPTH -> PUT_ABOVE\n";
   }
}

void Domi::put_above() {

    control_pos_ori();

    if (!has_error(0.01, 0.01) && !has_ori_error(0.02)) {
        state = Domi::PUT_DEPTH;
        set_des_pos_ori_duration(x_put, y_put, z_depth, ori_put, 5);
        std::cout<<"\nPUT_ABOVE -> PUT_DEPTH\n";
    }
}

void Domi::put_depth() {

    control_pos_ori();

    if (!has_error(0.007, 0.01)) {
        state = Domi::PUT_GRASP;
        set_des_pos_ori_duration(x_put, y_put, z_depth, ori_put, 5);
        rest_end = get_time() + 2;
        std::cout<<"\nPUT_DEPTH -> PUT_GRASP\n";
    }
}

void Domi::put_grasp() {

    control_pos_ori();
    schunkGripper->SetDesiredPosition(50, 0, 100);

    if (get_time() >= rest_end) {
        state = Domi::PUT_REVERSE_DEPTH;
        set_des_pos_ori_duration(x_put, y_put, z_above, ori_put, 5);
        std::cout<<"\nGET_GRASP -> GET_REVERSE_DEPTH\n";
    }
}

void Domi::put_reverse_depth() {

    control_pos_ori();

    if (!has_error(0.01, 0.01)) {
        state = Domi::MOVE_TO_REST;
        set_des_pos_ori_duration(x_rest, y_rest, z_above, 0, 5);
        std::cout<<"\nPUT_REVERSE_DEPTH -> MOVE_TO_REST\n";
   }
}


void Domi::set_des_pos_ori_duration(double x, double y, double z, double ori, double duration) {
    x_des(0) = x;
    x_des(1) = y;
    x_des(2) = z;
    R_des(0,0) = 0;  R_des(0,1) = -1; R_des(0,2) = 0;
    R_des(1,0) = -1; R_des(1,1) = 0;  R_des(1,2) = 0;
    R_des(2,0) = 0;  R_des(2,1) = 0;  R_des(2,2) = -1; // = pointing straight down

    // Setting the desired angle for joint 7

    ori_des = -ori;

    // Keeping it inside the range

    if (ori_des > M_PI){
        ori_des -= 2*M_PI;
    }
    else if (ori_des < -M_PI){
        ori_des += 2*M_PI;
    }

    if (ori_des > 174.0f * M_PI / 180.0f){
        ori_des = 174.0f * M_PI / 180.0f;
    }
    else if (ori_des < -174.0f * M_PI / 180.0f){
        ori_des = -174.0f * M_PI / 180.0f;
    }


    t_init = get_time();
    t_dur = duration;
    t_elap = 0;
}

void Domi::control_pos_ori() {
    dyn_scl.computeGCModel(&rio.sensors_,&rgcm);

    // Current position and orientation
    x = rwrist->T_o_lnk_ * hpos;
    R = rhand->T_o_lnk_.rotation();
    ori_cur = get_z_rot();

    if (t_elap == 0) {
        // We just started a new trajectory. Get x_init
        x_init = x;
        ori_init = ori_cur;
    }
    
    // Compute your Jacobians
    // hand - control orientation
    dyn_scl.computeJacobianWithTransforms(J_hand,*rhand,rio.sensors_.q_,hpos);
    Jhv = J_hand.block(0,0,3,rio.dof_);
    Jhw = J_hand.block(3,0,3,rio.dof_);

    // wrist - control position
    dyn_scl.computeJacobianWithTransforms(J_wrist,*rwrist,rio.sensors_.q_,hpos);
    Jwv = J_wrist.block(0,0,3,rio.dof_);
    
    // Current velocity
    dx = Jwv * rio.sensors_.dq_;
    
    // Trajectory
    if (t_dur == 0) {
        x_via = x_des;
        ori_via = ori_des;
    } else {
        t_elap = std::min(get_time() - t_init, t_dur);
        x_via(0) = x_init(0) + (3/std::pow(t_dur,2))*(x_des(0) - x_init(0))*std::pow(t_elap,2) - (2/std::pow(t_dur,3))*(x_des(0) - x_init(0))*std::pow(t_elap,3);
        x_via(1) = x_init(1) + (3/std::pow(t_dur,2))*(x_des(1) - x_init(1))*std::pow(t_elap,2) - (2/std::pow(t_dur,3))*(x_des(1) - x_init(1))*std::pow(t_elap,3);
        x_via(2) = x_init(2) + (3/std::pow(t_dur,2))*(x_des(2) - x_init(2))*std::pow(t_elap,2) - (2/std::pow(t_dur,3))*(x_des(2) - x_init(2))*std::pow(t_elap,3);   

        // Joint 7 trajectory generation
        ori_via = ori_init + (3/std::pow(t_dur,2))*(ori_des - ori_init)*std::pow(t_elap,2) - (2/std::pow(t_dur,3))*(ori_des - ori_init)*std::pow(t_elap,3);     
    }

    // Current angular velocity
    w = Jhw * rio.sensors_.dq_;
    
    // Rotation in the x axis 
//    R_des(0,0) = 1.0; R_des(0,1) = 0.0;       R_des(0,2) = 0.0;
//    R_des(1,0) = 0.0; R_des(1,1) = cos(M_PI); R_des(1,2) = -sin(M_PI);
//    R_des(2,0) = 0.0; R_des(2,1) = sin(M_PI); R_des(2,2) = cos(M_PI);
    
    // Use a potential function to enforce joint and joint velocity limits
    q_limit_gain = Eigen::VectorXd::Zero(7);
    dq_limit_gain = Eigen::VectorXd::Zero(7);

    for (int i = 0; i < 7; i = i + 1) {
        if (rio.sensors_.q_(i) >= q_lim(i)) {
            q_limit_gain(i) =  q_lim(i) -  rio.sensors_.q_(i);
            std::cout<<"\n"<<"q limit reached";
        } else if (rio.sensors_.q_(i) <= -q_lim(i)) {
            q_limit_gain(i) =  -q_lim(i) -  rio.sensors_.q_(i);
            std::cout<<"\n"<<"q limit reached";
        }

        if (rio.sensors_.dq_(i) >= dq_lim(i)) {
            dq_limit_gain(i) =  dq_lim(i) -  rio.sensors_.dq_(i);
            std::cout<<"\n"<<"dq limit reached";
        } else if (rio.sensors_.dq_(i) <= -dq_lim(i)) {
            dq_limit_gain(i) =  -dq_lim(i) -  rio.sensors_.dq_(i);
            std::cout<<"\n"<<"dq limit reached";
        }
    }
    
    // Angular error vector
    d_phi = -0.5*(R.col(0).cross(R_des.col(0)) + R.col(1).cross(R_des.col(1)) + R.col(2).cross(R_des.col(2)) );
    d_phi(2) = 0;

    // Operational space Inertia matrix
    M = Jwv * Jwv.transpose();  // A is identity!!!
    Lambda = M.inverse();
    MO = Jhw * Jhw.transpose();
    LambdaO = MO.inverse();

    // Null space damping of wrist position to stop it drooping
    Jwv_bar = Jwv.transpose()*Lambda;
    N = Eigen::MatrixXd::Identity(7,7) - Jwv.transpose()*Jwv_bar.transpose();

    // Operational space controller force
    F = Lambda * (kp * (x_via - x) - kv * dx);
    FO = LambdaO * (kpO * (-d_phi) - kvO * w);      // coupling effects of position controller with orientation

    // Joint torques
    Gamma = Jwv.transpose() * F; 
    Gamma(5) = 0;                                   // set 2nd last joint position control command = 0 then apply orientation control
    Gamma += Jhw.transpose()*FO;


    // Setting up the control for the last joint
    if (state == Domi::GET_ABOVE || state == Domi::PUT_ABOVE) {
        Gamma(6) = kpO * (ori_via - ori_cur) - kvO * rio.sensors_.dq_(6);
    }

    // Joint space control
    Gamma += /*N */ kq * - rio.sensors_.q_ - kdq * rio.sensors_.dq_ + kq_lim * q_limit_gain + kdq_lim * dq_limit_gain;
    
    // Set hard constraints on torque limits
    for (int i = 0; i < 7; i = i + 1) {
        if (Gamma(i) >= torque_lim(i)) {
            Gamma(i) = torque_lim(i);
            // std::cout<<"\n"<<"torque limit reached";
        }
        else if (Gamma(i) <= -torque_lim(i)) {
            Gamma(i) = -torque_lim(i);
            // std::cout<<"\n"<<"torque limit reached";
        }
    }
    if (iter % 1000 == 0){
        std::cout<<"\n"<< Gamma.transpose();
        Eigen::Vector3d x_err = (rwrist->T_o_lnk_ * hpos) - x_des;
        std::cout<<"\n"<<"Via Point:     \t"<< x_via.transpose();
        std::cout<<"\n"<<"Position Error:\t"<< x_err.transpose();
        std::cout<<"\n"<<"Angular Error: \t"<<d_phi.transpose();
        std::cout<<"\n"<<"Position:      \t"<<x.transpose();
        std::cout<<"\n"<<"Angle:      \t"<<ori_cur;
        std::cout<<"\n";
    }

    // Gamma *= 0;
    // Gamma -= kdq * rio.sensors_.dq_;
    
    // Apply gravity compensation after torque limits
    if (is_simulator) {
        Gamma -= rgcm.force_gc_grav_;
    } else {
        Gamma += Jhv.transpose()*g;
    }


    // Send the torque command to the robot
    rio.actuators_.force_gc_commanded_ = Gamma;
}

double Domi::get_z_rot() {
    // Extract z angle from rotation matrix. Assume R is approximately a rotation about the z axis.
    double ori = std::asin(R(1,1));
    if (R(0,1) > 0) {
        ori = M_PI - ori;
    }
    if (ori > M_PI) {
        ori -= 2*M_PI;
    }
    return ori;
}

bool Domi::has_error(double tol_pos, double tol_ori) {
    
    Eigen::Vector3d x_err = (rwrist->T_o_lnk_ * hpos) - x_des;
    if (std::abs(x_err(0)) > tol_pos ||
        std::abs(x_err(1)) > tol_pos ||
        std::abs(x_err(2)) > tol_pos) {
        return true;
    }
    
    Eigen::Matrix3d R = rhand->T_o_lnk_.rotation();
    Eigen::Vector3d d_phi = -0.5*(R.col(0).cross(R_des.col(0)) + R.col(1).cross(R_des.col(1)) + R.col(2).cross(R_des.col(2)));
    if (std::pow(d_phi(0),2) + std::pow(d_phi(1),2) > tol_ori)  {
        return true;
    }
    
    return false;
}

bool Domi::has_ori_error(double tol) {
    return std::abs(get_z_rot() - ori_des) > tol;
}

inline double Domi::get_time() {
    return is_simulator ? iter*dt : sutil::CSystemClock::getSysTime();
}

void Domi::compliant_control() {
    dyn_scl.computeGCModel(&rio.sensors_,&rgcm);

    if (iter == 0) {
        schunkGripper->SetDesiredPosition(50, 0, 100);
        R_des(0,0) = 0;  R_des(0,1) = -1; R_des(0,2) = 0;
        R_des(1,0) = -1; R_des(1,1) = 0;  R_des(1,2) = 0;
        R_des(2,0) = 0;  R_des(2,1) = 0;  R_des(2,2) = -1;
    }

    // Current position and orientation
    x = rwrist->T_o_lnk_ * hpos;
    R = rhand->T_o_lnk_.rotation();
    ori_cur = get_z_rot();

    // Compute your Jacobians
    // hand - control orientation
    dyn_scl.computeJacobianWithTransforms(J_hand,*rhand,rio.sensors_.q_,hpos);
    Jhv = J_hand.block(0,0,3,rio.dof_);
    Jhw = J_hand.block(3,0,3,rio.dof_);

    // Current angular velocity
    w = Jhw * rio.sensors_.dq_;
    
    // Angular error vector
    d_phi = -0.5*(R.col(0).cross(R_des.col(0)) + R.col(1).cross(R_des.col(1)) + R.col(2).cross(R_des.col(2)) );
    d_phi(2) = 0;

    // Operational space Inertia matrix
    MO = Jhw * Jhw.transpose();
    LambdaO = MO.inverse();

    // Operational space controller force
    FO = LambdaO * (kpO * (-d_phi) - kvO * w);      // coupling effects of position controller with orientation

    // Joint torques
    Gamma = Jhw.transpose()*FO;

    // Joint space control
    Gamma += /*N */ kq * - rio.sensors_.q_ - kdq * rio.sensors_.dq_;
    
    // Set hard constraints on torque limits
    for (int i = 0; i < 7; i = i + 1) {
        if (Gamma(i) >= torque_lim(i)) {
            Gamma(i) = torque_lim(i);
            // std::cout<<"\n"<<"torque limit reached";
        }
        else if (Gamma(i) <= -torque_lim(i)) {
            Gamma(i) = -torque_lim(i);
            // std::cout<<"\n"<<"torque limit reached";
        }
    }
    if (iter % 1000 == 0){
        std::cout<<"\n"<< Gamma.transpose();
        std::cout<<"\n"<<"Position:      \t"<<x.transpose();
        std::cout<<"\n"<<"Angle:      \t"<<ori_cur;
        std::cout<<"\n";
    }

    // Gamma *= 0;
    // Gamma -= kdq * rio.sensors_.dq_;
    
    // Apply gravity compensation after torque limits
    if (is_simulator) {
        Gamma -= rgcm.force_gc_grav_;
    } else {
        Gamma += Jhv.transpose()*g;
    }


    // Send the torque command to the robot
    rio.actuators_.force_gc_commanded_ = Gamma;

    ++iter;
}
