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

Domi::Domi(scl::SGcModel &_rgcm,                // Robot data structure with dynamic quantities 
           scl::SRobotIO &_rio,                 // I/O data structure
           scl::CDynamicsScl &_dyn_scl,         // Robot Kinematics and dynamics computation object
           scl::SRigidBodyDyn *_rhand,          // Orientation (last link)
           scl::SRigidBodyDyn *_rwrist,         // Position     (second to last link)
           const Eigen::Vector3d &_hpos,
           double _dt)
           :
           state(Domi::GET_ABOVE),
           rgcm(_rgcm),
           rio(_rio),
           dyn_scl(_dyn_scl),
           rhand(_rhand),
           rwrist(_rwrist),
           hpos(_hpos),
           dt(_dt),
           iter(0),
           py_fp(NULL) {
           
    is_simulator = (dt > 0);
    
    q_lim = Eigen::VectorXd::Zero(7);
    dq_lim = Eigen::VectorXd::Zero(7);
    torque_lim = Eigen::VectorXd::Zero(7);
           
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
    } else {
        kpO = 150;
        kvO = 0;
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
    // TODO: CHANGE ME
//    x_des(0) = 0.000884997;
//    x_des(1) = -0.605546;
//    x_des(2) = 0.613931;
//    R_des(0,0) = 0.0015229; R_des(0,1) = -0.999999;       R_des(0,2) = -0.00577387;
//    R_des(1,0) = -0.995643; R_des(1,1) = -0.0015701; R_des(1,2) = 0.0932387;
//    R_des(2,0) = -0.0932395; R_des(2,1) = 0.000432878; R_des(2,2) = -0.995644;

    control_pos_ori();
    
//    if (!has_error()) {
//        state = Domi::REST;
//        rest_end = get_time() + 10;
//    }
}

void Domi::rest() {
    if ((is_simulator && iter >= rest_end) ||
        (!is_simulator && sutil::CSystemClock::getSysTime() >= rest_end)) {
        state = Domi::OBSERVATION;
        return;
    }
    
    // Hold current position and orientation
    control_pos_ori();
}

void Domi::observation() {
    std::cout<<".";
    if (!py_fp) {
        py_fp = popen("python ../cv/cv.py", "r");
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
        double x_get = atof(strtok_r(NULL, ",", &saveptr));
        double y_get = atof(strtok_r(NULL, ",", &saveptr));
        double ori_get = atof(strtok_r(NULL, ",", &saveptr));
        double x_put = atof(strtok_r(NULL, ",", &saveptr));
        double y_put = atof(strtok_r(NULL, ",", &saveptr));
        double ori_put = atof(strtok_r(NULL, ",", &saveptr));
        
        std::cout<<"\ndomino ["<<domino_end_1<<"|"<<domino_end_2<<"] from ("<<x_get<<","<<y_get<<","<<ori_get<<") to ("<<x_put<<","<<y_put<<","<<ori_put<<")\n";
        
        // TODO: use output from CV
        
        state = Domi::GET_ABOVE;
    }
}

void Domi::get_above() {

    if (iter == 0) {
        set_des_pos_ori_duration(0, -0.7, 0.5, 0, 10);
    }

    control_pos_ori();

    if (!has_error(0.005, 0.001)) {
        state = Domi::GET_DEPTH;
        set_des_pos_ori_duration(0, -0.7, 0.3, 0, 10);
   }
}

void Domi::get_depth() {
    control_pos_ori();

    if (!has_error(0.005, 0.001)) {
        state = Domi::GET_GRASP;
        set_des_pos_ori_duration(0, -0.7, 0.5, 0, 10);
   }
}

void Domi::get_grasp() {
    control_pos_ori();
}

void Domi::get_reverse_depth() {
    
}

void Domi::put_above() {
    
}

void Domi::put_depth() {
    
}

void Domi::put_grasp() {
    
}

void Domi::put_reverse_depth() {
    
}

void Domi::set_des_pos_ori_duration(double x, double y, double z, double ori, double duration) {
    x_des(0) = x;
    x_des(1) = y;
    x_des(2) = z;
    R_des(0,0) = 0;  R_des(0,1) = -1; R_des(0,2) = 0;
    R_des(1,0) = -1; R_des(1,1) = 0;  R_des(1,2) = 0;
    R_des(2,0) = 0;  R_des(2,1) = 0;  R_des(2,2) = -1; // = pointing straight down

    //TODO: multiply by z rotation as a function of desired ori

    t_init = get_time();
    t_dur = duration;
    t_elap = 0;
}

void Domi::control_pos_ori() {
    dyn_scl.computeGCModel(&rio.sensors_,&rgcm);

    if (t_elap == 0) {
        // We just started a new trajectory. Get x_init
        x_init = rwrist->T_o_lnk_ * hpos;
    }
    
    // Compute your Jacobians
    // hand - control orientation
    dyn_scl.computeJacobianWithTransforms(J_hand,*rhand,rio.sensors_.q_,hpos);
    Jhv = J_hand.block(0,0,3,rio.dof_);
    Jhw = J_hand.block(3,0,3,rio.dof_);
    // wrist - control position
    dyn_scl.computeJacobianWithTransforms(J_wrist,*rwrist,rio.sensors_.q_,hpos);
    Jwv = J_wrist.block(0,0,3,rio.dof_);
    
    // Current position and velocity
    x = rwrist->T_o_lnk_ * hpos;
    dx = Jwv * rio.sensors_.dq_;
    
    // Trajectory
    if (t_dur == 0) {
        x_via = x_des;
    } else {
        t_elap = std::min(get_time() - t_init, t_dur);
        x_via(0) = x_init(0) + (3/std::pow(t_dur,2))*(x_des(0) - x_init(0))*std::pow(t_elap,2) - (2/std::pow(t_dur,3))*(x_des(0) - x_init(0))*std::pow(t_elap,3);
        x_via(1) = x_init(1) + (3/std::pow(t_dur,2))*(x_des(1) - x_init(1))*std::pow(t_elap,2) - (2/std::pow(t_dur,3))*(x_des(1) - x_init(1))*std::pow(t_elap,3);
        x_via(2) = x_init(2) + (3/std::pow(t_dur,2))*(x_des(2) - x_init(2))*std::pow(t_elap,2) - (2/std::pow(t_dur,3))*(x_des(2) - x_init(2))*std::pow(t_elap,3);
    }

    // Current orientation and angular velocity
    R = rhand->T_o_lnk_.rotation();
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
    Gamma +=  Jhw.transpose()*FO;

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
        std::cout<<"\n"<<"Position Error:\t"<< x_err.transpose();
        std::cout<<"\n"<<"Angular Error: \t"<<d_phi.transpose();
        std::cout<<"\n"<<"Position:      \t"<<x.transpose();
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

bool Domi::has_error(double tol_pos, double tol_ori) {
    
    Eigen::Vector3d x_err = (rwrist->T_o_lnk_ * hpos) - x_des;
    if (std::abs(x_err(0)) > tol_pos ||
        std::abs(x_err(1)) > tol_pos ||
        std::abs(x_err(2)) > tol_pos) {
        return true;
    }
    
    Eigen::Matrix3d R = rhand->T_o_lnk_.rotation();
    Eigen::Vector3d d_phi = -0.5*(R.col(0).cross(R_des.col(0)) + R.col(1).cross(R_des.col(1)) + R.col(2).cross(R_des.col(2)));
    if (std::abs(d_phi(0)) > tol_ori ||
        std::abs(d_phi(1)) > tol_ori ||
        std::abs(d_phi(2)) > tol_ori) {
        return true;
    }
    
    return false;
}

inline double Domi::get_time() {
    return is_simulator ? iter*dt : sutil::CSystemClock::getSysTime();
}
