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

Domi::Domi(scl::SGcModel &_rgcm,
           scl::SRobotIO &_rio,
           scl::CDynamicsScl &_dyn_scl,
           scl::SRigidBodyDyn *_rhand,
           scl::SRigidBodyDyn *_rwrist,
           const Eigen::Vector3d &_hpos,
           bool _is_simulator) :
    state(Domi::OBSERVATION),
    rgcm(_rgcm),
    rio(_rio),
    dyn_scl(_dyn_scl),
    rhand(_rhand),
    rwrist(_rwrist),
    hpos(_hpos),
    is_simulator(_is_simulator),
    py_fp(NULL) {}

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
}

void Domi::move_to_rest() {
    // TODO: CHANGE ME
    x_des(0) = 0;
    x_des(1) = 0;
    x_des(2) = 0;
    R_des;
    
    control_pos_ori();
    
    if (!has_error()) {
        state = Domi::REST;
        rest_end = sutil::CSystemClock::getSysTime() + 10;
    }
}

void Domi::rest() {
    if (sutil::CSystemClock::getSysTime() >= rest_end) {
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
    
}

void Domi::get_depth() {
    
}

void Domi::get_grasp() {
    
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

void Domi::control_pos_ori() {
    /*
	// We compute all the transformations and the jacobian of a point on
	// the end-effector
	dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);
	Eigen::Vector3d hpos(0.0,0.0,0.0);
	Eigen::MatrixXd J;
	Eigen::Vector3d x_des;
	dyn_scl.computeJacobian(J,*end_effector,rio.sensors_.q_,hpos);

	// Compute position of end-effector
	Eigen::Vector3d x(3);
	x = end_effector->T_o_lnk_*hpos;

	// First time initialization, this gets called again if robot leaves
	// operating mode
	if(first_time)
	{
		x_init = end_effector->T_o_lnk_*hpos;
		first_time = false;
	}

	tcurr = tcurr + 0.002;
	// radius of circle and frequency

	double ampl = 0.1;
	double freq = 0.2; 
	
	// Computing the desired trajectory

	x_des(0) = x_init(0) + ampl * sin(tcurr * freq * 2 * 3.14159);
	x_des(1) = x_init(1) + ampl * cos(tcurr * freq * 2 * 3.14159);
	x_des(2) = x_init(2) - 0.1;

	// Compute velocity of end-effector
	Eigen::Vector3d dx;
	Eigen::MatrixXd Jv = J.block(0,0,3,LBRState::NUMBER_OF_JOINTS);
	dx = Jv*rio.sensors_.dq_;


	rio.actuators_.force_gc_commanded_ = Jv.transpose() * (800*(x_des-x) - 30*dx);
	rio.actuators_.force_gc_commanded_ += -2.0*rio.sensors_.dq_;
    */
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
