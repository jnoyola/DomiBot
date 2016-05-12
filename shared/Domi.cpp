#include "Domi.h"

#include <iostream>

#include <fcntl.h>

Domi::Domi(scl::SGcModel &_rgcm,
           scl::SRobotIO &_rio,
           scl::CDynamicsScl &_dyn_scl,
           scl::SRigidBodyDyn *_rhand,
           scl::SRigidBodyDyn *_rwrist,
           Eigen::Vector3d &_hpos,
           bool _is_simulator) :
    state(Domi::MOVE_TO_REST),
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
    }
}

void Domi::move_to_rest() {
    // TODO: CHANGE ME
    x_des(0) = 0;
    x_des(1) = 0;
    x_des(2) = 0;
    R_des;
    
    if (!has_error()) {
        state = Domi::REST;
        return;
    }
    
    control_pos_ori();
}

void Domi::rest() {
    // Move to OBSERVATION state after 10 seconds
}

void Domi::observation() {
    if (!py_fp) {
        py_fp = popen("python ../cv_tests/cv.py", "r");
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
    
    // Assume all output is on one line
    if (fgets(py_out, sizeof(py_out), py_fp)) {
        std::cout<<py_out;
    
        // TODO: use output from CV
        
        state = Domi::GET_ABOVE;
    }
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
