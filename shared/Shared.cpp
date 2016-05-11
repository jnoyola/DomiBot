#include "Shared.h"

#include <cmath>

// Initialization
Shared::State Shared::state = Shared::MOVE_TO_REST;
Eigen::Vector3d Shared::hpos(0,0,0.05);

void Shared::mainloop(scl::SGcModel rgcm,
                      scl::SRobotIO rio,
                      scl::CDynamicsScl dyn_scl,
                      scl::SRigidBodyDyn *rhand,
                      scl::SRigidBodyDyn *rwrist) {

    switch (state) {
    case Shared::MOVE_TO_REST:
        move_to_rest(rgcm, rio, dyn_scl, end_effector);
        break;
    case Shared::REST:
        rest();
        break;
    }
}

void Shared::move_to_rest(scl::SGcModel rgcm,
                          scl::SRobotIO rio,
                          scl::CDynamicsScl dyn_scl,
                          scl::SRigidBodyDyn *rhand,
                          scl::SRigidBodyDyn *rwrist) {
    // TODO: CHANGE ME
    x_des(0) = 0;
    x_des(1) = 0;
    x_des(2) = 0;
    R_des;
    
    if (has_no_error(rhand, rwrist)) {
        state = Shared::REST;
        return;
    }
    
    control_pos_ori(rgcm, rio, dyn_scl, end_effector);
}

void Shared::rest() {
    // Move to OBSERVATION state after 10 seconds
}

bool Shared::has_no_error(scl::SRigidBodyDyn *rhand,
                          scl::SRigidBodyDyn *rwrist,
                          double tol_pos = 0.01,
                          double tol_ori = 0.01) {
    
    Eigen::Vector3d x_err = (rwrist->T_o_lnk_ * hpos) - x_des;
    if (abs(x_err(0)) > tol_pos ||
        abs(x_err(1)) > tol_pos ||
        abs(x_err(2)) > tol_pos) {
        return false;
    }
    
    R = rhand->T_o_lnk_.rotation();
    Eigen::Vector3d d_phi = -0.5*(R.col(0).cross(R_des.col(0)) + R.col(1).cross(R_des.col(1)) + R.col(2).cross(R_des.col(2)));
    if (abs(d_phi(0)) > tol_ori ||
        abs(d_phi(1)) > tol_ori ||
        abs(d_phi(2)) > tol_ori) {
        return false;
    }
    
    return true;
}

void Shared::control_pos_ori(scl::SGcModel rgcm,
                             scl::SRobotIO rio,
                             scl::CDynamicsScl dyn_scl,
                             scl::SRigidBodyDyn *rhand,
                             scl::SRigidBodyDyn *rwrist) {
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