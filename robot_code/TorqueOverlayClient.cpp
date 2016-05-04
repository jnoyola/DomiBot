// version: 1.7
/**

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," 
without warranty of any kind, including without limitation the warranties 
of merchantability, fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable 
for any particular purpose. In no event shall KUKA be responsible for loss 
or damages arising from the installation or use of the Software, 
including but not limited to any indirect, punitive, special, incidental 
or consequential damages of any character including, without limitation, 
damages for loss of goodwill, work stoppage, computer failure or malfunction, 
or any and all other commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by KUKA. 
Should the Software prove defective, KUKA is not liable for the entire cost 
of any service and repair.


COPYRIGHT

All Rights Reserved
Copyright (C)  2015 
KUKA Roboter GmbH
Augsburg, Germany

This material is the exclusive property of KUKA Roboter GmbH and must be returned 
to KUKA Roboter GmbH immediately upon request.  
This material and the information illustrated or contained herein may not be used, 
reproduced, stored in a retrieval system, or transmitted in whole 
or in part in any way - electronic, mechanical, photocopying, recording, 
or otherwise, without the prior written consent of KUKA Roboter GmbH.  



*/
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "TorqueOverlayClient.h"


#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#include <vector>
#include <chrono>
#include <iostream>

// scl_lib
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <Eigen/Dense>
// SCL variables defined globally for ease of use
scl::SRobotParsed rds;     //Robot data structure....
scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
scl::SRobotIO rio;         //I/O data structure
scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
scl::CParserScl p;         //This time, we'll parse the tree from a file...
scl::SRigidBodyDyn *end_effector;

// Print counter
int print = 0;

// Function for velocity filtering
// instantaneous joint velocity buffer for filter use
const size_t DQ_BUFF_SIZE = 30; //We store a few more than what is needed by the filter
const size_t DQ_FILTER_WINDOW = 10;
static std::chrono::high_resolution_clock::time_point t_last;
std::vector<Eigen::VectorXd> dq_buff;
void filter_MA(std::vector<Eigen::VectorXd> &buffer, Eigen::VectorXd &ret_vector, uint data_size, uint window_size);

// Constructor (called once before connection)
TorqueOverlay::TorqueOverlay() 
{
	printf("Hold position initialized\n");
	for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ _torques[i] = 0.0;}

	// Initialize scl
	bool flag = p.readRobotFromFile(
		"specs/kuka_iiwa/iiwaCfg.xml",
		"specs/","iiwaBot",rds);
	flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
	flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
	flag = flag && rio.init(rds);             //Set up the I/O data structure

	// Save pointer to the end effector
	end_effector = rgcm.rbdyn_tree_.at("end-effector");
	printf("Done init\n");
}

// Main loop, called each time there is an update
// 
// At this point, the rio data structures have the current values
// and the dynamic quantities have been computed.
// 
// You need only set rio.actuators_.force_gc_commanded_ with the
// torque values you want
//
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
// 
// Do not add gravity to the commanded torque, the robot does its own
// gravity compensation.
//
// Also, do not use the mass matrix (joint space or op space). The specs
// have not been properly calibrated, but "Identity" should be fine.
//
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING

bool first_time = true;
double tcurr = 0;
Eigen::Vector3d x_init;
void TorqueOverlay::mainloop()
{
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


	// Simple joint space control
	/*
	Eigen::VectorXd q_init(LBRState::NUMBER_OF_JOINTS);
	for(int i=0; i < LBRState::NUMBER_OF_JOINTS; i++)
	{
		q_init(i) = _init_joint_pos[i];
	}
	rio.actuators_.force_gc_commanded_ = 50.0*(q_init-rio.sensors_.q_)-10.0*rio.sensors_.dq_;
	*/

	/*
	if((++print) >= 200)
	{
		print = 0;
		//std::cout << dt << "\n";
		std::cout << "-------------------------------\n";
		//std::cout << new_q.transpose() << "\n";
		//std::cout << new_dq.transpose() << "\n";
		std::cout << x_init.transpose() << "\n";
		std::cout << x.transpose() << "\n";
		//std::cout << rio.actuators_.force_gc_commanded_.transpose() << "\n";
	}
	//*/
}

















//////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////
TorqueOverlay::~TorqueOverlay()
{
}

void TorqueOverlay::waitForCommand()
{
   // In waitForCommand(), the joint values have to be mirrored. Which is done, by calling
   // the base method.
   LBRClient::waitForCommand();
   
   // If we want to command torques, we have to command them all the time; even in
   // waitForCommand(). This has to be done due to consistency checks. In this state it is 
   // only necessary, that some torque values are sent. The LBR does not take the 
   // specific value into account.
   if (robotState().getClientCommandMode() == TORQUE)
   {
      robotCommand().setTorque(_torques);
   }
}

void TorqueOverlay::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   switch (newState)
   {
      case COMMANDING_ACTIVE:
      {
        printf("Controller active\n");
				t_last = std::chrono::high_resolution_clock::now(); 
        memcpy(_init_joint_pos, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
        break;
      }
      default:
      {
        printf("Controller deactivated\n");
				first_time = true;
        for (int i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
        {
          _torques[i] = 0.0;
        }
        break;
      }
   }
}

void filter_MA(std::vector<Eigen::VectorXd> &buffer, Eigen::VectorXd &ret_vector, uint data_size, uint window_size) {
  ret_vector.setZero(data_size);
  // if buffer is empty, return 0
  if (buffer.size() < window_size) {
    return;
  }
  for (uint i = buffer.size() - window_size; i < buffer.size(); ++i) {
    ret_vector += buffer[i];
  }
  ret_vector /= static_cast<double>(window_size);
}

void TorqueOverlay::command()
{
	// Computed elapsed time since last tick
	auto t_curr = std::chrono::high_resolution_clock::now();
	const double kMicrosInSec = 1000000.0;
	double dt = std::chrono::duration_cast<std::chrono::microseconds>(t_curr-t_last).count()/kMicrosInSec;
	
	if(dt < 0.001) dt = 0.001;

	// Read the position and set the desired position (to prevent stop)
  const double* cur_joint_pos = robotState().getMeasuredJointPosition();
  robotCommand().setJointPosition(cur_joint_pos);

	// Create eigen variables
	Eigen::VectorXd new_q(LBRState::NUMBER_OF_JOINTS);
	for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++)
	{
		new_q(i) = cur_joint_pos[i];
		if(i == 3) new_q(i) *= -1;
	}
	if(first_time)
	{
		rio.sensors_.q_ = new_q;
		// This gets changed to false in mainloop
	}

	// Compute raw velocity and do filtering
	Eigen::VectorXd raw_dq(LBRState::NUMBER_OF_JOINTS);
	raw_dq = (new_q - rio.sensors_.q_)/dt;
	dq_buff.push_back(raw_dq);
	if(dq_buff.size() > DQ_BUFF_SIZE) dq_buff.erase(dq_buff.begin());
	Eigen::VectorXd new_dq(LBRState::NUMBER_OF_JOINTS);
	filter_MA(dq_buff, new_dq, LBRState::NUMBER_OF_JOINTS, DQ_FILTER_WINDOW);

	// State is now new_q and new_dq, update simulator
	rio.sensors_.q_ = new_q;
	rio.sensors_.dq_ = new_dq;

	mainloop();

  // Check for correct ClientCommandMode.
  if (robotState().getClientCommandMode() == TORQUE)
  { 
    for (int i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
    {
      _torques[i] = rio.actuators_.force_gc_commanded_(i);
			if(i == 3) _torques[i] *= -1;
    }
    // Set superposed joint torques.
    robotCommand().setTorque(_torques);
  }

	t_last = t_curr;
}


