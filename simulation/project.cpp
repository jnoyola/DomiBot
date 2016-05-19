/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

scl is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.
Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

scl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
scl. If not, see <http://www.gnu.org/licenses/>.
 */
/* \file scl_tutorial4_control_gc_op.cpp
 *
 *  Created on: Jul 30, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/util/DatabaseUtils.hpp>

//For timing
#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <thread>

//Freeglut windowing environment
#include <GL/freeglut.h>

 // writing txt files
#include <fstream>

#include "../Shared/Domi.h"

// Define all the variables globally so they can be accessed across threads
scl::SRobotParsed rds;     //Robot data structure....
scl::SGraphicsParsed rgr;  //Robot graphics data structure...
scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
scl::SRobotIO rio;         //I/O data structure
scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
scl::CDynamicsTao dyn_tao; //Robot physics integrator
scl::CParserScl p;         //This time, we'll parse the tree from a file...

// See the bottom for the implementations
void jointSpaceControl();
void opSpacePositionControl();
void opSpaceOrientationControl();
void opSpacePositionOrientationControl();

// These functions are used for threading, you shouldn't need to change them
void graphicsLoop()
{
  while(scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
  {
    glutMainLoopEvent();
    const timespec ts = {0, 15000000};
    nanosleep(&ts,NULL);
  }
}
// A few global variables to make the sleeps consistent
double dt = 0.0001;
long controlSleepTime = 100000;

/** A sample application to demonstrate a physics simulation in scl.
 *
 * Moving forward from tutorial 3, we will now control the 6 DOF
 * demo robot (r6bot) with the physics engine running.
 *
 * SCL Modules used:
 * 1. data_structs
 * 2. dynamics (physics)
 * 4. dynamics (control matrices)
 * 4. graphics (chai)
 * */
int main(int argc, char** argv)
{
  std::cout<<"\n***************************************\n";
  std::cout<<"Standard Control Library Control Types";
  std::cout<<"\n***************************************\n";

  sutil::CSystemClock::start();

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  // bool flag = p.readRobotFromFile("../../specs/Puma/PumaCfg.xml","../../specs/","PumaBot",rds);
  const std::string fname("kuka_iiwa/iiwaCfg.xml");
  bool flag = p.readRobotFromFile(fname,"./","iiwaBot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_tao.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  #ifdef __APPLE__
    flag = flag && rio.init(rds);             //Set up the I/O data structure
  #else
    flag = flag && rio.init(rds.name_,rds.dof_);
  #endif
  for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
  if(false == flag){ return 1; }            //Error check.

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  // flag = p.readGraphicsFromFile("../../specs/Puma/PumaCfg.xml","PumaBotStdView",rgr);
  flag = p.readGraphicsFromFile(fname,"iiwaBotStdView",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/

  /****************************************/
  /********* Controller  ******************/
  /****************************************/
  std::thread controlThread(opSpacePositionOrientationControl);

  // Start up the graphics
  graphicsLoop();
  // Wait for control thread to die
  controlThread.join();

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}

void opSpacePositionOrientationControl()
{
  double tstart, tcurr;
  long iter = 0;
  bool flag = false;

  std::cout<<"\n\n***************************************************************"
      <<"\n Starting op space position + orientation controller..."
      <<"\n***************************************************************\n";
  tstart = sutil::CSystemClock::getSysTime(); iter = 0;

  scl::SRigidBodyDyn *rhand = rgcm.rbdyn_tree_.at("end-effector");
  scl::SRigidBodyDyn *rwrist = rgcm.rbdyn_tree_.at("link_5");

  const Eigen::Vector3d hpos(0,0,0.05); //control position of op-point wrt. hand
  
  /*******************************/
  /******* Domi Shared API *******/
  /*******************************/
  Domi domi(rgcm, rio, dyn_scl, rhand, rwrist, hpos, true);
  
  /*
  while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running) {
    domi.mainloop();
    dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, controlSleepTime}; nanosleep(&ts,NULL);
  }
  */
  
  /*******************************/
  /**** Everything below here ****/
  /**** should be done in the ****/
  /**** Domi mainloop methods ****/
  /*******************************/


  // Define the matrices and vectors you will need
  Eigen::MatrixXd J_hand, J_wrist, Jhv, Jhw, Jwv, Jww, A, A_inv, Lambda, LambdaO, R_init, M, MO;
  Eigen::Matrix3d R, R_des;
  Eigen::Vector3d w, x, x_des, x_init, dx, x_final;
  Eigen::VectorXd Gamma, g, F, FO, q_desired;
  // Eigen::VectorXd joint_lim, torque_lim; 
 

  while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
  {
    // get current time and compute the model
    tcurr = sutil::CSystemClock::getSysTime();
    dyn_scl.computeGCModel(&rio.sensors_,&rgcm);


    // initial position
    if(false == flag) { x_init = rhand->T_o_lnk_ * hpos; flag = true; }
    
    // Compute your Jacobians
    // Jacobians
    // hand - control orientation
    dyn_scl.computeJacobianWithTransforms(J_hand,*rhand,rio.sensors_.q_,hpos);
    Jhv = J_hand.block(0,0,3,rio.dof_);
    Jhw = J_hand.block(3,0,3,rio.dof_);
    // wrist - control position
    dyn_scl.computeJacobianWithTransforms(J_wrist,*rwrist,rio.sensors_.q_,hpos);
    Jwv = J_wrist.block(0,0,3,rio.dof_);
    Jww = J_wrist.block(3,0,3,rio.dof_);

    // Mass Matrix and its inverse    A is the identity
    // A = rgcm.M_gc_;
    // A_inv = rgcm.M_gc_inv_;
    
    // gains    
    double kpO = 200;   // 200
    double kvO = 20;    // 20
    double kp = 150;    // 100
    double kv = 25;     // 30
    double kq = 15;     // 15
    double kdq = 4.0;   // 4
    double kq_lim = 10.0;  // 5
    double kdq_lim = 10.0; // 5
    
    
    // current position and velocity
    x = rwrist->T_o_lnk_ * hpos;
    dx = Jwv * rio.sensors_.dq_;

    // desired position
    double t_1 = 5.0;
    double t_2 = 10.0;
    double t_3 = 15.0;
    double t_4 = 20.0;
    double t_f = 3.0;

    double ti = iter*dt;
    Eigen::Vector3d x_final1, x_final2, x_final3, x_final4;
    x_final1 = Eigen::VectorXd::Zero(3);
    x_final1(0) = 0.4;  x_final1(1) = 0.4;  x_final1(2) = 0.2;
    x_final2 = Eigen::VectorXd::Zero(3);
    x_final2(0) = -0.4;  x_final2(1) = 0.4;  x_final2(2) = 0.2;
    x_final3 = Eigen::VectorXd::Zero(3);
    x_final3(0) = 0.4;  x_final3(1) = 0.4;  x_final3(2) = 0.2;
    x_final4 = Eigen::VectorXd::Zero(3);
    x_final4(0) = -0.4;  x_final4(1) = 0.4;  x_final4(2) = 0.2;

    if (ti <= t_1)
      {t_f = t_1;
       x_final = x_final1;}
    else if (ti <= t_2 && ti > t_1)
      {t_f = t_2;
       x_final = x_final2;
       x_init = x_final1;}
    else if (ti <= t_3 && ti > t_2)
      {t_f = t_3;
       x_final = x_final3;
       x_init = x_final2;}
    else if (ti <= t_4)
      {t_f = t_4;
       x_final = x_final4;
       x_init = x_final3;}
    else if (ti > t_4)
      {ti = t_4; break;}


    x_des(0) = x_init(0) + (3/std::pow(t_f,2))*(x_final(0) - x_init(0))*std::pow(ti,2) - (2/std::pow(t_f,3))*(x_final(0) - x_init(0))*std::pow(ti,3);
    x_des(1) = x_init(1) + (3/std::pow(t_f,2))*(x_final(1) - x_init(1))*std::pow(ti,2) - (2/std::pow(t_f,3))*(x_final(1) - x_init(1))*std::pow(ti,3);
    x_des(2) = x_init(2) + (3/std::pow(t_f,2))*(x_final(2) - x_init(2))*std::pow(ti,2) - (2/std::pow(t_f,3))*(x_final(2) - x_init(2))*std::pow(ti,3);



    // current orientation and angular velocity
    R = rhand->T_o_lnk_.rotation();
    w = Jhw * rio.sensors_.dq_;
    
    // desired orientation
    // Rotation in the y axis 
    // R_des(0,0) = cos(M_PI);  R_des(0,1) = 0.0; R_des(0,2) = sin(M_PI);
    // R_des(1,0) = 0.0;        R_des(1,1) = 1.0; R_des(1,2) = 0.0;
    // R_des(2,0) = -sin(M_PI); R_des(2,1) = 0.0; R_des(2,2) = cos(M_PI);

    // Rotation in the x axis 
    R_des(0,0) = 1.0; R_des(0,1) = 0.0; R_des(0,2) = 0.0;
    R_des(1,0) = 0.0; R_des(1,1) = cos(M_PI); R_des(1,2) = -sin(M_PI);
    R_des(2,0) = 0.0; R_des(2,1) = sin(M_PI); R_des(2,2) = cos(M_PI);

    // Rotation in the z axis 
    // R_des(0,0) = cos(M_PI);  R_des(0,1) = -sin(M_PI); R_des(0,2) = 0.0;
    // R_des(1,0) = sin(M_PI);  R_des(1,1) = cos(M_PI);  R_des(1,2) = 0.0;
    // R_des(2,0) = 0.0;        R_des(2,1) = 0.0;        R_des(2,2) = 1.0;



    // In simulation, we artificially add hard joint and torque constraints
    // In real life, we would use a potential function

    Eigen::VectorXd q_lim, dq_lim, q_limit_gain, dq_limit_gain;
    q_lim = Eigen::VectorXd::Zero(7);
    dq_lim = Eigen::VectorXd::Zero(7);
    q_lim << 160.0*M_PI/180.0, 110.0*M_PI/180.0, 160.0*M_PI/180.0, 110.0*M_PI/180.0, 160.0*M_PI/180.0, 110.0*M_PI/180.0, 165.0*M_PI/180.0;
    dq_lim << 98.0*M_PI/180.0, 98.0*M_PI/180.0, 100.0*M_PI/180.0, 130.0*M_PI/180.0, 140.0*M_PI/180.0, 180.0*M_PI/180.0, 180.0*M_PI/180.0;
    
    q_limit_gain = Eigen::VectorXd::Zero(7);
    dq_limit_gain = Eigen::VectorXd::Zero(7);


    for (int i = 0; i < 7; i = i + 1)
      {
        if (rio.sensors_.q_(i) >= q_lim(i))
          {q_limit_gain(i) =  q_lim(i) -  rio.sensors_.q_(i);
          std::cout<<"\n"<<"q limit reached";}
        if (rio.sensors_.q_(i) <= -q_lim(i))
          {q_limit_gain(i) =  -q_lim(i) -  rio.sensors_.q_(i); 
           std::cout<<"\n"<<"q limit reached";}

        if (rio.sensors_.dq_(i) >= dq_lim(i))
                // {rio.sensors_.dq_(0) = 166.0;
          {dq_limit_gain(i) =  dq_lim(i) -  rio.sensors_.dq_(i);   
            std::cout<<"\n"<<"dq limit reached";}
        if (rio.sensors_.dq_(i) <= -dq_lim(i))
                // {rio.sensors_.dq_(0) = 166.0;
          {dq_limit_gain(i) =  -dq_lim(i) -  rio.sensors_.dq_(i);   
            std::cout<<"\n"<<"dq limit reached";}
           
      }


    // angular error vector
    Eigen::Vector3d d_phi;
    d_phi = -0.5*(R.col(0).cross(R_des.col(0)) + R.col(1).cross(R_des.col(1)) + R.col(2).cross(R_des.col(2)) );

    // Operational space Inertia matrix
    M = Jwv * Jwv.transpose();  // A is identity!!!
    Lambda = M.inverse();
    MO = Jhw * Jhw.transpose();
    LambdaO = MO.inverse();


    // joint space gravity
    g = rgcm.force_gc_grav_;

    // null space damping of wrist position to stop it drooping
    Eigen::MatrixXd Jwv_bar = Jwv.transpose()*Lambda;
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(7,7) - Jwv.transpose()*Jwv_bar.transpose();

    // Operational space controller force
    F = Lambda * (kp * (x_des - x) - kv * dx);
    FO = LambdaO * (kpO * (-d_phi) - kvO * w);
    
    // joint torques    !!! WE DO NOT WANT GRAVITY CONTROL !!!
    Gamma = Eigen::VectorXd::Zero(7);
    Gamma = Jwv.transpose() * F  + Jhw.transpose()*FO - g;

    // send the torque command to the simulated robot
    Eigen::MatrixXd J_bar = Jhw.transpose()*LambdaO;
    // ns_damping = (Eigen::MatrixXd::Identity(6,6) - Jhw.transpose() * J_bar.transpose()) * rio.sensors_.dq_;
    rio.actuators_.force_gc_commanded_ = Gamma;   // position + orientation control

    q_desired = Eigen::VectorXd::Zero(7);
    // q_desired << 1.00267,1.26118,-0.482714,0.967511,0.545266,1.02197,0.537638;
    // q_desired << 2.8271,1.34687,-1.04559,1.51003,1.2214,1.08341,1.472;
    // joint space control
    rio.actuators_.force_gc_commanded_ += N * kq * (q_desired - rio.sensors_.q_) - kdq * rio.sensors_.dq_ + kq_lim * q_limit_gain - kdq_lim * dq_limit_gain;  

// >>> set hard constraints on torque limits.
    Eigen::VectorXd torque_lim;
    torque_lim = Eigen::VectorXd::Zero(7);
    torque_lim << 175.0, 175.0, 109.0, 109.0, 109.0, 39.0, 39.0;

    for (int i = 0; i < 7; i = i + 1)
      {
        if (rio.actuators_.force_gc_commanded_(i) >= torque_lim(i))
          rio.actuators_.force_gc_commanded_(i) = torque_lim(i);
        if (rio.actuators_.force_gc_commanded_(i) <= -torque_lim(i))
          rio.actuators_.force_gc_commanded_(i) = -torque_lim(i);
      }


    // Integrate the dynamics
    dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, controlSleepTime}; nanosleep(&ts,NULL);

    
    if(iter % 1000 == 0)
    {
       // std::cout<<"\n"<<iter*dt<<"\t" << d_phi.transpose();
       // std::cout<<"\n" << rio.sensors_.dq_.transpose() << "\n";
       std::cout<<"\n"<<iter*dt<<"\t"<< d_phi.transpose();
       std::cout<<"\n"<<iter*dt<<"\t"<< (x_des - x).transpose();
       std::cout<<"\n"<<iter*dt<<"\t"<< rio.sensors_.q_.transpose()*180.0/M_PI;
       std::cout<<"\n"<<iter*dt<<"\t"<< rio.sensors_.dq_.transpose()*180.0/M_PI;
       std::cout<<"\n"<<iter*dt<<"\t"<< rio.actuators_.force_gc_commanded_.transpose()<<"\n";
    }
  }
}


