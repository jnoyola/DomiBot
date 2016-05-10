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

  /***************************************************/
  /********* Chose your controller  ******************/
  /***************************************************/
  // std::thread controlThread(jointSpaceControl);
  // std::thread controlThread(opSpacePositionControl);
  // std::thread controlThread(opSpaceOrientationControl);
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

void jointSpaceControl()
{
  long iter = 0;
  int counter = 0;

  std::cout<<"\n\n***************************************************************"
           <<"\n Starting joint space (generalized coordinate) controller..."
           <<"\n***************************************************************\n";


  

  while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
  {
    dyn_scl.computeGCModel(&rio.sensors_,&rgcm);

    Eigen::VectorXd q_desired(6); // desired position
    Eigen::VectorXd g(6);         // joint space gravity
    Eigen::VectorXd Gamma(6);

    /*********************************************************/
    /*********************************************************/
    /*************     YOUR CODE HERE     ********************/
    /*********************************************************/
    /*********************************************************/

    // define and chose your gains 
    double kp = 400;
    double kv = 150;

    // desired position to set
    q_desired << 0.5,0.5,0.5,0.5,0.5,0.5;

    // gravity vector
    // g << 0,0,0,0,0,0;
    g = rgcm.force_gc_grav_;

    // torque command to the motors
    // compute your controller here
    Gamma = Eigen::VectorXd::Zero(6);


    // Gamma = kp * (q_desired - rio.sensors_.q_) - kv * rio.sensors_.dq_;
    // Gamma = kp * (q_desired - rio.sensors_.q_) - kv * rio.sensors_.dq_ - g;
    Gamma = rgcm.M_gc_ * (kp * (q_desired - rio.sensors_.q_) - kv * rio.sensors_.dq_) - g ;
    
    
    /*********************************************************/
    /*********************************************************/
    /*************     END OF YOUR CODE     ******************/
    /*********************************************************/
    /*********************************************************/

    rio.actuators_.force_gc_commanded_ = Gamma;

    dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, controlSleepTime}; nanosleep(&ts,NULL);

    // if(iter % 10000 == 0){
    if(iter % 100 == 0){
        std::cout<<"\n"<< iter*dt<<"\t"<<(q_desired - rio.sensors_.q_).transpose();
        // std::cout<<"\nJoint torques: "<<Gamma.transpose();
        // std::cout<<"\nJoint position: "<<rio.sensors_.q_.transpose();
    }
  }
}




void opSpacePositionControl()
{
  double tstart, tcurr;
  long iter = 0;
  bool flag = false;

  const Eigen::Vector3d hpos(0,0,0.05); //control position of op-point wrt. hand
  Eigen::MatrixXd J, Jv, A, A_inv, Lambda, M;
  Eigen::Vector3d x, x_des, x_init, dx;
  Eigen::VectorXd Gamma, g, F;
  scl::SRigidBodyDyn *rhand = rgcm.rbdyn_tree_.at("end-effector");


  std::cout<<"\n\n***************************************************************"
      <<"\n Starting op space position controller..."
      <<"\n***************************************************************\n";
  tstart = sutil::CSystemClock::getSysTime(); iter = 0;
  while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
  {
    // current time and compute model
    tcurr = sutil::CSystemClock::getSysTime();
    dyn_scl.computeGCModel(&rio.sensors_,&rgcm);

    // find initial position
    if(false == flag) { x_init = rhand->T_o_lnk_ * hpos; flag = true; }
    
    // Jacobians
    dyn_scl.computeJacobianWithTransforms(J,*rhand,rio.sensors_.q_,hpos);
    Jv = J.block(0,0,3,rio.dof_);

    // Mass Matrix and its inverse
    A = rgcm.M_gc_;
    A_inv = rgcm.M_gc_inv_;

    /*********************************************************/
    /*********************************************************/
    /*************     YOUR CODE HERE     ********************/
    /*********************************************************/
    /*********************************************************/

    // gains    
    double kp = 400;
    double kv = 20;

    // radius of circle and frequency
    double sin_ampl = 0.15;
    double frequency = 0.3;

    // current position and velocity
    // x << 0,0,0;
    // dx << 0,0,0;

    x = rhand->T_o_lnk_ * hpos;
    dx = Jv * rio.sensors_.dq_;

    // compute desired position
    x_des(0) = x_init(0) + sin(tcurr*frequency*2*3.14159)*sin_ampl + 0.1; 
    x_des(1) = x_init(1) + cos(tcurr*frequency*2*3.14159)*sin_ampl + 0.2; 
    x_des(2) = x_init(2) - 0.1;


    // Compute operational space Inertia matrix
    Lambda = Eigen::MatrixXd::Identity(3,3);

    M = J * A_inv * J.transpose();
    Lambda = M.inverse();


    // Operational space controller force
    F = Eigen::Vector3d::Zero();

    F = Lambda * (kp * (x_des - x) - kv * dx);


    // joint space gravity
    // g << 0,0,0,0,0,0;
    g = rgcm.force_gc_grav_;

    // compute the joint torques
    Gamma = Eigen::VectorXd::Zero(6);

    Gamma = Jv.transpose()  * F - g;
    /*********************************************************/
    /*********************************************************/
    /*************     END OF YOUR CODE     ******************/
    /*********************************************************/
    /*********************************************************/

    Eigen::VectorXd ns_damping(6);
    Eigen::MatrixXd J_bar = A_inv*Jv.transpose()*Lambda;
    ns_damping = (Eigen::MatrixXd::Identity(6,6)-Jv.transpose()*J_bar.transpose())*rio.sensors_.dq_;

    rio.actuators_.force_gc_commanded_ = Gamma  - 15.0*ns_damping;

    // Integrate the dynamics
    dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, controlSleepTime}; nanosleep(&ts,NULL);

    if(iter % 1000 == 0)
    {
        // std::cout<<"\nPosition error: "<<(x_des-x).transpose(); // <<". Norm: "<<(x_des-x).norm();
        std::cout << "\n"<< dt*iter<<"\t"<<x.transpose();
        // std::cout<<"\nJoint torques: "<<rio.actuators_.force_gc_commanded_.transpose() << "\n";
    }
  }
}










void opSpaceOrientationControl()
{
  long iter = 0;
  bool flag = false;

  const Eigen::Vector3d hpos(0,0,0.05); //control position of op-point wrt. hand
  Eigen::MatrixXd J, Jw, A, A_inv, Lambda, R_init, M;
  Eigen::Matrix3d R, R_des;
  Eigen::Vector3d w;
  Eigen::VectorXd Gamma, g, F;
  scl::SRigidBodyDyn *rhand = rgcm.rbdyn_tree_.at("end-effector");


  std::cout<<"\n\n***************************************************************"
      <<"\n Starting op space orientation controller..."
      <<"\n***************************************************************\n";
  while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
  {
    dyn_scl.computeGCModel(&rio.sensors_,&rgcm);

    // Jacobians
    dyn_scl.computeJacobianWithTransforms(J,*rhand,rio.sensors_.q_,hpos);

    // Mass Matrix and its inverse
    A = rgcm.M_gc_;
    A_inv = rgcm.M_gc_inv_;

    // Initialize rotation matrices
    R = Eigen::Matrix3d::Identity(3,3);
    R_des = Eigen::Matrix3d::Identity(3,3);

    // initial position
    if(false == flag) {R_init = rhand->T_o_lnk_.rotation(); flag = true; }

    /*********************************************************/
    /*********************************************************/
    /*************     YOUR CODE HERE     ********************/
    /*********************************************************/
    /*********************************************************/

    // gains    
    double kp = 2000;
    double kv = 20;

    // find the task jacobians
    // Jw = Eigen::MatrixXd::Zero(3,rio.dof_);
    Jw = J.block(3,0,3,rio.dof_);

    // current orientation and angular velocity
    // R = Eigen::Matrix3d::Identity(3,3);
    R = rhand->T_o_lnk_.rotation();
    w = Jw * rio.sensors_.dq_;

    // desired orientation
    R_des = Eigen::Matrix3d::Identity(3,3);
    R_des(0,0) = cos(M_PI/3.0);  R_des(0,1) = 0.0; R_des(0,2) = sin(M_PI/3.0);
    R_des(1,0) = 0.0;            R_des(1,1) = 1.0; R_des(1,2) = 0.0;
    R_des(2,0) = -sin(M_PI/3.0); R_des(2,1) = 0.0; R_des(2,2) = cos(M_PI/3.0);

    // angular error vector
    Eigen::Vector3d d_phi;
    d_phi = -0.5*(  R.col(0).cross(R_des.col(0)) + R.col(1).cross(R_des.col(1)) + R.col(2).cross(R_des.col(2)) );
    // std::cout<<dt*iter<<"\t"<<d_phi.transpose()<<"\n";

    // Operational space Inertia matrix
    // Lambda = Eigen::MatrixXd::Identity(3,3);
    M = J * A_inv * J.transpose();
    Lambda = M.inverse();

    // Operational space controller force
    // F = Eigen::Vector3d::Zero();
    F = -Lambda * (kp * d_phi + kv * w);


    // joint space gravity
    // g << 0,0,0,0,0,0;
    g = rgcm.force_gc_grav_;

    // joint torques
    // Gamma = Eigen::VectorXd::Zero(6);
    Gamma = Jw.transpose()*F - g;


    /*********************************************************/
    /*********************************************************/
    /*************     END OF YOUR CODE     ******************/
    /*********************************************************/
    /*********************************************************/

    Eigen::VectorXd ns_damping(6);
    Eigen::MatrixXd J_bar = A_inv*Jw.transpose()*Lambda;
    ns_damping = (Eigen::MatrixXd::Identity(6,6)-Jw.transpose()*J_bar.transpose())*rio.sensors_.dq_;

    rio.actuators_.force_gc_commanded_ = Gamma  - 3.0*ns_damping;

    // Integrate the dynamics
    dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, controlSleepTime}; nanosleep(&ts,NULL);

    // if(iter % 10000 == 0)
    if(iter % 100 == 0)
    {
      std::cout<<"\n"<<iter*dt<<"\t" << d_phi.transpose();
        // std::cout<<"\nangular Position error: "<<d_phi.transpose() <<". Norm: "<<d_phi.norm();
        // std::cout<<"\nJoint torques: "<<rio.actuators_.force_gc_commanded_.transpose() << "\n";
        // std::cout<<"rotation matrix:\n"<<R<<"\n";
        // std::cout<<"desired rotation matrix:\n"<<R_des<<"\n";
    }
  }
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
  /*********************************************************/
  /***********************   WRIST      ********************/
  /*********************************************************/
  scl::SRigidBodyDyn *rwrist = rgcm.rbdyn_tree_.at("link_5");
  /*********************************************************/
  /*********************************************************/
  /*********************************************************/

  const Eigen::Vector3d hpos(0,0,0.05); //control position of op-point wrt. hand

  /*********************************************************/
  /*********************************************************/
  /*************     YOUR CODE HERE     ********************/
  /*********************************************************/
  /*********************************************************/


  // Define the matrices and vectors you will need
  Eigen::MatrixXd J_hand, J_wrist, Jhv, Jhw, Jwv, Jww, A, A_inv, Lambda, LambdaO, R_init, M, MO;
  Eigen::Matrix3d R, R_des;
  Eigen::Vector3d w, x, x_des, x_init, dx;
  Eigen::VectorXd Gamma, g, F, FO, q_desired; 
 

  while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
  {
    // get current time and compute the model
    tcurr = sutil::CSystemClock::getSysTime();
    dyn_scl.computeGCModel(&rio.sensors_,&rgcm);


    // initial position
    if(false == flag) { x_init = rhand->T_o_lnk_ * hpos; flag = true; }
    
    // Compute your Jacobians
    // Jacobians
    dyn_scl.computeJacobianWithTransforms(J_hand,*rhand,rio.sensors_.q_,hpos);
    Jhv = J_hand.block(0,0,3,rio.dof_);
    Jhw = J_hand.block(3,0,3,rio.dof_);

    dyn_scl.computeJacobianWithTransforms(J_wrist,*rwrist,rio.sensors_.q_,hpos);
    Jwv = J_wrist.block(0,0,3,rio.dof_);
    Jww = J_wrist.block(3,0,3,rio.dof_);

    // Mass Matrix and its inverse    A is the identity
    // A = rgcm.M_gc_;
    // A_inv = rgcm.M_gc_inv_;
    
    // gains    
    double kpO = 200;
    double kvO = 20;
    double kp = 100;
    double kv = 30;
    
    // position trajectory parameters
    double sin_ampl = 0.15;
    double frequency = 0.3;

    
    // current position and velocity
    x = rwrist->T_o_lnk_ * hpos;
    dx = Jwv * rio.sensors_.dq_;

    // desired position
    // x_des(0) = x_init(0) + sin(tcurr*frequency*2*M_PI)*sin_ampl + 0.1; 
    // x_des(1) = x_init(1) + cos(tcurr*frequency*2*M_PI)*sin_ampl + 0.5; 
    // x_des(2) = x_init(2) - 0.5;

      if (iter*dt < 7.0)
      {x_des(0) = 0.5; 
      x_des(1) = 0.5; 
      x_des(2) = 0.2;}

      else if (iter*dt < 11.0)
        {x_des(0) = 0.5; 
         x_des(1) = -0.5; 
         x_des(2) = 0.2;}

      else if (iter*dt < 15.0)
        {x_des(0) = -0.5; 
         x_des(1) = -0.5; 
         x_des(2) = 0.2;}

      else if (iter*dt < 19.0)
      {x_des(0) = -0.5; 
       x_des(1) = 0.5; 
       x_des(2) = 0.2;}



    // current orientation and angular velocity
    R = rhand->T_o_lnk_.rotation();
    w = Jhw * rio.sensors_.dq_;
    
    // desired orientation
    // Rotation in the y axis 
    R_des(0,0) = cos(M_PI);  R_des(0,1) = 0.0; R_des(0,2) = sin(M_PI);
    R_des(1,0) = 0.0;        R_des(1,1) = 1.0; R_des(1,2) = 0.0;
    R_des(2,0) = -sin(M_PI); R_des(2,1) = 0.0; R_des(2,2) = cos(M_PI);

    // Rotation in the x axis 
    // R_des(0,0) = 1.0; R_des(0,1) = 0.0; R_des(0,2) = 0.0;
    // R_des(1,0) = 0.0; R_des(1,1) = cos(M_PI); R_des(1,2) = -sin(M_PI);
    // R_des(2,0) = 0.0; R_des(2,1) = sin(M_PI); R_des(2,2) = cos(M_PI);

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

    q_desired = Eigen::VectorXd::Zero(6);
    q_desired << 1.00267,1.26118,-0.482714,0.967511,0.545266,1.02197,0.537638;
    // joint space control
    rio.actuators_.force_gc_commanded_ += 0.0 * (q_desired - rio.sensors_.q_) - 2.0 * rio.sensors_.dq_;  

    // Integrate the dynamics
    dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, controlSleepTime}; nanosleep(&ts,NULL);

    
    if(iter % 1000 == 0)
    {
       std::cout<<"\n"<<iter*dt<<"\t" << d_phi.transpose();
       std::cout<<"\n" << rio.sensors_.q_ << "\n";
       std::cout<<"\n" << x_init.transpose() << "\n";
    }

    /*********************************************************/
    /*********************************************************/
    /*************     END OF YOUR CODE     ******************/
    /*********************************************************/
    /*********************************************************/
  }
}


