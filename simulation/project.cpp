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
/* \file scl_tutorial5_multi_task.cpp
 *
 *  Created on: Aug 10, 2014
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
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>
//scl functions to simplify dynamic typing and data sharing
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/util/DatabaseUtils.hpp>

//For timing
#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>

//Freeglut windowing environment
#include <GL/freeglut.h>

// writing txt files
#include <fstream>

/** A sample application to demonstrate a physics simulation in scl.
 *
 * Moving forward from tutorial 4, we will now control the 6 DOF
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
  std::cout<<"Standard Control Library Tutorial #5";
  std::cout<<"\n***************************************\n";

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGraphicsParsed rgr;  //Robot graphics data structure...
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SRobotIO rio;         //I/O data structure
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
  scl::CDynamicsTao dyn_tao; //Robot physics integrator
  scl::CParserScl p;         //This time, we'll parse the tree from a file...

  scl::SControllerMultiTask rctr_ds; //A multi-task controller data structure
  scl::CControllerMultiTask rctr;    //A multi-task controller
  std::vector<scl::STaskBase*> rtasks;              //A set of executable tasks
  std::vector<scl::SNonControlTaskBase*> rtasks_nc; //A set of non-control tasks
  std::vector<scl::sString2> ctrl_params;        //Used to parse extra xml tags
  scl::STaskOpPos* rtask_hand;       //Will need to set hand desired positions etc.

  sutil::CSystemClock::start(); //Start the clock

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  // bool flag = p.readRobotFromFile("./Puma.xml","./","PumaBot",rds);
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
  if(false == flag){ return 1; }            //Error check.

  /******************************Set up Controller Specification************************************/
  // Read xml file info into task specifications.
  // flag = p.readTaskControllerFromFile("./PumaCfg.xml","opc",rtasks,rtasks_nc,ctrl_params);
  flag = p.readTaskControllerFromFile(fname,"opc",rtasks,rtasks_nc,ctrl_params);
  flag = flag && rctr_ds.init("opc",&rds,&rio,&rgcm); //Set up the control data structure..
  // Tasks are initialized after find their type with dynamic typing.
  flag = flag && scl_registry::registerNativeDynamicTypes();
  flag = flag && scl_util::initMultiTaskCtrlDsFromParsedTasks(rtasks,rtasks_nc,rctr_ds);
  flag = flag && rctr.init(&rctr_ds,&dyn_scl);        //Set up the controller (needs parsed data and a dyn object)
  if(false == flag){ return 1; }            //Error check.

  rtask_hand = dynamic_cast<scl::STaskOpPos*>( *(rctr_ds.tasks_.at("hand")) );
  if(NULL == rtask_hand)  {return 1;}       //Error check

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  // flag = p.readGraphicsFromFile("./PumaCfg.xml","PumaBotStdView",rgr);
  flag = p.readGraphicsFromFile(fname,"iiwaBotStdView",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the r6bot's physics. \nWill test two different controllers.\n Press (x) to exit at anytime.";
  long long iter = 0; double dt=0.0001;

  double tstart, tcurr; flag = false;
  int graphics_ctr = 0;

  // Controller : Operational space controller
  std::cout<<"\n\n***************************************************************"
      <<"\n Starting op space (task coordinate) controller..."
      <<"\n This will move the hand in a circle. x =sin(t), y=cos(t)."
      <<"\n***************************************************************";
  tstart = sutil::CSystemClock::getSysTime(); iter = 0;

  std::ofstream myfile;
  myfile.open("error.txt");

  while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
  {
    tcurr = sutil::CSystemClock::getSysTime();

    // Move the hand in a sine wave
    // rtask_hand->x_goal_(0) = 0.2*sin((tcurr-tstart)*3.14/4) + 0.5;
    // rtask_hand->x_goal_(1) = 0.5;
    // rtask_hand->x_goal_(2) = 0.2*cos((tcurr-tstart)*3.14/4) + 0.2;
    rtask_hand->x_goal_(0) = 0.7;
    rtask_hand->x_goal_(1) = 0.5;
    rtask_hand->x_goal_(2) = 0.2;
    

    // Compute control forces (note that these directly have access to the io data ds).
    rctr.computeDynamics();
    rctr.computeControlForces();
    rctr.getControlForces();

    // Integrate the dynamics
    dyn_tao.integrate(rio,dt); iter++;



    if (rio.actuators_.force_gc_commanded_(0) > 176)
      {rio.actuators_.force_gc_commanded_(0) = 176;}
    if (rio.actuators_.force_gc_commanded_(1) > 176)
      {rio.actuators_.force_gc_commanded_(1) = 176;}
    if (rio.actuators_.force_gc_commanded_(2) > 110)
      {rio.actuators_.force_gc_commanded_(2) = 110;}
  	if (rio.actuators_.force_gc_commanded_(3) > 110)
      {rio.actuators_.force_gc_commanded_(3) = 110;}
    if (rio.actuators_.force_gc_commanded_(4) > 110)
      {rio.actuators_.force_gc_commanded_(4) = 110;}
    if (rio.actuators_.force_gc_commanded_(5) > 40)
      {rio.actuators_.force_gc_commanded_(5) = 40;}
 	if (rio.actuators_.force_gc_commanded_(6) > 40)
      {rio.actuators_.force_gc_commanded_(6) = 40;}



    if (rio.actuators_.force_gc_commanded_(0) < -176)
      {rio.actuators_.force_gc_commanded_(0) = -176;}
    if (rio.actuators_.force_gc_commanded_(1) < -176)
      {rio.actuators_.force_gc_commanded_(1) = -176;}
    if (rio.actuators_.force_gc_commanded_(2) < -110)
      {rio.actuators_.force_gc_commanded_(2) = -110;}
  	if (rio.actuators_.force_gc_commanded_(3) < -110)
      {rio.actuators_.force_gc_commanded_(3) = -110;}
    if (rio.actuators_.force_gc_commanded_(4) < -110)
      {rio.actuators_.force_gc_commanded_(4) = -110;}
    if (rio.actuators_.force_gc_commanded_(5) < -40)
      {rio.actuators_.force_gc_commanded_(5) = -40;}
  	if (rio.actuators_.force_gc_commanded_(6) < -40)
      {rio.actuators_.force_gc_commanded_(6) = -40;}


    // if(iter % 5000 == 0){std::cout<<"\nTracking error: "<<(rtask_hand->x_goal_-rtask_hand->x_).transpose()
    // <<". Norm: "<<(rtask_hand->x_goal_-rtask_hand->x_).norm(); }
    if(iter % 1000 == 0)
      {std::cout<<"\n"<<(rtask_hand->x_goal_-rtask_hand->x_).transpose()
    <<"\t"<<dt*iter; 
	myfile << dt*iter << "\t" << (rtask_hand->x_goal_-rtask_hand->x_).transpose() << "\n";
    	}
      // {std::cout<<"\n" << rio.actuators_.force_gc_commanded_.transpose()<<"\t"<<tcurr;}

    if(graphics_ctr >= 150)
    {
      glutMainLoopEvent();
      graphics_ctr = -1;
    }
    graphics_ctr++;
    const timespec ts = {0, 200000};
    nanosleep(&ts,NULL);

  }

  myfile.close();

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\n\tSystem time = "<<sutil::CSystemClock::getSysTime()-tstart;
  std::cout<<"\n\tSimulated time = "<<static_cast<double>(iter)*dt;
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}


