/*
 * Copyright (C) 2014 Stanford Robotics Lab
 * Author: Mohammad Khansari
 * email:  khansari@cs.stanford.edu
 *
 * This code was written based on another code written by Keegen Go
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef GRIPPER_HPP_
#define GRIPPER_HPP_

#include <QThread>
#include <QElapsedTimer>

// Gripper Library
#include <rl/hal/WeissWsg50.h>
#include <rl/util/Timer.h>

#define MinSchunkGripperPos 0.0 //0.006
#define MaxSchunkGripperPos 0.11

/*
 * The gripper is meant to be controlled form a separate thread.
 * It watches the given bit and opens or closes to match it.
 */
class SchunkGripper : public QThread{
public:

    // Constructs object and attempts to connect to gripper.
    // Likely that program will hang if unable to connect
    void run();

    void stop();

    float GetForce();

    int GetSystemState();

    void SetDesiredPosition(const double &desPos, const double &minRange = MinSchunkGripperPos, const double &maxRange = MaxSchunkGripperPos);
		
		float GetPosition();

private:
    rl::hal::WeissWsg50 gripper;
    double desiredGripperPosition;
    bool b_gripperAvailable;
    bool b_runThread;
};


#endif /* GRIPPER_HPP_ */
