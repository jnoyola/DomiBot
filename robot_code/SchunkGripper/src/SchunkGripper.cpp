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

#include "SchunkGripper.h"
#include <iostream>

void SchunkGripper::run()
{
    b_runThread = true;
    b_gripperAvailable = false;
    try{
        gripper.open();
        b_gripperAvailable = true;
    }catch (const std::exception& e)
    {
        std::cout << "Unable to connect to the schunk gripper!" << std::endl;
        std::cerr << e.what() << std::endl;
        b_gripperAvailable = false;
    }

    if (b_gripperAvailable)
    {
        gripper.start();
        gripper.doSetAcceleration(1.0f);

        gripper.doSetForceLimit(75.0f);

        struct timespec wait;
        wait.tv_nsec  = 1000000; // 1ms
        wait.tv_sec   = 0;

        desiredGripperPosition = 0.06;


        rl::hal::WeissWsg50::SystemState state = gripper.getSystemState();
        std::cout << "Schunk Device State: " << state << std::endl;
        gripper.doSetAcceleration(4.5); //(5.0);

        while (b_runThread)
        {
            try{
                gripper.doPrePositionFingers(desiredGripperPosition, 0.35); //0.4);
            }catch (const std::exception& e)
            {
                std::cerr << "doPrePositionFingers failed: " << e.what() << std::endl;

                std::string error = e.what();

                if (error == "Broken pipe") {
                    std::cerr << "Trying to reconnect to gripper" << std::endl;
                    gripper.stop();
                    gripper.close();
                    b_gripperAvailable = false;
                    try{
                        gripper.open();
                        b_gripperAvailable = true;
                        std::cerr << "Connected to gripper" << std::endl;
                    }catch (const std::exception& e)
                    {
                        std::cout << "Unable to connect to the schunk gripper!" << std::endl;
                        std::cerr << e.what() << std::endl;
                        b_gripperAvailable = false;
                        gripper.stop();
                        gripper.close();
                        return;
                    }
                }
            }
            nanosleep(&wait,NULL);
        }

        gripper.stop();
        gripper.close();
    }
}

int SchunkGripper::GetSystemState()
{
    rl::hal::WeissWsg50::SystemState state = gripper.getSystemState();
    return state;
}


void SchunkGripper::stop()
{
    b_runThread = false;
}

void SchunkGripper::SetDesiredPosition(const double &desPos, const double &minRange, const double &maxRange)
{
    if (b_gripperAvailable)
    {
        if (minRange<maxRange)
        {
            if (desPos>maxRange)
                desiredGripperPosition = MaxSchunkGripperPos;
            else if (desPos<minRange)
                desiredGripperPosition = MinSchunkGripperPos;
            else
                desiredGripperPosition = (desPos - minRange)/(maxRange - minRange)*(MaxSchunkGripperPos-MinSchunkGripperPos) + MinSchunkGripperPos;
        }else{
            std::cout << "SchunkGripper: minRange should be smaller that maxRange!" << std::endl;
        }
    }
}

float SchunkGripper::GetForce()
{
    if (b_gripperAvailable)
        return gripper.getForce();
    else
        return 0.0;
}

float SchunkGripper::GetPosition()
{
    if (b_gripperAvailable)
        return gripper.getOpeningWidth();
    else
        return -1.0;
}

/*
void SchunkGripper::Control(bool *ok, bool *toClose)
{
    struct timespec wait;
    wait.tv_nsec  = 5000000; // 5ms
    wait.tv_sec   = 0;

    double open_pos = 0.06;
    double close_pos = 0.045;


    rl::hal::WeissWsg50::SystemState state = gripper.getSystemState();
    gripper.doPrePositionFingers(open_pos, 0.4);

    while(*ok) {
        try {
            if(*toClose) {
                if(open) {
                    open = false;
                    // Close the Gripper
                    gripper.doPrePositionFingers(close_pos, 0.4);
                }
            }
            else {
                if(!open) {
                    open = true;
                    // Open the Gripper
                    gripper.doStop(); // interrupt the gripper with whatever it was doing before
                    gripper.doPrePositionFingers(open_pos, 0.4);
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
        rl::hal::WeissWsg50::SystemState cur_state = gripper.getSystemState();
        if(cur_state != state){
            std::cout << cur_state << "\n";
            state = cur_state;
        }

        nanosleep(&wait,NULL);
    }

    gripper.stop();
    gripper.close();
}
*/
