//Trajectory generation
//Author: Ryan Williams
/*
/* This code is used to generate a sequence of trajectories to be used as desired state inputs by the KUKA robot
in order to play dominoes */
	// define sequence of trajectory, by state
	// Start with trajectory to a home location (not necessary)
//trajectory1 = 

//Assumed rest location (home) is reached, calculate new trajectory to first domino (no intermediate points)'
	//start with end effector desired cartesian positioning
EE_DCp_x = x_0 + 3/t_f*(x_f - x_0)*t^2 - (2/t_f^3)*(x_f - x_0)*t^3;
EE_DCp_y = y_0 + 3/t_f*(y_f - y_0)*t^2 - (2/t_f^3)*(y_f - y_0)*t^3;
EE_DCp_z = z_0; //Just move in the x-y plane

/* Note that the above trajectory is based off of a known starting and ending location, and a known total time'
it should take to get there: t_f. Additionally, these values may need heavier discretization as the controller
may not be able to reduce the error enough before another desired location is called
Also note, if this trajectory planning does not work well, we may have to introduce via points, which would
require additional polynomials */

// Full state trajectory Joint Space - input to joint space controller (for all 7 states)
//Note: q_desired, q_initial, and q_final is a 7 state vector.
q_desired = q_initial + 3/t_f*(q_final - q_initial)*t^2 - (2/t_f^3)*(q_final - q_initial)*t^3;
*/

//Now to actual implement in a basic state machine
int state = 0
//Trajectory from home to above domino: 
if ( (x_EE != x_CV || y_EE != y_CV) && state == 0){
	q_desired = q_initial + 3/t_f*(q_final - q_initial)*t^2 - (2/t_f^3)*(q_final - q_initial)*t^3;
}
elseif{ x_EE == x_CV && y_EE == y_CV)
	state = 1;
	long time_current = time;
	long time_chill = 0;
	for(time_chill< 2000;) //hover above domino for 2 seconds
	{
		qdot_desired = 0;
		//call controller here (joint space) or transform to opspace
		time_chill = time - time_current;
	}
	
if (state == 1 && q_dot == 0){
    z1 = z;
	c = (z_cv - z1)/t_f_down; //set t_f_down as how long it should take the domino to get to the floor
	state == 2;
   }
if(state == 2){
	z_des = z1 +c*t;
	if(z == z_des && q_dot == 0){
		state ==3;
		z2 = z;
		c2 = (z1 - z2)/t_f_down
		}
	}
if (state == 3){
	z_des = z2 +c2*t;
}