#ifndef _DOMI_H
#define _DOMI_H

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <Eigen/Dense>
#include <stdio.h>

// For Gripper
#include "SchunkGripper.h"

class Domi
{
   
public:

    // Pass _dt <= 0 for the actual robot!!
    Domi(scl::SGcModel &_rgcm,
         scl::SRobotIO &_rio,
         scl::CDynamicsScl &_dyn_scl,
         scl::SRigidBodyDyn *_rhand,
         scl::SRigidBodyDyn *_rwrist,
         const Eigen::Vector3d &_hpos,
         double _dt);

    void mainloop();



    void compliant_control();
    
private:

    typedef enum e_State {
        MOVE_TO_REST,
        REST,
        OBSERVATION,
        GET_ABOVE,
        GET_DEPTH,
        GET_GRASP,
        GET_REVERSE_DEPTH,
        PUT_ABOVE,
        PUT_DEPTH,
        PUT_GRASP,
        PUT_REVERSE_DEPTH
    } State;
    State state;
    
    scl::SGcModel &rgcm;
    scl::SRobotIO &rio;
    scl::CDynamicsScl &dyn_scl;
    scl::SRigidBodyDyn *rhand;
    scl::SRigidBodyDyn *rwrist;
    const Eigen::Vector3d &hpos;
    
    Eigen::MatrixXd J_hand, J_wrist, Jhv, Jhw, Jwv, A, A_inv, Lambda, LambdaO, M, MO, Jwv_bar, N;
    Eigen::Matrix3d R, R_des;
    Eigen::Vector3d w, x, x_via, x_des, x_init, dx, d_phi, g;
    Eigen::VectorXd Gamma, F, FO, q_lim, dq_lim, q_limit_gain, dq_limit_gain, torque_lim;
    
    double kpO, kvO, kp, kv, kq, kdq, kq_lim, kdq_lim, ori_init, ori_des, ori_via, ori_cur;
    
    double t_init, t_dur, t_elap;
    
    SchunkGripper *schunkGripper; // Gripper object

    long iter;
    double dt;
    bool is_simulator;
    
    // For running python code
    FILE *py_fp;
    char py_out[128];
    
    // For timing REST state
    double rest_end;

    // Domino position and orientation
    double x_get, y_get, ori_get, x_put, y_put, ori_put;
    const double x_rest, y_rest, z_above, z_depth;
    
    void move_to_rest();
    void rest();
    void observation();
    void get_above();
    void get_depth();
    void get_grasp();
    void get_reverse_depth();
    void put_above();
    void put_depth();
    void put_grasp();
    void put_reverse_depth();
    
    void set_des_pos_ori_duration(double x, double y, double z, double ori, double duration);
    
    void control_pos_ori();

    double get_z_rot();
    
    bool has_error(double tol_pos = 0.01,
                   double tol_ori = 0.01);

    bool has_ori_error(double tol = 0.02);
    
    double get_time();
};

#endif
