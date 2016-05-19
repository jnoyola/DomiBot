#ifndef _DOMI_H
#define _DOMI_H

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>

#include <Eigen/Dense>

#include <stdio.h>

class Domi
{
   
public:

    Domi(scl::SGcModel &_rgcm,
         scl::SRobotIO &_rio,
         scl::CDynamicsScl &_dyn_scl,
         scl::SRigidBodyDyn *_rhand,
         scl::SRigidBodyDyn *_rwrist,
         const Eigen::Vector3d &_hpos,
         bool is_simulator);

    void mainloop();
    
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
    
    bool is_simulator;
    
    Eigen::Vector3d x_des;
    Eigen::Matrix3d R_des;
    
    // For timing REST state
    double rest_end;
    
    // For running python code
    FILE *py_fp;
    char py_out[128];
    
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
    
    void control_pos_ori();
    
    bool has_error(double tol_pos = 0.01,
                   double tol_ori = 0.01);
};

#endif
