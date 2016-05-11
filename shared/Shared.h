#ifndef _SHARED_H
#define _SHARED_H

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>

#include <Eigen/Dense>

class Shared
{
   
public:

    static void mainloop(scl::SGcModel rgcm,
                         scl::SRobotIO rio,
                         scl::CDynamicsScl dyn_scl,
                         scl::SRigidBodyDyn *rhand,
                         scl::SRigidBodyDyn *rwrist);
    
private:

    typedef enum e_State {
        MOVE_TO_REST
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
    static State state;
    
    static Eigen::Vector3d hpos;
    
    static Eigen::Vector3d x_des;
    static Eigen::Matrix3d R_des;
    
    static void move_to_rest(scl::SGcModel rgcm,
                             scl::SRobotIO rio,
                             scl::CDynamicsScl dyn_scl,
                             scl::SRigidBodyDyn *rhand,
                             scl::SRigidBodyDyn *rwrist);
    
    static void rest();
    
    static bool has_no_error(scl::SRigidBodyDyn *rhand,
                             scl::SRigidBodyDyn *rwrist,
                             double tol_pos = 0.01,
                             double tol_ori = 0.01);
    
    static void control_pos_ori(scl::SGcModel rgcm,
                                scl::SRobotIO rio,
                                scl::CDynamicsScl dyn_scl,
                                scl::SRigidBodyDyn *rhand,
                                scl::SRigidBodyDyn *rwrist);
};

#endif
