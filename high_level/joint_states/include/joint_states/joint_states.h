/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once

#include <tuple>
#include <array>
#include <cassert>

#include "singleton/singleton.h"
#include "joint_data/joint_data.h"
#include "robot_params/robot_params.h"
#include "math/math_utils.h"

#define JOINT_ID_ASSERT(ID) assert(ID >= 0 && ID < JointData::NUM_OF_JOINTS)

class JointStates:public Singleton<JointStates>{
private:

    std::array<RobotParams, JointData::NUM_OF_JOINTS > params_;

    void init();

    JointStates();
    friend class Singleton<JointStates>;
public:
    ~JointStates();

    //Getter

    inline double getAngle(JointData::LinkID _joint_id){
        JOINT_ID_ASSERT(_joint_id);
        if(params_[_joint_id].q > MathUtils::TWO_PI)
            params_[_joint_id].q = params_[_joint_id].q - MathUtils::TWO_PI;
        if(params_[_joint_id].q < -MathUtils::TWO_PI)
            params_[_joint_id].q = params_[_joint_id].q + MathUtils::TWO_PI;
        return params_[_joint_id].q;
    }

    inline double getLastAngle(JointData::LinkID _joint_id){
        JOINT_ID_ASSERT(_joint_id);
        if(params_[_joint_id].last_q > MathUtils::TWO_PI)
            params_[_joint_id].last_q = params_[_joint_id].last_q - MathUtils::TWO_PI;
        if(params_[_joint_id].last_q < -MathUtils::TWO_PI)
            params_[_joint_id].last_q = params_[_joint_id].last_q + MathUtils::TWO_PI;
        return params_[_joint_id].last_q;
    }

    inline double getVel(JointData::LinkID _joint_id) const{
        JOINT_ID_ASSERT(_joint_id);
        return params_[_joint_id].dq;
    }

    inline double getAccel(JointData::LinkID _joint_id) const{
        JOINT_ID_ASSERT(_joint_id);
        return params_[_joint_id].ddq;
    }

    inline colvec getRotDir(JointData::LinkID _joint_id) const{
        JOINT_ID_ASSERT(_joint_id);
        return params_[_joint_id].a;
    }

    inline colvec getPos(JointData::LinkID _joint_id) const{
        JOINT_ID_ASSERT(_joint_id);
        return params_[_joint_id].p;
    }

    inline mat getRotMat(JointData::LinkID _joint_id) const{
        JOINT_ID_ASSERT(_joint_id);
        return params_[_joint_id].R;
    }

    inline JointData::LinkID getChild(JointData::LinkID _joint_id) const{
        JOINT_ID_ASSERT(_joint_id);
        return static_cast<JointData::LinkID>(params_[_joint_id].child);
    }

    inline JointData::LinkID getMother(JointData::LinkID _joint_id) const{
        JOINT_ID_ASSERT(_joint_id);
        return static_cast<JointData::LinkID>(params_[_joint_id].mother);
    }

    inline JointData::LinkID getSister(JointData::LinkID _joint_id) const{
        JOINT_ID_ASSERT(_joint_id);
        return static_cast<JointData::LinkID>(params_[_joint_id].sister);
    }

    inline colvec getRelativePos(JointData::LinkID _joint_id){
        JOINT_ID_ASSERT(_joint_id);
//        params_[_joint_id].b = params_[_joint_id].p - params_[getMother(_joint_id)].p;
        return params_[_joint_id].b;
    }

    //Setter

    inline double& setAngle(JointData::LinkID _joint_id){
        JOINT_ID_ASSERT(_joint_id);
        params_[_joint_id].last_q = params_[_joint_id].q;
        return params_[_joint_id].q;
    }

    inline double& setVel(JointData::LinkID _joint_id){
        JOINT_ID_ASSERT(_joint_id);
        return params_[_joint_id].dq;
    }

    inline double& setAccel(JointData::LinkID _joint_id){
        JOINT_ID_ASSERT(_joint_id);
        return params_[_joint_id].ddq;
    }

    inline colvec& setPos(JointData::LinkID _joint_id){
        JOINT_ID_ASSERT(_joint_id);
        return params_[_joint_id].p;
    }

    inline colvec& setRotDir(JointData::LinkID _joint_id){
        JOINT_ID_ASSERT(_joint_id);
        return params_[_joint_id].a;
    }

    inline mat& setRotMat(JointData::LinkID _joint_id){
        JOINT_ID_ASSERT(_joint_id);
        return params_[_joint_id].R;
    }

};
