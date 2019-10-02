/**
 * @author koseng : lintangerlangga@gmail.com
 * @brief In one project of specific robot the kinematics are rarely changed
 *        so the singleton is choosen, keep in mind the singleton will allocated
 *        more than one if called in another node, because node is representation
 *        of a program/executable which imply to different address in each node
 */

#pragma once

#include <armadillo>

#include "math/math_utils.h"
#include "joint_states/joint_states.h"

using MathUtils::hat;
using MathUtils::wedge;

class Kinematics:public Singleton<Kinematics>{
public:
    Kinematics();
    void forwardKinematics(JointData::LinkID _joint_id);
    mat calcRodrigues(const colvec& _joint_axis, double _angle);

};
