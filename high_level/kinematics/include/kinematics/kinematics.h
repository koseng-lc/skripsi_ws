/**
 * @author koseng : lintangerlangga@gmail.com
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
