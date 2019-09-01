#include "joint_states/joint_states.h"

using namespace RobotSpec;
using namespace JointData;

JointStates::JointStates(){
    init();
}

JointStates::~JointStates(){

}

void JointStates::init(){    

    params_[R_SHO_PITCH].p << .0
                           << -TRUNK_WIDTH * .5 - JOINT_RADIUS
                           << TRUNK_HEIGHT - JOINT_RADIUS;
    params_[L_SHO_PITCH].p << .0
                           << TRUNK_WIDTH * .5 + JOINT_RADIUS
                           << TRUNK_HEIGHT - JOINT_RADIUS;
    params_[R_SHO_ROLL].p << .0
                          << -TRUNK_WIDTH * .5 - JOINT_RADIUS
                          << TRUNK_HEIGHT - JOINT_RADIUS;
    params_[L_SHO_ROLL].p << .0
                          << TRUNK_WIDTH * .5 + JOINT_RADIUS
                          << TRUNK_HEIGHT - JOINT_RADIUS;
    params_[R_ELBOW].p << params_[R_SHO_ROLL].p.at(0)
                       << params_[R_SHO_ROLL].p.at(1)
                       << params_[R_SHO_ROLL].p.at(2) - 2.0 * JOINT_RADIUS - ARM_HEIGHT;
    params_[L_ELBOW].p << params_[L_SHO_ROLL].p.at(0)
                       << params_[L_SHO_ROLL].p.at(1)
                       << params_[L_SHO_ROLL].p.at(2) - 2.0 * JOINT_RADIUS - ARM_HEIGHT;
    constexpr auto C_1_QTR_PI = cos(MathUtils::ONE_QTR_PI);
    params_[R_HIP_YAW].p << .0
                         << -(SHOULDER_WIDTH * HIP2SHO_RATIO * .5) - JOINT_RADIUS * C_1_QTR_PI
                         << -JOINT_RADIUS * C_1_QTR_PI;
    params_[L_HIP_YAW].p << .0
                         << (SHOULDER_WIDTH * HIP2SHO_RATIO * .5) + JOINT_RADIUS * C_1_QTR_PI
                         << -JOINT_RADIUS * C_1_QTR_PI;
    params_[R_HIP_ROLL].p << .0
                          << -(SHOULDER_WIDTH * HIP2SHO_RATIO * .5) - JOINT_RADIUS * C_1_QTR_PI
                          << -JOINT_RADIUS * C_1_QTR_PI;
    params_[L_HIP_ROLL].p << .0
                          << (SHOULDER_WIDTH * HIP2SHO_RATIO * .5) + JOINT_RADIUS * C_1_QTR_PI
                          << -JOINT_RADIUS * C_1_QTR_PI;
    params_[R_HIP_PITCH].p << .0
                           << -(SHOULDER_WIDTH * HIP2SHO_RATIO * .5) - JOINT_RADIUS * C_1_QTR_PI
                           << -JOINT_RADIUS * C_1_QTR_PI;
    params_[L_HIP_PITCH].p << .0
                           << (SHOULDER_WIDTH * HIP2SHO_RATIO * .5) + JOINT_RADIUS * C_1_QTR_PI
                           << -JOINT_RADIUS * C_1_QTR_PI;
    params_[R_KNEE].p << params_[R_HIP_PITCH].p.at(0)
                      << params_[R_HIP_PITCH].p.at(1)
                      << params_[R_HIP_PITCH].p.at(2) - (2.0 * JOINT_RADIUS) - THIGH_HEIGHT;
    params_[L_KNEE].p << params_[L_HIP_PITCH].p.at(0)
                      << params_[L_HIP_PITCH].p.at(1)
                      << params_[L_HIP_PITCH].p.at(2) - (2.0 * JOINT_RADIUS) - THIGH_HEIGHT;
    params_[R_ANK_PITCH].p << params_[R_KNEE].p.at(0)
                           << params_[R_KNEE].p.at(1)
                           << params_[R_KNEE].p.at(2) - (2.0 * JOINT_RADIUS) - THIGH_HEIGHT;
    params_[L_ANK_PITCH].p << params_[L_KNEE].p.at(0)
                           << params_[L_KNEE].p.at(1)
                           << params_[L_KNEE].p.at(2) - (2.0 * JOINT_RADIUS) - THIGH_HEIGHT;
    params_[R_ANK_ROLL].p << params_[R_KNEE].p.at(0)
                          << params_[R_KNEE].p.at(1)
                          << params_[R_KNEE].p.at(2) - (2.0 * JOINT_RADIUS) - THIGH_HEIGHT;
    params_[L_ANK_ROLL].p << params_[L_KNEE].p.at(0)
                          << params_[L_KNEE].p.at(1)
                          << params_[L_KNEE].p.at(2) - (2.0 * JOINT_RADIUS) - THIGH_HEIGHT;
    params_[HEAD_PAN].p << .0
                        << .0
                        << TRUNK_HEIGHT + SHOULDER_HEIGHT + HEAD_RADIUS;
    params_[HEAD_TILT].p << .0
                         << .0
                         << TRUNK_HEIGHT + SHOULDER_HEIGHT + HEAD_RADIUS;
    params_[TRUNK].p << .0
                     << .0
                     << TRUNK_HEIGHT * .5;       

    params_[R_SHO_PITCH].child = R_SHO_ROLL;
    params_[R_SHO_PITCH].sister = L_SHO_PITCH;
    params_[R_SHO_PITCH].mother = TRUNK;

    params_[L_SHO_PITCH].child = L_SHO_ROLL;
    params_[L_SHO_PITCH].sister = R_HIP_YAW;
    params_[L_SHO_PITCH].mother = TRUNK;

    params_[R_SHO_ROLL].child = R_ELBOW;
    params_[R_SHO_ROLL].sister = NONE;
    params_[R_SHO_ROLL].mother = R_SHO_PITCH;

    params_[L_SHO_ROLL].child = L_ELBOW;
    params_[L_SHO_ROLL].sister = NONE;
    params_[L_SHO_ROLL].mother = L_SHO_PITCH;

    params_[R_ELBOW].child = NONE;
    params_[R_ELBOW].sister = NONE;
    params_[R_ELBOW].mother = R_SHO_ROLL;

    params_[L_ELBOW].child = NONE;
    params_[L_ELBOW].sister = NONE;
    params_[L_ELBOW].mother = L_SHO_ROLL;

    params_[R_HIP_YAW].child = R_HIP_ROLL;
    params_[R_HIP_YAW].sister = L_HIP_YAW;
    params_[R_HIP_YAW].mother = TRUNK;

    params_[L_HIP_YAW].child = L_HIP_ROLL;
    params_[L_HIP_YAW].sister = HEAD_PAN;
    params_[L_HIP_YAW].mother = TRUNK;

    params_[R_HIP_ROLL].child = R_HIP_PITCH;
    params_[R_HIP_ROLL].sister = NONE;
    params_[R_HIP_ROLL].mother = R_HIP_YAW;

    params_[L_HIP_ROLL].child = L_HIP_PITCH;
    params_[L_HIP_ROLL].sister = NONE;
    params_[L_HIP_ROLL].mother = L_HIP_YAW;

    params_[R_HIP_PITCH].child = R_KNEE;
    params_[R_HIP_PITCH].sister = NONE;
    params_[R_HIP_PITCH].mother = R_HIP_ROLL;

    params_[L_HIP_PITCH].child = L_KNEE;
    params_[L_HIP_PITCH].sister = NONE;
    params_[L_HIP_PITCH].mother = L_HIP_ROLL;

    params_[R_KNEE].child = R_ANK_PITCH;
    params_[R_KNEE].sister = NONE;
    params_[R_KNEE].mother = R_HIP_PITCH;

    params_[L_KNEE].child = L_ANK_PITCH;
    params_[L_KNEE].sister = NONE;
    params_[L_KNEE].mother = L_HIP_PITCH;

    params_[R_ANK_PITCH].child = R_ANK_ROLL;
    params_[R_ANK_PITCH].sister = NONE;
    params_[R_ANK_PITCH].mother = R_KNEE;

    params_[L_ANK_PITCH].child = L_ANK_ROLL;
    params_[L_ANK_PITCH].sister = NONE;
    params_[L_ANK_PITCH].mother = L_KNEE;

    params_[R_ANK_ROLL].child = NONE;
    params_[R_ANK_ROLL].sister = NONE;
    params_[R_ANK_ROLL].mother = R_ANK_PITCH;

    params_[L_ANK_ROLL].child = NONE;
    params_[L_ANK_ROLL].sister = NONE;
    params_[L_ANK_ROLL].mother = L_ANK_PITCH;

    params_[HEAD_PAN].child = HEAD_TILT;
    params_[HEAD_PAN].sister = NONE;
    params_[HEAD_PAN].mother = TRUNK;

    params_[HEAD_TILT].child = NONE;
    params_[HEAD_TILT].sister = NONE;
    params_[HEAD_TILT].mother = HEAD_PAN;

    params_[TRUNK].child = R_SHO_PITCH;
    params_[TRUNK].sister = NONE;
    params_[TRUNK].mother = NONE;

    params_[R_SHO_PITCH].a << .0 << 1.0 << .0;
    params_[L_SHO_PITCH].a << .0 << 1.0 << .0;
    params_[R_SHO_ROLL].a << 1.0 << .0 << .0;
    params_[L_SHO_ROLL].a << 1.0 << .0 << .0;
    params_[R_ELBOW].a << .0 << 1.0 << .0;
    params_[L_ELBOW].a << .0 << 1.0 << .0;
    params_[R_HIP_YAW].a << .0 << .0 << 1.0;
    params_[L_HIP_YAW].a << .0 << .0 << 1.0;
    params_[R_HIP_ROLL].a << 1.0 << .0 << .0;
    params_[L_HIP_ROLL].a << 1.0 << .0 << .0;
    params_[R_HIP_PITCH].a << .0 << 1.0 << .0;
    params_[L_HIP_PITCH].a << .0 << 1.0 << .0;
    params_[R_KNEE].a << .0 << 1.0 << .0;
    params_[L_KNEE].a << .0 << 1.0 << .0;
    params_[R_ANK_PITCH].a << .0 << 1.0 << .0;
    params_[L_ANK_PITCH].a << .0 << 1.0 << .0;
    params_[R_ANK_ROLL].a << 1.0 << .0 << .0;
    params_[L_ANK_ROLL].a << 1.0 << .0 << .0;
    params_[HEAD_PAN].a << .0 << .0 << 1.0;
    params_[HEAD_TILT].a << .0 << 1.0 << .0;

    params_[R_SHO_PITCH].c << params_[R_SHO_PITCH].p.at(0)
                           << params_[R_SHO_PITCH].p.at(1)
                           << params_[R_SHO_PITCH].p.at(2) - JOINT_RADIUS - ARM_HEIGHT * .5;
    params_[L_SHO_PITCH].c << params_[L_SHO_PITCH].p.at(0)
                           << params_[L_SHO_PITCH].p.at(1)
                           << params_[L_SHO_PITCH].p.at(2) - JOINT_RADIUS - ARM_HEIGHT * .5;
    params_[R_SHO_ROLL].c << params_[R_SHO_ROLL].p.at(0)
                          << params_[R_SHO_ROLL].p.at(1)
                          << params_[R_SHO_ROLL].p.at(2)- JOINT_RADIUS - ARM_HEIGHT * .5;
    params_[L_SHO_ROLL].c << params_[L_SHO_ROLL].p.at(0)
                          << params_[L_SHO_ROLL].p.at(1)
                          << params_[L_SHO_ROLL].p.at(2) - JOINT_RADIUS - ARM_HEIGHT * .5;
    params_[R_ELBOW].c << params_[R_ELBOW].p.at(0)
                       << params_[R_ELBOW].p.at(1)
                       << params_[R_ELBOW].p.at(2) - JOINT_RADIUS - FOREARM_HEIGHT * .5;
    params_[L_ELBOW].c << params_[L_ELBOW].p.at(0)
                       << params_[L_ELBOW].p.at(1)
                       << params_[L_ELBOW].p.at(2) - JOINT_RADIUS - FOREARM_HEIGHT * .5;
    params_[R_HIP_YAW].c << params_[R_HIP_YAW].p.at(0)
                         << params_[R_HIP_YAW].p.at(1)
                         << params_[R_HIP_YAW].p.at(2) - JOINT_RADIUS - (THIGH_HEIGHT * .5);
    params_[L_HIP_YAW].c << params_[L_HIP_YAW].p.at(0)
                         << params_[L_HIP_YAW].p.at(1)
                         << params_[L_HIP_YAW].p.at(2) - JOINT_RADIUS - (THIGH_HEIGHT * .5);
    params_[R_HIP_ROLL].c << params_[R_HIP_ROLL].p.at(0)
                          << params_[R_HIP_ROLL].p.at(1)
                          << params_[R_HIP_ROLL].p.at(2) - JOINT_RADIUS - (THIGH_HEIGHT * .5);
    params_[L_HIP_ROLL].c << params_[L_HIP_ROLL].p.at(0)
                          << params_[L_HIP_ROLL].p.at(1)
                          << params_[L_HIP_ROLL].p.at(2) - JOINT_RADIUS - (THIGH_HEIGHT * .5);
    params_[R_HIP_PITCH].c << params_[R_HIP_PITCH].p.at(0)
                           << params_[R_HIP_PITCH].p.at(1)
                           << params_[R_HIP_PITCH].p.at(2) - JOINT_RADIUS - (THIGH_HEIGHT * .5);
    params_[L_HIP_PITCH].c << params_[L_HIP_PITCH].p.at(0)
                           << params_[L_HIP_PITCH].p.at(1)
                           << params_[L_HIP_PITCH].p.at(2) - JOINT_RADIUS - (THIGH_HEIGHT * .5);
    params_[R_KNEE].c << params_[R_KNEE].p.at(0)
                      << params_[R_KNEE].p.at(1)
                      << params_[R_KNEE].p.at(2) - JOINT_RADIUS - (CALF_HEIGHT * .5);
    params_[L_KNEE].c << params_[L_KNEE].p.at(0)
                      << params_[L_KNEE].p.at(1)
                      << params_[L_KNEE].p.at(2) - JOINT_RADIUS - (CALF_HEIGHT * .5);
    params_[R_ANK_PITCH].c << params_[R_ANK_PITCH].p.at(0)
                           << params_[R_ANK_PITCH].p.at(1)
                           << params_[R_ANK_PITCH].p.at(2) - JOINT_RADIUS - FOOT_HEIGHT * .5;
    params_[L_ANK_PITCH].c << params_[L_ANK_PITCH].p.at(0)
                           << params_[L_ANK_PITCH].p.at(1)
                           << params_[L_ANK_PITCH].p.at(2) - JOINT_RADIUS - FOOT_HEIGHT * .5;
    params_[HEAD_PAN].c << params_[HEAD_PAN].p.at(0)
                        << params_[HEAD_PAN].p.at(1)
                        << params_[HEAD_PAN].p.at(2);
    params_[HEAD_PAN].c << params_[HEAD_PAN].p.at(0)
                        << params_[HEAD_PAN].p.at(1)
                        << params_[HEAD_PAN].p.at(2);

    for(auto& params:params_){
        params.R << 1.0 << .0 << .0 << endr
                 << .0 << 1.0 << .0 << endr
                 << .0 << .0 << 1.0 << endr;
        params.v.fill(.0);
        params.w.fill(.0);
        params.q = .0;
        params.dq = .0;
        params.ddq = .0;
        params.m = .1;
        if(params.mother == JointData::NONE)
            params.b = params.p;
        else
            params.b = params.p - params_[params.mother].p;

    }

}
