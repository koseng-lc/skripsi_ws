/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once

#include <map>

namespace JointData{

static const int NUM_OF_JOINTS = 21;
enum LinkID{
    R_SHO_PITCH = 0,
    L_SHO_PITCH = 1,
    R_SHO_ROLL = 2,
    L_SHO_ROLL = 3,
    R_ELBOW = 4,
    L_ELBOW = 5,
    R_HIP_YAW = 6,
    L_HIP_YAW = 7,
    R_HIP_ROLL = 8,
    L_HIP_ROLL = 9,
    R_HIP_PITCH = 10,
    L_HIP_PITCH = 11,
    R_KNEE = 12,
    L_KNEE = 13,
    R_ANK_PITCH = 14,
    L_ANK_PITCH = 15,
    R_ANK_ROLL = 16,
    L_ANK_ROLL = 17,
    HEAD_PAN = 18,
    HEAD_TILT = 19,
    TRUNK = 20,
    NONE = -1
};

static const LinkID RIGHT_ARM[] = {R_SHO_PITCH, R_SHO_ROLL, R_ELBOW};
static const LinkID LEFT_ARM[] = {L_SHO_PITCH, L_SHO_ROLL, L_ELBOW};
static const LinkID BOTH_ARM[] = {R_SHO_PITCH, L_SHO_PITCH, R_SHO_ROLL, L_SHO_ROLL, R_ELBOW, L_ELBOW};
static const LinkID RIGHT_LEG[] = {R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANK_PITCH, R_ANK_ROLL};
static const LinkID LEFT_LEG[] = {L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANK_PITCH, L_ANK_ROLL};
static const LinkID BOTH_LEG[] = {R_HIP_YAW, L_HIP_YAW, R_HIP_ROLL, L_HIP_ROLL, R_HIP_PITCH, L_HIP_PITCH,
                               R_KNEE, L_KNEE, R_ANK_PITCH, L_ANK_PITCH, R_ANK_ROLL, L_ANK_ROLL};
static const LinkID ALL_ID[] = {R_SHO_PITCH, L_SHO_PITCH, R_SHO_ROLL, L_SHO_ROLL, R_ELBOW, L_ELBOW,
                                R_HIP_YAW, L_HIP_YAW, R_HIP_ROLL, L_HIP_ROLL, R_HIP_PITCH, L_HIP_PITCH,
                                R_KNEE, L_KNEE, R_ANK_PITCH, L_ANK_PITCH, R_ANK_ROLL, L_ANK_ROLL,
                                HEAD_PAN, HEAD_TILT};

static std::map<LinkID, std::string > ID2NAME{
    {R_SHO_PITCH, "r_sho_pitch"},
    {L_SHO_PITCH, "l_sho_pitch"},
    {R_SHO_ROLL, "r_sho_roll"},
    {L_SHO_ROLL, "l_sho_roll"},
    {R_ELBOW, "r_elbow"},
    {L_ELBOW, "l_elbow"},
    {R_HIP_YAW, "r_hip_yaw"},
    {L_HIP_YAW, "l_hip_yaw"},
    {R_HIP_ROLL, "r_hip_roll"},
    {L_HIP_ROLL, "l_hip_roll"},
    {R_HIP_PITCH, "r_hip_pitch"},
    {L_HIP_PITCH, "l_hip_pitch"},
    {R_KNEE, "r_knee"},
    {L_KNEE, "l_knee"},
    {R_ANK_PITCH, "r_ank_pitch"},
    {L_ANK_PITCH, "l_ank_pitch"},
    {R_ANK_ROLL, "r_ank_roll"},
    {L_ANK_ROLL, "l_ank_roll"},
    {HEAD_PAN, "head_pan"},
    {HEAD_TILT, "head_tilt"},
    {TRUNK, "trunk"}
};

enum Axis{
    X = 0b1,
    Y = 0b10,
    Z = 0b100,
};

}
