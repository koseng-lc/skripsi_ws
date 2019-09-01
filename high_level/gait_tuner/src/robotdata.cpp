#include "gait_tuner/robotdata.h"

RobotData *RobotData::instance = 0;

RobotData::RobotData():data(JointData::NUM_OF_JOINTS){
    data[JointData::R_SHO_PITCH].name = "r_sho_pitch";
    data[JointData::R_SHO_PITCH].ID = JointData::R_SHO_PITCH;
    data[JointData::R_SHO_PITCH].child = JointData::R_SHO_ROLL;
    data[JointData::R_SHO_PITCH].sister = JointData::L_SHO_PITCH;

    data[JointData::L_SHO_PITCH].name = "l_sho_pitch";
    data[JointData::L_SHO_PITCH].ID = JointData::L_SHO_PITCH;
    data[JointData::L_SHO_PITCH].child = JointData::L_SHO_ROLL;
    data[JointData::L_SHO_PITCH].sister = JointData::R_HIP_YAW;

    data[JointData::R_SHO_ROLL].name = "r_sho_roll";
    data[JointData::R_SHO_ROLL].ID = JointData::R_SHO_ROLL;
    data[JointData::R_SHO_ROLL].child = JointData::R_ELBOW;
    data[JointData::R_SHO_ROLL].sister = -1;

    data[JointData::L_SHO_ROLL].name = "l_sho_roll";
    data[JointData::L_SHO_ROLL].ID = JointData::L_SHO_ROLL;
    data[JointData::L_SHO_ROLL].child = JointData::L_ELBOW;
    data[JointData::L_SHO_ROLL].sister = -1;

    data[JointData::R_ELBOW].name = "r_elbow";
    data[JointData::R_ELBOW].ID = JointData::R_ELBOW;
    data[JointData::R_ELBOW].child = -1;
    data[JointData::R_ELBOW].sister = -1;

    data[JointData::L_ELBOW].name = "l_elbow";
    data[JointData::L_ELBOW].ID = JointData::L_ELBOW;
    data[JointData::L_ELBOW].child = -1;
    data[JointData::L_ELBOW].sister = -1;

    data[JointData::R_HIP_YAW].name = "r_hip_yaw";
    data[JointData::R_HIP_YAW].ID = JointData::R_HIP_YAW;
    data[JointData::R_HIP_YAW].child = JointData::R_HIP_ROLL;
    data[JointData::R_HIP_YAW].sister = JointData::L_HIP_YAW;

    data[JointData::L_HIP_YAW].name = "l_hip_yaw";
    data[JointData::L_HIP_YAW].ID = JointData::L_HIP_YAW;
    data[JointData::L_HIP_YAW].child = JointData::L_HIP_ROLL;
    data[JointData::L_HIP_YAW].sister = JointData::HEAD_PAN;

    data[JointData::R_HIP_ROLL].name = "r_hip_roll";
    data[JointData::R_HIP_ROLL].ID = JointData::R_HIP_ROLL;
    data[JointData::R_HIP_ROLL].child = JointData::R_HIP_PITCH;
    data[JointData::R_HIP_ROLL].sister = -1;

    data[JointData::L_HIP_ROLL].name = "l_hip_roll";
    data[JointData::L_HIP_ROLL].ID = JointData::L_HIP_ROLL;
    data[JointData::L_HIP_ROLL].child = JointData::L_HIP_PITCH;
    data[JointData::L_HIP_ROLL].sister = -1;

    data[JointData::R_HIP_PITCH].name = "r_hip_pitch";
    data[JointData::R_HIP_PITCH].ID = JointData::R_HIP_PITCH;
    data[JointData::R_HIP_PITCH].child = JointData::R_KNEE;
    data[JointData::R_HIP_PITCH].sister = -1;

    data[JointData::L_HIP_PITCH].name = "l_hip_pitch";
    data[JointData::L_HIP_PITCH].ID = JointData::L_HIP_PITCH;
    data[JointData::L_HIP_PITCH].child = JointData::L_KNEE;
    data[JointData::L_HIP_PITCH].sister = -1;

    data[JointData::R_KNEE].name = "r_knee";
    data[JointData::R_KNEE].ID = JointData::R_KNEE;
    data[JointData::R_KNEE].child = JointData::R_ANK_PITCH;
    data[JointData::R_KNEE].sister = -1;

    data[JointData::L_KNEE].name = "l_knee";
    data[JointData::L_KNEE].ID = JointData::L_KNEE;
    data[JointData::L_KNEE].child = JointData::L_ANK_PITCH;
    data[JointData::L_KNEE].sister = -1;

    data[JointData::R_ANK_PITCH].name = "r_ank_pitch";
    data[JointData::R_ANK_PITCH].ID = JointData::R_ANK_PITCH;
    data[JointData::R_ANK_PITCH].child = JointData::R_ANK_ROLL;
    data[JointData::R_ANK_PITCH].sister = -1;

    data[JointData::L_ANK_PITCH].name = "l_ank_pitch";
    data[JointData::L_ANK_PITCH].ID = JointData::L_ANK_PITCH;
    data[JointData::L_ANK_PITCH].child = JointData::L_ANK_ROLL;
    data[JointData::L_ANK_PITCH].sister = -1;

    data[JointData::R_ANK_ROLL].name = "r_ank_roll";
    data[JointData::R_ANK_ROLL].ID = JointData::R_ANK_ROLL;
    data[JointData::R_ANK_ROLL].child = -1;
    data[JointData::R_ANK_ROLL].sister = -1;

    data[JointData::L_ANK_ROLL].name = "l_ank_roll";
    data[JointData::L_ANK_ROLL].ID = JointData::L_ANK_ROLL;
    data[JointData::L_ANK_ROLL].child = -1;
    data[JointData::L_ANK_ROLL].sister = -1;

    data[JointData::HEAD_PAN].name = "head_pan";
    data[JointData::HEAD_PAN].ID = JointData::HEAD_PAN;
    data[JointData::HEAD_PAN].child = JointData::HEAD_TILT;
    data[JointData::HEAD_PAN].sister = -1;

    data[JointData::HEAD_TILT].name = "head_tilt";
    data[JointData::HEAD_TILT].ID = JointData::HEAD_TILT;
    data[JointData::HEAD_TILT].child = -1;
    data[JointData::HEAD_TILT].sister = -1;

    data[JointData::TRUNK].name = "trunk";
    data[JointData::TRUNK].ID = JointData::TRUNK;
    data[JointData::TRUNK].child = JointData::R_SHO_PITCH;
    data[JointData::TRUNK].sister = -1;
}
