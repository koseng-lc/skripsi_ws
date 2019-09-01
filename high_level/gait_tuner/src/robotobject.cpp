#include "gait_tuner/robotobject.h"

using namespace RobotSpec;
using namespace JointData;

//extern JointStates& jstates_ref;

RobotObject::RobotObject()
    : trunk_pos(QVector3D(JointStates::getInstance().getPos(TRUNK).at(1),
                          JointStates::getInstance().getPos(TRUNK).at(2) - SHOULDER_HEIGHT * .5,
                          JointStates::getInstance().getPos(TRUNK).at(0)))
    , shoulder_pos(QVector3D(JointStates::getInstance().getPos(TRUNK).at(1),
                             JointStates::getInstance().getPos(TRUNK).at(2) * 2.0 - SHOULDER_HEIGHT * .5,
                             JointStates::getInstance().getPos(TRUNK).at(0)))
    , r_sho_joint_pos(arma2qt(JointStates::getInstance().getPos(R_SHO_ROLL)))
    , l_sho_joint_pos(arma2qt(JointStates::getInstance().getPos(L_SHO_ROLL)))
    , r_elbow_joint_pos(arma2qt(JointStates::getInstance().getPos(R_ELBOW)))
    , l_elbow_joint_pos(arma2qt(JointStates::getInstance().getPos(L_ELBOW)))
    , r_hip_joint_pos(arma2qt(JointStates::getInstance().getPos(R_HIP_PITCH)))
    , l_hip_joint_pos(arma2qt(JointStates::getInstance().getPos(L_HIP_PITCH)))
    , r_knee_joint_pos(arma2qt(JointStates::getInstance().getPos(R_KNEE)))
    , l_knee_joint_pos(arma2qt(JointStates::getInstance().getPos(L_KNEE)))
    , r_ank_joint_pos(arma2qt(JointStates::getInstance().getPos(R_ANK_ROLL)))
    , l_ank_joint_pos(arma2qt(JointStates::getInstance().getPos(L_ANK_ROLL))){

    for(auto id:ALL_ID){
        init_joint_pos[id] = arma2qt(JointStates::getInstance().getPos(id));
        init_rotary_axis[id] = arma2qt(JointStates::getInstance().getRotDir(id));
    }

    setCurrentInstance(TRUNK);
    genTrunk(target_, target_inst_, trunk_pos,
             QVector3D(TRUNK_WIDTH, TRUNK_HEIGHT - SHOULDER_HEIGHT, TRUNK_DEPTH),
             HIP2SHO_RATIO);
    genBlock(target_, target_inst_, shoulder_pos,
             QVector3D(SHOULDER_WIDTH, SHOULDER_HEIGHT, SHOULDER_DEPTH));

    setCurrentInstance(R_SHO_ROLL);
    genSphere(target_, target_inst_, r_sho_joint_pos, JOINT_RADIUS);
    genBlock(target_, target_inst_,
             QVector3D(r_sho_joint_pos.x(),
                       r_sho_joint_pos.y() - JOINT_RADIUS - ARM_HEIGHT * .5,
                       r_sho_joint_pos.z()),
             QVector3D(ARM_WIDTH, ARM_HEIGHT, ARM_DEPTH));

    setCurrentInstance(L_SHO_ROLL);
    genSphere(target_, target_inst_, l_sho_joint_pos, JOINT_RADIUS);
    genBlock(target_, target_inst_,
             QVector3D(l_sho_joint_pos.x(),
                       l_sho_joint_pos.y() - JOINT_RADIUS - ARM_HEIGHT * .5,
                       l_sho_joint_pos.z()),
             QVector3D(ARM_WIDTH, ARM_HEIGHT, ARM_DEPTH));

    setCurrentInstance(R_ELBOW);
    genSphere(target_, target_inst_, r_elbow_joint_pos, JOINT_RADIUS);
    genBlock(target_, target_inst_,
             QVector3D(r_elbow_joint_pos.x(),
                       r_elbow_joint_pos.y() - JOINT_RADIUS - FOREARM_HEIGHT * .5,
                       r_elbow_joint_pos.z()),
             QVector3D(FOREARM_WIDTH, FOREARM_HEIGHT, FOREARM_DEPTH));

    setCurrentInstance(L_ELBOW);
    genSphere(target_, target_inst_, l_elbow_joint_pos, JOINT_RADIUS);
    genBlock(target_, target_inst_,
             QVector3D(l_elbow_joint_pos.x(),
                       l_elbow_joint_pos.y() - JOINT_RADIUS - FOREARM_HEIGHT * .5,
                       l_elbow_joint_pos.z()),
             QVector3D(FOREARM_WIDTH, FOREARM_HEIGHT, FOREARM_DEPTH));

    setCurrentInstance(R_HIP_PITCH);
    genSphere(target_, target_inst_, r_hip_joint_pos, JOINT_RADIUS);
    genBlock(target_, target_inst_,
             QVector3D(r_hip_joint_pos.x(),
                       r_hip_joint_pos.y() - JOINT_RADIUS - (THIGH_HEIGHT * .5),
                       r_hip_joint_pos.z()),
             QVector3D(THIGH_WIDTH, THIGH_HEIGHT, THIGH_DEPTH));

    setCurrentInstance(L_HIP_PITCH);
    genSphere(target_, target_inst_, l_hip_joint_pos, JOINT_RADIUS);
    genBlock(target_, target_inst_,
             QVector3D(l_hip_joint_pos.x(),
                       l_hip_joint_pos.y() - JOINT_RADIUS - (THIGH_HEIGHT * .5),
                       l_hip_joint_pos.z()),
             QVector3D(THIGH_WIDTH, THIGH_HEIGHT, THIGH_DEPTH));

    setCurrentInstance(R_KNEE);
    genSphere(target_, target_inst_, r_knee_joint_pos, JOINT_RADIUS);
    genBlock(target_, target_inst_,
             QVector3D(r_knee_joint_pos.x(),
                       r_knee_joint_pos.y() - JOINT_RADIUS - (CALF_HEIGHT * .5),
                       r_knee_joint_pos.z()),
             QVector3D(CALF_WIDTH, CALF_HEIGHT, CALF_DEPTH));

    setCurrentInstance(L_KNEE);
    genSphere(target_, target_inst_, l_knee_joint_pos, JOINT_RADIUS);
    genBlock(target_, target_inst_,
             QVector3D(l_knee_joint_pos.x(),
                       l_knee_joint_pos.y() - JOINT_RADIUS - (CALF_HEIGHT * .5),
                       l_knee_joint_pos.z()),
             QVector3D(CALF_WIDTH, CALF_HEIGHT, CALF_DEPTH));

    setCurrentInstance(R_ANK_ROLL);
    genSphere(target_, target_inst_, r_ank_joint_pos, JOINT_RADIUS);
    genBlock(target_, target_inst_,
             QVector3D(r_ank_joint_pos.x(),
                       r_ank_joint_pos.y() - JOINT_RADIUS - (FOOT_HEIGHT * .5),
                       r_ank_joint_pos.z()),
             QVector3D(FOOT_WIDTH, FOOT_HEIGHT, FOOT_DEPTH));

    setCurrentInstance(L_ANK_ROLL);
    genSphere(target_, target_inst_, l_ank_joint_pos, JOINT_RADIUS);
    genBlock(target_, target_inst_,
             QVector3D(l_ank_joint_pos.x(),
                       l_ank_joint_pos.y() - JOINT_RADIUS - (FOOT_HEIGHT * .5),
                       l_ank_joint_pos.z()),
             QVector3D(FOOT_WIDTH, FOOT_HEIGHT, FOOT_DEPTH));

    setCurrentInstance(HEAD_TILT);
    genSphere(target_, target_inst_,
              QVector3D(.0, TRUNK_HEIGHT + HEAD_RADIUS, .0), HEAD_RADIUS);

}

/*
Format v1
      /  \
     v3--v2
*/
void RobotObject::genTriangle(Obj &_target, Inst &_target_inst,
                      QVector3D v1,
                      QVector3D v2,
                      QVector3D v3){
    QVector3D n = QVector3D::crossProduct(v1-v3,v1-v2).normalized();
    _target.push_back(v1.x());
    _target.push_back(v1.y());
    _target.push_back(v1.z());
    _target.push_back(n.x());
    _target.push_back(n.y());
    _target.push_back(n.z());
    _target_inst.push_back(current_instance_);

    _target.push_back(v2.x());
    _target.push_back(v2.y());
    _target.push_back(v2.z());
    _target.push_back(n.x());
    _target.push_back(n.y());
    _target.push_back(n.z());
    _target_inst.push_back(current_instance_);

    _target.push_back(v3.x());
    _target.push_back(v3.y());
    _target.push_back(v3.z());
    _target.push_back(n.x());
    _target.push_back(n.y());
    _target.push_back(n.z());
    _target_inst.push_back(current_instance_);
}

/*
Format v1---v2
       |     |
       v4---v3
*/
void RobotObject::genRectangle(Obj &_target, Inst &_target_inst, QVector3D v1, QVector3D v2, QVector3D v3, QVector3D v4){
    genTriangle(_target, _target_inst, v1, v2, v3);
    genTriangle(_target, _target_inst, v1, v3, v4);
}

/*
Format v1---v2
      / |   /|
     v4---v3 |
     |  |  | |
     |  |  | |
     | v5---v6
     |/    |/
     v8---v7
*/
void RobotObject::genBlock(Obj &_target, Inst &_target_inst, QVector3D v1, QVector3D v2, QVector3D v3, QVector3D v4,
              QVector3D v5, QVector3D v6, QVector3D v7, QVector3D v8){
    //up-bottom
    genRectangle(_target, _target_inst, v1, v2, v3, v4);
    genRectangle(_target, _target_inst, v5, v6, v7, v8);

    //front-rear
    genRectangle(_target, _target_inst, v4, v3, v7, v8);
    genRectangle(_target, _target_inst, v2, v1, v5, v6);

    //left-right
    genRectangle(_target, _target_inst, v1, v4, v8, v5);
    genRectangle(_target, _target_inst, v3, v2, v6, v7);
}

void RobotObject::genBlock(Obj &_target, Inst &_target_inst, QVector3D center, QVector3D param){
    genBlock(_target, _target_inst,
                      QVector3D(center.x() - param.x()*.5, center.y() + param.y()*.5, center.z() - param.z()*.5),
                      QVector3D(center.x() + param.x()*.5, center.y() + param.y()*.5, center.z() - param.z()*.5),
                      QVector3D(center.x() + param.x()*.5, center.y() + param.y()*.5, center.z() + param.z()*.5),
                      QVector3D(center.x() - param.x()*.5, center.y() + param.y()*.5, center.z() + param.z()*.5),
                      QVector3D(center.x() - param.x()*.5, center.y() - param.y()*.5, center.z() - param.z()*.5),//--------
                      QVector3D(center.x() + param.x()*.5, center.y() - param.y()*.5, center.z() - param.z()*.5),
                      QVector3D(center.x() + param.x()*.5, center.y() - param.y()*.5, center.z() + param.z()*.5),
                      QVector3D(center.x() - param.x()*.5, center.y() - param.y()*.5, center.z() + param.z()*.5));
}

void RobotObject::genTrunk(Obj &_target, Inst &_target_inst, QVector3D center, QVector3D param, float hip_len_ratio){
    float manip_len = hip_len_ratio*param.x();
    genBlock(_target, _target_inst,
                      QVector3D(center.x() - param.x()*.5, center.y() + param.y()*.5, center.z() - param.z()*.5),
                      QVector3D(center.x() + param.x()*.5, center.y() + param.y()*.5, center.z() - param.z()*.5),
                      QVector3D(center.x() + param.x()*.5, center.y() + param.y()*.5, center.z() + param.z()*.5),
                      QVector3D(center.x() - param.x()*.5, center.y() + param.y()*.5, center.z() + param.z()*.5),
                      QVector3D(center.x() - manip_len*.5, center.y() - param.y()*.5, center.z() - param.z()*.5),//--------
                      QVector3D(center.x() + manip_len*.5, center.y() - param.y()*.5, center.z() - param.z()*.5),
                      QVector3D(center.x() + manip_len*.5, center.y() - param.y()*.5, center.z() + param.z()*.5),
                      QVector3D(center.x() - manip_len*.5, center.y() - param.y()*.5, center.z() + param.z()*.5));
}

void RobotObject::genSphere(Obj &_target, Inst &_target_inst, QVector3D center, float radius){
    constexpr int num_stack = 101;
    constexpr int num_sector = 101;
    constexpr float stack_step = M_PI / num_stack;
    constexpr float sector_step = 2*M_PI / num_sector;
    for(int i = 0; i < num_stack; i++){
        float c_st_p = cos(i * stack_step);
        float s_st_p = sin(i * stack_step);
        float c_st_n = cos((i+1) * stack_step);
        float s_st_n = sin((i+1) * stack_step);
        for(int j = 0; j < num_sector; j++){
            float c_se_p = cos(j * sector_step);
            float s_se_p = sin(j * sector_step);
            float c_se_n = cos((j+1) * sector_step);
            float s_se_n = sin((j+1) * sector_step);
            QVector3D v1(center.x() + radius * s_st_p * s_se_p, center.y() + radius * c_st_p, center.z() + radius * s_st_p * c_se_p);
            QVector3D v2(center.x() + radius * s_st_p * s_se_n, center.y() + radius * c_st_p, center.z() + radius * s_st_p * c_se_n);
            QVector3D v3(center.x() + radius * s_st_n * s_se_p, center.y() + radius * c_st_n, center.z() + radius * s_st_n * c_se_p);
            QVector3D v4(center.x() + radius * s_st_n * s_se_n, center.y() + radius * c_st_n, center.z() + radius * s_st_n * c_se_n);
//            qDebug() << v1 << " ; " << v2 << " ; " << v3 << " ; " << v4;
//            switch(i){
//            case 0:{
                genTriangle(_target, _target_inst, v1, v4, v3);
                genTriangle(_target, _target_inst, v1, v2, v4);
//            }
//            }
        }
    }

}
