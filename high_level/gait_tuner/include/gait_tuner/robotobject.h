/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once
#include <GL/gl.h>

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QMatrix3x3>
#include <QMatrix4x4>

#include "joint_data/joint_data.h"
#include "joint_states/joint_states.h"
#include "kinematics/kinematics.h"

typedef QVector<GLfloat> Obj;
typedef QVector<GLfloat > Inst;

class RobotObject{
private:
    Obj target_;
    Inst target_inst_;

    GLfloat current_instance_;

    void setCurrentInstance(GLfloat _instance){
        current_instance_ = _instance;
    }

    void genTriangle(Obj &_target,
                     Inst &_target_inst,
                     QVector3D v1,
                     QVector3D v2,
                     QVector3D v3);
    void genRectangle(Obj &_target, Inst &_target_inst, QVector3D v1, QVector3D v2, QVector3D v3, QVector3D v4);

    void genBlock(Obj &_target, Inst &_target_inst, QVector3D v1, QVector3D v2, QVector3D v3, QVector3D v4,
                  QVector3D v5, QVector3D v6, QVector3D v7, QVector3D v8);
    void genBlock(Obj &_target, Inst &_target_inst, QVector3D center, QVector3D param);

    void genTrunk(Obj &_target, Inst &_target_inst, QVector3D center, QVector3D param, float hip_len_ratio);

    void genSphere(Obj &_target, Inst &_target_inst, QVector3D center, float radius);

    QVector3D trunk_pos;
    QVector3D shoulder_pos;
    QVector3D r_sho_joint_pos;
    QVector3D l_sho_joint_pos;
    QVector3D r_elbow_joint_pos;
    QVector3D l_elbow_joint_pos;
    QVector3D r_hip_joint_pos;
    QVector3D l_hip_joint_pos;
    QVector3D r_knee_joint_pos;
    QVector3D l_knee_joint_pos;
    QVector3D r_ank_joint_pos;
    QVector3D l_ank_joint_pos;
    QVector3D head_joint_pos;    

public:
    RobotObject();

    std::array<QVector3D, JointData::NUM_OF_JOINTS > init_joint_pos;
    std::array<QVector3D, JointData::NUM_OF_JOINTS > init_rotary_axis;

    // Real robot coordinate different with OpenGL coordinate
    static inline QVector3D arma2qt(const arma::colvec& _in){
        QVector3D res(_in.at(1), _in.at(2), _in.at(0));
        return res;
    }

    static inline arma::colvec qt2arma(const QVector3D& _in){
        return arma::colvec{_in.z(),_in.x(),_in.y()};
    }

    static inline QMatrix3x3 arma2qt(const arma::mat& _in){
        arma::mat first(_in);
        first.swap_cols(0,1);
        first.swap_cols(1,2);
        first.swap_rows(0,1);
        first.swap_rows(1,2);
        float last_data[] = {(float)first.at(0,0),(float)first.at(0,1),(float)first.at(0,2),
                              (float)first.at(1,0),(float)first.at(1,1),(float)first.at(1,2),
                              (float)first.at(2,0),(float)first.at(2,1),(float)first.at(2,2)};
        QMatrix3x3 last(last_data);
        return last;
    }

//    static inline mat qt2arma(const QMatrix3x3 &_in){
////        mat first(double)
//        mat a(0,0,fill::zeros);
//        return a;
//    }

    int totalData() const{
        return target_.size(); // Count all the data in target_
    }

    int vertexCount() const{
        return target_.size()/6; // Each vertex consists 6 data
    }

    const float* robotData() const{
        return target_.constData();
    }

    int instCount() const{
        return target_inst_.size();
    }

    const float* instData() const{
        return target_inst_.constData();
    }
};
