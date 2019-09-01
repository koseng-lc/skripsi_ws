/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <high_level_msgs/VizStates.h>

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QPainter>
#include <QKeyEvent>
#include <QMouseEvent>

#include <iostream>
#include <sstream>
#include <memory>

#include "gait_tuner/robotobject.h"
#include "gait_tuner/robotdata.h"
#include "joint_states/joint_states.h"
#include "gait_engine/gait_engine.h"
#include "walking/walking.h"
#include "kinematics/kinematics.h"

class Viz:public QOpenGLWidget, protected QOpenGLFunctions{
    Q_OBJECT
private:

//    ros::NodeHandle nh_;

//    ros::Subscriber viz_states_sub_;
//    void vizStatesCb(const high_level_msgs::VizStatesConstPtr &_msg);
//    high_level_msgs::VizStates viz_states_;

    QOpenGLBuffer buffer_;
    QOpenGLBuffer instance_buffer_;
    QOpenGLVertexArrayObject vao_;
    QOpenGLShaderProgram* shader_;

    QMatrix4x4 m_proj_; // camera(view) to homogenous
    int m_proj_loc_;
    QMatrix4x4 m_view_; // world to camera(view)
    int m_view_loc_;
    QMatrix4x4 m_model_[JointData::NUM_OF_JOINTS]; // model to world
    int m_model_loc_;

    int m_normal_loc_;

    int light_pos_loc_;

    int vertices_count_;

    void keyPressEvent(QKeyEvent *e);

    QPoint mouse_ref_;
    void mousePressEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);

//    JointStates joint_states_;
    QMatrix4x4 current_tf_;
    QMatrix4x4 current_rot_;
    QMatrix4x4 joint_axis_rot_[JointData::NUM_OF_JOINTS];
    QVector3D joint_pos_[JointData::NUM_OF_JOINTS];
    QVector3D joint_axis_[JointData::NUM_OF_JOINTS];

    void updateModel(JointData::LinkID _joint_id);
    void updateJointPos(JointData::LinkID _joint_id);
    void updateRotaryAxis(JointData::LinkID _joint_id);

    void settingActions();

    RobotObject* robot_object_;
protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
public:
    Viz();
    ~Viz();

    void updateAngle(JointData::LinkID _joint_id, double _angle);
    void updateAngle();
private slots:
//    void updateRobotPose(int _joint_id, float _value);
    void actionForState(JointData::LinkID _joint_id, double _delta_theta);
};
