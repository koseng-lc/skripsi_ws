/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "joint_states/joint_states.h"

class JointControl{
private:
    ros::NodeHandle nh_;

    ros::Subscriber joint_angle_sub_;
    sensor_msgs::JointState joint_angle_data_;
    void jointAngleCb(const sensor_msgs::JointStateConstPtr& _msg);

public:
    typedef bool MOTION_SIGNATURE;

    void init();
    void execute();

    JointControl();
    ~JointControl();

};
