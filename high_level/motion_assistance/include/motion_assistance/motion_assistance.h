/**
 * @author koseng : lintangerlangga@gmail.com
 * @brief Motion framework will processed in this node
 */

/*
 * How to add another motion module :
 * - Include your motion module header file first [1]
 * - Declare your motion module as a argument template MotionInterface(the MotionInterface is pointer)
 *   like [2]
 * - Call addMotionList and pass the pointer(if you use smart pointer call get())
 * - It will automatically call your motion initialization
 * - The use of pointer is to prevent "Object Slicing", which is the information contained
 *   in derived class are missing
 */

#pragma once

#include <ros/ros.h>
#include <high_level_msgs/ActiveMotion.h>

#include <memory>
#include <utility>

#include "motion_handler/motion_handler.h"
#include "motion_interface/motion_interface.h"

//[1] - Include motion module
#include "joint_control/joint_control.h"
#include "walking/walking.h"

class MotionAssistance{
private:
    ros::NodeHandle nh_;

    ros::Subscriber active_motion_sub_;
    void activeMotionCb(const high_level_msgs::ActiveMotionConstPtr &_msg);
    high_level_msgs::ActiveMotion active_motion_;

    std::unique_ptr<MotionHandler > motion_handler_;

    //[2] - Add your motion module
    MotionInterface<Walking >* walking_;
    MotionInterface<JointControl >* joint_control_;

    void process();

public:
    MotionAssistance();
    ~MotionAssistance();

    void init();
    void run();

};
