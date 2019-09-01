/**
* @author koseng : lintangerlangga@gmail.com
* @brief This module is responsible to transmit command to actuators
* @details Actuators are the most risk component it is possible to dangerous or harmful the user
*          so the framework are made such that there only one way to communicate with actuators
*/

#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <high_level_msgs/VizStates.h>

#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>

#include "joint_states/joint_states.h"
#include "motion_interface/motion_interface.h"

class MotionHandler{
public:
    void addMotionList(std::string&& _motion_name, MotionBase* _motion_base);
    void begin();
    void motionExecutor();

    // string is little bit complex data type, i prefer to steal it
    // but also provide the standard one
    void setActiveMotion(const std::string &_motion_name);
    void setActiveMotion(std::string&& _motion_name);
    void stopMotion();

    MotionHandler();
    ~MotionHandler();

private:
    ros::NodeHandle nh_;

    ros::Publisher viz_states_pub_;
    high_level_msgs::VizStates viz_states_;

    std::map<std::string, MotionBase* > motion_list_;
    std::string active_motion_;

    bool running_;
    static const int SYSTEM_CYCLE;
    /**
     * @brief This individual thread is to make sure the system cycle for communicate with actuators not disturbed
     */
    boost::thread executor_thread_;
    /**
     * @brief To lock the flag variable(running_) when the destructor was called
     */
    boost::mutex mutex_;

};
