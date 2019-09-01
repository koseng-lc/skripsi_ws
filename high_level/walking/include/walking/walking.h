/**
* @author koseng : lintangerlangga@gmail.com
* @brief This module is responsible for executing each process in
*        walking behaviour
* @details Specialization expectedly helped in this manner of code
*          because gait and control are separated
*/

#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Quaternion.h>
#include <high_level_msgs/GaitEngineData.h>
#include <high_level_msgs/IntuitiveLeg.h>
#include <high_level_msgs/MotionPrimitive.h>
#include <high_level_msgs/WalkCmd.h>
#include <high_level_msgs/WalkInput.h>
#include <high_level_cfg/GaitEngineConfig.h>

#include <boost/thread.hpp>

#include "gait_engine/gait_engine.h"
#include "gait_control/gait_control.h"

class Walking{
public:

    typedef bool MOTION_SIGNATURE;

    Walking();
    ~Walking();

    void execute();
    void init();

private:

    // this node handle will utilize global callback
    ros::NodeHandle nh_;

    // and the other one use custom callback queue
    ros::NodeHandle walking_nh_;
    ros::CallbackQueue cb_queue_;

    ros::Subscriber imu_sub_;
    void imuCb(const geometry_msgs::QuaternionConstPtr &_msg);

    bool walk_cmd_;
    ros::Subscriber walk_cmd_sub_;
    void walkCmdCb(const high_level_msgs::WalkCmdConstPtr &_msg);

    ros::Subscriber walk_input_sub_;
    void walkInputCb(const high_level_msgs::WalkInputConstPtr &_msg);

    high_level_msgs::IntuitiveLeg intuitive_leg_;
    ros::Publisher intuitive_leg_pub_;

    high_level_msgs::MotionPrimitive motion_primitives_;
    ros::Publisher motion_primitives_pub_;

    high_level_msgs::GaitEngineData g_eng_data_;
    ros::Publisher g_eng_data_pub_;

    dynamic_reconfigure::Server<high_level_cfg::GaitEngineConfig > g_eng_server_;
    dynamic_reconfigure::Server<high_level_cfg::GaitEngineConfig >::CallbackType g_eng_cb_;
    void gaitEngineConfigCb(high_level_cfg::GaitEngineConfig &_config, uint32_t level);

    void publishData();

    boost::thread queue_thread_;
    void queueThread();

    GaitEngine gait_engine_;
    GaitControl gait_control_;
};
