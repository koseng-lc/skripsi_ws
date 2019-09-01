#include "walking/walking.h"

Walking::Walking()
    : walk_cmd_(false){

}

Walking::~Walking(){
    queue_thread_.join();
}

void Walking::execute(){

    if(walk_cmd_){
        gait_engine_.execute();
        gait_control_.execute();
        publishData();
    }
}

void Walking::init(){
    gait_engine_.init();
    gait_control_.init();

    queue_thread_ = boost::thread{boost::bind(&Walking::queueThread, this)};
}

void Walking::queueThread(){

    walking_nh_.setCallbackQueue(&cb_queue_);

    imu_sub_ = walking_nh_.subscribe("/data_provider/imu", 1, &Walking::imuCb, this);
    walk_cmd_sub_ = nh_.subscribe("/walking/cmd", 1, &Walking::walkCmdCb, this);
    walk_input_sub_ = nh_.subscribe("/walking/input", 1, &Walking::walkInputCb, this);

    intuitive_leg_pub_ = walking_nh_.advertise<high_level_msgs::IntuitiveLeg >("/walking/intuitive_leg", 1);
    motion_primitives_pub_ = walking_nh_.advertise<high_level_msgs::MotionPrimitive >("/walking/motion_primitives", 1);
    g_eng_data_pub_ = walking_nh_.advertise<high_level_msgs::GaitEngineData >("/walking/gait_engine/data", 1);

    g_eng_cb_ = boost::bind(&Walking::gaitEngineConfigCb, this, _1, _2);
    g_eng_server_.setCallback(g_eng_cb_);

    // special for any important data that should be real time
    ros::AsyncSpinner spinner(1, &cb_queue_);
    spinner.start();

    ros::Rate loop_rate(30);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::waitForShutdown();
}

void Walking::imuCb(const geometry_msgs::QuaternionConstPtr &_msg){

}

void Walking::walkCmdCb(const high_level_msgs::WalkCmdConstPtr &_msg){
    walk_cmd_ = _msg->walk_status;
}

void Walking::walkInputCb(const high_level_msgs::WalkInputConstPtr &_msg){
    gait_engine_.setInput({_msg->v_x, _msg->v_y, _msg->v_phi});
}

void Walking::gaitEngineConfigCb(high_level_cfg::GaitEngineConfig &_config, uint32_t level){
    (void)level;
    gait_engine_.applyConfig(_config);
}

void Walking::publishData(){

    g_eng_data_.motion_phase = gait_engine_.getMotionPhase();

    auto r_patt{gait_engine_.getIntuitiveLegValue(GaitEngine::RIGHT)};
    high_level_msgs::IntuitiveLeg::_r_pattern_type copy_r_patt{r_patt.at(0),r_patt.at(1),r_patt.at(2),
                                                               r_patt.at(3),r_patt.at(4),r_patt.at(5)};
    g_eng_data_.intuitive_leg.r_pattern = copy_r_patt;

    auto l_patt{gait_engine_.getIntuitiveLegValue(GaitEngine::LEFT)};
    high_level_msgs::IntuitiveLeg::_l_pattern_type copy_l_patt{l_patt.at(0),l_patt.at(1),l_patt.at(2),
                                                               l_patt.at(3),l_patt.at(4),l_patt.at(5)};
    g_eng_data_.intuitive_leg.l_pattern = copy_l_patt;

    auto r_sig{gait_engine_.getMotionPrimitiveSignal(GaitEngine::RIGHT)};
    high_level_msgs::MotionPrimitive::_r_signal_type copy_r_sig{r_sig.at(0),r_sig.at(1),r_sig.at(2),
                                                                r_sig.at(3),r_sig.at(4),r_sig.at(5),
                                                                r_sig.at(6),r_sig.at(7),r_sig.at(8),
                                                                r_sig.at(9),r_sig.at(10),r_sig.at(11)};
    g_eng_data_.motion_primitive.r_signal = copy_r_sig;

    auto l_sig{gait_engine_.getMotionPrimitiveSignal(GaitEngine::LEFT)};
    high_level_msgs::MotionPrimitive::_l_signal_type copy_l_sig{l_sig.at(0),l_sig.at(1),l_sig.at(2),
                                                                l_sig.at(3),l_sig.at(4),l_sig.at(5),
                                                                l_sig.at(6),l_sig.at(7),l_sig.at(8),
                                                                l_sig.at(9),l_sig.at(10),l_sig.at(11)};
    g_eng_data_.motion_primitive.l_signal = copy_l_sig;

    g_eng_data_pub_.publish(g_eng_data_);
}
