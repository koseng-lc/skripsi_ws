#include "motion_assistance/motion_assistance.h"

MotionAssistance::MotionAssistance()
    : motion_handler_{new MotionHandler}
    , active_motion_sub_{nh_.subscribe("/motion/active_motion", 1,
                                       &MotionAssistance::activeMotionCb,
                                       this)}{

    init();
}

MotionAssistance::~MotionAssistance(){
    delete joint_control_;
    delete walking_;
}

void MotionAssistance::activeMotionCb(const high_level_msgs::ActiveMotionConstPtr &_msg){
    active_motion_.header = _msg->header;
    active_motion_.motion_name = _msg->motion_name;

    // move not steal the info, only convert to rvalue reference
    motion_handler_->setActiveMotion(std::move(active_motion_.motion_name));
}

void MotionAssistance::process(){

}

void MotionAssistance::init(){

    joint_control_ = new MotionInterface<JointControl >;
    walking_ = new MotionInterface<Walking >;

    active_motion_.motion_name = "none";

//    std::cout << "Joint Control Addr : " << &joint_control_ << std::endl;
//    std::cout << "Walking Addr : " << &walking_ << std::endl;

    motion_handler_->addMotionList("joint_control", dynamic_cast<MotionBase* >(joint_control_));
    motion_handler_->addMotionList("walk", dynamic_cast<MotionBase* >(walking_));

    motion_handler_->begin();
}

void MotionAssistance::run(){

    ros::Rate loop_rate(60);

    while(ros::ok()){
        ros::spinOnce();
        process();
        loop_rate.sleep();
    }
}
