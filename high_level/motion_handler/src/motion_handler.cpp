#include "motion_handler/motion_handler.h"

const int MotionHandler::SYSTEM_CYCLE = 8; // milliseconds

MotionHandler::MotionHandler()
    : viz_states_pub_{nh_.advertise<high_level_msgs::VizStates>("/viz/states", 1)}
    , running_{false}{

    active_motion_ = "none";

}

MotionHandler::~MotionHandler(){
    mutex_.lock();
    running_ = false;
    executor_thread_.join();
    ROS_INFO("[MotionHandler] MotionHandler has terminated !!!");
    mutex_.unlock();
}

void MotionHandler::begin(){
    executor_thread_ = boost::thread{boost::bind(&MotionHandler::motionExecutor, this)};
    running_ = true;
}

void MotionHandler::motionExecutor(){
    while(running_){
//        auto t1 = boost::chrono::high_resolution_clock::now();

        if(active_motion_.compare("none") != 0){

//            std::cout << "Curr Addr : " << &motion_list_[active_motion_] << std::endl;

            motion_list_[active_motion_]->execute();

            for(const auto id:JointData::ALL_ID){
                viz_states_.angular_displacement[id] = JointStates::getInstance().getAngle(id)
                        - JointStates::getInstance().getLastAngle(id);
            }

            viz_states_pub_.publish(viz_states_);
        }

//        static tf::TransformBroadcaster tf_bc_;
//        tf::Transform transform;
//        transform.setOrigin(tf::Vector3(.0,.0,.0));
//        tf::Quaternion rot_;
//        rot_.setRPY(0,0,M_PI);
//        transform.setRotation(rot_);
//        tf_bc_.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world",JointData::ID2NAME[JointData::R_KNEE]));

        boost::this_thread::sleep_for(boost::chrono::milliseconds{SYSTEM_CYCLE});

//        auto t2 = boost::chrono::high_resolution_clock::now();
//        auto elapsed_time = boost::chrono::duration_cast<boost::chrono::milliseconds>(t2-t1);
//        std::cout << elapsed_time << std::endl;
    }
}

void MotionHandler::addMotionList(std::string&& _motion_name,
                                  MotionBase* _motion_base){
    _motion_base->init();
    //insertion prevent duplicate key-to-value couple
    motion_list_.insert(std::pair<std::string, MotionBase* >(_motion_name, _motion_base));
}

void MotionHandler::setActiveMotion(const std::string &_motion_name){
    active_motion_ = _motion_name;
}

void MotionHandler::setActiveMotion(std::string&& _motion_name){
    active_motion_ = _motion_name;
}

void MotionHandler::stopMotion(){
    active_motion_ = "none";
}
