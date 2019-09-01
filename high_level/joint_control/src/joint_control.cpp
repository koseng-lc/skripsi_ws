#include "joint_control/joint_control.h"

JointControl::JointControl(){

}

JointControl::~JointControl(){

}

void JointControl::jointAngleCb(const sensor_msgs::JointStateConstPtr &_msg){
    joint_angle_data_.position = _msg->position;

}

void JointControl::init(){
    for(auto id:JointData::ALL_ID){
        joint_angle_data_.name.push_back(JointData::ID2NAME[id]);
        joint_angle_data_.position.push_back(.0);
    }
    joint_angle_sub_ = nh_.subscribe("/joint_control/data", 1, &JointControl::jointAngleCb, this);
}

void JointControl::execute(){
    for(const auto id:JointData::ALL_ID){
        JointStates::getInstance().setAngle(id) = joint_angle_data_.position[id];
    }
}
