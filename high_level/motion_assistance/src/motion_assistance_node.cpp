#include "motion_assistance/motion_assistance.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "motion_assistance_node");

    MotionAssistance motion_assistance;
    motion_assistance.run();

    return 0;
}
