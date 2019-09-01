#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include "data_provider/data_provider.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "data_provider_node");

    DataProvider data_provider;
    data_provider.routine();

    return 0;
}
