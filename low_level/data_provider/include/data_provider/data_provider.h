/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once

#include <ros/ros.h>

#include <boost/thread.hpp>

class DataProvider{
private:
    ros::NodeHandle nh_;
    void init();
public:
    DataProvider();
    ~DataProvider();

    void routine();
};
