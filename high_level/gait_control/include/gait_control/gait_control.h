/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once

#include <ros/ros.h>

#include <boost/thread.hpp>

class GaitControl{
private:

public:
    GaitControl();
    ~GaitControl();

    void setInput();
    void init();
    void execute();
};
