/**
 * @author koseng : lintangerlangga@gmail.com
 * @brief Common abstract class for motion object
 */

#pragma once

class MotionBase{
public:
    virtual void execute() = 0;
    virtual void init() = 0;

private:
    template <typename T>
    friend class MotionInterface;

    MotionBase(){}
};
