/**
 * @author koseng : lintangerlangga@gmail.com
 * @brief In this fashion, so every motion object should be obligate the rule
 *        that is has MOTION_SIGNATURE, execute and init functtion.
 * @details Due to SFINAE, deduced type will be passed the function or any other member
 *          which is not available in that object, with template feature we can evaluate the motion object
 *          in compile time
 */

#pragma once

#include <memory>

#include "motion_base/motion_base.h"

template <typename T>
class MotionInterface:public MotionBase{
private:
    typename T::MOTION_SIGNATURE signatureCheck(T* _motion){
        (void)_motion;
        return true;
    }

    constexpr auto executeCheck(T* _motion) -> decltype(_motion->execute(), void()){}

    constexpr auto initCheck(T* _motion) -> decltype(_motion->init(), void()){}

    std::unique_ptr<T> motion_;
public:
    MotionInterface(){}

    void execute(){
        motion_->execute();
    }

    void init(){
        motion_ = std::unique_ptr<T>(new T);
        motion_->init();
    }
};
