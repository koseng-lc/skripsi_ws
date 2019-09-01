/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once

#include <ros/package.h>
#include <high_level_cfg/GaitEngineConfig.h>

#include <sstream>
#include <cassert>

#include <armadillo>
#include <yaml-cpp/yaml.h>

#include "joint_states/joint_states.h"
#include "kinematics/kinematics.h"

using namespace arma;

class GaitEngine{

public:

    GaitEngine();

    void execute();
    void init();

    enum VelocityAxis{
        Vx = 0,
        Vy,
        Vphi
    };

    enum Leg{
        LEFT = -1,
        RIGHT = 1
    };

    //Motion primitives
    enum MotionPrimtive{
        // Halt Position
        P_HALT_LEG_EXT = 0,
        P_HALT_LEG_ROLL,
        P_HALT_LEG_PITCH,
        P_HALT_FOOT_ROLL,
        P_HALT_FOOT_PITCH,
        // Leg Lifting
        P_LIFT_LEG_EXT,
        // Leg Swing
        P_SWING_LEG_PITCH,
        P_SWING_LEG_ROLL,
        P_SWING_LEG_YAW,
        // Hip Swing Lateral
        P_SWING_HIP_LATERAL,
        // Leaning
        P_LEAN_LEG_PITCH,
        P_LEAN_LEG_ROLL
    };

    // Intuitive Leg
    enum IntuitiveLegParam{
        LEG_EXT = 0,
        YAW_LEG,
        ROLL_LEG,
        PITCH_LEG,
        PITCH_FOOT,
        ROLL_FOOT
    };

    enum Config{
        C1 = 0, C2, C3, C4, C5, C6, C7, C8, C9, C_TAU_0, C_TAU_1,
        C10, C11, C12, C13, C14, C15, C16, C17, C18, C19, C20,
        C21, C22, C23, C24, C25, C26, C27
    };

    inline void setInput(const colvec& _input_vel){
        control_interface_.setInputVelocity(_input_vel);
    }

    inline double getInput(VelocityAxis _v_axis){
        return control_interface_.getInputVelocity(_v_axis);
    }

    inline double getTargetAngle(JointData::LinkID _joint_id) const{
        assert(_joint_id >= JointData::R_HIP_YAW && _joint_id <= JointData::L_ANK_ROLL);
        return target_angle_.at(_joint_id - JointData::R_HIP_YAW);
    }

    inline double getMotionPhase() const{return common_phase_;}

    inline colvec getMotionPrimitiveSignal(Leg _leg) const{
        return motion_pattern_.getActivationSignals(_leg);
    }

    inline colvec getIntuitiveLegValue(Leg _leg) const{
        return leg_interface_.getIntuitiveLegValue(_leg);
    }

    void applyConfig(const high_level_cfg::GaitEngineConfig &_config);

private:

    double common_phase_;
    static colvec vel_;
    static colvec abs_vel_;
    static colvec target_angle_;
//    static std::vector<double > config_;
    static high_level_cfg::GaitEngineConfig config_;

    static inline double& setTargetAngle(JointData::LinkID _joint_id){
        assert(_joint_id >= JointData::R_HIP_YAW && _joint_id <= JointData::L_ANK_ROLL);
        return target_angle_.at(_joint_id - JointData::R_HIP_YAW); // offset to first leg joint id
    }


    class ControlInterface{
    private:
        colvec input_vel_;
    public:
        ControlInterface();
        inline void setInputVelocity(const colvec& _input_vel){
            input_vel_ = _input_vel;
        }

        inline double getInputVelocity(VelocityAxis _v_axis) const{
            return input_vel_.at(_v_axis);
        }

        inline void maintainMotionPhase(double &_motion_phase){
            _motion_phase += config_.C25 +
                             abs_vel_.at(Vx)*config_.C26 +
                             abs_vel_.at(Vy)*config_.C27;

            if(_motion_phase > MathUtils::PI)_motion_phase -= MathUtils::TWO_PI;
        }

        void process(double &_motion_phase); // maintain motion phase
    }control_interface_;

    class MotionPattern{
    private:

        colvec r_signals_;
        colvec l_signals_;        

        double motion_phase_;

        double gamma_const_;

        double start_sine_point_;
        double end_sine_point_;

        // Halt
        inline double haltLegExt();
        inline double haltLegRoll(Leg _leg);
        inline double haltLegPitch();
        inline double haltFootRoll();
        inline double haltFootPitch();
        // Leg Lift
        inline double liftLegExt();
        // Leg Swing
        inline void updateGammaConst();
        inline double legSwingPitch();
        inline double legSwingRoll(Leg _leg);
        inline double legSwingYaw(Leg _leg);
        // Lateral Hip Swing
        inline void updateStartAndStopEndPoints();
        inline double hipSwing();
        //Lean
        inline double leanPitch();
        inline double leanRoll();

        void setMotionPhase(double _motion_phase);

    public:
        MotionPattern();

        void process(double _motion_phase);

        inline colvec getActivationSignals(Leg _leg) const{
            return (_leg == RIGHT) ? r_signals_ : l_signals_;
        }

        inline double getActivationSignals(MotionPrimtive _type, Leg _leg) const{
            return (_leg == RIGHT) ? r_signals_.at(_type) : l_signals_.at(_type);
        }

    }motion_pattern_;

    class LegInterface{
    private:
        colvec r_pattern_;
        colvec l_pattern_;
    public:
        LegInterface();

        void process(const colvec& r_activation_signals, const colvec& l_activation_signals);

        inline double& setIntuitiveLegValue(IntuitiveLegParam _param, Leg _leg);

        inline colvec getIntuitiveLegValue(Leg _leg) const{
            return (_leg == RIGHT) ? r_pattern_ : l_pattern_;
        }

        inline double getIntuitiveLegValue(IntuitiveLegParam _param, Leg _leg) const;

        void intuitiveLegToJointAngle();
    }leg_interface_;

    void loadConfig();

};

/*
 * Control Interface - Input from high level
 * Motion Pattern - generate pattern as rythmic activation signal
 * Leg Interface - pattern are translated into joint angle based on intuitive leg
 */

/*
 * Nested class for prevent other program to use this class
 */

/*
* NOTE :
* - the leg interface assume the thigh and shank are equal lengths
* - push amplitude must be less than halt leg extension
*/
