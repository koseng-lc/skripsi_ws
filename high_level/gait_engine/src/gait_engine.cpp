#include "gait_engine/gait_engine.h"

colvec GaitEngine::vel_({.0, .0, .0});
colvec GaitEngine::abs_vel_({.0, .0, .0});
colvec GaitEngine::target_angle_({.0, .0, .0, .0, .0, .0,
                                  .0, .0, .0, .0, .0, .0});
high_level_cfg::GaitEngineConfig GaitEngine::config_;

GaitEngine::GaitEngine()
    : common_phase_(.0){
//    init();    
}

void GaitEngine::init(){
    loadConfig();
}

void GaitEngine::execute(){

    control_interface_.process(common_phase_);
    motion_pattern_.process(common_phase_);
    leg_interface_.process(motion_pattern_.getActivationSignals(RIGHT),
                           motion_pattern_.getActivationSignals(LEFT));
    control_interface_.maintainMotionPhase(common_phase_);


    for(const auto id:JointData::BOTH_LEG){
        JointStates::getInstance().setAngle(id) = getTargetAngle(id);
    }

//    std::cout << "[GaitEngine] Motion Phase : " << common_phase_ << std::endl;

}

void GaitEngine::loadConfig(){
    YAML::Node config_file_;

    try{
        std::stringstream file_path;
        file_path << ros::package::getPath("high_level_cfg") << "/config/gait_config.yaml";
        config_file_ = YAML::LoadFile(file_path.str().c_str());
    }catch(std::exception &e){
        std::cerr << "[GaitEngine] Unable to load config file: " << e.what() << std::endl;
    }

    config_.C1 = config_file_["C1"].as<double>();
    config_.C2 = config_file_["C2"].as<double>();
    config_.C3 = config_file_["C3"].as<double>();
    config_.C4 = config_file_["C4"].as<double>();
    config_.C5 = config_file_["C5"].as<double>();
    config_.C6 = config_file_["C6"].as<double>();
    config_.C7 = config_file_["C7"].as<double>();
    config_.C8 = config_file_["C8"].as<double>();
    config_.C9 = config_file_["C9"].as<double>();
    config_.C_TAU_0 = config_file_["C_TAU_0"].as<double>();
    config_.C_TAU_1 = config_file_["C_TAU_1"].as<double>();
    config_.C10 = config_file_["C10"].as<double>();
    config_.C11 = config_file_["C11"].as<double>();
    config_.C12 = config_file_["C12"].as<double>();
    config_.C13 = config_file_["C13"].as<double>();
    config_.C14 = config_file_["C14"].as<double>();
    config_.C15 = config_file_["C15"].as<double>();
    config_.C16 = config_file_["C16"].as<double>();
    config_.C17 = config_file_["C17"].as<double>();
    config_.C18 = config_file_["C18"].as<double>();
    config_.C19 = config_file_["C19"].as<double>();
    config_.C20 = config_file_["C20"].as<double>();
    config_.C21 = config_file_["C21"].as<double>();
    config_.C22 = config_file_["C22"].as<double>();
    config_.C23 = config_file_["C23"].as<double>();
    config_.C24 = config_file_["C24"].as<double>();
    config_.C25 = config_file_["C25"].as<double>();
    config_.C26 = config_file_["C26"].as<double>();
    config_.C27 = config_file_["C27"].as<double>();

}

void GaitEngine::applyConfig(const high_level_cfg::GaitEngineConfig &_config){
    config_ = _config;
}

// Control Interface

GaitEngine::ControlInterface::ControlInterface()
    : input_vel_({.0, .0, .0}){

}

void GaitEngine::ControlInterface::process(double &_motion_phase){
    vel_.at(Vx) += std::max(-config_.C22, std::min(input_vel_.at(Vx) - vel_.at(Vx), config_.C22));
    vel_.at(Vy) += std::max(-config_.C23, std::min(input_vel_.at(Vy) - vel_.at(Vy), config_.C23));
    vel_.at(Vphi) += std::max(-config_.C24, std::min(input_vel_.at(Vphi) - vel_.at(Vphi), config_.C24));

    abs_vel_.at(Vx) = std::fabs(vel_.at(Vx));
    abs_vel_.at(Vy) = std::fabs(vel_.at(Vy));
    abs_vel_.at(Vphi) = std::fabs(vel_.at(Vphi));
}

// Motion Pattern

GaitEngine::MotionPattern::MotionPattern()
    : r_signals_{.0, .0, .0, .0, .0, .0, .0, .0, .0, .0, .0, .0}
    , l_signals_{.0, .0, .0, .0, .0, .0, .0, .0, .0, .0, .0, .0}
    , gamma_const_(.0)
    , start_sine_point_(.0)
    , end_sine_point_(.0){
}

inline double GaitEngine::MotionPattern::haltLegExt(){
    return config_.C1;
}

inline double GaitEngine::MotionPattern::haltLegRoll(Leg _leg){
    return _leg * config_.C2;
}

inline double GaitEngine::MotionPattern::haltLegPitch(){
    return config_.C3;
}

inline double GaitEngine::MotionPattern::haltFootRoll(){
    return config_.C4;
}

inline double GaitEngine::MotionPattern::haltFootPitch(){
    return config_.C5;
}

inline double GaitEngine::MotionPattern::liftLegExt(){
    double max_vel = std::max(abs_vel_.at(Vx), abs_vel_.at(Vy));
    return (this->motion_phase_ <= 0) ? sin(this->motion_phase_)*(config_.C6 + config_.C7*max_vel) :
                                        sin(this->motion_phase_)*(config_.C8 + config_.C9*max_vel);
}

inline double GaitEngine::MotionPattern::legSwingPitch(){
    return vel_.at(Vx) >= 0 ? gamma_const_ * vel_.at(Vx) * config_.C10 :
                              gamma_const_ * vel_.at(Vx) * config_.C11;
}

inline double GaitEngine::MotionPattern::legSwingRoll(Leg _leg){
    return -gamma_const_ * vel_.at(Vy) * config_.C12
           - _leg * std::max(abs_vel_.at(Vy) * config_.C13,
                             abs_vel_.at(Vphi) * config_.C14);
}

inline double GaitEngine::MotionPattern::legSwingYaw(Leg _leg){
    return gamma_const_ * vel_.at(Vphi) * config_.C15
           -_leg * abs_vel_.at(Vphi) * config_.C16;
}

inline double GaitEngine::MotionPattern::hipSwing(){
    auto delta_const = config_.C_TAU_0 - config_.C_TAU_1 + MathUtils::TWO_PI;
    return config_.C17 * (sin(start_sine_point_ * MathUtils::PI / delta_const)
                           - sin(end_sine_point_ * MathUtils::PI / delta_const));
}

inline double GaitEngine::MotionPattern::leanPitch(){
    return (vel_.at(Vx) >= .0) ? vel_.at(Vx) * config_.C18 : vel_.at(Vx) * config_.C19;
}

inline double GaitEngine::MotionPattern::leanRoll(){
    return vel_.at(Vphi) * abs_vel_.at(Vx) * config_.C20;
}

inline void GaitEngine::MotionPattern::updateGammaConst(){
    auto diff = config_.C_TAU_1 - config_.C_TAU_0;
    if(motion_phase_ >= config_.C_TAU_0 && motion_phase_ < config_.C_TAU_1)
        gamma_const_ = cos(((motion_phase_ - config_.C_TAU_0)/diff) * MathUtils::PI);
    else if(motion_phase_ >= config_.C_TAU_1 && motion_phase_ < MathUtils::PI)
        gamma_const_ = (2.0 * (motion_phase_ - config_.C_TAU_1)/(MathUtils::TWO_PI - diff)) - 1.0;
    else if(motion_phase_ >= -MathUtils::PI && motion_phase_ < config_.C_TAU_0)
        gamma_const_ = (2.0 * (motion_phase_ + MathUtils::TWO_PI - config_.C_TAU_1)/(MathUtils::TWO_PI - diff)) - 1.0;
}

inline void GaitEngine::MotionPattern::updateStartAndStopEndPoints(){
    if(motion_phase_ < config_.C_TAU_0)
        start_sine_point_ = motion_phase_ - config_.C_TAU_1 + MathUtils::TWO_PI;
    else if(motion_phase_ > config_.C_TAU_1)
        start_sine_point_ = motion_phase_ - config_.C_TAU_1;
    else
        start_sine_point_ = .0;

    if(motion_phase_ + MathUtils::PI < config_.C_TAU_0)
        end_sine_point_ = motion_phase_ - config_.C_TAU_1 + 3.0 * MathUtils::PI;
    else if(motion_phase_ + MathUtils::PI > config_.C_TAU_1)
        end_sine_point_ = motion_phase_ - config_.C_TAU_1 + MathUtils::PI;
    else
        end_sine_point_ = .0;

}

void GaitEngine::MotionPattern::setMotionPhase(double _motion_phase){
    motion_phase_ = _motion_phase;
    if(motion_phase_ > MathUtils::PI)
        motion_phase_ -= MathUtils::TWO_PI;
    updateGammaConst();
    updateStartAndStopEndPoints();
}

void GaitEngine::MotionPattern::process(double _motion_phase){

    setMotionPhase(_motion_phase);

    r_signals_[P_HALT_LEG_EXT] = haltLegExt();
    r_signals_[P_HALT_LEG_ROLL] = haltLegRoll(RIGHT);
    r_signals_[P_HALT_LEG_PITCH] = haltLegPitch();
    r_signals_[P_HALT_FOOT_ROLL] = haltFootRoll();
    r_signals_[P_HALT_FOOT_PITCH] = haltFootPitch();

    r_signals_[P_LIFT_LEG_EXT] = liftLegExt();

    r_signals_[P_SWING_LEG_PITCH] = legSwingPitch();
    r_signals_[P_SWING_LEG_ROLL] = legSwingRoll(RIGHT);
    r_signals_[P_SWING_LEG_YAW] = legSwingYaw(RIGHT);

    r_signals_[P_SWING_HIP_LATERAL] = hipSwing();

    r_signals_[P_LEAN_LEG_PITCH] = leanPitch();
    r_signals_[P_LEAN_LEG_ROLL] = leanRoll();

    setMotionPhase(_motion_phase + MathUtils::PI);

    l_signals_[P_HALT_LEG_EXT] = haltLegExt();
    l_signals_[P_HALT_LEG_ROLL] = haltLegRoll(LEFT);
    l_signals_[P_HALT_LEG_PITCH] = haltLegPitch();
    l_signals_[P_HALT_FOOT_ROLL] = haltFootRoll();
    l_signals_[P_HALT_FOOT_PITCH] = haltFootPitch();

    l_signals_[P_LIFT_LEG_EXT] = liftLegExt();

    l_signals_[P_SWING_LEG_PITCH] = legSwingPitch();
    l_signals_[P_SWING_LEG_ROLL] = legSwingRoll(LEFT);
    l_signals_[P_SWING_LEG_YAW] = legSwingYaw(LEFT);

    l_signals_[P_SWING_HIP_LATERAL] = hipSwing();

    l_signals_[P_LEAN_LEG_PITCH] = leanPitch();
    l_signals_[P_LEAN_LEG_ROLL] = leanRoll();

}

// Leg Interface

GaitEngine::LegInterface::LegInterface()
    : r_pattern_{.0, .0, .0, .0, .0, .0}
    , l_pattern_{.0, .0, .0, .0, .0, .0}{
}

void GaitEngine::LegInterface::process(const colvec& r_signals, const colvec& l_signals){
    setIntuitiveLegValue(LEG_EXT, RIGHT) = r_signals[P_HALT_LEG_EXT]
                                           + r_signals[P_LIFT_LEG_EXT];

    setIntuitiveLegValue(YAW_LEG, RIGHT) = r_signals[P_SWING_LEG_YAW];

    setIntuitiveLegValue(ROLL_LEG, RIGHT) = r_signals[P_HALT_LEG_ROLL]
                                            + r_signals[P_SWING_HIP_LATERAL]
                                            + r_signals[P_SWING_LEG_ROLL]
                                            + r_signals[P_LEAN_LEG_ROLL];

    setIntuitiveLegValue(PITCH_LEG, RIGHT) = r_signals[P_HALT_LEG_PITCH]
                                             + r_signals[P_SWING_LEG_PITCH]
                                             + r_signals[P_LEAN_LEG_PITCH];

    setIntuitiveLegValue(PITCH_FOOT, RIGHT) = r_signals[P_HALT_FOOT_PITCH];

    setIntuitiveLegValue(ROLL_FOOT, RIGHT) = r_signals[P_HALT_FOOT_ROLL];
    //-----------
    setIntuitiveLegValue(LEG_EXT, LEFT) = l_signals[P_HALT_LEG_EXT]
                                        + l_signals[P_LIFT_LEG_EXT];

    setIntuitiveLegValue(YAW_LEG, LEFT) = l_signals[P_SWING_LEG_YAW];

    setIntuitiveLegValue(ROLL_LEG, LEFT) = l_signals[P_HALT_LEG_ROLL]
                                           + l_signals[P_SWING_HIP_LATERAL]
                                           + l_signals[P_SWING_LEG_ROLL]
                                           + l_signals[P_LEAN_LEG_ROLL];

    setIntuitiveLegValue(PITCH_LEG, LEFT) = l_signals[P_HALT_LEG_PITCH]
                                            + l_signals[P_SWING_LEG_PITCH]
                                            + l_signals[P_LEAN_LEG_PITCH];

    setIntuitiveLegValue(PITCH_FOOT, LEFT) = l_signals[P_HALT_FOOT_PITCH];

    setIntuitiveLegValue(ROLL_FOOT, LEFT) = l_signals[P_HALT_FOOT_ROLL];

    intuitiveLegToJointAngle();
}

inline double& GaitEngine::LegInterface::setIntuitiveLegValue(IntuitiveLegParam _param, Leg _leg){
    return (_leg == RIGHT) ? r_pattern_.at(_param) : l_pattern_.at(_param);
}

inline double GaitEngine::LegInterface::getIntuitiveLegValue(IntuitiveLegParam _param, Leg _leg) const{
    return (_leg == RIGHT) ? r_pattern_.at(_param) : l_pattern_.at(_param);
}

void GaitEngine::LegInterface::intuitiveLegToJointAngle(){
    auto lambda_const = acos(1.0 - getIntuitiveLegValue(LEG_EXT, RIGHT));

    auto yaw_angle = -getIntuitiveLegValue(YAW_LEG, RIGHT);

    mat tf_mat(2, 2);
    tf_mat << cos(yaw_angle) << sin(yaw_angle) << endr
           << -sin(yaw_angle) << cos(yaw_angle) << endr;

    colvec untfed_angle;
    untfed_angle << getIntuitiveLegValue(PITCH_LEG, RIGHT) << getIntuitiveLegValue(ROLL_LEG, RIGHT);
    colvec tfed_angle = tf_mat * untfed_angle;    

    GaitEngine::setTargetAngle(JointData::R_HIP_YAW) = yaw_angle;
    GaitEngine::setTargetAngle(JointData::R_HIP_ROLL) = tfed_angle.at(1);
    GaitEngine::setTargetAngle(JointData::R_HIP_PITCH) = tfed_angle.at(0) - lambda_const;
    GaitEngine::setTargetAngle(JointData::R_KNEE) = 2.0 * lambda_const;
    GaitEngine::setTargetAngle(JointData::R_ANK_PITCH) = getIntuitiveLegValue(PITCH_FOOT, RIGHT) - tfed_angle.at(0) - lambda_const;
    GaitEngine::setTargetAngle(JointData::R_ANK_ROLL) = getIntuitiveLegValue(ROLL_FOOT, RIGHT) - tfed_angle.at(1);

    //------------

    lambda_const = acos(1.0 - getIntuitiveLegValue(LEG_EXT, LEFT));

    yaw_angle = -getIntuitiveLegValue(YAW_LEG, LEFT);

    tf_mat.clear();
    tf_mat.set_size(2, 2);
    tf_mat << cos(yaw_angle) << sin(yaw_angle) << endr
           << -sin(yaw_angle) << cos(yaw_angle) << endr;

    untfed_angle.clear();
    untfed_angle << getIntuitiveLegValue(PITCH_LEG, LEFT) << getIntuitiveLegValue(ROLL_LEG, LEFT);

    tfed_angle = tf_mat * untfed_angle;

    GaitEngine::setTargetAngle(JointData::L_HIP_YAW) = yaw_angle;
    GaitEngine::setTargetAngle(JointData::L_HIP_ROLL) = tfed_angle.at(1);
    GaitEngine::setTargetAngle(JointData::L_HIP_PITCH) = tfed_angle.at(0) - lambda_const;
    GaitEngine::setTargetAngle(JointData::L_KNEE) = 2.0 * lambda_const;
    GaitEngine::setTargetAngle(JointData::L_ANK_PITCH) = getIntuitiveLegValue(PITCH_FOOT, LEFT) - tfed_angle.at(0) - lambda_const;
    GaitEngine::setTargetAngle(JointData::L_ANK_ROLL) = getIntuitiveLegValue(ROLL_FOOT, LEFT) - tfed_angle.at(1);

}
