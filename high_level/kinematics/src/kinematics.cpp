#include "kinematics/kinematics.h"

//JointStates& jstates_ref = JointStates::getInstance();

Kinematics::Kinematics(){

}

void Kinematics::forwardKinematics(JointData::LinkID _joint_id){
//    std::cout << "Kinematics : " << &JointStates::getInstance() << std::endl;
    if(_joint_id == JointData::NONE){
        return;
    }else if(_joint_id != JointData::TRUNK){
        auto mother_id = JointStates::getInstance().getMother(_joint_id);
        auto mother_rot_mat = JointStates::getInstance().getRotMat(mother_id);

//        if(_joint_id == JointData::R_SHO_PITCH){
//            std::cout << "----- " << JointData::ID2NAME[_joint_id] << std::endl;
//            std::cout << "q : " << JointStates::getInstance().getAngle(_joint_id) << std::endl;
//            JointStates::getInstance().getRotMat(_joint_id).print("R : ");
//            JointStates::getInstance().getRotMat(mother_id).print("R(mother) : ");
//            JointStates::getInstance().getPos(mother_id).print("p(mother) : ");
//            JointStates::getInstance().getPos(_joint_id).print("p : ");
//            JointStates::getInstance().getRelativePos(_joint_id).print("b : ");
//        }

        JointStates::getInstance().setPos(_joint_id) = JointStates::getInstance().getPos(mother_id) + mother_rot_mat * JointStates::getInstance().getRelativePos(_joint_id);
        JointStates::getInstance().setRotMat(_joint_id) = mother_rot_mat
                                           * calcRodrigues(JointStates::getInstance().getRotDir(_joint_id), JointStates::getInstance().getAngle(_joint_id));

//        JointStates::getInstance().getRotMat(_joint_id).print("R : ");
//        JointStates::getInstance().getPos(_joint_id).print("p : ");

    }

    forwardKinematics(JointStates::getInstance().getSister(_joint_id));
    forwardKinematics(JointStates::getInstance().getChild(_joint_id));



}

mat Kinematics::calcRodrigues(const colvec& _joint_axis, double _angle){
    mat identity(3,3,fill::eye);
//    identity.print(" I : ");
    mat a_hat(hat(_joint_axis));
//    _joint_axis.print(" a : ");
//    a_hat.print("a_hat : ");
    mat rotation_matrix = identity
                          + a_hat * sin(_angle)
                          + a_hat * a_hat * (1.0 - cos(_angle));
    return rotation_matrix;
}
