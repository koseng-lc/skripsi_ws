/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once

#include <array>

#include <armadillo>

#include "robot_params/robot_spec.h"
#include "joint_data/joint_data.h"

using namespace arma;

struct RobotParams {

public:

    enum{
        X = 0,
        Y,
        Z
    };

    /*
     * Sister ID
     */
    JointData::LinkID sister;

    /*
     * Child ID
     */
    JointData::LinkID child;

    /*
     * Parent ID
     */
    JointData::LinkID mother;

    /*
     * Position in World Coordinates
     */
    colvec::fixed<3> p;

    /*
     * Attitude in World Coordinates
     */
    mat::fixed<3,3> R;

    /*
     * Linear Velocity in World Coordinates
     */
    colvec::fixed<3> v;

    /*
     * Angular Velocity in World Coordinates
     */
    colvec::fixed<3> w;

    /*
     * Joint Angle
     */
    double q;

    /*
     * Last Joint Angle
     */
    double last_q;

    /*
     * Joint Velocity
     */
    double dq;

    /*
     * Joint Acceleration
     */
    double ddq;

    /*
     * Joint Axis Vector(Relative to Parent)
     */
    colvec::fixed<3> a;

    /*
     * Joint Relative Position(Relative to Parent)
     */
    colvec::fixed<3> b;

    /*
     * Shape(Vertex Information, Link Local)
     */
    colvec::fixed<3> vertex;

    /*
     * Shape(Vertex Information (Point Connection))
     */
    colvec::fixed<3> face;

    /*
     * Mass
     */
    double m;

    /*
     * Center of Mass
     */
    colvec::fixed<3> c;

    /*
     * Moment of Intertia
     */
    colvec::fixed<3> I;

    RobotParams();

    ~RobotParams();
};
