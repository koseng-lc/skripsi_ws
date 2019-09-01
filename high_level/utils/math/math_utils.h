/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once

#include <armadillo>

namespace MathUtils{

using namespace arma;

static constexpr auto PI = 3.14159265359;
static constexpr auto TWO_PI = 2 * PI;
static constexpr auto HALF_PI = PI * .5;
static constexpr auto ONE_QTR_PI = PI * .25;

static constexpr auto DEG2RAD = PI / 180.0;
static constexpr auto RAD2DEG = 1.0 / DEG2RAD;

//prevent multiple definition and take that reference only
static inline colvec wedge(const mat& _in){
    if(approx_equal(_in.t(), -_in, "absdiff", .0)){
        return colvec{_in.at(2,1), _in.at(0,2), _in.at(1,0)};
    }
    return colvec{.0, .0, .0};
}

static inline mat hat(const colvec& _in){
    mat res;
    res << .0 << -_in.at(2) << _in.at(1) << endr
        << _in.at(2) << .0 << -_in.at(0) << endr
        << -_in.at(1) << _in.at(0) << .0 << endr;
    return res;
}

}

