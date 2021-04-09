#ifndef darwinopKinematics_h_DEFINED
#define darwinopKinematics_h_DEFINED

#include <math.h>
#include <vector>
#include "Transform.h"

enum {LEG_LEFT = 0, LEG_RIGHT = 1};

//const double PI = 3.14159265358979323846;
const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);


const double neckOffsetZ = .038+0.054;//OP, calculated from spec
const double neckOffsetX = .02;//OP, calculated from spec
const double shoulderOffsetX = .013;//OP, calculated from spec
const double shoulderOffsetY = .082; //op, spec 
const double shoulderOffsetZ = .026; //OP, calculated from spec
const double handOffsetX = .058;
const double handOffsetZ = .0159;
const double upperArmLength = .077;  //OP, spec
const double lowerArmLength = .157;  //OP, spec

const double hipOffsetY = .0507;    //OP, measured
const double hipOffsetZ = .1557;    //OP, Calculated from spec
const double hipOffsetX = .01556;    //OP, Calculated from spec
const double thighLength = .120;  //OP, spec
const double tibiaLength = .120;  //OP, spec
const double footHeight = .05527;   //OP, spec
const double kneeOffsetX = .0;     //OP
const double dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
const double aThigh = atan(kneeOffsetX/thighLength);
const double dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX);
const double aTibia = atan(kneeOffsetX/tibiaLength);

Transform darwinop_kinematics_forward_head(const double *q);
Transform darwinop_kinematics_forward_larm(const double *q);
Transform darwinop_kinematics_forward_rarm(const double *q);
Transform darwinop_kinematics_forward_lleg(const double *q);
Transform darwinop_kinematics_forward_rleg(const double *q);

std::vector<double>
darwinop_kinematics_inverse_leg(
			   const Transform trLeg,
			   const int leg,
			   double unused=0);

std::vector<double>
darwinop_kinematics_inverse_lleg(const Transform trLeg, double unused=0);

std::vector<double>
darwinop_kinematics_inverse_rleg(const Transform trLeg, double unused=0);

std::vector<double>
darwinop_kinematics_inverse_legs(
			    const double *pLLeg,
			    const double *pRLeg,
			    const double *pTorso,
			    int legSupport=0);

std::vector<double> darwinop_kinematics_inverse_arm(
			    const double *dArm
			    );
#endif
