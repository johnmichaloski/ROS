//

// RCSInterface.h
//

/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */
#pragma once

#include "RCS.h"
#include <vector>
#include "Kinematics.h"
//#include "Trajectory.h"
#include "trajectoryMaker.h"
#include "MotionControl.h"
#include "Controller.h"
#include "Demo.h"
namespace RCS {

    class BangBangInterpreter : public IRCSInterpreter {
    protected:
        IKinematicsSharedPtr _kinematics; /**<  kinematics pointer */
        boost::shared_ptr<RCS::CController>_nc;
        //NearestJointsLookup &_hints;
    public:
        std::vector<double> minrange;
        std::vector<double> maxrange;
        BangBangInterpreter(boost::shared_ptr<RCS::CController> nc, 
        IKinematicsSharedPtr k); //  ,
        //NearestJointsLookup &hints);
        virtual RCS::CanonCmd ParseCommand(RCS::CanonCmd cmd);
        virtual void SetRange(std::vector<double> minrange, std::vector<double> maxrange);
    };
};