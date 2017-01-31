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
#include <vector>

#include "RCS.h"
#include "Kinematics.h"
#include "trajectoryMaker.h"
#include "MotionControl.h"
#include "Controller.h"
#include "Demo.h"
#include "Debug.h"
#include "gotraj/gotraj.h"
using namespace gomotion;

namespace RCS {

    class BangBangInterpreter : public IRCSInterpreter {
    protected:
        IKinematicsSharedPtr _kinematics; /**<  kinematics pointer */
        boost::shared_ptr<RCS::CController>_nc;
    public:
        std::vector<double> minrange;
        std::vector<double> maxrange;
        BangBangInterpreter(boost::shared_ptr<RCS::CController> nc,
                IKinematicsSharedPtr k); //  ,
        virtual int ParseCommand(RCS::CanonCmd, RCS::CanonCmd&,
                RCS::CanonWorldModel instatus, RCS::CanonWorldModel &outstatus);
        virtual void SetRange(std::vector<double> minrange, std::vector<double> maxrange);
    };
    

    class GoInterpreter : public IRCSInterpreter {
    protected:
        IKinematicsSharedPtr _kinematics; /**<  kinematics pointer */
        boost::shared_ptr<RCS::CController>_nc;
        uint64_t _lastcmdid;
        boost::shared_ptr<GoTraj> _go;
    public:
        void Init(std::vector<double> jnts) ;
        std::vector<double> minrange;
        std::vector<double> maxrange;
        GoInterpreter(boost::shared_ptr<RCS::CController> nc,
                IKinematicsSharedPtr k); //  ,
        virtual int ParseCommand(RCS::CanonCmd incmd, RCS::CanonCmd& outcmd,
                RCS::CanonWorldModel instatus, RCS::CanonWorldModel &outstatus);
        virtual int ParseJointCommand(RCS::CanonCmd cmd, RCS::CanonCmd &outcmd,
                RCS::CanonWorldModel instatus, RCS::CanonWorldModel &outstatus);
        virtual int ParseWorldCommand(RCS::CanonCmd cmd, RCS::CanonCmd &outcmd,
                RCS::CanonWorldModel instatus, RCS::CanonWorldModel &outstatus);
        virtual int ParseStopCommand(RCS::CanonCmd cmd, RCS::CanonCmd &outcmd,
                RCS::CanonWorldModel instatus, RCS::CanonWorldModel &outstatus);
        virtual void SetRange(std::vector<double> minrange, std::vector<double> maxrange);
    };
};
