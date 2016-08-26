

#pragma once
#include <stdexcept>
#include "RCS.h"
#include "Controller.h"
#include <stdarg.h>   
#include <boost/shared_ptr.hpp>
#include "boost/bind.hpp" 
#include "boost/function.hpp" 
using namespace tf;

namespace KinematicChain {

    class MotionEquation {
    public:
        typedef boost::function<RCS::Pose(size_t) > ChainFunc;

        enum EqTypes { DONE = 0, EQUALS, ROBOT, TOOL, BASE, SENSOR, GOAL, WORLD, TABLE};

        MotionEquation() {

        }

        static RCS::Pose Placeholder(size_t) {
            return RCS::Pose(tf::Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));
        }

        int make_equation(std::string name, boost::shared_ptr<IKinematics> kin,
                int m1, ...) {
            int type;
            bool bFlag = true;
            ChainFunc pose;
            Name() = name;
            Kin() = kin;

            EQUATION.clear();
            EQUATION.push_back(m1);
            va_list ap;
            va_start(ap, m1);

            while (bFlag) {
                type = va_arg(ap, int);
                if (type == MotionEquation::DONE) {
                    bFlag = false;
                    continue;
                }
                EQUATION.push_back(type);
            }
            va_end(ap);

            std::vector<int>::iterator it;
            if ((it = std::find(EQUATION.begin(), EQUATION.end(), MotionEquation::EQUALS)) == EQUATION.end())
                return -1;
            if (std::count(EQUATION.begin(), EQUATION.end(), MotionEquation::EQUALS) > 1)
                return -1;
            if (std::count(EQUATION.begin(), EQUATION.end(), MotionEquation::ROBOT) > 1) // only 1 robot for now
                return -1;
            if (std::count(EQUATION.begin(), EQUATION.end(), MotionEquation::ROBOT) == 0) // must be one robot
                return -1;
            if (EQUATION.size() < 3)
                return -1;


            int pos = std::distance( EQUATION.begin(), it);
            if (pos < 1)
                return -1;

            CHAIN.resize(EQUATION.size(), boost::bind(&MotionEquation::Placeholder, _1));
            Poses.resize(EQUATION.size(), RCS::Pose(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0)));
        }

        virtual RCS::Pose FindGoal() {
            // For now assume Robot on LHS
            int eqpos = std::distance( EQUATION.begin(), std::find(EQUATION.begin(), EQUATION.end(), EQUALS));
            int robotpos = std::distance(EQUATION.begin(), std::find(EQUATION.begin(), EQUATION.end(), ROBOT));
            RCS::Pose goal(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));
            for (size_t i = eqpos+1; i < EQUATION.size(); i++) {
                ChainFunc func = CHAIN[i]; // boost::bind(&MotionEquation::Placeholder, _1) ; // *RHS.at(i);
                goal = goal * func(i);
            }
            bool bPre = true;
            for (size_t i = 0; i < eqpos; i++) {
                if (i == robotpos)
                {
                    bPre = false;
                    continue;
                }
                
                if (bPre)
                    goal = CHAIN[i](i).inverse() * goal; // (*LHS.at(i))(i).inverse() * goal;
                else
                    goal = goal * CHAIN[i](i).inverse(); // ((*LHS.at(i))(i).inverse());
            }
            return goal;
        }

        virtual std::vector<double> Solve() {
            RCS::Pose goal = FindGoal();
            std::cout << "Solve Pose= " << DumpPose(goal).c_str();
            std::cout << "Current Joints" << VectorDump(Cnc.status.currentjoints.position).c_str();
            std::vector<double> armjoints;
            armjoints.insert(armjoints.begin(), Cnc.status.currentjoints.position.begin(), Cnc.status.currentjoints.position.begin() + 6);
            std::cout << "Arm Joints" << VectorDump(armjoints).c_str();
            std::vector<double> joints = Kin()->IK(goal, armjoints);
            std::cout << "Solve Joints" << VectorDump(joints).c_str();
            return joints;
        }
        int FindIndex(size_t i) {
            int pos = std::distance(EQUATION.begin(), std::find(EQUATION.begin(), EQUATION.end(), i));
            assert(pos >= 0 && pos < EQUATION.size());
            return pos;
        }

        virtual void SetPoseCallback(EqTypes i, ChainFunc pose = boost::bind(&MotionEquation::Placeholder, _1)) {
            int n = FindIndex(i);
            CHAIN[n] = pose;
        }

        virtual void SetPose(EqTypes i, RCS::Pose pose = RCS::Pose(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0))) {
            int n = FindIndex(i);
            Poses[n] = pose;
        }

        virtual RCS::Pose Identity(size_t i) {
            return RCS::Pose(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));
        }

        virtual RCS::Pose GetPose(size_t i) {
            return Poses[i];
        }
        VAR(Name, std::string);
        VAR(Kin, boost::shared_ptr<IKinematics>);
        std::vector<ChainFunc> CHAIN;
        std::vector<int> EQUATION;
        std::vector<RCS::Pose> Poses;

    };

}