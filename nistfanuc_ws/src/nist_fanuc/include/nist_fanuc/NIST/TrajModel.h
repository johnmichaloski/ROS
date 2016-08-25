

#pragma once
#include <stdexcept>
#include "RCS.h"
#include "Controller.h"
#include <stdarg.h>   
#include <boost/shared_ptr.hpp>

namespace MCL {
    
    const static RCS::Pose constant;
    const static RCS::Pose p_vector;
    const static RCS::Pose EQUALS;
    const static RCS::Pose TOOL;



    class MotionEquation {
	public:
		static const int DONE=0;
		static const int EQUALS=1;
		static const int ROBOT=2;
		static const int TOOL=3;
		static const int BASE=4;
		static const int SENSOR=5;
		static const int GOAL=6;
		static const int WORLD=7;

        MotionEquation() {
            World().setIdentity();
            WorldInv().setIdentity();
            Tool().setIdentity();
            ToolInv().setIdentity();
            Base().setIdentity();
            BaseInv().setIdentity();
        }
		int make_equation(std::string name, boost::shared_ptr<IKinematics> kin,
			int m1, RCS::Pose p1, ...)
		{
			int type;
			bool bFlag=true;
			RCS::Pose pose;
			Name() = name;
			Kin() = kin;

			EQUATION.clear();
			EQUATION.push_back(m1);
            va_list ap;
            va_start(ap, p1);
			while(bFlag)
			{
				type = va_arg( ap, int );
				if(type == DONE)
				{
					bFlag=true;
					continue;
				}
				EQUATION.push_back(type);
				if(type == EQUALS)
				{
					hs=RHS();
					pose=RCS::Pose(Quaterion(0,0,0,1), Vector3(0,0,0)); // identity
				}
				else pose = va_arg( ap, RCS::Pose );
				CHAIN.push_back(pose);
			}
			va_end(ap);
			EQUATION::iterator it;
			if((it=std::find(EQUATION.begin(), EQUATION.end(), EQUALS)) == EQUATION.end())
				return -1;
			if(std::count (EQUATION.begin(), EQUATION.end(), EQUALS)>1)
				return -1;
			if(std::count (EQUATION.begin(), EQUATION.end(), ROBOT)>1) // only 1 robot for now
				return -1;
			if(std::count (EQUATION.begin(), EQUATION.end(), ROBOT)==0) // must be one robot
				return -1;
			if(EQUATION.size()<3)
				return -1;
			size_t pos = std::distance(it, EQUATION.begin()) ;
			if(pos < 1)
				return -1;

			//
			LHS.clear(); RHS.clear();
			for(int i=0; i< pos; i++)
			{
				LHS.push_back(CHAIN[i]);
			}
			for(int i=pos+1; i< CHAIN.size(); i++)
			{
				RHS.push_back(CHAIN[i]);
			}
		}


        virtual RCS::Pose SetBase(RCS::Pose in) {
            Base()=p;
            BaseInv()=p.inverse();
		}
        virtual RCS::Pose SetWorld(RCS::Pose p) {
            World()=p;
            WorldInv()=p.inverse();
        }
        virtual RCS::Pose SetTool(RCS::Pose p) {
            Tool()=p;
            ToolInv()=p.inverse();
        }

        virtual RCS::Pose FindGoal() {
			if(LHS.size() == 0 || RHS.size() == 0)
				 throw std::runtime_error(" RCS::Pose Solve() no equation");
            // For now assume Robot on LHS
			size_t robotpos = std::distance( std::find(LHS.begin(), LHS.end(), ROBOT), LHS.begin());
			RCS::Pose goal(Quaterion(0,0,0,1), Vector3(0,0,0));
			for(size_t i=0; i< RHS.size(); i++)
			{
				goal=goal*RHS[i];
			}
			bool bPre=true;
 			for(size_t i=0; i< LHS.size(); i++)
			{
				if(i==robotpos)
					bPre=false;
				if(bPre)
					goal=LHS[i].inverse() * goal;
				else
					goal=goal*(LHS[i].inverse());
			}
			return goal;
		}
       virtual std::vector<double> Solve() {
		    RCS::Pose goal= FindGoal();
			std::vector<double> joints = Kin().IK(goal, Cnc.status.currentjoints);
			return joints;
		}
		VAR(Base, RCS::Pose);
		VAR(Tool, RCS::Pose);
		VAR(World, RCS::Pose);
		VAR(BaseInv, RCS::Pose);
		VAR(ToolInv, RCS::Pose);
		VAR(WorldInv, RCS::Pose);
		VAR(Name, std::string);
		VAR(Kin, boost::shared_ptr<IKinematics>);
		VAR(LHS,std::vector<RCS::Pose>);
		VAR(RHS,std::vector<RCS::Pose>);
		VAR(CHAIN,std::vector<RCS::Pose>);
		VAR(EQUATION,std::vector<int>);

    };
    

    class Transform {
    public:

        std::string name;
        std::vector<boost::shared_ptr<RCS::Pose>> arg1;

        make_transform(std::string name, boost::shared_ptr<RCS::Pose> arg1) {
            va_list ap;
            va_start(ap, arg1);
            
            while (size_t i = 0; i < numArgs; i++) {
                int num = va_arg(args, int); // get next argument
                sum += num;
            }

            int m;
            int n = strlen(name) + 1028;
            std::string tmp(n, '0');


            // Kind of a bogus way to insure that we don't
            // exceed the limit of our buffer
            while ((m = _vsnprintf(&tmp[0], n - 1, format, ap)) < 0) {
                n = n + 1028;
                tmp.resize(n, '0');
            }
            va_end(ap);
            return tmp.substr(0, m);
        }

    };

};
