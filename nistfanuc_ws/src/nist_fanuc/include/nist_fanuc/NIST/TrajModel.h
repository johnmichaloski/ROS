

#pragma once
#include "RCS.h"
#include <stdarg.h>   
#include <boost/shared_ptr.hpp>

namespace MCL {
    
    const static RCS::Pose constant;
    const static RCS::Pose p_vector;
    const static RCS::Pose EQUALS;
    const static RCS::Pose TOOL;

    class TrajModel {
    public:

        TrajModel() {
            _world.setIdentity();
            _sensor.setIdentity();
            _robot.setIdentity();
            _ee.setIdentity();
        }

        RCS::Pose Identity() {
            RCS::Pose I;
            I.setIdentity();
            return I;
        }

        virtual RCS::Pose World(RCS::Pose in) {
            return _world;
        }

        virtual RCS::Pose Sensor(RCS::Pose in) {
            return _sensor;
        }

        virtual RCS::Pose Robot(RCS::Pose in) {
            return _robot;
        }

        virtual RCS::Pose EndEffector(RCS::Pose in) {
            return _ee;
        }

        virtual RCS::Pose SolveGoal() {
            return _world(sensor) * _ee.inverse();
        }

        RCS::Pose _world;
        RCS::Pose _sensor;
        RCS::Pose _robot;
        RCS::Pose _ee;
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
