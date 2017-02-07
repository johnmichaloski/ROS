

#include <vector>
#include <string>
#include <exception>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <boost/shared_ptr.hpp>



namespace gomotion {
struct genserkins;
#if 1
    struct kin_exception : public std::exception {
        std::string exmsg;

        kin_exception() {
            exmsg = "gokin exception";
        }

        kin_exception(std::string err) {
            exmsg = err;
        }

        virtual const char* what() const throw () {
            return exmsg.c_str();
        }
    };
#endif
    struct GoKin {
        GoKin();
        void SetParams(std::vector< std::vector<double>> params); // either URDF or DH
        tf::Pose fwd(const std::vector<double>joint);
        std::vector<double> inv(const tf::Pose world, std::vector<double>jointest);
        void SetAngleUnitsPerRadian(double d);
        void SetLengthUnitsPerMeter(double d);
    protected:
        boost::shared_ptr<gomotion::genserkins> _pGenserkins;
    };
};