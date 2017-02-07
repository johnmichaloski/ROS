

#include "gokin/gokin.h"
#include "gokin/genserkins.h"

namespace gomotion {

    static go_pose ConvertTfPose(tf::Pose pose) {
        tf::Quaternion q = pose.getRotation();
        return go_pose_this(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
                q.w(), q.x(), q.y(), q.z());

    }

    static tf::Pose ConvertGoPose(go_pose p) {
        return tf::Pose(tf::Quaternion(p.rot.x, p.rot.y, p.rot.z, p.rot.s),
                tf::Vector3(p.tran.x, p.tran.y, p.tran.z));
    }

    GoKin::GoKin() {
        _pGenserkins = boost::shared_ptr<genserkins>(genserkins::create());
        _pGenserkins->SetAngleUnits(1.0);
        _pGenserkins->SetLengthUnits(1.0);
    }

    void GoKin::SetAngleUnitsPerRadian(double d) {
        _pGenserkins->SetAngleUnits(d);
    }

    void GoKin::SetLengthUnitsPerMeter(double d) {
        _pGenserkins->SetLengthUnits(d);
    }

    void GoKin::SetParams(std::vector< std::vector<double>> params) // either URDF or DH
    {
        if (_pGenserkins->set_params(params) != GO_RESULT_OK) {
            throw std::runtime_error("gokin error"); // kin_exception();
        }
    }

    tf::Pose GoKin::fwd(std::vector<double> joints) {
        go_pose world;
        _pGenserkins->fwd(&joints[0], &world);
        tf::Pose pose = ConvertGoPose(world);
        return pose;
    }

    std::vector<double> GoKin::inv(tf::Pose world, std::vector<double>jointest) {
        go_pose pos = ConvertTfPose(world);
        _pGenserkins->inv(&pos, &jointest[0]);
        return jointest;
    }
};