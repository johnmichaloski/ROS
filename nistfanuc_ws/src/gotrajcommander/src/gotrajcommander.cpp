

#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/python/converter/registry.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

//#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include "gotraj/gotraj.h"
#include "gotrajcommander/py_conversions.h"
#include "gotrajcommander/Debug.h"
using namespace boost::python;
//using namespace gomotion;
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(InitPose_member_overloads, GoTraj::InitPose, 3, 4)
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(InitJoints_member_overloads, GoTraj::InitJoints, 3, 4)

inline geometry_msgs::Pose ConvertPose(tf::Pose m) {
    geometry_msgs::Pose p;
    p.position.x = m.getOrigin().x();
    p.position.y = m.getOrigin().y();
    p.position.z = m.getOrigin().z();
    p.orientation.x = m.getRotation().x();
    p.orientation.y = m.getRotation().y();
    p.orientation.z = m.getRotation().z();
    p.orientation.w = m.getRotation().w();
    return p;
}

std::string DumpGoTrajParams(gomotion::GoTrajParams params) {
    std::stringstream s;
    s << "Vel=" << params.vel << ":Acc=" << params.acc << "Jerk=" << params.jerk << "\n";
    return s.str();
}

struct GoTraj // : public gomotion::GoTraj, boost::python::wrapper<gomotion::GoTraj>
{
    boost::shared_ptr<gomotion::GoTraj> gm;

    GoTraj() {
        gm = boost::shared_ptr<gomotion::GoTraj>(new gomotion::GoTraj());
    }

    int Init(object here, float time) {
        gomotion::JointState jts;
        std::cout << "Cross our fingers\n";
        jts.position = py_bindings_tools::typeFromList<double > (here.attr("position"));
        //	boost::python::list l =  extract<boost::python::list>(here.attr("position"));
        //	std::vector<double> jts = py_list_to_std_vector<double>(l);
        // THis doesn't work
        //JointState j =  extract<JointState>(here);
        std::cout << "Msg Joints " << RCS::VectorDump<double>(jts.position).c_str() << "\n";

        gm->Init(jts, time);
        std::cout << "Eureka\n";
        return 0;
    }

    bool IsDone() {
        return gm->IsDone();
    }

    int InitJoints(object here,
            object there,
            object params,
            bool bCoordinated) {
        gomotion::JointState herejts, therejts;
        herejts.position = py_bindings_tools::typeFromList<double > (here.attr("position"));
        std::cout << "Here Joints " << RCS::VectorDump<double>(herejts.position).c_str() << "\n";
        therejts.position = py_bindings_tools::typeFromList<double > (there.attr("position"));
        std::cout << "THere Joints " << RCS::VectorDump<double>(therejts.position).c_str() << "\n";
        std::vector<gomotion::GoTrajParams> jtparams = py_bindings_tools::typeFromList<gomotion::GoTrajParams > (params);
        std::cout << "Params " << RCS::VectorDump<double>(therejts.position).c_str() << "\n";
        return gm->InitJoints(herejts, therejts, jtparams, bCoordinated);
    }

    void AppendJoints(object there) {
        gomotion::JointState therejts;
        therejts.position = py_bindings_tools::typeFromList<double > (there.attr("position"));
        std::cout << "Append There Joints " << RCS::VectorDump<double>(therejts.position).c_str() << "\n";
        gm->AppendJoints(therejts);
    }

    gomotion::JointState NextJoints() {
        gomotion::JointState nextjts = gm->NextJoints();
        return nextjts;
    }

    int InitPose(object here,
            object there,
            gomotion::GoTrajParams tparams,
            gomotion::GoTrajParams rparams) {
        tf::Pose h = ToPose(here);
        tf::Pose t = ToPose(there);
        std::cout << "Here Position " << RCS::DumpPoseSimple(h).c_str() << "\n";
        std::cout << "There Position " << RCS::DumpPoseSimple(t).c_str() << "\n";
        std::cout << "tparams " << DumpGoTrajParams(tparams);
        std::cout << "rparams " << DumpGoTrajParams(rparams);
        gm->InitPose(h, t, tparams, rparams);
        return 0;
    }

    geometry_msgs::Pose NextPose() {
        tf::Pose pose = gm->NextPose();
        //std::cout << "Next Position " << RCS::DumpPoseSimple(pose).c_str() << "\n";
        return ConvertPose(pose);
        //object obj=FromPose(pose);
        //return obj;
    }

    void AppendPose(object where) {
        tf::Pose pose = ToPose(where);
        gm->AppendPose(pose);
    }

    void InitStop() {
        gm->InitStop();
    }
protected:

    tf::Pose ToPose(object here) {
        std::vector<double> p, q;
        p.push_back(extract<double>(here.attr("position").attr("x")));
        p.push_back(extract<double>(here.attr("position").attr("y")));
        p.push_back(extract<double>(here.attr("position").attr("z")));
        q.push_back(extract<double>(here.attr("orientation").attr("x")));
        q.push_back(extract<double>(here.attr("orientation").attr("y")));
        q.push_back(extract<double>(here.attr("orientation").attr("z")));
        q.push_back(extract<double>(here.attr("orientation").attr("w")));

        tf::Vector3 pos(p[0], p[1], p[2]);
        tf::Quaternion quat(q[0], q[1], q[2], q[3]);
        tf::Pose pose = tf::Pose(quat, pos);
        return pose;
    }

    // This doesn't work - you need to create PyObject with correct info
    object FromPose(tf::Pose pose) {
        object here;
        here.attr("position").attr("x") = pose.getOrigin().x();
        here.attr("position").attr("y") = pose.getOrigin().y();
        here.attr("position").attr("z") = pose.getOrigin().z();
        here.attr("orientation").attr("x") = pose.getRotation().x();
        here.attr("orientation").attr("y") = pose.getRotation().y();
        here.attr("orientation").attr("z") = pose.getRotation().z();
        here.attr("orientation").attr("w") = pose.getRotation().w();
        return here;
    }
};

typedef std::vector<double> MyList;

BOOST_PYTHON_MODULE(gotrajcommander) {

    class_<geometry_msgs::Point>("Point")
            .def_readwrite("x", &geometry_msgs::Point::x)
            .def_readwrite("y", &geometry_msgs::Point::y)
            .def_readwrite("z", &geometry_msgs::Point::z)
            ;
    class_<geometry_msgs::Quaternion>("Quaternion")
            .def_readwrite("x", &geometry_msgs::Quaternion::x)
            .def_readwrite("y", &geometry_msgs::Quaternion::y)
            .def_readwrite("z", &geometry_msgs::Quaternion::z)
            .def_readwrite("w", &geometry_msgs::Quaternion::w)
            ;
    class_<geometry_msgs::Pose>("Pose")
            .def_readwrite("position", &geometry_msgs::Pose::position)
            .def_readwrite("orientation", &geometry_msgs::Pose::orientation)
            ;

    class_<MyList>("MyList")
            .def(vector_indexing_suite<MyList>());

    class_<gomotion::JointState >("JointState")
            .def_readwrite("header", &sensor_msgs::JointState_<std::allocator<void> >::header)
            .def_readwrite("name", &sensor_msgs::JointState_<std::allocator<void> >::name)
            .def_readwrite("position", &sensor_msgs::JointState_<std::allocator<void> >::position)
            .def_readwrite("velocity", &sensor_msgs::JointState_<std::allocator<void> >::velocity)
            .def_readwrite("effort", &sensor_msgs::JointState_<std::allocator<void> >::effort)
            ;

    class_<gomotion::GoTrajParams>("GoTrajParams", init<double, double, double>())
            .def_readwrite("vel", &gomotion::GoTrajParams::vel)
            .def_readwrite("acc", &gomotion::GoTrajParams::acc)
            .def_readwrite("jerk", &gomotion::GoTrajParams::jerk)
            ;

    class_<GoTraj>("GoTraj", init<>())
            .def("Init", &GoTraj::Init)
            .def("InitJoints", &GoTraj::InitJoints)
            .def("IsDone", &GoTraj::IsDone, "Is motion done that is is queue empty")
            .def("AppendJoints", &GoTraj::AppendJoints, "Append another set of joints to motion queue ")
            .def("NextJoints", &GoTraj::NextJoints, "Get next joints from motion queue")
            .def("InitPose", &GoTraj::InitPose, "InitPose on motion queue")
            .def("NextPose", &GoTraj::NextPose, "Get next pose from motion queue")
            .def("AppendPose", &GoTraj::AppendPose, "Append another pose onto motion queue")
            .def("InitStop", &GoTraj::InitStop, "Stop motion in queue")
    // FIXME: The time based gotraj has not been implemented in python
#if 0
            .def("InitPose", &GoTraj::InitPose, InitPose_member_overloads(
            args("here", "there", "tparams", "rparams"), "InitPose"
            )
            )
            .def("InitJoints", &GoTraj::InitJoints, InitJoints_member_overloads(
            args("here", "there", "params", "bCoordinated"), "Init Joints"
            )
            )
#endif

            ;

}

