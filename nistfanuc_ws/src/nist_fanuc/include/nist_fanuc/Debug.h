

#pragma once

#include <stdarg.h>
#include <vector>
#include "RCS.h"

template<typename T>
inline std::string VectorDump(std::vector<T> v) {
    std::stringstream s;

    for (size_t i = 0; i < v.size(); i++) {
        s << v[i] << ":";
    }
    s << std::endl;
    return s.str();
}

namespace RCS {

       static const char * sCmd[] = { "CANON_NOOP",
        "CANON_INIT_CANON ",
        "CANON_END_CANON",
        "CANON_MOVE_JOINT",
        "CANON_MOVE_TO",
       "CANON_DWELL",
        "CANON_MESSAGE",
       "CANON_MOVE_THRU ",
        "CANON_SET_COORDINATED_MOTION",
        "CANON_STOP_MOTION",
        "CANON_SET_GRIPPER",
        "CANON_OPEN_GRIPPER ",
        "CANON_CLOSE_GRIPPER",
        "CANON_DRAW_OBJECT",
        "CANON_ERASE_OBJECT",
        "CANON_SET_GRIPPER_POSE",
        "CANON_PICK ",
        "CANON_PLACE "};
    /*!
     * \brief DumpPose takes a urdf pose  and generates a string describing pose. 
     * Can be used as std::cout << DumpPose(pose); 

     */
    inline std::string DumpPose(RCS::Pose & pose) {
        std::stringstream s;

        s << "Translation = " << 1000.0 * pose.getOrigin().x() << ":" << 1000.0 * pose.getOrigin().y() << ":" << 1000.0 * pose.getOrigin().z() << std::endl;
        double roll, pitch, yaw;
        getRPY(pose, roll, pitch, yaw);
        s << "Rotation = " << Rad2Deg(roll) << ":" << Rad2Deg(pitch) << ":" << Rad2Deg(yaw) << std::endl;
        s << "Quaternion = " << pose.getRotation().x() << ":" << pose.getRotation().y() << ":" << pose.getRotation().z() << ":" << pose.getRotation().w() ;
        //s << Crcl::DumpRotationAsCrcl(pose)<< std::endl;
        return s.str();
    }

    inline std::string DumpPoseSimple(RCS::Pose & pose) {
        std::stringstream s;

        s << "Translation = " << 1000.0 * pose.getOrigin().x() << ":" << 1000.0 * pose.getOrigin().y() << ":" << 1000.0 * pose.getOrigin().z();
        double roll, pitch, yaw;
        getRPY(pose, roll, pitch, yaw);
        s << "Rotation = " << Rad2Deg(roll) << ":" << Rad2Deg(pitch) << ":" << Rad2Deg(yaw) ;
        return s.str();
    }

    /*!
     * \brief DumpJoints takes a list of joints and generates a string describing pose. 
     * Can be used as std::cout << DumpPose(pose); 
     */

    inline std::string DumpJoints(JointState joints) {
        std::stringstream s;
        s << VectorDump<double> (joints.position);
        return s.str();
    }

    /*!
     * \brief DumpPose takes a urdf pose  and generates a string describing pose. 
     * Can be used as std::cout << DumpPose(pose); 
     */
    inline std::ostream & operator<<(std::ostream & os, RCS::Pose & pose) {
        std::stringstream s;
        s << "Translation = " << 1000.0 * pose.getOrigin().x() << ":" << 1000.0 * pose.getOrigin().y() << ":" << 1000.0 * pose.getOrigin().z() << std::endl;
        double roll, pitch, yaw;
        getRPY(pose, roll, pitch, yaw);
        s << "Rotation = " << Rad2Deg(roll) << ":" << Rad2Deg(pitch) << ":" << Rad2Deg(yaw) << std::endl;
        s << "Quaterion = " << pose.getRotation().x() << ":" << pose.getRotation().y() << ":" << pose.getRotation().z() << ":" << pose.getRotation().w() ;
        os << s.str();
    }

    /*!
     * \brief DumpQuaterion takes a urdf quaterion  and generates a string describing x,y,z,w coordinates. 
     * Can be used as std::cout << DumpQuaterion(urdf::rotation); 
     */
    inline std::string DumpQuaterion(std::ostream & os, const RCS::Rotation & rot) {
        std::stringstream s;
        s << "Quaterion = ";
        s << boost::format("X=%8.4f") % rot.x() << ":";
        s << boost::format("Y=%8.4f") % rot.y() << ":";
        s << boost::format("Z=%8.4f") % rot.z() << ":";
        s << boost::format("W=%8.4f") % rot.w() << ":";
//       s << std::endl;
        return s.str();
    }
};

inline std::ostream & operator<<(std::ostream & os, const RCS::CanonCmd & cc) {
    os << "Cmd = " << cc.crclcommand << " Status = " << cc.status << std::endl << "Joints = ";

    for (size_t i = 0; i < cc.joints.position.size(); i++) {
        os << i << "=" << Rad2Deg(cc.joints.position[i]) << ":";
    }
    os << std::endl;
    return os;
}

template<typename T>
inline std::vector<T> ToVector(int n, ...) {
    std::vector<T> ds;
    va_list args; // define argument list variable
    va_start(args, n); // init list; point to last
    //   defined argument

    for (int i = 0; i < n; i++) {
        double d = va_arg(args, T); // get next argument
        ds.push_back(d);
    }
    va_end(args); // clean up the system stack
    return ds;
}

template<typename T>
inline std::string MapDump(std::map<std::string, T> m) {
    std::stringstream s;

    for (typename std::map<std::string, T>::iterator it = m.begin(); it != m.end(); it++) {
        s << (*it).first << "=" << (*it).second << std::endl;
    }
    s << std::endl;
    return s.str();
}

