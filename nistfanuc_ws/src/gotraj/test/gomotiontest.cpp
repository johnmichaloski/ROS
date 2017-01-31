
#include <gtest/gtest.h>
#include <typeinfo>
#include <cxxabi.h>
#include <iostream>
#include <fstream>
#include <stdarg.h>
/*!
 * \brief StrFormat  accepts a traditional C format string and expects parameter to follow on calling stack and will
 * produce a string from it.
 * \param fmt is the C format string.
 */
std::string StrFormat(const char *fmt, ...) {
    va_list argptr;
    va_start(argptr, fmt);
    int m;
    int n = (int) strlen(fmt) + 1028;
    std::string tmp(n, '0');
    while ((m = vsnprintf(&tmp[0], n - 1, fmt, argptr)) < 0) {
        n = n + 1028;
        tmp.resize(n, '0');
    }
    va_end(argptr);
    return tmp.substr(0, m);
}

// ROS
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>

// Go motion
#include "gomotion/gomath.h"
#include "gomotion/gotypes.h"
#include "gomotion/gointerp.h"
#include "gomotion/gotraj.h"
#include "gomotion/gomotion.h"
#include "gomotion/gomove.h"

#ifndef Deg2Rad
#define Deg2Rad(Ang)    ( (double) ( Ang * M_PI / 180.0 ) )
#define Rad2Deg(Ang)    ( (double) ( Ang * 180.0 / M_PI ) )
#define MM2Meter(d)     ( (double) ( d / 1000.00 ) )
#define Meter2MM(d)     ( (double) ( d * 1000.00 ) )
#endif

#if defined(GO_RESULT_CHAR)
int go_result_char = 1;

#elif defined(GO_RESULT_SHORT)
int go_result_short = 1;

#else
int go_result_int = 1;
#endif

#if defined(GO_REAL_FLOAT)
int go_real_float = 1;

#elif defined(GO_REAL_LONG_DOUBLE)
int go_real_long_double = 1;

#else
int go_real_double = 1;
#endif

#if defined(GO_INTEGER_SHORT)
int go_integer_short = 1;

#elif defined(GO_INTEGER_LONG)
int go_integer_long = 1;

#elif defined(GO_INTEGER_LONG_LONG)
int go_integer_long_long = 1;

#else
int go_integer_int = 1;
#endif

#if defined(GO_FLAG_USHORT)
int go_flag_ushort = 1;

#elif defined(GO_FLAG_UINT)
int go_flag_uint = 1;

#else
int go_flag_uchar = 1;
#endif

/* the global ad-hoc flag for tracing code */
go_flag gocode = 0;

using namespace gomotion;
static go_pose ConvertTfPose(tf::Pose pose) {
    go_pose p;
    p.tran.x = pose.getOrigin().x();
    p.tran.y = pose.getOrigin().y();
    p.tran.z = pose.getOrigin().z();
    p.rot.x = pose.getRotation().x();
    p.rot.y = pose.getRotation().y();
    p.rot.z = pose.getRotation().z();
    p.rot.s = pose.getRotation().w();
    return p;
}

static tf::Pose ConvertGoPose(go_pose p) {
    tf::Pose pose;
    pose.setOrigin(tf::Vector3(p.tran.x, p.tran.y, p.tran.z));
    pose.setRotation(tf::Quaternion(p.rot.x, p.rot.y, p.rot.z, p.rot.s));
    return pose;
}
    
/* user defined functions that must exit to link*/
int go_init() {
    return 0;
}

int go_exit() {
    return 0;
}

enum {
    QUEUE_SIZE = 4
}; /* < 3 tests for good length ha:ndling */
go_real gotime, deltat = 0.01;
go_motion_spec gms, space[QUEUE_SIZE];
go_motion_queue gmq;
go_position position;
go_integer i;

TEST(GoMotion, type) {
    /*
     *  Primitive types: go_result, go_real, go_integer, go_flag
     */
    // Mangling causes the names to be cutoff.... 
    // Workaround from http://stackoverflow.com/questions/81870/is-it-possible-to-print-a-variables-type-in-standard-c
    int status = 0;
    char* demangled = abi::__cxa_demangle(typeid (go_real).name(), 0, 0, &status);

    std::cout << "go motion linear units  = METER\n" ;
    std::cout << "go motion angular units = RADIAN\n" ;

    std::cout << "go_real type = " << demangled << "\n";
    std::cout << "go_real size = " << sizeof (go_real) << "\n";

    demangled = abi::__cxa_demangle(typeid (go_integer).name(), 0, 0, &status);
    std::cout << "go_integer type = " << demangled << "\n";
    std::cout << "go_integer size = " << sizeof (go_integer) << "\n";

    demangled = abi::__cxa_demangle(typeid (go_flag).name(), 0, 0, &status);
    std::cout << "go_flag type = " <<demangled << "\n";
    std::cout << "go_flag size = " << sizeof (go_flag) << "\n";

}

TEST(GoMotion, conversion){
    tf::Quaternion q(Deg2Rad(180.0), 0.0, Deg2Rad(180.0));
    tf::Pose pose(q, 
            tf::Vector3(-0.060, -0.060, 0.010) );
    std::cout << "Conversion1: tf to go_pose\n";
    std::cout << "    X=" << pose.getOrigin().x() << ":Y=" << pose.getOrigin().y() << ":Z=" << pose.getOrigin().z() << "\n";
    std::cout << "    QX=" << q.x() << ":QY=" << q.y() << ":QZ=" << q.z() << ":QW=" << q.w() << "\n";

    std::cout << "    go_pose=\n";
    go_pose p = ConvertTfPose(pose);
    std::cout << "    X=" << p.tran.x << ":Y=" << p.tran.y << ":Z=" << p.tran.z << "\n";
    std::cout << "    QX=" << p.rot.x << ":QY=" << p.rot.y << ":QZ=" << p.rot.z << ":QW=" << p.rot.s << "\n";
    
    std::cout << "Conversion2: go_pose to tf\n";
    pose = ConvertGoPose( p);
    q=pose.getRotation();
    std::cout << "C   tf=\n";
    std::cout << "    X=" << pose.getOrigin().x() << ":Y=" << pose.getOrigin().y() << ":Z=" << pose.getOrigin().z() << "\n";
    std::cout << "    QX=" << q.x() << ":QY=" << q.y() << ":QZ=" << q.z() << ":QW=" << q.w() << "\n";

}

TEST(GoMotion, init) {
    ASSERT_EQ(0, go_init());
    ASSERT_EQ(GO_RESULT_OK, gmq.init(space, QUEUE_SIZE, deltat));
}

TEST(GoMotion, world) {
    gotime = 0.0;
    ASSERT_TRUE(GO_RESULT_OK == gmq.set_type(GO_MOTION_WORLD));

    gms.init();
    gms.set_type(GO_MOTION_LINEAR);
    gms.set_id(1);
    position.u.pose = go_pose_this(0, 0, 0, 1, 0, 0, 0);
    gmq.set_here(&position); // jlm added otherwise no starting position

    gms.set_end_position(&position);
    // set_tpar( go_real vel, go_real acc, go_real jerk)
    gms.set_tpar(1., 1., 1.); // set translation parameters
    gms.set_rpar(1., 1., 1.); // set rotation parameters

    position.u.pose.tran.x = 1.0;
    gms.set_end_position(&position);
    ASSERT_EQ (GO_RESULT_OK , gmq.append(&gms)) ;

    position.u.pose.tran.y = 2.0;
    gms.set_end_position(&position);
    ASSERT_EQ  (GO_RESULT_OK , gmq.append(&gms)) ;
    
    position.u.pose.tran.x = 3.0;
    gms.set_end_position(&position);
    ASSERT_EQ  (GO_RESULT_OK , gmq.append(&gms)) ;

    std::ofstream myfile;
    myfile.open("test1.log");
    while (!gmq.is_empty()) {
        gmq.interp(&position);
        myfile<< StrFormat("%f %f %f\n", 
                (double) position.u.pose.tran.x, 
                (double) position.u.pose.tran.y,
                (double) position.u.pose.tran.z);
        gotime += deltat;
    }
    myfile.close();


    //   ASSERT_NEAR(t[0], k[0], 1e-6);
}

TEST(GoMotion, world2) {
    gotime = 0.0;
    ASSERT_TRUE(GO_RESULT_OK == gmq.set_type(GO_MOTION_WORLD));

    gms.init();
    gms.set_type(GO_MOTION_LINEAR);
    gms.set_id(1);
    position.u.pose = go_pose_this(0, 0, 0, 1, 0, 0, 0);
    gmq.set_here(&position); // jlm added otherwise no starting position

    gms.set_end_position(&position);
    gms.set_tpar(1., 1., 1.); // set translation parameters
    gms.set_rpar(1., 1., 1.); // set rotation parameters
    position.u.pose.tran.x = -1.0;
    position.u.pose.tran.y = -1.0;
    gms.set_end_position(&position);
    if (GO_RESULT_OK != gmq.append(&gms)) {
        fprintf(stderr, "can't append\n");
    }
    std::ofstream myfile;
    myfile.open("test2.log");

    while (!gmq.is_empty()) {
        gmq.interp(&position);
        myfile << StrFormat("%f %f %f\n", (double) position.u.pose.tran.x, 
                (double) position.u.pose.tran.y,
                (double) position.u.pose.tran.z);
        gotime += deltat;
    }
    myfile.close();
}

int main(int argc, char **argv) {
    /* initialize random seed: */
    srand(time(NULL));
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
