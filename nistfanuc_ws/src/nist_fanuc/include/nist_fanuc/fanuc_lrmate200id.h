//

// fanuc_lrmate200id.h
//

/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I.
*/

/***
        fanuc_lrmate200id fanuc;

        // Have to be converted to radians
        double goal[] = { -0.0005, 29.930, -29.947, 0.12, -59.935, 0.062};
        std::vector<double> goaljts(goal, goal + sizeof(goal) / sizeof(double) ) ;
        // transform angles from degree to radians
        std::transform(goaljts.begin(), goaljts.end(), goaljts.begin(),
            std::bind1st(std::multiplies<double>(),  M_PI / 180.0 ));

        urdf::Pose pose = fanuc.fanuc_lrmate200id_kin_fwd(&goaljts[0]);
        std::cout << pose;

        double goal1[] = { -0.0005, 29.916, -29.949, 179.997, 60.509, 178.810};
        std::vector<double> goal1jts(goal1, goal1 + sizeof(goal1) / sizeof(double) ) ;
        // transform angles from degree to radians
        std::transform(goal1jts.begin(), goal1jts.end(), goal1jts.begin(),
            std::bind1st(std::multiplies<double>(),  M_PI / 180.0 ));

        urdf::Pose pose1 = fanuc.fanuc_lrmate200id_kin_fwd(&goal1jts[0]);
        std::cout << pose1;

Translation =    522.6895:    -0.1496:    69.2683
Rotation    =    179.8958:    -0.1518:     0.1216
Translation =    521.9315:    -0.0009:    69.4712
Rotation    =    179.9945:     0.3920:    -1.1920

        */
#pragma once
#include "RCS.h"
//#include "DenavitHartenberg.h"

#define WRIST_OFFSET              0.080 /*  nominal offset in -Z for wrist */

#define THREE21_KIN_NUM_JOINTS    6

#define THREE21_SHOULDER_RIGHT    0x01
#define THREE21_ELBOW_DOWN        0x02
#define THREE21_WRIST_FLIP        0x04
#define THREE21_SINGULAR          0x08

struct three21_kin
{
  double a1;
  double a2;
  double a3;
  double d2;
  double d3;
  double d4;
  long   iflags;

  // genser_struct gk;
};

struct  fanuc_lrmate200id_kin_struct
{
  three21_kin tk;
  tf::Pose  t7;           /* final tool transform */
  tf::Pose  t7_inv;       /* its inverse */
};

class fanuc_lrmate200id
{
public:
    fanuc_lrmate200id(void);
    ~fanuc_lrmate200id(void);
    tf::Pose fanuc_lrmate200id_kin_fwd(const double *motors);
    std::vector<double> fanuc_lrmate200id_kin_inv(const tf::Pose & pos);

    std::vector<double> add_fkgearing(std::vector<double> motors) {
        std::vector<double> joints(motors.size(), 0.0);
        /* gearing equations */
        joints[0] = motors[0];
        joints[1] = motors[1] - M_PI_2;
        joints[2] = -(motors[1] + motors[2]);
        joints[3] = -motors[3];
        joints[4] = -motors[4];
        joints[5] = -motors[5];
        return joints;
    }
  // -------------------------------------------
  fanuc_lrmate200id_kin_struct kins;
};
