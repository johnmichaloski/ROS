// 

// Kinematics.h
//

// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied
// or intended

#pragma once
#include <string>
#include <vector>
#include "Canon.h"

class IkFastKinematics
{
public:
  IkFastKinematics(void);
  ~IkFastKinematics(void);
  static RCS::Pose JointsToPose (std::vector<double> & jv);
  static size_t    PoseToJoints (RCS::Pose & pose, 
	  std::vector<std::vector<double>> & newjoints);
  static std::vector<double>      NearestJoints ( 
	  std::vector<double>  oldjoints,
	  std::vector<std::vector<double>> & newjoints);
};
