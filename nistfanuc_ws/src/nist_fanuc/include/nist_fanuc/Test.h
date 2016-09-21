

#pragma once

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
*/


inline void TestFk(boost::shared_ptr<IKinematics> kin, std::string kinname){
        JointState kincmd;
        RCS::Pose kinpose;
        kincmd.name = kin->JointNames();
 #if 1
// rviz says link_6 at 0.465, 0, 0.695
        kincmd.position = ToVector<double>(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        kinpose = kin->FK(kincmd.position);
        LOG_DEBUG << kinname.c_str() <<  "Test FK Pose 1" << RCS::DumpPoseSimple(kinpose).c_str();
#endif
        kincmd.position = ToVector<double>(6, 1.4, 0.0, 0.0, 0.0, 0.0, 0.00);
        kinpose = kin->FK(kincmd.position);
        LOG_DEBUG << kinname.c_str() <<  "Test FK Pose 2 " << RCS::DumpPoseSimple(kinpose).c_str();
}
inline void TestIk(boost::shared_ptr<IKinematics> kin, std::string kinname){
#if 0
 	tf::Pose pose(tf::Quaternion(0,0,0,1), tf::Vector3(.465,0,.695 ));
 	std::vector<double> oldjoints= ToVector<double>(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
#endif	
	tf::Pose pose(Conversion::RPYDegrees(0,0,80), tf::Vector3(0.079,0.458,0.695 ));
 	std::vector<double> oldjoints=ToVector<double>(6, 1.4,  0.0,0.0,0.0,0.0,0.0);
        std::vector<double> joints = kin->IK(pose, oldjoints);
        LOG_DEBUG << kinname.c_str() <<  "IK Pose 1" << RCS::DumpPoseSimple(pose).c_str();
        LOG_DEBUG << kinname.c_str() <<  "Test IK Joints 1" << VectorDump<double>(joints).c_str();

}
