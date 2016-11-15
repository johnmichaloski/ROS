//

// Kinematics.h
//

// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied
// or intended

#include "Kinematics.h"
#include <algorithm>
#ifndef WIN32
#define WIN32
#endif
#include "ikfast.h"

#include "Globals.h"
#include "urdf_model\eigenmath.h"

IkFastKinematics::IkFastKinematics(void)
{ }
IkFastKinematics::~IkFastKinematics(void)
{ }
static double SIGN (double x)
{
	return ( x >= 0.0f ) ? +1.0f : -1.0f;
}
static double NORM (double a, double b, double c, double d)
{
	return sqrt(a * a + b * b + c * c + d * d);
}

// Convert rotation matrix to quaternion (Daisuke Miyazaki)
// http://pastebin.com/NikwbL3k
static urdf::Rotation Convert2Rotation (IkReal *eerot)
{
	double q0 = ( eerot[0] + eerot[4] + eerot[8] + 1.0f ) / 4.0f;
	double q1 = ( eerot[0] - eerot[4] - eerot[8] + 1.0f ) / 4.0f;
	double q2 = ( -eerot[0] + eerot[4] - eerot[8] + 1.0f ) / 4.0f;
	double q3 = ( -eerot[0] - eerot[4] + eerot[8] + 1.0f ) / 4.0f;

	if ( q0 < 0.0f )
	{
		q0 = 0.0f;
	}

	if ( q1 < 0.0f )
	{
		q1 = 0.0f;
	}

	if ( q2 < 0.0f )
	{
		q2 = 0.0f;
	}

	if ( q3 < 0.0f )
	{
		q3 = 0.0f;
	}
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);

	if ( ( q0 >= q1 ) && ( q0 >= q2 ) && ( q0 >= q3 ) )
	{
		q0 *= +1.0f;
		q1 *= SIGN(eerot[7] - eerot[5]);
		q2 *= SIGN(eerot[2] - eerot[6]);
		q3 *= SIGN(eerot[3] - eerot[1]);
	}
	else if ( ( q1 >= q0 ) && ( q1 >= q2 ) && ( q1 >= q3 ) )
	{
		q0 *= SIGN(eerot[7] - eerot[5]);
		q1 *= +1.0f;
		q2 *= SIGN(eerot[3] + eerot[1]);
		q3 *= SIGN(eerot[2] + eerot[6]);
	}
	else if ( ( q2 >= q0 ) && ( q2 >= q1 ) && ( q2 >= q3 ) )
	{
		q0 *= SIGN(eerot[2] - eerot[6]);
		q1 *= SIGN(eerot[3] + eerot[1]);
		q2 *= +1.0f;
		q3 *= SIGN(eerot[7] + eerot[5]);
	}
	else if ( ( q3 >= q0 ) && ( q3 >= q1 ) && ( q3 >= q2 ) )
	{
		q0 *= SIGN(eerot[3] - eerot[1]);
		q1 *= SIGN(eerot[6] + eerot[2]);
		q2 *= SIGN(eerot[7] + eerot[5]);
		q3 *= +1.0f;
	}
	else
	{
		throw std::exception("Error while converting to quaternion! \n");
	}
	double r = NORM(q0, q1, q2, q3);
	q0 /= r;
	q1 /= r;
	q2 /= r;
	q3 /= r;
	urdf::Rotation q;
	q.setFromQuaternion(q0, q1, q2, q3);
	return q;
}
// / !brief Computes the end effector coordinates given the joint values. This function is used to double check ik
// Elsewhere: Returns the forward kinematic solution given the joint angles (in radians)
RCS::Pose IkFastKinematics::JointsToPose (std::vector<double> & jv)
{
	// Handle gearing of joints
	std::vector<double> joints;
	joints.insert(joints.begin(), jv.begin(), jv.end());
	//joints(1) = thetas[1] - M_PI_2;
	joints[2] +=  jv[1] ;

	// IkReal j[6]={ 0.0, 0.0, 0.0, 0.0, 0.0};
	IkReal eetrans[4];
	IkReal eerot[9];

	// / Computes the end effector coordinates given the joint values using ikfast. This function is used to double check ik
	// Units? joint angles in radians or degree
	// Elsewhere: Returns the forward kinematic solution given the joint angles (in radians)
	ComputeFk(&joints[0], eetrans, eerot);

	RCS::Pose pose;
	pose.position.x = eetrans[0];
	pose.position.y = eetrans[1];
	pose.position.z = eetrans[2]-0.33;

	// RosMatrix m = _3x3matrixConvert(eerot);
	// pose.rotation=_quatFromMatrix( m);
	pose.rotation = Convert2Rotation(eerot);
	return pose;
}
// Convert input effector pose, in w x y z quaternion notation, to rotation matrix.
// Must use doubles, else lose precision compared to directly inputting the rotation matrix.
// Found at http://kaist-ros-pkg.googlecode.com/svn/trunk/arm_kinematics_tools/src/ikfastdemo/ikfastdemo.cpp
static void Convert2RotationMatrix (const urdf::Rotation & quat, IkReal *eerot)
{
	double                   qq1 = 2 * quat.x * quat.x;
	double                   qq2 = 2 * quat.y * quat.y;
	double                   qq3 = 2 * quat.z * quat.z;
	eerot[3 * 0 + 0] = 1 - qq2 - qq3;
	eerot[3 * 0 + 1] = 2 * ( quat.x * quat.y - quat.w * quat.z );
	eerot[3 * 0 + 2] = 2 * ( quat.x * quat.z + quat.w * quat.y );
	eerot[3 * 1 + 0] = 2 * ( quat.x * quat.y + quat.w * quat.z );
	eerot[3 * 1 + 1] = 1 - qq1 - qq3;
	eerot[3 * 1 + 2] = 2 * ( quat.y * quat.z - quat.w * quat.x );
	eerot[3 * 2 + 0] = 2 * ( quat.x * quat.z - quat.w * quat.y );
	eerot[3 * 2 + 1] = 2 * ( quat.y * quat.z + quat.w * quat.x );
	eerot[3 * 2 + 2] = 1 - qq1 - qq2;
}
size_t IkFastKinematics::PoseToJoints (RCS::Pose & pose, std::vector<std::vector<double>> & joints)
{
	// Inverse kinematics
	ikfast::IkSolutionList<IkReal> solutions;

	std::vector<IkReal> vfree(GetNumFreeParameters( ) );
	IkReal              eetrans[3] = { pose.position.x, pose.position.y, pose.position.z + .33};

	IkReal eerot[9];
	Convert2RotationMatrix(pose.rotation, eerot);
	if ( Globals.IsDebug( ) )
	{
		std::cout << Globals.StrFormat("IKFAST IK\n");
		std::cout << Globals.StrFormat("Pos  X=%6.4f Y=%6.4f Z=%6.4f\n", eetrans[0], eetrans[1], eetrans[2]);
		std::cout << Globals.StrFormat("XROT I=%6.4f J=%6.4f K=%6.4f\n", eerot[0], eerot[1], eerot[2]);
		std::cout << Globals.StrFormat("ZROT I=%6.4f J=%6.4f K=%6.4f\n", eerot[6], eerot[7], eerot[8]);
	}
	bool bSuccess = ComputeIk(eetrans, eerot, vfree.size( ) > 0 ? &vfree[0] : NULL, solutions);

	if ( !bSuccess )
	{
		std::cerr << Globals.StrFormat("Failed to get ik solution\n");
		return -1;
	}

	// There are no redundant joints, so no free dof

	std::cerr << Globals.StrFormat("Found %d ik solutions:\n", (int) solutions.GetNumSolutions( ) );
	std::vector<IkReal> solvalues(GetNumJoints( ) );

	for ( std::size_t i = 0; i < solutions.GetNumSolutions( ); ++i )
	{
		const ikfast::IkSolutionBase<IkReal> & sol = solutions.GetSolution(i);

		if ( Globals.IsDebug( ) )
		{
			std::cerr << Globals.StrFormat("sol%d (free=%d): ", (int) i, (int) sol.GetFree( ).size( ) );
		}
		std::vector<IkReal> vsolfree(sol.GetFree( ).size( ) );
		sol.GetSolution(&solvalues[0], vsolfree.size( ) > 0 ? &vsolfree[0] : NULL);

		if ( Globals.IsDebug( ) )
		{
			for ( std::size_t j = 0; j < solvalues.size( ); ++j )
			{
				std::cerr << Globals.StrFormat("%6.4f, ", Rad2Deg(solvalues[j]) );
			}
			std::cerr << Globals.StrFormat("\n");
		}

		std::vector<double> jnts;

		for ( std::size_t j = 0; j < solvalues.size( ); ++j )
		{
			jnts.push_back(solvalues[j]);
		}
		jnts[2] -=  jnts[1] ;
		joints.push_back( jnts);
	}
	return solutions.GetNumSolutions( );
}
std::vector<double>    IkFastKinematics::NearestJoints ( 
	std::vector<double>  oldjoints,
	std::vector<std::vector<double>> & newjoints)
{
	std::vector<double> finaljoints;
	Eigen::VectorXd    oldjointvec =       ConvertJoints(oldjoints);
	double min=std::numeric_limits<double>::infinity();
	size_t index=0;
	for(size_t i=0; i< newjoints.size(); i++)
	{
		Eigen::VectorXd newjointvec = ConvertJoints(newjoints[i] );
		double diff = (oldjointvec-newjointvec).norm();
		if( diff < min)
		{	
			min=diff;
			index=i;
		}
	}
	// save "best" solution - closset ignoring importance of wrist
	finaljoints.insert(finaljoints.begin(), newjoints[index].begin(), newjoints[index].end());
	return finaljoints;
}
