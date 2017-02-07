/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef H
#define H

#include <string>
#include <vector>

#include "gokin/gotypes.h"		/* go_result, go_real */
#include "gokin/gomath.h"		/* go_pose */

using namespace gomotion;
/** \brief Describes the type of kinematics used.

IDENTITY means that the joints and world coordinates are the
same, as for slideway machines (XYZ milling machines). The EMC will allow
changing from joint to world mode and vice versa. Also, the EMC will set
the actual world position to be the actual joint positions (not commanded)
by calling the forward kinematics each trajectory cycle.

FORWARD_ONLY means that only the forward kinematics exist.
Since the EMC requires at least the inverse kinematics, this should simply
terminate the EMC.

INVERSE_ONLY means that only the inverse kinematics exist.
The forwards won't be called, and the EMC will only allow changing from
joint to world mode at the home position.

BOTH means that both the forward and inverse kins are defined.
Like IDENTITY, the EMC will allow changing between world and
joint modes. However, the kins are assumed to be somewhat expensive
computationally, and the forwards won't be called at the trajectory rate
to compute actual world coordinates from actual joint values.
*/

typedef enum {
	IDENTITY = 1,	/* forward=inverse, both well-behaved, e.g., 3 joints = xyz */
	FORWARD_ONLY,	/* forward but no inverse */
	INVERSE_ONLY,	/* inverse but no forward */
	BOTH		/* forward and inverse both */
} go_kin_type;




struct gokin {

	/** \brief This function sets the parameters used to calculate the Fk and IK.
	* Can be either Denavit Hartenberg (DH) or ROS URDF format depending* on go_kin 
	* implementation. 
	* For DH, that means outer vector is number of axis, inner vector is the 4 DH parameters, a, alpha, d and theta.
	* For URDF, outer vector is number of axis, and inner vector is xyz of origin, rpy of origin, and xyz axis of rotation.
	* \param params is a vector of vectors to define parameters.
	*/
	virtual go_result set_params (std::vector< std::vector<double>> params)=0;  // either URDF or DH

	/** \brief This function gets a descriptive and hopefully unique name so
	that the controller can adjust the meaning of the parameters passed
	to set_parameters() */
	virtual std::string get_name(void)=0;

	/** \brief size returns how big the kinematics structure is for
	the particular implementation */
	virtual go_integer size(void)=0;

	/** \brief pass the name of the kins you want */
	//virtual go_result select(std::string name)=0;

	/** \brief init() initializes the kinematics pointed to by 'kins' */
	virtual go_result init()=0;

	/** \brief returns the actual number of joints, possibly less than the max
	supported depending on how many links were present in the call
	to set_parameters */
	virtual go_integer num_joints()=0;

	/** \brief
	The forward kinematics take joint values and determine world coordinates,
	using any forward kinematics flags to resolve any ambiguities. The inverse
	flags are set to indicate their value appropriate to the joint values
	passed in.
	*/
	virtual go_result fwd(
		const go_real *joint,
		go_pose * world)=0; 

	/** \brief
	The inverse kinematics take world coordinates and determine joint values,
	given the inverse kinematics flags to resolve any ambiguities. The forward
	flags are set to indicate their value appropriate to the world coordinates
	passed in.
	*/
	virtual go_result inv(
		const go_pose * world,
		go_real *joints)=0; 

	virtual go_kin_type get_type()=0;

//	go_result set_parameters( go_link * params, go_integer num)=0;
//	go_result get_parameters( go_link * params, go_integer num)=0;

	virtual go_result jac_inv(
		const go_pose * pos,
		const go_vel * vel,
		const go_real * joints,
		go_real * jointvels)=0;

	virtual go_result jac_fwd(
		const go_real * joints,
		const go_real * jointvels,
		const go_pose * pos,
		go_vel * vel)=0;

	virtual go_result set_flags(
		go_flag fflags,
		go_flag iflags)=0;

	virtual go_result get_flags(
		go_flag *fflags,
		go_flag *iflags)=0;
};
#endif
