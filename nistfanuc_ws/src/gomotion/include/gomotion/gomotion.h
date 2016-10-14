/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*!
\file gomotion.h

\brief Declarations for motion queue manipulation
*/

#ifndef GOMOTION_H
#define GOMOTION_H

#include "gomotion/gotypes.h"		/* go_result, go_count */
#include "gomotion/gomath.h"		/* go_pose */
#include "gomotion/gotraj.h"		/* go_traj_cv,a,j_spec */

#define GO_MOTION_JOINT_NUM 8	/* number of joints supported */
namespace gomotion {
	enum {
		/* types of motion queueing done */
		GO_MOTION_NONE,
		GO_MOTION_JOINT,
		GO_MOTION_UJOINT,
		GO_MOTION_WORLD,

		/* these are used to further specify types of world motion */
		GO_MOTION_LINEAR,
		GO_MOTION_CIRCULAR
	};

	/*
	Depending upon whether you are doing joint interpolation or
	world coordinate interpolation (as specified by your call to
	go_motion_queue_set_type()), fill in joint[] or pose accordingly.
	*/
	struct go_position {
		union {
			go_real joint[GO_MOTION_JOINT_NUM];
			go_pose pose;
		} u;

		void zero_joints();
		void zero_pose();
	} ;

	struct go_motion_params {
		go_real vel;			/*< max vel for each motion */
		go_real acc;			/*< max accel for each motion */
		go_real jerk;			/*< max jerk for each motion */
	} ;

	/*
	In the following comments, LIN, CIR and ALL refer to linear moves,
	circular moves and both, respectively. If they are not in parens,
	e.g., ALL:, you need to provide them for that type of motion.
	If they are in parens, e.g., (CIR), they are computed for you.
	*/

	struct go_motion_linear_params {
		go_cart uvec;			/* (LIN) unit vector along line */
		go_quat uquat;		/* (LIN) unit quaternion along rotation */
	} ;

	struct go_motion_circular_params {
		/*! The vector to the circle center. */
		go_cart center;

		/*! The normal vector that defines the plane of the circle. */
		go_cart normal;

		/*! The normal vector expressed as a unit rotation. */
		go_quat qnormal;

		/*! The unit vector from center to start, projected onto the normal plane. */
		go_cart urcsp;

		/*! The starting radius. */
		go_real rstart;

		/*! The vector from the normal plane to the start. */
		go_cart zstart;

		/*! The signed total angular displacement around normal vector */
		go_real thtot;

		/*! The signed displacement from start to end projected radii */
		go_real rtot;

		/*! The signed displacement from start to end z off-normals  */
		go_real ztot;

		/*! The inverse of total approximate arc length. If negative, no
		translation motion is taking place. */
		go_real stotinv;

		/*! The number of turns in the circle. 0 means partial CCW, -1 means
		partial CW, otherwise more turns are added in each direction. */
		go_integer turns;
	} ;

	struct go_motion_spec {
		go_flag type;			/* ALL: GO_MOTION_JOINT,LINEAR,CIRCULAR */
		go_integer id;		/* ALL: id echoed as current move */
		go_real totalt;		/* (ALL) total planned time for the motion */
		go_position start;		/* (ALL)  start pose wrt world; prev end */
		go_position end;		/* ALL: target position, joints or pose */
		go_quat uquat;		/* (ALL) unit rotation from start to end */
		/* For joint moves, each pertains to the indexed joint. For world moves,
		[0] pertains to trans, [1] pertains to rot, and the [2]-[*] are unused. */
		go_motion_params par[GO_MOTION_JOINT_NUM];
		/* specific params for linear and circular moves */
		union {
			go_motion_linear_params lpar;	/* (LIN) linear params */
			go_motion_circular_params cpar;	/* CIR: some circular params */
		} u;
		/* (ALL) times for the various tran phases of CV, CA and CJ profiles */
		/* For world motion [0] is for tran, [1] is for rot, rest unused */
		go_traj_cj_spec cj[GO_MOTION_JOINT_NUM];

		go_result init();

		go_result set_type( go_flag type);
		go_flag get_type();

		go_result set_id( go_integer id);
		go_integer get_id();

		go_result set_jpar( go_integer i, go_real vel, go_real acc, go_real jerk);
		go_result set_tpar( go_real vel, go_real acc, go_real jerk);
		go_result set_rpar( go_real vel, go_real acc, go_real jerk);
		go_result set_cpar( go_cart * center, go_cart * normal, go_integer turns);
		go_result set_time( go_real time);
		go_result set_end_position( go_position * end);
		go_result set_end_pose( go_pose * end);
	} ;

	struct go_scale_spec{
		go_traj_ca_spec scale_spec;
		go_flag scaling;		/*< non-zero means we're scaling time */
		go_flag scale_dir;		/*< non-zero means add scale to scale_b  */
		go_flag scale_isneg;		/*< non-zero means negative scale  */
		go_real scale_b;		/*< original base scale factor */
		go_real scale;		/*< current scale factor */
		go_real scale_next;		/*< pending scale factor */
		go_real scale_v_next;		/*< pending d(scale)/dt */
		go_real scale_a_next;		/*< pending d^2(scale)/dt^2 */
		go_real scale_t;		/*< time into scaling */

		go_result init( go_real scale);
		go_result set(
			go_real scale,
			go_real scale_v,
			go_real scale_a);
		go_result eval(
			go_real deltat, 
			go_real * scale);
	} ;

	struct go_motion_queue{
		go_flag _type;			/*< go_MOTION_JOINT for joint, otherwise world */
		go_position _here;		/*< initial starting point */
		go_position _there;		/*< where the end is */
		go_motion_spec *_startptr;	/*< ptr to original start */
		go_motion_spec *_endptr;	/*< the original end */
		go_motion_spec *_start;	/*< ptr to first entry of the queue */
		go_motion_spec *_end;		/*< ptr to last (empty) entry */
		go_integer _size;		/*< size of whole queue */
		go_integer _joint_num;		/*< number of joints to be interpolated */
		go_integer _number;		/*< number of motions on queue */
		go_integer _last_id;		/*< id of last motion appended */
		go_real _deltat;		/*< cycle time */
		go_real _time;			/*< time into the current spec */
		go_scale_spec _timescale;	/*< walked-in time scale factor */

		go_result init(
			go_motion_spec * space,
			go_integer size,
			go_real deltat);

		go_result reset();

		go_result set_type(go_flag type);

		go_flag get_type();

		go_result set_joint_number(go_integer joints);

		go_integer get_joint_number();

		go_result set_here(const go_position * here);

		go_result set_cycle_time(go_real deltat);

		go_result set_scale(
			go_real scale,
			go_real scale_v,
			go_real scale_a);

		go_result append(const go_motion_spec * motion);

		go_result number( go_integer * number);

		go_result size(go_integer * size);

		go_result head(	go_motion_spec * motion);

		go_result here(	go_position * position);

		go_result there(go_position * position);

		go_result interp(go_position * position);

		go_result stop(go_motion_queue * queue);

		go_result set_id(go_integer id);

		go_integer last_id();

		go_flag is_empty();

		go_result erase();

		go_result drop_pending();

		go_result stop(); 
	protected:
		go_result append_joint(
		const go_motion_spec * motion);
		
		go_result  append_ujoint(
		const go_motion_spec * motion);

		go_result append_world(
		const go_motion_spec * motion);
		
		go_result interp_joint(	go_real * joint);

		go_result interp_world( go_pose * pose);

		go_result 
		interp_world(go_motion_spec * motion, go_real time, go_pose * pose);
	} ;

};

#endif /* GOMOTION_H */
