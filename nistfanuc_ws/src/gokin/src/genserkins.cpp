/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I. 
 */

/*
genserkins.c

These are the forward and inverse kinematic functions for a general
serial-link manipulator. Thanks to Herman Bruyninckx and John
Hallam at http://www.roble.info/ for this.
 */

#include <math.h>
#include "gokin/gotypes.h"  /* go_result, go_integer */
#include "gokin/gomath.h"  /* go_pose */
#include "gokin/gokin_.h"  /* type */
#include "gokin/genserkins.h"  /* these decls */
#include <iostream>
#include <assert.h>
/*
Set ROTATE_JACOBIANS_BACK if you want the Jacobian matrix to be
expressed in the {0} frame. This means Cartesian velocities will be
assumed relative to the {0} frame. If this is not defined, then the
Jacobian will be relative to the final frame, and Cartesian speeds
are assumed to be in the final frame.

Normally, things are expressed in the {0} frame, so rotating
Jacobians back is consistent with this.
 */
#define ROTATE_JACOBIANS_BACK 1

enum {
    go_DEFAULT_MAX_ITERATIONS = 100
};
using namespace gomotion;

/**
; Units are [mm], [deg]
LENGTH_UNITS_PER_M = 1000.0
ANGLE_UNITS_PER_RAD = 57.295779513082323

 */
genserkins::genserkins() {
    m_per_length_units = 1000.0;
    rad_per_angle_units = 57.295779513082323;
}

genserkins * genserkins::create() {
    return new genserkins();

}

/* how many joints we can support */
enum {
    MAX_JOINT_NUM = 8
};

go_result genserkins::set_params(std::vector< std::vector<double>> params) {
    go_link link;
    for (size_t i = 0; i < params.size(); i++) {
        std::vector<double> d = params[i];
        go_rpy rpy;
        go_cart cart;
        link.u.urdf.pose.tran.x = (go_real) (m_per_length_units * d[0]);
        link.u.urdf.pose.tran.y = (go_real) (m_per_length_units * d[1]);
        link.u.urdf.pose.tran.z = (go_real) (m_per_length_units * d[2]);
        rpy.r = (go_real) (rad_per_angle_units * d[3]);
        rpy.p = (go_real) (rad_per_angle_units * d[4]);
        rpy.y = (go_real) (rad_per_angle_units * d[5]);
        go_rpy_quat_convert(&rpy, &link.u.urdf.pose.rot);
        cart.x = (go_real) (m_per_length_units * d[6]);
        cart.y = (go_real) (m_per_length_units * d[7]);
        cart.z = (go_real) (m_per_length_units * d[8]);
        if (GO_RESULT_OK != go_cart_unit(&cart, &cart)) {
            std::cerr << "bad entry: [" << i << "] URDF = \n";
            return GO_RESULT_ERROR;

        }
        link.u.urdf.axis = cart;
        link.type = GO_LINK_URDF;
        links.push_back(link);
    }
    // Should be number into language
    if (links.size() < 6) {
        std::cerr << "Minimum 6 joints for general serial kinematic gokin";
        return GO_RESULT_ERROR;
    }
    if (links.size() > go_MAX_JOINTS) {
        std::cerr << "Maximum 8 joints for general serial kinematic gokin";
        return GO_RESULT_ERROR;
    }
    return GO_RESULT_OK;
}

go_integer genserkins::size(void) {
    return (go_integer) links.size();
}

go_result genserkins::init() {
    go_integer t;

    links.clear();
#if 0
    links.resize(go_MAX_JOINTS);
    /* clear them all and make them revolute joints */
    for (t = 0; t < go_MAX_JOINTS; t++) {
        links[t].u.dh.a = 0;
        links[t].u.dh.alpha = 0;
        links[t].u.dh.d = 0;
        links[t].u.dh.theta = 0;
        links[t].type = GO_LINK_DH;
        links[t].quantity = GO_QUANTITY_ANGLE;
        go_body_init(&links[t].body);
    }

    /* set a select few to make it PUMA-like */

    links[1].u.dh.alpha = -GO_PI_2;

    links[2].u.dh.a = 0.300;
    links[2].u.dh.d = 0.070;

    links[3].u.dh.a = 0.050;
    links[3].u.dh.alpha = -GO_PI_2;
    links[3].u.dh.a = 0.400;

    links[4].u.dh.alpha = GO_PI_2;

    links[5].u.dh.alpha = -GO_PI_2;

    link_num = 6;
#endif
    iterations = 0;
    max_iterations = go_DEFAULT_MAX_ITERATIONS;

    return GO_RESULT_OK;
}

std::string genserkins::get_name(void) {
    return "genserkins";
}

go_integer genserkins::num_joints() {

    return links.size();
}

go_result genserkins::set_parameters(go_link * params, go_integer num) {
    go_integer t;
    assert(0);
    if (num > go_MAX_JOINTS) return GO_RESULT_BAD_ARGS;

    for (t = 0; t < num; t++) {
        /* we only handle serial-type link params */
        if (params[t].type != GO_LINK_DH &&
                params[t].type != GO_LINK_PP &&
                params[t].type != GO_LINK_URDF) return GO_RESULT_BAD_ARGS;
        links[t] = params[t];
    }

    return GO_RESULT_OK;
}

go_result genserkins::get_parameters(go_link * params, go_integer num) {
    go_integer t;

    /* check if they have enough space to hold the params */
    if (num < links.size()) return GO_RESULT_BAD_ARGS;

    for (t = 0; t < links.size(); t++) {
        params[t] = links[t];
    }

    return GO_RESULT_OK;
}

/* forward decls for these extended functions */

//extern go_result compute_jfwd(go_link *link_params, int link_number, go_matrix *Jfwd, go_pose *T_L_0);
//extern go_result compute_jinv(go_matrix *Jfwd, go_matrix *Jinv, go_real *weights);

go_result genserkins::jac_inv(
        const go_pose * pos,
        const go_vel * vel,
        const go_real * joints,
        go_real * jointvels) {
    GO_MATRIX_DECLARE(Jfwd, Jfwd_stg, 6, go_MAX_JOINTS);
    GO_MATRIX_DECLARE(Jinv, Jinv_stg, go_MAX_JOINTS, 6);
    go_pose T_L_0;
#ifdef ROTATE_JACOBIANS_BACK
#else
    go_quat Rinv;
#endif
    go_link linkout[go_MAX_JOINTS];
    go_real vw[6];
    go_vector weights[go_MAX_JOINTS];
    go_integer link;
    go_result retval;

    go_matrix_init(Jfwd, Jfwd_stg, 6, links.size());
    go_matrix_init(Jinv, Jinv_stg, go_MAX_JOINTS, 6);

    for (link = 0; link < links.size(); link++) {
        go_link_joint_set(&links[link], joints[link], &linkout[link]);
        weights[link] = (GO_QUANTITY_LENGTH == links[link].quantity ? links[link].body.mass : GO_QUANTITY_ANGLE == links[link].quantity ? links[link].body.inertia[2][2] : 1);
    }
    retval = compute_jfwd(linkout, links.size(), &Jfwd, &T_L_0);
    if (GO_RESULT_OK != retval) return retval;
    retval = compute_jinv(&Jfwd, &Jinv, weights);
    if (GO_RESULT_OK != retval) return retval;

#ifdef ROTATE_JACOBIANS_BACK
#else
    go_quat_inv(&T_L_0.rot, &Rinv);
    go_quat_cart_mult(&Rinv, &vel->v, &vel->v);
    go_quat_cart_mult(&Rinv, &vel->w, &vel->w);
#endif

    vw[0] = vel->v.x;
    vw[1] = vel->v.y;
    vw[2] = vel->v.z;
    vw[3] = vel->w.x;
    vw[4] = vel->w.y;
    vw[5] = vel->w.z;

    return go_matrix_vector_mult(&Jinv, vw, jointvels);
}

go_result genserkins::jac_fwd(
        const go_real * joints,
        const go_real * jointvels,
        const go_pose * pos,
        go_vel * vel) {
    GO_MATRIX_DECLARE(Jfwd, Jfwd_stg, 6, go_MAX_JOINTS);
    go_pose T_L_0;
#ifdef ROTATE_JACOBIANS_BACK
#else
    go_quat Rinv;
#endif
    go_link linkout[go_MAX_JOINTS];
    go_real vw[6];
    go_integer link;
    go_result retval;

    go_matrix_init(Jfwd, Jfwd_stg, 6, links.size());

    for (link = 0; link < links.size(); link++) {
        retval = go_link_joint_set(&links[link], joints[link], &linkout[link]);
        if (GO_RESULT_OK != retval) return retval;
    }

    retval = compute_jfwd(linkout, links.size(), &Jfwd, &T_L_0);
    if (GO_RESULT_OK != retval) return retval;

    go_matrix_vector_mult(&Jfwd, jointvels, vw);
    vel->v.x = vw[0];
    vel->v.y = vw[1];
    vel->v.z = vw[2];
    vel->w.x = vw[3];
    vel->w.y = vw[4];
    vel->w.z = vw[5];

#ifdef ROTATE_JACOBIANS_BACK
#else
    go_quat_inv(&T_L_0.rot, &Rinv);
    go_quat_cart_mult(&Rinv, &vel->v, &vel->v);
    go_quat_cart_mult(&Rinv, &vel->w, &vel->w);
#endif

    return GO_RESULT_OK;
}

go_result genserkins::fwd(
        const go_real *joints,
        go_pose * pos) {
    go_link linkout[go_MAX_JOINTS];
    go_integer link;
    go_result retval;

    for (link = 0; link < links.size(); link++) {
        retval = go_link_joint_set(&links[link], joints[link], &linkout[link]);
        if (GO_RESULT_OK != retval) return retval;
    }

    retval = go_link_pose_build(linkout, links.size(), pos);
    if (GO_RESULT_OK != retval) return retval;

    return GO_RESULT_OK;
}

go_result genserkins::inv(
        const go_pose * pos,
        go_real *joints) {

    GO_MATRIX_DECLARE(Jfwd, Jfwd_stg, 6, go_MAX_JOINTS);
    GO_MATRIX_DECLARE(Jinv, Jinv_stg, go_MAX_JOINTS, 6);
    go_pose T_L_0;
    go_real dvw[6];
    go_real jest[go_MAX_JOINTS];
    go_real dj[go_MAX_JOINTS];
    go_pose pest, pestinv, Tdelta;
    go_rvec rvec;
    go_cart cart;
    go_link linkout[go_MAX_JOINTS];
    go_vector weights[go_MAX_JOINTS];
    go_integer link;
    go_integer smalls;
    go_result retval;

    go_matrix_init(Jfwd, Jfwd_stg, 6, links.size());
    go_matrix_init(Jinv, Jinv_stg, links.size(), 6);

    /* jest[] is a copy of joints[], which is the joint estimate */
    for (size_t link = 0; link < links.size(); link++) {
        jest[link] = joints[link];
    }

    for (iterations = 0; iterations < max_iterations; iterations++) {
        /* update the Jacobians */
        for (link = 0; link < links.size(); link++) {
            go_link_joint_set(&links[link], jest[link], &linkout[link]);
            weights[link] = (GO_QUANTITY_LENGTH == links[link].quantity ? links[link].body.mass : GO_QUANTITY_ANGLE == links[link].quantity ? links[link].body.inertia[2][2] : 1);
        }
        retval = compute_jfwd(linkout, links.size(), &Jfwd, &T_L_0);
        if (GO_RESULT_OK != retval) return retval;
        retval = compute_jinv(&Jfwd, &Jinv, weights);
        if (GO_RESULT_OK != retval) return retval;

        /* pest is the resulting pose estimate given joint estimate */
        fwd(jest, &pest);
        /* pestinv is its inverse */
        go_pose_inv(&pest, &pestinv);
        /*
        Tdelta is the incremental pose from pest to pos, such that

        0        L         0
        . pest *  Tdelta =  pos, or
        L        L         L

        L         L          0
        .Tdelta =  pestinv *  pos
        L         0          L
         */
        go_pose_pose_mult(&pestinv, pos, &Tdelta);

        /*
        We need Tdelta in 0 frame, not pest frame, so rotate it
        back. Since it's effectively a velocity, we just rotate it, and
        don't translate it.
         */

#ifdef ROTATE_JACOBIANS_BACK
        go_quat_cart_mult(&pest.rot, &Tdelta.tran, &cart);
#else
        cart = Tdelta.tran;
#endif
        dvw[0] = cart.x;
        dvw[1] = cart.y;
        dvw[2] = cart.z;

        /* to rotate the rotation differential, convert it to a
        velocity and rotate that */
        go_quat_rvec_convert(&Tdelta.rot, &rvec);
        cart.x = rvec.x;
        cart.y = rvec.y;
        cart.z = rvec.z;
#ifdef ROTATE_JACOBIANS_BACK
        go_quat_cart_mult(&pest.rot, &cart, &cart);
#endif
        dvw[3] = cart.x;
        dvw[4] = cart.y;
        dvw[5] = cart.z;

        /* push the Cartesian velocity vector through the inverse Jacobian */
        go_matrix_vector_mult(&Jinv, dvw, dj);

        /* check for small joint increments, if so we're done */
        for (link = 0, smalls = 0; link < links.size(); link++) {
            if (GO_QUANTITY_LENGTH == linkout[link].quantity) {
                if (GO_TRAN_SMALL(dj[link])) smalls++;
            } else {
                if (GO_ROT_SMALL(dj[link])) smalls++;
            }
        }
        if (smalls == links.size()) {
            /* converged, copy jest[] out */
            for (link = 0; link < links.size(); link++) {
                joints[link] = jest[link];
            }
            return GO_RESULT_OK;
        }

        /* else keep iterating */
        for (link = 0; link < links.size(); link++) {
            jest[link] += dj[link];
            if (GO_QUANTITY_ANGLE == linkout[link].quantity) {
                if (jest[link] > GO_PI) jest[link] -= GO_2_PI;
                else if (jest[link] < -GO_PI) jest[link] += GO_2_PI;
            }
        }
    } /* for (iterations) */

    return GO_RESULT_ERROR;
}

/*
Extras, not callable using  wrapper but if you know you have
linked in these kinematics, go ahead and call these for your ad hoc
purposes.
 */

go_result genserkins::compute_jfwd(go_link * link_params, int link_number, go_matrix * Jfwd, go_pose * T_L_0) {
    GO_MATRIX_DECLARE(Jv, Jvstg, 3, go_MAX_JOINTS);
    GO_MATRIX_DECLARE(Jw, Jwstg, 3, go_MAX_JOINTS);
    GO_MATRIX_DECLARE(R_i_ip1, R_i_ip1stg, 3, 3);
    GO_MATRIX_DECLARE(scratch, scratchstg, 3, go_MAX_JOINTS);
    GO_MATRIX_DECLARE(R_inv, R_invstg, 3, 3);
    go_pose pose;
    go_quat quat;
    go_vector P_ip1_i[3];
    int row, col;

    /* init matrices to possibly smaller size */
    go_matrix_init(Jv, Jvstg, 3, link_number);
    go_matrix_init(Jw, Jwstg, 3, link_number);
    go_matrix_init(R_i_ip1, R_i_ip1stg, 3, 3);
    go_matrix_init(scratch, scratchstg, 3, link_number);
    go_matrix_init(R_inv, R_invstg, 3, 3);

    if (GO_LINK_URDF == link_params[0].type) {
        /* move about general axis */
        if (GO_QUANTITY_LENGTH == link_params[0].quantity) {
            Jv.el[0][0] = link_params[0].u.urdf.axis.x;
            Jv.el[1][0] = link_params[0].u.urdf.axis.y;
            Jv.el[2][0] = link_params[0].u.urdf.axis.z;
            Jw.el[0][0] = 0, Jw.el[1][0] = 0, Jw.el[2][0] = 0;
        } else {
            Jw.el[0][0] = link_params[0].u.urdf.axis.x;
            Jw.el[1][0] = link_params[0].u.urdf.axis.y;
            Jw.el[2][0] = link_params[0].u.urdf.axis.z;
            Jv.el[0][0] = 0, Jv.el[1][0] = 0, Jv.el[2][0] = 0;
        }
    } else {
        /* rotation or translate about Z by convention */
        Jv.el[0][0] = 0, Jv.el[1][0] = 0, Jv.el[2][0] = (GO_QUANTITY_LENGTH == link_params[0].quantity ? 1 : 0);
        Jw.el[0][0] = 0, Jw.el[1][0] = 0, Jw.el[2][0] = (GO_QUANTITY_ANGLE == link_params[0].quantity ? 1 : 0);
    }

    /* initialize inverse rotational transform */
    if (GO_LINK_DH == link_params[0].type) {
        go_dh_pose_convert(&link_params[0].u.dh, &pose);
    } else if (GO_LINK_PP == link_params[0].type) {
        pose = link_params[0].u.pp.pose;
    } else if (GO_LINK_URDF == link_params[0].type) {
        pose = link_params[0].u.urdf.pose;
    } else {
        return GO_RESULT_IMPL_ERROR;
    }

    *T_L_0 = pose;

    for (col = 1; col < link_number; col++) {
        /* T_ip1_i */
        if (GO_LINK_DH == link_params[col].type) {
            go_dh_pose_convert(&link_params[col].u.dh, &pose);
        } else if (GO_LINK_PP == link_params[col].type) {
            pose = link_params[col].u.pp.pose;
        } else if (GO_LINK_URDF == link_params[col].type) {
            pose = link_params[col].u.urdf.pose;
        } else {
            return GO_RESULT_IMPL_ERROR;
        }

        go_cart_vector_convert(&pose.tran, P_ip1_i);
        go_quat_inv(&pose.rot, &quat);
        go_quat_matrix_convert(&quat, &R_i_ip1);

        /* Jv */
        go_matrix_vector_cross(&Jw, P_ip1_i, &scratch);
        go_matrix_matrix_add(&Jv, &scratch, &scratch);
        go_matrix_matrix_mult(&R_i_ip1, &scratch, &Jv);

        if (GO_LINK_URDF == link_params[col].type) {
            if (GO_QUANTITY_LENGTH == link_params[col].quantity) {
                Jv.el[0][col] = link_params[col].u.urdf.axis.x;
                Jv.el[1][col] = link_params[col].u.urdf.axis.y;
                Jv.el[2][col] = link_params[col].u.urdf.axis.z;
            } else {
                Jv.el[0][col] = 0, Jv.el[1][col] = 0, Jv.el[2][col] = 0;
            }
        } else {
            Jv.el[0][col] = 0, Jv.el[1][col] = 0, Jv.el[2][col] = (GO_QUANTITY_LENGTH == link_params[col].quantity ? 1 : 0);
        }

        /* Jw */
        go_matrix_matrix_mult(&R_i_ip1, &Jw, &Jw);

        if (GO_LINK_URDF == link_params[col].type) {
            if (GO_QUANTITY_ANGLE == link_params[col].quantity) {
                Jw.el[0][col] = link_params[col].u.urdf.axis.x;
                Jw.el[1][col] = link_params[col].u.urdf.axis.y;
                Jw.el[2][col] = link_params[col].u.urdf.axis.z;
            } else {
                Jw.el[0][col] = 0, Jw.el[1][col] = 0, Jw.el[2][col] = 0;
            }
        } else {
            Jw.el[0][col] = 0, Jw.el[1][col] = 0, Jw.el[2][col] = (GO_QUANTITY_ANGLE == link_params[col].quantity ? 1 : 0);
        }

        if (GO_LINK_DH == link_params[col].type) {
            go_dh_pose_convert(&link_params[col].u.dh, &pose);
        } else if (GO_LINK_PP == link_params[col].type) {
            pose = link_params[col].u.pp.pose;
        } else if (GO_LINK_URDF == link_params[col].type) {
            pose = link_params[col].u.urdf.pose;
        } else {
            return GO_RESULT_IMPL_ERROR;
        }
        go_pose_pose_mult(T_L_0, &pose, T_L_0);
    }

#ifdef ROTATE_JACOBIANS_BACK
    /* rotate back into {0} frame */
    go_quat_matrix_convert(&T_L_0->rot, &R_inv);
    go_matrix_matrix_mult(&R_inv, &Jv, &Jv);
    go_matrix_matrix_mult(&R_inv, &Jw, &Jw);
#endif

    /* put Jv atop Jw in J */
    for (row = 0; row < 6; row++) {
        for (col = 0; col < link_number; col++) {
            if (row < 3) {
                Jfwd->el[row][col] = Jv.el[row][col];
            } else {
                Jfwd->el[row][col] = Jw.el[row - 3][col];
            }
        }
    }

    return GO_RESULT_OK;
}

go_result genserkins::compute_jinv(go_matrix * Jfwd, go_matrix * Jinv, go_vector *weights) {
    go_integer row, col;
    go_result retval;

    /* compute inverse, or pseudo-inverse */
    if (Jfwd->rows == Jfwd->cols) {
        retval = go_matrix_inv(Jfwd, Jinv);
        if (GO_RESULT_OK != retval) return retval;
    } else if (Jfwd->rows < Jfwd->cols) {
        /* underdetermined, optimize on smallest sum of square of speeds */
        /* JT(JJT)inv */
        GO_MATRIX_DECLARE(JT, JTstg, go_MAX_JOINTS, 6);
        GO_MATRIX_DECLARE(JJT, JJTstg, 6, 6);
        GO_MATRIX_DECLARE(Minv, Minvstg, go_MAX_JOINTS, go_MAX_JOINTS);

        go_matrix_init(JT, JTstg, Jfwd->cols, Jfwd->rows);
        go_matrix_init(JJT, JJTstg, Jfwd->rows, Jfwd->rows);
        go_matrix_init(Minv, Minvstg, Jfwd->cols, Jfwd->cols);

        for (row = 0; row < Jfwd->cols; row++) {
            for (col = 0; col < Jfwd->cols; col++) {
                if (row == col) {
                    Minv.el[row][col] = weights[row] > 0.0 ? 1.0 / weights[row] : 1.0;
                } else {
                    Minv.el[row][col] = 0.0;
                }
            }
        }

        go_matrix_transpose(Jfwd, &JT);
        go_matrix_matrix_mult(&Minv, &JT, &JT);
        go_matrix_matrix_mult(Jfwd, &JT, &JJT);
        retval = go_matrix_inv(&JJT, &JJT);
        if (GO_RESULT_OK != retval) {
            return retval;
        }
        go_matrix_matrix_mult(&JT, &JJT, Jinv);
        go_matrix_matrix_mult(&Minv, Jinv, Jinv);
    } else {
        /* overdetermined, do least-squares best fit */
        /* (JTJ)invJT */
        GO_MATRIX_DECLARE(JT, JTstg, go_MAX_JOINTS, 6);
        GO_MATRIX_DECLARE(JTJ, JTJstg, go_MAX_JOINTS, go_MAX_JOINTS);

        go_matrix_init(JT, JTstg, Jfwd->cols, Jfwd->rows);
        go_matrix_init(JTJ, JTJstg, Jfwd->cols, Jfwd->cols);
        go_matrix_transpose(Jfwd, &JT);
        go_matrix_matrix_mult(&JT, Jfwd, &JTJ);
        retval = go_matrix_inv(&JTJ, &JTJ);
        if (GO_RESULT_OK != retval) return retval;
        go_matrix_matrix_mult(&JTJ, &JT, Jinv);
    }

    return GO_RESULT_OK;
}

go_integer genserkins::inv_iterations() {
    return iterations;
}

go_result genserkins::inv_set_max_iterations(go_integer i) {
    if (i <= 0) return GO_RESULT_ERROR;
    max_iterations = i;
    return GO_RESULT_OK;
}

go_integer genserkins::inv_get_max_iterations() {
    return max_iterations;
}
