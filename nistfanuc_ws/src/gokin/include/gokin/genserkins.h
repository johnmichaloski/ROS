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
  genserkins.h
 */

#ifndef GENSERKINS_H
#define GENSERKINS_H
#include <string>
#include <vector>

#include "gokin/gotypes.h"  /* go_result, go_integer */
#include "gokin/gomath.h"  /* go_pose */
#include "gokin/gokin_.h"  /* type */


/*! 
  The maximum number of joints supported by the general serial
  kinematics. Make this at least 6; a device can have fewer than these.
 */
#define go_MAX_JOINTS 8

namespace gomotion {

    struct genserkins  {
        std::vector<go_link> links; /*!< The link description of the device. */
        //go_integer link_num; /*!< How many are actually present. */
        go_integer iterations; /*!< How many iterations were actually used to compute the inverse kinematics. */
        go_integer max_iterations; /*!< Number of iterations after which to give up and report an error. */
        go_real m_per_length_units;
        go_real rad_per_angle_units;
        genserkins::genserkins();
        static genserkins * create();
        go_result set_params(std::vector< std::vector<double>> params);

        virtual go_integer size(void);

        virtual go_result init();

        virtual std::string get_name(void);

        virtual go_integer num_joints();

        virtual go_result fwd(
                const go_real *joint,
                go_pose * world);

        virtual go_result inv(
                const go_pose * world,
                go_real *joints);

        virtual go_result set_parameters(go_link *params, go_integer num);

        virtual go_result get_parameters(go_link *params, go_integer num);

        virtual go_result jac_inv(
                const go_pose *pos,
                const go_vel *vel,
                const go_real *joints,
                go_real *jointvels);


        virtual go_result jac_fwd(
                const go_real *joints,
                const go_real *jointvels,
                const go_pose *pos,
                go_vel *vel);

        /*
          Extras, not callable using  wrapper but if you know you have
          linked in these kinematics, go ahead and call these for your ad hoc
          purposes.
         */

        virtual go_result compute_jfwd(go_link *link_params, int link_number, go_matrix *Jfwd, go_pose *T_L_0);

        virtual go_result compute_jinv(go_matrix *Jfwd, go_matrix *Jinv, go_vector *weights);

        /*! Returns the number of iterations used during the last call to the
          inverse kinematics functions */
        virtual go_integer inv_iterations();

        /*! Sets the maximum number of iterations to use in future calls to
          the inverse kinematics functions, after which an error will be
          reported */
        virtual go_result inv_set_max_iterations(go_integer i);

        /*! Returns the maximum number of iterations that will be used to
         compute inverse kinematics functions */
        virtual go_integer inv_get_max_iterations();

        virtual go_kin_type get_type() {
            return BOTH;
        }

        // These are not implemented in serkins

        virtual go_result set_flags(
                go_flag fflags,
                go_flag iflags) {
            return 0;
        }

        virtual go_result get_flags(
                go_flag *fflags,
                go_flag *iflags) {
            return 0;
        }

        void SetAngleUnits(double d) {
            rad_per_angle_units = 1.0 / d;
        }

        void SetLengthUnits(double d) {
            m_per_length_units = 1.0 / d;
        }


    };
}
#endif
