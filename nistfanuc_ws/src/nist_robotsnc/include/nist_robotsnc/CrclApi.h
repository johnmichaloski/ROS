#pragma once

/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include <ros/ros.h>

#include "RCS.h"
#include "Kinematics.h"
#include "Scene.h"
#include "Controller.h"
#include "Conversions.h"

/**
 * \brief CrclApi provides some inline Crcl commands that are queued to given CNC.
 */
class CrclApi {
protected:
    boost::shared_ptr<RCS::CController>_cnc;
    static int crclcommandnum; /**<  crcl command number for this robot */
public:
    double mydwell; /**<  global dwell time */
    tf::Quaternion QBend; /**< rotation so end effect is facing down (as opposed to sideways)*/
    static RCS::Pose retract;
    /*!
     * \brief Constructor of commands requires reference to Controller object.
     * \param cnc is pointer to CController instance.
     */
    CrclApi(boost::shared_ptr<RCS::CController> cnc) : _cnc(cnc) {
        mydwell = .50;
        QBend = cnc->QBend();
    }

    /*!
     * \brief Return pointer to CNC instance of this robot command object.
     * \return cnc is pointer to CController instance.
     */
    boost::shared_ptr<RCS::CController> cnc() {
        return _cnc;
    }
    /*!
     * \brief Robot picks up an object at pose with given objname.
     * \param pose of the given object to pick up.
     * \param objname name of the object that is being picked up.
     */
    void Pick(tf::Pose pose, std::string objname);
    /*!
     * \brief Robot moves to given Cartesian pose, and may move object .
     * \param pose of the given object to move.
     * \param objname name of the object that is being moved.
     */
    void MoveTo(tf::Pose pose, std::string objname = "");
    /*!
     * \brief Robot dwells for given dwell time in seconds. .
     * \param dwelltime time to dwell in seconds.
     * */
    void DoDwell(double dwelltime);
    //void AddGripperOffset();
    /*!
     * \brief Robot opens gripper. .
     */
    void OpenGripper();
    /*!
     * \brief Robot closes gripper. .
     */
    void CloseGripper();
    /*!
     * \brief Set the robot gripper to the given percentage (from 0..1). .
     * \param ee end effector percentage close(0)..open(1). Note gripper could be closed at 1!
     */
    void SetGripper(double ee);
    /*!
     * \brief Robot places up an object at pose with given objname. 
     * Retracts to given retraction offset from place pose.
     * \param pose of the given object to pick up.
     * \param objname name of the object that is being picked up.
     */
    void Place(tf::Pose pose, std::string objname);

    /*!
     * \brief Move object with given objname to given pose.
     * \param objname name of the object that is being picked up.
     * \param pose of the given object to pick up.
     * \param color sets the object color
     * 
     */
    void MoveObject(std::string objname, tf::Pose pose, std::string color);
    void GraspObject(std::string objname);
    void ReleaseObject(std::string objname);
    /*!
     * \brief Erases object with given objname .
     * \param objname name of the object that is being picked up.
     */
    void EraseObject(std::string objname);
    /*!
     * \brief Robot moves joints as defined joint number vector to positions.
     * Coordinated joint motion "assumed".
     * \param jointnum is a list of joints to move.
     * \param positions is final joint position 
     */
    void MoveJoints(std::vector<long unsigned int> jointnum,
            std::vector<double> positions);


};
