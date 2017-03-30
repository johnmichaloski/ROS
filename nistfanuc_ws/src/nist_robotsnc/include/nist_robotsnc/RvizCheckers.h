

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

////#include <rviz_visual_tools/rviz_visual_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>


#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "Scene.h"
#include "Globals.h"
#include "Checkers.h"
#include "Controller.h"

#if 0
#include <algorithm>
#include <stdarg.h>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <iterator> 
#include <fstream>   
#include <iomanip>
#include <sstream>


#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>


#include "Debug.h"
//#include "Demo.h"
#endif

#define LEFTRIGHT 1
#define UPDOWN  -1

struct RvizCheckers {
	Checkers::CheckersGame game; /**< game the robots are playing */
#ifndef ROWS
	static const int ROWS = 8; /**< number of rows on checkerboard */
	static const int COLS = 8; /**< number of columns on checkerboard */
#endif
	double HEIGHT; /**< rviz height of single checker */
	/** CHeckers board rviz dimensions */
	double XOFFSET; /**< x offset from world 0,0,0 */
	double YOFFSET; /**< y offset from world 0,0,0 */
	double ZOFFSET; /**< z offset from world 0,0,0 */
	double SQOFFSET; /**< offset between board positions on checkerboard */
	double RADIUS; /**< radius of checker cylinder piece */
	int BOARD_DIRECTION;
	int curplayer; /**< current checker player - either black or red */

	ros::Subscriber sub; /**< ROS subscriber to clicked_point message. */
	ros::NodeHandle &_nh; /**< Reference to ROS node handle for publish/subscribe etc. */
	bool bFlag;  /**< flag if ready. */
	int winner; /**< winning player */

	/*!
	*\brief RvizCheckers constructor, default board dimension setup, red player, and subscribe clicked_point.
	*/
	RvizCheckers(ros::NodeHandle &nh);
	/*!
	* \brief Rviz checkers board dimension setup.
	* \param xoffset the beginning x origin in meters
	* \param yoffset the beginning y origin in meters
	* \param zoffset the beginning z origin in meters
	* \param sqoffset the checker square offset in x,y direction in meters(it is square)
	* \param checkerradius radius of checker in meters
	* \param checkerheight height of checker in meters
	* \param boarddirection direction of checkboard UPDOWN or LEFTRIGHT

	*/
	void BoardSetup(double xoffset = -0.16, double yoffset = 0.20, double zoffset = 0.0, double sqoffset = 0.04,
		double checkerradius = 0.025, double checkerheight = 0.015, int boarddirection = UPDOWN);

	/*!
	*\brief Return underlying checker game. 
	\return reference to checkers game that rviz is playing
	*/
	Checkers::CheckersGame & Game();

	/*!
	*\brief Return ready flag. 
	\return bool true if ready
	*/
	bool & Ready() ;

	/*!
	*\brief callback when rviz point_clicked message is published 
	* \param msg geometry message sent by rviz
	*/
	void callback(const geometry_msgs::PointStamped::ConstPtr& msg) ;

	/*!
	*\brief Return physical position given row and column.
	* Must account for board direction.
	* \param msg geometry message sent by rviz
	* \return tf pose describing position and orientation
	*/
	tf::Pose GetPose(int row, int col);

	/*!
	*\brief Return physical position given row of starting position.
	* Must account for board direction.
	* \return double in meters describing row offset position.
	*/
	double RowOffset(int row);

	/*!
	*\brief Return physical position given column of starting position.
	* Must account for board direction.
	* \return double in meters describing column offset position.
	*/
	double ColOffset(int col);

	/*!
	*\brief Return middle position given row offset and column offset.
	* Must account for board direction.
	* \param rowoffset beginning physical position of checker row offset in meters
	* \param coloffset beginning physical position of checker column offset in meters
	* \return tf vector giving x,y and checkerboard z offset + checker height
	*/
	tf::Vector3 GetCentroid(double rowoffset, double coloffset) ;

	/*! 
	\brief computes upper limit of board square (top = checkerboard z offset + checker height). Used in up point/down point cuboid rviz drawing.
	* \param rowoffset beginning physical position of checker row offset in meters
	* \param coloffset beginning physical position of checker column offset in meters
	* \return tf vector giving x,y and checkerboard z offset + checker height
	*/
	tf::Vector3 GetUp(double rowoffset, double coloffset) ;

	/*!
	* \brief computes lower limit of board (bottom) Used in up point/down point cuboid rviz drawing.
	* \param rowoffset beginning physical position of checker row offset in meters
	* \param coloffset beginning physical position of checker column offset in meters
	* \return tf vector giving x,y plus sq offset and bottom of checker square z
	*/
	tf::Vector3 GetDown(double rowoffset, double coloffset) ;
	/*!
	* \brief Use interface to rviz to draw black and white checkerboard .
	*/
	void RvizBoardSetup() ;
	/*!
	* \brief Use interface to rviz to draw red and black checker pieces .
	*/
	void RvizPiecesSetup();

	/*!
	* \brief Check if move make piece a king .
	* \return bool true if move will make piece a king.
	*/
	bool IsKing(Checkers::Move m) ;

	/*!
	* \brief Check if  piece a lready a king. No double kings.
	* \return bool true if piece already king.
	*/
	bool IsAlreadyKing(Checkers::Move m);
	/*!
	* \brief Return current player.
	* \return int describing either red or black.
	*/
	int Player();
	/*!
	* \brief Find opponent of current player.
	* \return int describing either red or black.
	*/
	int Opponent() ;
	/*!
	* \brief Find next player (opponent of current player).
	* \return int describing either red or black.
	*/
	int NextPlayer() ;
	/*!
	* \brief Make a checkers move. .
	* \param player is red or black player.
	* \param from reference to original move piece position.
	* \param to reference to destination move piece position.
	* \return bool return true if winning move or opponent wins since there is no move.
	*/
	bool CheckersMove(int & player, Checkers::Move & from, Checkers::Move &to);

};

