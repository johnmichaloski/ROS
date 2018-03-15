

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
#include "RvizCheckers.h"

#include <algorithm>
#include <stdarg.h>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <iterator> 
#include <fstream>   
#include <iomanip>


//#include <boost/bind.hpp>
//#include <boost/thread/mutex.hpp>

#include "nist_robotsnc/Debug.h"

using namespace Conversion;

RvizCheckers::RvizCheckers(ros::NodeHandle &nh) : _nh(nh) {
	XOFFSET = -0.16;
	YOFFSET = -0.20;
	ZOFFSET=0.0;
	SQOFFSET = 0.04;
	RADIUS = 0.025;
	HEIGHT= 0.015;
	BOARD_DIRECTION=UPDOWN;

	curplayer = Checkers::RED;

	sub = _nh.subscribe("clicked_point", 10, &RvizCheckers::callback, this);
	bFlag = false;
}

void RvizCheckers::BoardSetup(double xoffset, double yoffset, double zoffset, double sqoffset,
    double checkerradius, double checkerheight, int boarddirection ) {
		XOFFSET = xoffset;
		YOFFSET = yoffset;
		ZOFFSET = zoffset;
		SQOFFSET = sqoffset;
		RADIUS = checkerradius;
		HEIGHT = checkerheight;
		BOARD_DIRECTION = boarddirection;
}
Checkers::CheckersGame & RvizCheckers::Game() {
	return game;
}

bool & RvizCheckers::Ready() {
	return bFlag;
}

void RvizCheckers::callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
	geometry_msgs::Point pt = msg->point;
	std::cout << Globals.StrFormat("geometry_msgs::PointStamped=%f:%f:%f\n", pt.x, pt.y, pt.z);
	bFlag = true;

}

tf::Pose RvizCheckers::GetPose(int row, int col) {
	// Input into this method assumes only correct row/col choices
	assert(row < ROWS);
	assert(col < COLS);
	assert((row + col) % 2 == 1);
	tf::Vector3 v;
	tf::Pose pose;
	if (BOARD_DIRECTION == LEFTRIGHT) {
		double rowoffset = XOFFSET + (SQOFFSET * row);
		double coloffset = YOFFSET + (col * SQOFFSET);
		v = tf::Vector3(rowoffset + SQOFFSET / 2.0, coloffset + SQOFFSET / 2.0, ZOFFSET+HEIGHT);
		pose = Convert<tf::Vector3, tf::Pose>(v);
	} else if (BOARD_DIRECTION == UPDOWN) {
		double rowoffset = YOFFSET + (SQOFFSET * row);
		double coloffset = XOFFSET + (col * SQOFFSET);
		v = tf::Vector3(coloffset + SQOFFSET / 2.0, rowoffset + SQOFFSET / 2.0, ZOFFSET+HEIGHT);
		pose = Convert<tf::Vector3, tf::Pose> (v);
	}
	return pose;
}

double RvizCheckers::RowOffset(int row) {
	if (BOARD_DIRECTION == LEFTRIGHT) {
		return XOFFSET + (SQOFFSET * row);
    } //else if (BOARD_DIRECTION == UPDOWN) {
		return YOFFSET + (SQOFFSET * row);
    //}
}

double RvizCheckers::ColOffset(int col) {
	if (BOARD_DIRECTION == LEFTRIGHT) {
		return YOFFSET + (SQOFFSET * col);
    } //else if (BOARD_DIRECTION == UPDOWN) {
		return XOFFSET + (SQOFFSET * col);
    //}
}

tf::Vector3 RvizCheckers::GetCentroid(double rowoffset, double coloffset) {
	if (BOARD_DIRECTION == LEFTRIGHT) {
		return tf::Vector3(rowoffset + SQOFFSET / 2.0, coloffset + SQOFFSET / 2.0, ZOFFSET+HEIGHT);
    } //else if (BOARD_DIRECTION == UPDOWN) {
		return tf::Vector3(coloffset + SQOFFSET / 2.0, rowoffset + SQOFFSET / 2.0, ZOFFSET+HEIGHT);
    //}
}

/** \brief computes upper limit of board (top = HEIGHT)*/
tf::Vector3 RvizCheckers::GetUp(double rowoffset, double coloffset) {
	tf::Vector3 up;
	if (BOARD_DIRECTION == LEFTRIGHT) {
		up = tf::Vector3(rowoffset, coloffset,ZOFFSET+ HEIGHT);
	} else if (BOARD_DIRECTION == UPDOWN) {
		up = tf::Vector3(coloffset, rowoffset,ZOFFSET+ HEIGHT);
	}
	return up;
}

/** \brief computes lower limit of board (bottom)*/
tf::Vector3 RvizCheckers::GetDown(double rowoffset, double coloffset) {
	tf::Vector3 down;
	if (BOARD_DIRECTION == LEFTRIGHT) {
		down = tf::Vector3(rowoffset + SQOFFSET, coloffset + SQOFFSET, ZOFFSET);
	} else if (BOARD_DIRECTION == UPDOWN) {
		down = tf::Vector3(coloffset + SQOFFSET, rowoffset + SQOFFSET, ZOFFSET);
	}
	return down;
}

void RvizCheckers::RvizBoardSetup() {
	tf::Quaternion qidentity(0.0, 0.0, 0.0, 1.0);

	for (size_t row = 0; row < ROWS; row++) {
		double rowoffset = RowOffset(row);
		for (size_t i = 0; i < COLS; i = i + 2) {
			double coloffset = ColOffset(i);
			if (row % 2 == 0) coloffset = coloffset + SQOFFSET; // red offset at zero
			//LOG_DEBUG << "Rowoffset="<<rowoffset << " Coloffset="<<coloffset << "\n";

			std::string sqname = Globals.StrFormat("Square[%d:%d]", row, i);
			SceneObject & obj(SceneObject::nullref);

			obj = pScene->CreateCuboid(sqname,
				"Checkerboard",
				tf::Pose(qidentity, GetUp(rowoffset, coloffset)),
				tf::Pose(qidentity, GetDown(rowoffset, coloffset)),
				Scene::GetColor("WHITE"));

			obj.centroid = GetCentroid(rowoffset, coloffset);

			sqname = Globals.StrFormat("Square[%d:%d]", row, i + 1);
			if (row % 2 == 0) coloffset = coloffset - SQOFFSET; // red offset at zero
			else coloffset = coloffset + SQOFFSET;

			obj = pScene->CreateCuboid(sqname,
				"Checkerboard",
				tf::Pose(qidentity, GetUp(rowoffset, coloffset)),
				tf::Pose(qidentity, GetDown(rowoffset, coloffset)),
				Scene::GetColor("BLACK"));
			obj.centroid = GetCentroid(rowoffset, coloffset);
		}
	}
}

void RvizCheckers::RvizPiecesSetup() {     
	tf::Quaternion qidentity(0.0, 0.0, 0.0, 1.0);
	LOG_DEBUG << Game().printDisplayFancy(this->Game().Board()).c_str();
	for (size_t row = 0; row < ROWS; row++) {
		double rowoffset = RowOffset(row);
		for (size_t i = 0; i < COLS; i = i + 2) {
			double coloffset = ColOffset(i);
			if (row % 2 == 0) coloffset = coloffset + SQOFFSET; // red offset at zero

			SceneObject & checker(SceneObject::nullref);
			double checkerheight = HEIGHT/2.0;
			size_t checkercol = (row % 2 == 0) ? i + 1 : i;
			std::string checkercolor = "CLEAR";

			int piece = this->Game().Board()[row][checkercol];
			if (ISRED(piece)) 
				checkercolor = "RED";
			if (ISBLACK(piece)) 
				checkercolor = "BLACK";
			if (ISKING(piece)) 
				checkerheight *= 2;

			if (checkercolor != "CLEAR") {
				tf::Pose epose = tf::Pose(qidentity,GetCentroid(rowoffset, coloffset));
				std::string checkername = Globals.StrFormat("Checker[%d:%d]", row, checkercol);
				checker = pScene->CreateCylinder(checkername,
					"Cylinder",
					epose, 
					Scene::GetColor(checkercolor),
					checkerheight,
					RADIUS,
					"Cylinder");
			}
		}
	}
}


bool RvizCheckers::IsKing(Checkers::Move m) {
	return ISKING(game.Board()[m.row][m.col]);
}

bool RvizCheckers::IsAlreadyKing(Checkers::Move m) {
	return ISKING(game.Board()[m.row][m.col]);
}

int RvizCheckers::Player() {
	return curplayer;
}

int RvizCheckers::Opponent() {
	return (curplayer == Checkers::BLACK) ? Checkers::RED : Checkers::BLACK;
}

int RvizCheckers::NextPlayer() {
	curplayer = Opponent();
	return curplayer;
}

bool RvizCheckers::CheckersMove(int & player, Checkers::Move & from, Checkers::Move &to) {
	static int movenum = 0;
	int opponent;
    winner=Checkers::EMPTY;
	player = Player();
	opponent = Opponent();

	// Generate moves from current position (from move).
	std::map<Checkers::Move, std::vector < Checkers::Move>> moves = game.GenerateMoveList(game.Board(), player);
	std::cout << game.DumpLegalMoves(moves);
	if (moves.size() == 0) {
		winner = opponent;
		return true;
	}
	// From entire list of possible moves, pick best current (m1) and move (m2).
	Checkers::Move m1, m2;
	//game.RandomMove(moves, m1, m2);
	game.MinMaxBestMove(moves, game.Board(), player, m1, m2);

	// m1 and m2 have best move
	m2.Start(player, m1.row, m1.col);
	std::cout << movenum++ << "move= "; 	game.Dump(std::cout, m2);

	// Make move on game board.
	game.Board() = game.MakeMove(game.Board(), player, m1.row, m1.col, m2);
	//game.printDisplayFancy(game.Board());

	// Check if winning move.
	from = m1;
	to = m2;
	if (game.IsWin(game.Board(), player)) {
		winner = player;
		return true;
	}
	NextPlayer();
	return false;
}



