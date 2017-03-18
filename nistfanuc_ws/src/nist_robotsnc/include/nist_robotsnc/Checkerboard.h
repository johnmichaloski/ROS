

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

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>


#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <map>
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


#include "Globals.h"
#include "Scene.h"
// Checkers makes move 
#include "Checkers.h"
#include "Controller.h"
#include "Debug.h"
//#include "Demo.h"

using namespace Conversion;
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

    ros::Subscriber sub;
    ros::NodeHandle &_nh;
    bool bFlag;

    RvizCheckers(ros::NodeHandle &nh) : _nh(nh) {
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

    void BoardSetup(double xoffset = -0.16, double yoffset = 0.20, double zoffset = 0.0, double sqoffset = 0.04,
            double checkerradius = 0.025, double checkerheight = 0.015, int boarddirection = UPDOWN) {
        XOFFSET = xoffset;
        YOFFSET = yoffset;
        ZOFFSET = zoffset;
        SQOFFSET = sqoffset;
        RADIUS = checkerradius;
        HEIGHT = checkerheight;
        BOARD_DIRECTION = boarddirection;
    }
    Checkers::CheckersGame & Game() {
        return game;
    }

    bool & Ready() {
        return bFlag;
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        geometry_msgs::Point pt = msg->point;
        std::cout << Globals.StrFormat("geometry_msgs::PointStamped=%f:%f:%f\n", pt.x, pt.y, pt.z);
        bFlag = true;

    }

    Eigen::Affine3d GetPose(int row, int col) {
        // Input into this method assumes only correct row/col choices
        assert(row < ROWS);
        assert(col < COLS);
        assert((row + col) % 2 == 1);
        Eigen::Vector3d v;
        Eigen::Affine3d pose;
        if (BOARD_DIRECTION == LEFTRIGHT) {
            double rowoffset = XOFFSET + (SQOFFSET * row);
            double coloffset = YOFFSET + (col * SQOFFSET);
            v = Eigen::Vector3d(rowoffset + SQOFFSET / 2.0, coloffset + SQOFFSET / 2.0, ZOFFSET+HEIGHT);
            pose = Convert<Eigen::Vector3d, Eigen::Affine3d>(v);
        } else if (BOARD_DIRECTION == UPDOWN) {
            double rowoffset = YOFFSET + (SQOFFSET * row);
            double coloffset = XOFFSET + (col * SQOFFSET);
            v = Eigen::Vector3d(coloffset + SQOFFSET / 2.0, rowoffset + SQOFFSET / 2.0, ZOFFSET+HEIGHT);
            pose = Convert<Eigen::Vector3d, Eigen::Affine3d> (v);
        }
        return pose;
    }

    double RowOffset(int row) {
        if (BOARD_DIRECTION == LEFTRIGHT) {
        return XOFFSET + (SQOFFSET * row);
       } else if (BOARD_DIRECTION == UPDOWN) {
        return YOFFSET + (SQOFFSET * row);
       }
    }

    double ColOffset(int col) {
        if (BOARD_DIRECTION == LEFTRIGHT) {
        return YOFFSET + (SQOFFSET * col);
       } else if (BOARD_DIRECTION == UPDOWN) {
        return XOFFSET + (SQOFFSET * col);
       }
    }

    Eigen::Vector3d GetCentroid(double rowoffset, double coloffset) {
        if (BOARD_DIRECTION == LEFTRIGHT) {
        return Eigen::Vector3d(rowoffset + SQOFFSET / 2.0, coloffset + SQOFFSET / 2.0, ZOFFSET+HEIGHT);
       } else if (BOARD_DIRECTION == UPDOWN) {
        return Eigen::Vector3d(coloffset + SQOFFSET / 2.0, rowoffset + SQOFFSET / 2.0, ZOFFSET+HEIGHT);
       }
    }

    /** \brief computes upper limit of board (top = HEIGHT)*/
    tf::Vector3 GetUp(double rowoffset, double coloffset) {
        Eigen::Vector3d up;
        if (BOARD_DIRECTION == LEFTRIGHT) {
            up = Eigen::Vector3d(rowoffset, coloffset,ZOFFSET+ HEIGHT);
        } else if (BOARD_DIRECTION == UPDOWN) {
            up = Eigen::Vector3d(coloffset, rowoffset,ZOFFSET+ HEIGHT);
        }
        return Convert<Eigen::Vector3d, tf::Vector3 >(up);
    }

    /** \brief computes lower limit of board (bottom)*/
    tf::Vector3 GetDown(double rowoffset, double coloffset) {
        Eigen::Vector3d down;
        if (BOARD_DIRECTION == LEFTRIGHT) {
            down = Eigen::Vector3d(rowoffset + SQOFFSET, coloffset + SQOFFSET, ZOFFSET);
        } else if (BOARD_DIRECTION == UPDOWN) {
            down = Eigen::Vector3d(coloffset + SQOFFSET, rowoffset + SQOFFSET, ZOFFSET);
        }
        return Convert<Eigen::Vector3d, tf::Vector3 >(down);
    }

    void RvizBoardSetup() {
        tf::Quaternion qidentity(0.0, 0.0, 0.0, 1.0);

        for (size_t row = 0; row < ROWS; row++) {
            double rowoffset = RowOffset(row);
            for (size_t i = 0; i < COLS; i = i + 2) {
                double coloffset = ColOffset(i);
                if (row % 2 == 0) coloffset = coloffset + SQOFFSET; // red offset at zero
                //LOG_DEBUG << "Rowoffset="<<rowoffset << " Coloffset="<<coloffset << "\n";

                std::string sqname = Globals.StrFormat("Square[%d:%d]", row, i);
                ObjectDB * obj;
 
                obj = pScene->CreateCuboid(sqname,
                        "Checkerboard",
                         Convert<tf::Pose, Eigen::Affine3d>(tf::Pose(qidentity, GetUp(rowoffset, coloffset))),
                         Convert<tf::Pose, Eigen::Affine3d>(tf::Pose(qidentity, GetDown(rowoffset, coloffset))),
                        "WHITE");
               
                obj->centroid = GetCentroid(rowoffset, coloffset);

                sqname = Globals.StrFormat("Square[%d:%d]", row, i + 1);
                if (row % 2 == 0) coloffset = coloffset - SQOFFSET; // red offset at zero
                else coloffset = coloffset + SQOFFSET;

                obj = pScene->CreateCuboid(sqname,
                        "Checkerboard",
                        Convert<tf::Pose, Eigen::Affine3d>(tf::Pose(qidentity, GetUp(rowoffset, coloffset))),
                        Convert<tf::Pose, Eigen::Affine3d>(tf::Pose(qidentity, GetDown(rowoffset, coloffset))),
                        "BLACK");
                obj->centroid = GetCentroid(rowoffset, coloffset);
            }
        }
    }

    void RvizPiecesSetup() {     
        LOG_DEBUG << Game().printDisplayFancy(this->Game().Board()).c_str();
        for (size_t row = 0; row < ROWS; row++) {
            double rowoffset = RowOffset(row);
            for (size_t i = 0; i < COLS; i = i + 2) {
                double coloffset = ColOffset(i);
                if (row % 2 == 0) coloffset = coloffset + SQOFFSET; // red offset at zero

                ObjectDB * checker;
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
                    Eigen::Affine3d epose = Eigen::Affine3d(Eigen::Translation3d(GetCentroid(rowoffset, coloffset)));
                     std::string checkername = Globals.StrFormat("Checker[%d:%d]", row, checkercol);
                    checker = pScene->CreateCylinder(checkername,
                            "Cylinder",
                            epose, //FIXME: base offset?
                            checkercolor,
                            checkerheight,
                            RADIUS,
                            "Cylinder");
                }
            }
        }
    }

    void SetCheckerColor(Checkers::Move m, std::string color) { // rviz_visual_tools::colors color) {
        std::string checkername = Globals.StrFormat("Checker[%d:%d]", m.row, m.col);
        Eigen::Affine3d pose = pScene->FindPose(checkername);
        pScene->ChangeColor(checkername, color);
    }

    bool IsKing(Checkers::Move m) {
        return ISKING(game.Board()[m.row][m.col]);
    }

    bool IsAlreadyKing(Checkers::Move m) {
        return ISKING(game.Board()[m.row][m.col]);
    }

    int Player() {
        return curplayer;
    }

    int Opponent() {
        return (curplayer == Checkers::BLACK) ? Checkers::RED : Checkers::BLACK;
    }

    int NextPlayer() {
        curplayer = Opponent();
        return curplayer;
    }

    bool CheckersMove(int & player, Checkers::Move & from, Checkers::Move &to) {
        static int movenum = 0;
        player = Player();
        int opponent;
        int winner;

        //player = (player == RED) ? BLACK : RED;
        opponent = (player == Checkers::BLACK) ? Checkers::RED : Checkers::BLACK;
        std::map<Checkers::Move, std::vector < Checkers::Move>> moves = game.GenerateMoveList(game.Board(), player);
        std::cout << game.DumpLegalMoves(moves);
        if (moves.size() == 0) {
            winner = opponent;
            return true;
        }
        Checkers::Move m1, m2;
        //game.RandomMove(moves, m1, m2);
        game.MinMaxBestMove(moves, game.Board(), player, m1, m2);
        m2.Start(player, m1.row, m1.col);
        std::cout << movenum++ << "move= ";
        game.Dump(std::cout, m2);
        game.Board() = game.MakeMove(game.Board(), player, m1.row, m1.col, m2);
        //game.printDisplayFancy(game.Board());

        from = m1;
        to = m2;
        if (game.IsWin(game.Board(), player)) {
            winner = player;
            return true;
        }
        NextPlayer();
        return false;
    }

};

