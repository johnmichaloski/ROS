

#pragma once
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
using namespace Conversion;

struct RvizCheckers {
    //static boost::mutex _reader_mutex; /**< for mutexed reading access  */
    static const int rows = 8;
    static const int cols = 8;
    double xoffset;
    double rowoffset;
    double yoffset;
    double offset;
    Checkers::Checkers game;
    int curplayer;
    ros::Subscriber sub;
    ros::NodeHandle &_nh;
    bool bFlag;

    RvizCheckers(ros::NodeHandle &nh) : _nh(nh) {
        xoffset = 0.20;
        rowoffset = 0.04;
        yoffset = -0.20;
        offset = 0.04;
        curplayer = Checkers::RED;
        sub = _nh.subscribe("clicked_point", 10, &RvizCheckers::callback, this);
        bFlag = false;
    }

    bool & Ready() { return bFlag; }
    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        geometry_msgs::Point pt = msg->point;
        std::cout << Globals.StrFormat("geometry_msgs::PointStamped=%f:%f:%f\n", pt.x, pt.y, pt.z);
        bFlag = true;

    }

    Eigen::Affine3d GetPose(int row, int col) {
        assert((row + col) % 2 == 1);
        double rowoffset = xoffset + (offset * row);
        double coloffset = yoffset + (col * offset);
        if (row % 2 == 0) coloffset = coloffset + offset;
        Eigen::Affine3d pose = visual_tools->convertPointToPose((
                Eigen::Vector3d(rowoffset + offset / 2.0, coloffset + offset / 2.0, 0.01))
                );
        return pose;
    }

    // std::vector<std::vector<Eigen::Affine3d> > boardposes;

    void RvizSetup() {
        tf::Quaternion qidentity(0.0, 0.0, 0.0, 1.0);

        for (size_t row = 0; row < rows; row++) {

            double rowoffset = xoffset + (offset * row);
            for (size_t i = 0; i <= cols; i = i + 2) {
                double coloffset = yoffset + (i * offset);
                if (row % 2 == 0) coloffset = coloffset + offset; // red offset at zero

                Eigen::Vector3d up(rowoffset, coloffset, 0.01);
                Eigen::Vector3d down(rowoffset + offset, coloffset + offset, 0.0);
                std::string sqname = Globals.StrFormat("Square[%d:%d]", row, i);
                ObjectDB * obj = new ObjectDB(sqname,
                        "Checkerboard",
                        RcsPose2Affine3d(RCS::Pose(qidentity, vectorEigenToTF<Eigen::Vector3d>(up))),
                        RcsPose2Affine3d(RCS::Pose(qidentity, vectorEigenToTF<Eigen::Vector3d>(down))),
                        rviz_visual_tools::WHITE);
                obj->centroid = Eigen::Vector3d(rowoffset + offset / 2.0, coloffset + offset / 2.0, 0.01);
                ObjectDB::Save(obj);

                ObjectDB * checker;
                size_t checkercol = (row % 2 == 0) ? i + 1 : i; //  coloffset = coloffset + offset; // red offset at zero
                rviz_visual_tools::colors checkercolor = rviz_visual_tools::CLEAR;
                if (row < 3) checkercolor = rviz_visual_tools::RED;
                if (row >= 5) checkercolor = rviz_visual_tools::BLACK;
                if (checkercolor != rviz_visual_tools::CLEAR) {
                    std::string checkername = Globals.StrFormat("Checker[%d:%d]", row, checkercol);
                    checker = new ObjectDB(checkername,
                            "Cylinder",
                            Eigen::Affine3d(Eigen::Translation3d(obj->centroid)),
                            checkercolor,
                            0.01,
                            0.02,
                            "Cylinder");
                    ObjectDB::Save(checker);
                }

                if (row % 2 == 0) coloffset = coloffset - offset; // red offset at zero
                else coloffset = coloffset + offset;
                Eigen::Vector3d bup(rowoffset, coloffset, 0.01);
                Eigen::Vector3d bdown(rowoffset + offset, coloffset + offset, 0.0);
                sqname = Globals.StrFormat("Square[%d:%d]", row, i);
                obj = new ObjectDB(sqname,
                        "Checkerboard",
                        RcsPose2Affine3d(RCS::Pose(qidentity, vectorEigenToTF<Eigen::Vector3d>(bup))),
                        RcsPose2Affine3d(RCS::Pose(qidentity, vectorEigenToTF<Eigen::Vector3d>(bdown))),
                        rviz_visual_tools::BLACK);
                obj->centroid = Eigen::Vector3d(rowoffset + offset / 2.0, coloffset + offset / 2.0, 0.01);
                ObjectDB::Save(obj);
            }
        }
    }

    void SetCheckerColor(Checkers::Move m, rviz_visual_tools::colors color) {
        std::string checkername = Globals.StrFormat("Checker[%d:%d]", m.row, m.col);
        Eigen::Affine3d pose = ObjectDB::FindPose(checkername);
        ChangeColor(checkername, color);
        //UpdateScene(checkername, pose, color);
    }

    // Robot doesn't move yet

    void PhysicalMove(int player, int i, int j, Checkers::Move m) {
        std::string errmsg;
        std::string typemove("move");
        //SetCheckerColor(Checkers::Move(i, j), rviz_visual_tools::CLEAR);
        if (m.bJump) {
            typemove = "jump";
            Checkers::Move jumped = m.Diff(Checkers::Move(i, j));
            std::string jumpedcheckername = Globals.StrFormat("Checker[%d:%d]", jumped.row, jumped.col);
            DeleteObject(jumpedcheckername);
        }
        // If blank space there can be no actual checkername2 object
        std::string checkername1 = Globals.StrFormat("Checker[%d:%d]", i, j);
        std::string checkername2 = Globals.StrFormat("Checker[%d:%d]", m.row, m.col);
        Eigen::Affine3d pose = GetPose(m.row, m.col);
        std::cout << Globals.StrFormat("[%d,%d]=%f,%f\n", m.row, m.col, pose(0,3),pose(1,3) );
        ObjectDB * obj = ObjectDB::Find(checkername1);
        assert(obj != NULL);
        Pick(Conversion::Affine3d2RcsPose(obj-> pose) , checkername1);
        Place(Conversion::Affine3d2RcsPose(pose), checkername1);
        UpdateScene(checkername1, pose, obj->color);
        ros::spinOnce();
        obj->name = checkername2; // change checker name (old space is blank)
        // Fixme: check if board moves is for kings

    }
    // Single jump only for now

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
        if (game.IsWin(game.Board(), player)) {
            winner = player;
            return true;
        }
        from = m1;
        to = m2;
        NextPlayer();
        return false;
    }

};