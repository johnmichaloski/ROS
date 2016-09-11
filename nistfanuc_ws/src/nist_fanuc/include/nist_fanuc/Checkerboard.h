

#pragma once
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "Globals.h"
#include <boost/bind.hpp>
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
#include "Scene.h"
using namespace Conversion;
using namespace rviz_visual_tools;
#define ROWS 8
#define COLS 8



//#define RED (rviz_visual_tools::RED) 
//#define BLACK (rviz_visual_tools::BLACK) 
#define EMPTY  NULL

#define ISRED(c) (c != NULL && c->color == rviz_visual_tools::RED)
#define ISBLACK(c) (c!=NULL && c->color == rviz_visual_tools::BLACK)
#define ISPLAYER(c,player) (c!=NULL && c->color == player)
#define ISEMPTY(c) (c == NULL)
#define ISKING(c) ( c != NULL && c.King)
#define SIGN(x) (x < 0) ? -1 : (x > 0)

struct BoardType : std::vector<std::vector<ObjectDB *> > {

    BoardType() {
        this->resize(ROWS, std::vector<ObjectDB *> (COLS, NULL));
    }
};

struct Move {

    Move() {
        player = 0;
        bJump = bDoubleJumps = false;
        row = col = srow = scol = 0;
        score = 0.0;
    }

    Move(int row, int col, bool bJump = false) {
        this->row = row;
        this->col = col;
        this->bJump = bJump;
        player = 0;
        bDoubleJumps = false;
        srow = scol = 0;
        score = 0.0;
    }
    int row;
    int col;
    int player;
    bool bJump;
    bool bDoubleJumps;
    std::vector<Move> doublejumps;
    int srow;
    int scol;
    double score;

    void Start(int player, int row, int col) {
        this->player = player;
        srow = row;
        scol = col;
    }

    // For std::map use

    friend bool operator<(const Move &left, const Move &other) {
        return ( (left.row * 8 + left.col)< (other.row * 8 + other.col));
    }
    // For serialization

    friend std::ostream & operator<<(std::ostream & output_out, const Move & move_in) {
        return output_out << move_in.player << "\t" << move_in.srow << "\t" << move_in.scol << "\t"
                << move_in.row << "\t" << move_in.col << "\t" << move_in.bJump;
    }

    friend std::istream& operator>>(std::istream& s_in, Move & move_out) {
        s_in >> move_out.player >> move_out.srow >> move_out.scol >> move_out.row >> move_out.col >> move_out.bJump;
        return s_in;
    }

};

/**
  bool publishCylinder(const Eigen::Affine3d &pose, colors color = BLUE, double height = 0.1, double radius = 0.01,
                       const std::string &ns = "Cylinder");
 geometry_msgs::Pose convertPose(const Eigen::Affine3d &pose);
 */
struct Checkerboard {
    double xoffset;
    double rowoffset;
    double yoffset;
    double offset;
    BoardType board;

    Checkerboard() {
        xoffset = 1.0;
        rowoffset = 0.04;
        yoffset = -0.5;
        offset = 0.04;

    }

    BoardType & Board() {
        return board;
    }

    void RvizSetup() {
        tf::Quaternion qidentity(0.0, 0.0, 0.0, 1.0);

        for (size_t row = 0; row < 8; row++) {

            double rowoffset = xoffset + (offset * row);
            for (size_t i = 0; i <= 8; i = i + 2) {
                double coloffset = yoffset + (i * offset);
                if (row % 2 == 0) coloffset = coloffset + offset; // red offset at zero

                Eigen::Vector3d up(rowoffset, coloffset, 0.01);
                Eigen::Vector3d down(rowoffset + offset, coloffset + offset, 0.0);

                ObjectDB * obj = new ObjectDB(Globals.StrFormat("Square[%d:%d]", row, i),
                        "Checkerboard",
                        RcsPose2Affine3d(RCS::Pose(qidentity, vectorEigenToTF<Eigen::Vector3d>(up))),
                        RcsPose2Affine3d(RCS::Pose(qidentity, vectorEigenToTF<Eigen::Vector3d>(down))),
                        rviz_visual_tools::WHITE);
                obj->centroid = Eigen::Vector3d(rowoffset + offset / 2.0, coloffset + offset / 2.0, 0.01);
                ObjectDB::Save(obj);

                ObjectDB * checker;
                size_t checkercol = (row % 2 == 0)? i+1 : i; //  coloffset = coloffset + offset; // red offset at zero
               rviz_visual_tools::colors checkercolor = rviz_visual_tools::BLUE;
                if (row < 3) checkercolor = rviz_visual_tools::RED;
                if (row >= 5) checkercolor = rviz_visual_tools::BLACK;
                std::string checkername = Globals.StrFormat("Checker[%d:%d]", row, checkercol);
                checker = new ObjectDB(checkername,
                        "Cylinder",
                        Eigen::Affine3d(Eigen::Translation3d(obj->centroid)),
                        checkercolor,
                        0.01,
                        0.02,
                        "Cylinder");

                board[row][i] = checker;
                ObjectDB::Save(checker);


                if (row % 2 == 0) coloffset = coloffset - offset; // red offset at zero
                else coloffset = coloffset + offset;
                Eigen::Vector3d bup(rowoffset, coloffset, 0.01);
                Eigen::Vector3d bdown(rowoffset + offset, coloffset + offset, 0.0);
                obj = new ObjectDB(Globals.StrFormat("Square[%d:%d]", row, i),
                        "Checkerboard",
                        RcsPose2Affine3d(RCS::Pose(qidentity, vectorEigenToTF<Eigen::Vector3d>(bup))),
                        RcsPose2Affine3d(RCS::Pose(qidentity, vectorEigenToTF<Eigen::Vector3d>(bdown))),
                        rviz_visual_tools::BLACK);
                obj->centroid = Eigen::Vector3d(rowoffset + offset / 2.0, coloffset + offset / 2.0, 0.01);
                ObjectDB::Save(obj);
            }
        }
    }

    void KingMe(BoardType & inboard) {
        for (int i = 0; i < 8; i++) {
            if (ISBLACK(inboard[0][i]))
                inboard[0][i]->King = true;

            if (ISRED(inboard[7][i]))
                inboard[7][i]->King = true;
        }
    }

    void Jump(BoardType & inboard, int i, int j, Move move) {
        ObjectDB * temp;
        //std::cout <<  StrFormat("SWAP: %d,%d to %d,%d\n", i, j, move.row, move.col).c_str();
        //        temp = inboard[i][j];
        //        inboard[i][j] = inboard[move.row][move.col];
        //        inboard[move.row][move.col] = temp;
        KingMe(inboard); // check if kinged
    }

    bool IsWin(BoardType & inboard, int player) {
        int opponent = (player == BLACK) ? RED : BLACK;
        for (int i = 0; i < ROWS; i++)
            for (int j = 0; j < COLS; j++) {
                if (ISPLAYER(inboard[i][j], opponent))
                    return false;
            }
        return true;
    }

    void SetCheckerColor(Move m, rviz_visual_tools::colors color) {
        std::string checkername = Globals.StrFormat("Checker[%d:%d]", m.row, m.col);
        Eigen::Affine3d pose = ObjectDB::FindPose(checkername);
        ChangeColor(checkername, color);
        //UpdateScene(checkername, pose, color);
    }
    // Robot doesn't move yet
#if 1

    BoardType PhysicalMove(BoardType inboard, int player, int i, int j, Move m) {
        std::string errmsg;
        std::string typemove("move");
        if (m.bJump)
            typemove = "jump";
#if 0
        int rowsign = SIGN(m.row - i);
        int colsign = SIGN(m.col - j);
        if (abs(i - m.row) == 2) {
            // Clear
            SetCheckerColor(Move(i + (rowsign * 1), j + (colsign * 1), rviz_visual_tools::CLEAR)
         }
#endif
        Jump(inboard, i, j, m);
        SetCheckerColor(Move(i, j), rviz_visual_tools::CLEAR);
         ros::spinOnce();
        SetCheckerColor(m, player);

                // Single jump only for now
#if 0
                Move move(m.row, m.col, m.bJump);
        if (move.doublejumps.size() > 0) {
            Move m2 = move.doublejumps[0];
                    inboard = MakeMove(inboard, player, m.row, m.col, m2);
        }

        // Save move - only good if no mixmax lookahead
        move.Start(player, i, j);
                allmoves.push_back(move);
#endif
                return inboard;
    }
#endif
};