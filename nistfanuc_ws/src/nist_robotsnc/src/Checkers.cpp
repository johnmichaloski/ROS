

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

#include "Checkers.h"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <iterator> 
#include <fstream>   
#include <iomanip>
#include <cfloat>

namespace Checkers {



	Move::Move() {
		player = 0;
		bJump = bDoubleJumps = false;
		row = col = srow = scol = 0;
		score = 0.0;
	}

	Move::Move(int row, int col, bool bJump = false) {
		this->row = row;
		this->col = col;
		this->bJump = bJump;
		player = 0;
		bDoubleJumps = false;
		srow = scol = 0;
		score = 0.0;
	}


	void Move::Start(int player, int row, int col) {
		this->player = player;
		srow = row;
		scol = col;
	}

	BoardType::BoardType() {
		this->resize(ROWS, std::vector<int> (COLS, EMPTY));
	}

	BoardType &CheckersGame::Board() {
		return aboard;
	}

	CheckersGame::CheckersGame() {
		NewGame();
	}
	void CheckersGame::NewGame(){
		aboard[0][1] = RED;
		aboard[0][3] = RED;
		aboard[0][5] = RED;
		aboard[0][7] = RED;
		aboard[1][0] = RED;
		aboard[1][2] = RED;
		aboard[1][4] = RED;
		aboard[1][6] = RED;
		aboard[2][1] = RED;
		aboard[2][3] = RED;
		aboard[2][5] = RED;
		aboard[2][7] = RED;

		aboard[5][0] = BLACK;
		aboard[5][2] = BLACK;
		aboard[5][4] = BLACK;
		aboard[5][6] = BLACK;
		aboard[6][1] = BLACK;
		aboard[6][3] = BLACK;
		aboard[6][5] = BLACK;
		aboard[6][7] = BLACK;
		aboard[7][0] = BLACK;
		aboard[7][2] = BLACK;
		aboard[7][4] = BLACK;
		aboard[7][6] = BLACK;
	}

	void CheckersGame::Restore(std::string filename, std::vector<Move> & moves) {
		std::ifstream f(filename.c_str());
		if (!f)
			throw std::runtime_error(std::string("Checkers game could not open file : " + filename + " for reading!").c_str());
		moves.clear();
		std::copy(std::istream_iterator<Move>(f), std::istream_iterator<Move>(), std::back_inserter(moves));
	}

	void CheckersGame::Save(std::string filename) {
		std::ofstream f(filename.c_str());
		if (!f)
			throw std::runtime_error(std::string("Checkers game could not open file : " + filename + " for writing!").c_str());
		std::ostream_iterator<Move> out_it(f, "\n");
		std::copy(allmoves.begin(), allmoves.end(), out_it);
	}

	void CheckersGame::Dump(std::ostream &str, const Move move) {
		std::string typemove = move.bJump ? "jump" : "move";
		std::string splayer = ISBLACK(move.player) ? "BLACK" : "RED";
		str << StrFormat("%s %s from %d,%d to %d,%d\n", splayer.c_str(), typemove.c_str(), move.srow, move.scol, move.row, move.col);

	}

	void CheckersGame::Dump(std::ostream &str, const std::vector<Move> &moves) {
		for (size_t i = 0; i < moves.size(); i++) {
			std::string typemove = moves[i].bJump ? "jump" : "move";
			std::string splayer = ISBLACK(moves[i].player) ? "BLACK" : "RED";
			str << StrFormat("%s %s from %d,%d to %d,%d\n", splayer.c_str(), typemove.c_str(), moves[i].srow, moves[i].scol, moves[i].row, moves[i].col);

		}
	}

	bool CheckersGame::IsKing(int player, Move m) {
		return ISKING(Board()[m.row][m.col]);
	}

	bool CheckersGame::LegalRow(int n) {
		return n >= 0 && n < ROWS;
	}

	char CheckersGame::value2symbol(int i) {
		switch (i) {
		case EMPTY: return ' ';
		case RED: return 'r';
		case BLACK: return 'b';
		case RED + KING: return 'R';
		case BLACK + KING: return 'B';
		}
		return ('?');
	}

	std::string CheckersGame::printDisplayFancy(BoardType inboard) {
		std::stringstream str;
		int rr, cc;
		str << "  +---+---+---+---+---+---+---+---+\n";
		for (rr = 0; rr < ROWS; ++rr) {
			str << StrFormat("%d |", rr).c_str();
			for (cc = 0; cc < COLS; ++cc) {
				str << StrFormat(" %c |", value2symbol(inboard[rr][cc])).c_str();
			}
			str << "\n";
			str << "  +---+---+---+---+---+---+---+---+\n";
		}
		str << "    0   1   2   3   4   5   6   7\n";
		return str.str();
	}

	int CheckersGame::symbol2value(char c) {
		switch (c) {
		case ' ': return 1;
		case '@': return 1;
		case 'r': return RED;
		case 'b': return BLACK;
		case 'R': return RED + KING;
		case 'B': return BLACK + KING;
		}
		return 0;
	}

	void CheckersGame::Deserialize(std::istream& s_in, BoardType &board_out) {
		int rr, cc;
		int dummy;
		char dummyc;
		char piece;
		char str[256];
		std::string line;
		s_in >> line;

		// This will not parse the \n
		//s_in.get(str, sizeof ("    +---+---+---+---+---+---+---+---+\n"));
		int row = 0;
		while (row < ROWS && !s_in.eof()) {
			int col = 0;
			char checker[4] = {' ', ' ', ' ', ' '};
			std::getline(s_in, line);
			if (line.size() < 1)
				continue;
			if(line[0]=='+')
				continue;
			ReplaceAll(line, "   ", " @ ");
			std::cout << line << "\n";
			// blank characters are skipped if leading space in front of %c
			// maybe replace all blank space blank with blank@blank and treat @ as 0
			if (row % 2 == 0) {
				col++;
				sscanf(line.c_str(), "%d | @  | %c | @ | %c | @ | %c | @ | %c |",
					&dummy, &checker[0], &checker[1], &checker[2], &checker[3]);
			} else {
				sscanf(line.c_str(), "%d | %c | @ | %c | @ | %c | @ | %c |",
					&dummy, &checker[0], &checker[1], &checker[2], &checker[3]);
			}
			std::cout << dummy << checker[0] << checker[1] << checker[2] << &checker[3] << "\n";
			board_out[row][col] = symbol2value(checker[0]);
			board_out[row][col + 2] = symbol2value(checker[1]);
			board_out[row][col + 4] = symbol2value(checker[2]);
			board_out[row][col + 6] = symbol2value(checker[3]);
			row++;
			std::getline(s_in, line);
		}
	}

	void CheckersGame::KingMe(BoardType & inboard) {
		for (int i = 0; i < 8; i++) {
			if (ISBLACK(inboard[0][i]))
				inboard[0][i] |= KING;

			if (ISRED(inboard[7][i]))
				inboard[7][i] |= KING;
		}
	}

	void CheckersGame::Jump(BoardType & inboard, int i, int j, Move move) {
		int temp;
		temp = inboard[i][j];
		inboard[i][j] = inboard[move.row][move.col]; //????????????
		inboard[move.row][move.col] = temp;
		KingMe(inboard); // check if kinged
	}

	bool CheckersGame::IsWin(BoardType & inboard, int player) {
		int opponent = (player == BLACK) ? RED : BLACK;
		for (int i = 0; i < ROWS; i++)
			for (int j = 0; j < COLS; j++)
				if ((inboard[i][j] & opponent) > 0) // King
					return false;
		return true;
	}

	std::vector<Move> CheckersGame::BuildMoves(BoardType inboard, int player, Move from, bool bJumpOnly = false) {
		std::vector<Move> moves;
		int row = from.row;
		int col = from.col;
		int nJump;
		int opponent = (player == BLACK) ? RED : BLACK;
		bool bKing = ISKING(inboard[row][col]);
		int tries = (bKing) ? 2 : 1;
		int nDir = ISBLACK(player) ? -1 : 1;

		for (size_t n = 0; n < tries; n++) {
			nJump = 1;
			if (n == 1) nDir = -1 * nDir; //second time for kings

			if (!bJumpOnly && (col + nJump < 8)
				&& LegalRow(row + nDir * nJump)
				&& inboard[row + nDir * nJump][col + nJump] == EMPTY)
				moves.push_back(Move(row + nDir * nJump, col + nJump));

			if (!bJumpOnly && (col - nJump >= 0)
				&& LegalRow(row + nDir * nJump)
				&& inboard[row + nDir * nJump][col - nJump] == EMPTY)
				moves.push_back(Move(row + nDir * nJump, col - nJump));

			nJump = 2;
			if ((col + nJump < 8) && LegalRow(row + nDir * nJump) &&
				inboard[row + nDir * nJump][col + nJump] == EMPTY &&
				inboard[row + nDir * (nJump - 1)][col + nJump - 1] == opponent) {
					Move move(row + nDir*nJump, col + nJump, true);
					BoardType newboard = MakeMove(inboard, player, row, col, move);
					move.doublejumps = BuildMoves(newboard, player, move, true);
					moves.push_back(move);
			}

			if ((col - nJump >= 0) && LegalRow(row + nDir * nJump) &&
				inboard[row + nDir * nJump][col - nJump] == EMPTY &&
				inboard[row + nDir * (nJump - 1)][col - nJump + 1] == opponent) {
					Move move(row + nDir*nJump, col - nJump, true);
					BoardType newboard = MakeMove(inboard, player, row, col, move);
					move.doublejumps = BuildMoves(newboard, player, move, true);
					moves.push_back(move);
					//moves.push_back(Move(row+nDir*nJump,col-nJump,true));
			}
		}
		return moves;
	}

	std::map<Move, std::vector<Move>> CheckersGame::GenerateMoveList(BoardType inboard, int player) {
		int nDir = 1;

		int nJump = 1;
		std::map<Move, std::vector < Move>> sqmoves;
		if (ISBLACK(player)) {
			nDir = -1;
		}
		for (int i = 0; i < ROWS; i++) {
			for (int j = 0; j < COLS; j++) {
				// current board position have the player piece?
				if ((inboard[i][j] & player) == 0)   
					continue;
				std::vector<Move> moves;
				moves = BuildMoves(inboard, player, Move(i, j));
				if (moves.size() != 0)
					sqmoves[Move(i, j)] = moves;
			}
		}
		return sqmoves;
	}

	std::string CheckersGame::DumpLegalMoves(std::map<Move, std::vector<Move>> &moves) {
		std::map<Move, std::vector < Move>>::iterator it;
		std::string str;
		for (it = moves.begin(); it != moves.end(); it++) {
			if ((*it).second.size() == 0)
				continue;
			str += StrFormat("[%d,%d]=", (*it).first.row, (*it).first.col);
			for (size_t i = 0; i < (*it).second.size(); i++) {
				Move & move((*it).second[i]);
				str += StrFormat("%s(%d,%d),", move.bJump ? "J" : "", move.row, move.col);
			}
			str += "\n";
		}
		return str;
	}

	std::string CheckersGame::LegalMove(const BoardType & inboard, int player, int i, int j, Move m) {
		int k = m.row;
		int l = m.col;

		if (i < 0 && ROWS <= i) { // keeping in bounds
			return StrFormat("i is out of bounds\n");
		}
		if (j < 0 && COLS <= j) {
			return StrFormat("j is out of bound");
		}

		if (k < 0 && ROWS <= k) {
			return StrFormat("k is out of bounds");
		}
		if (l < 0 && COLS <= l) {
			return StrFormat("l is out of bounds\n");
		}

		// check player is moving his own piece.
		if ((player == RED && !ISRED(inboard[i][j])) || (player == BLACK && !ISBLACK(inboard[i][j]))) {
			return StrFormat("move your own piece!\n").c_str();
		}

		//make sure they are jumping to a empty loacation
		if (inboard[k][l] != EMPTY) {
			return StrFormat("You must move to a empty location");
		}

		return "";
	}

	BoardType CheckersGame::MakeMove(BoardType inboard, int player, int i, int j, Move m) {
		std::string errmsg;
		std::string typemove("move");
		if (m.bJump)
			typemove = "jump";

		//if(!(errmsg=LegalMove( player, i, j, m)).empty())
		//{
		//	std::cout << errmsg<< std::endl;
		//	return errmsg;
		//}
		int rowsign = SIGN(m.row - i);
		int colsign = SIGN(m.col - j);
		if (abs(i - m.row) == 2) {
			inboard[i + (rowsign * 1)][j + (colsign * 1)] = EMPTY;
		}
		Jump(inboard, i, j, m);
		Move move(m.row, m.col, m.bJump);
		if (m.doublejumps.size() > 0) {
			Move m2 = m.doublejumps[0];
			inboard = MakeMove(inboard, player, m.row, m.col, m2);
		}
		// Save move - only good if no mixmax lookahead
		move.Start(player, i, j);
		allmoves.push_back(move);

		return inboard;
	}

	double CheckersGame::Eval(BoardType &inBoard, int player) // add in player, subtract opponent
	{
		int sum = 0;
		int opponent = (player == BLACK) ? RED : BLACK;
		// For each square on checkerboard evaluate. Add 1 for me, -1 for opponent.
		// If king,  add 5 for me, -5 for opponent
		for (int i = 0; i < ROWS; i++)
		{
			for (int j = 0; j < COLS; j++) {
				int nKing = 1;
				if (ISKING(inBoard[i][j])) nKing = 5;
				if ((inBoard[i][j] & player) > 0) {
					sum = sum + nKing * 1;
				} else if ((inBoard[i][j] & opponent) > 0) {
					sum = sum - nKing * 1;
				}
			}
		}
		// return sum from scanning all checkerboard squares
		return sum;
	}

	bool CheckersGame::RandomMove(std::map<Move, std::vector<Move>> &moves, Move &m1, Move &m2) {
		if (moves.size() == 0)
			return false;

		std::map<Move, std::vector < Move>>::iterator it;

		int n = rand() % moves.size();
		it = moves.begin();
		for (size_t i = 0; i < n; i++) it++;
		int m = rand() % (*it).second.size();
		m1 = (*it).first;
		m2 = (*it).second[m];
		return true;
	}

	double CheckersGame::DoubleJumpEval(Move &m) {
		double val = 0;
		double bestScore = 0;
		for (size_t i = 0; i < m.doublejumps.size(); i++) {
			val = 10;
			// 1 versus 2 double jumps
			val += DoubleJumpEval(m.doublejumps[i]);
			if (val > bestScore)
				bestScore = val;
		}

		return bestScore;
	}

	double CheckersGame::MinMaxEval(int depth, int player, BoardType curBoard, double signFactor) {
		int opponent = (player == BLACK) ? RED : BLACK;
		float posValue = -FLT_MAX; // value of given move - FIXME should be -signFactor *
		if (depth >= MAX_DEPTH)
			return Eval(curBoard, opponent);

		// Generate moves from current board for given player
		std::map<Move, std::vector < Move>> moves = GenerateMoveList(curBoard, player);

		// For each player position on checkerboard
                // treat all moves as positive return negative if signFactor
		std::map<Move, std::vector < Move>>::iterator it;
		for (it = moves.begin(); it != moves.end(); it++) {
			// no moves associated with player checkerboard position
			if ((*it).second.size() == 0)
				continue;
			// Evaluate each move in vector.
			Move from((*it).first);
			for (size_t i = 0; i < (*it).second.size(); i++) {
				Move & to((*it).second[i]);
				BoardType newBoard = MakeMove(curBoard, player, from.row, from.col, to);
				float newValue = /*signFactor *  */ MinMaxEval(depth + 1, opponent, newBoard, -signFactor);
				if (to.bJump)
					newValue += /*signFactor *  */ 10;  // this hopefully skews evaluation towards jumps
				if (newValue > posValue)
					posValue = newValue;
			}
		}

		return signFactor*posValue;
	}

	bool CheckersGame::MinMaxBestMove(std::map<Move, std::vector<Move>> &moves, BoardType curBoard, int player, Move &m1, Move &m2) {
		double bestScore = -LDBL_MAX; // for this move, start with worst possible score.
		int opponent = (player == BLACK) ? RED : BLACK;

		Move bestfrom, bestto;
		std::map<Move, std::vector < Move>>::iterator it;
		std::map<Move, std::vector < Move>> bestmoves;

		// Go through all the moveable piece positions, some don't have legal moves in vector
		for (it = moves.begin(); it != moves.end(); it++) {
			// No legal move?
			if ((*it).second.size() == 0)
				continue;

			// With each legal move at piece position, evaluate move and subsequent moves
			for (size_t i = 0; i < (*it).second.size(); i++) {
				Move & move((*it).second[i]);

				// Make move from current board position to move position
				BoardType newBoard = MakeMove(curBoard, player, (*it).first.row, (*it).first.col, move);
				// Evaluate board from opponent view. Why negative once?
				move.score = MinMaxEval(50, opponent, newBoard, -1.0);

				if (move.bJump)
					move.score += 10;
				// Check if double jump - if so, increase score
				move.score += DoubleJumpEval(move);
				if (move.score > bestScore) {
					m1 = (*it).first;
					m2 = move;
					bestScore = move.score;
					bestmoves.clear();
					bestmoves[(*it).first] = std::vector<Move>();
					bestmoves[(*it).first].push_back(move);
				}
			}
		}

                if(bestScore = -LDBL_MAX)
                    return false;
		return true;
	}

	std::string CheckersGame::TestBoard() {
		return "  +---+---+---+---+---+---+---+---+\n"
			"0 |   | r |   | B |   | r |   | r |\n"
			"  +---+---+---+---+---+---+---+---+\n"
			"1 |   |   |   |   |   |   | r |   |\n"
			"  +---+---+---+---+---+---+---+---+\n"
			"2 |   |   |   |   |   | r |   |   |\n"
			"  +---+---+---+---+---+---+---+---+\n"
			"3 |   |   | r |   |   |   |   |   |\n"
			"  +---+---+---+---+---+---+---+---+\n"
			"4 |   |   |   |   |   | b |   | b |\n"
			"  +---+---+---+---+---+---+---+---+\n"
			"5 |   |   |   |   |   |   |   |   |\n"
			"  +---+---+---+---+---+---+---+---+\n"
			"6 |   | b |   | b |   |   |   | b |\n"
			"  +---+---+---+---+---+---+---+---+\n"
			"7 | b |   |   |   |   |   | b |   |\n"
			"  +---+---+---+---+---+---+---+---+\n"
			"    0   1   2   3   4   5   6   7\n";
	}

};
