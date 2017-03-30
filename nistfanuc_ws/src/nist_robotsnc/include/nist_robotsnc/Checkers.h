

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

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <math.h>
#include <stdarg.h>
#include <string.h>

namespace Checkers {
#define ROWS 8
#define COLS 8

	static const int EMPTY = 1;
	static const int RED = 2;
	static const int BLACK = 4;
	static const int KING = 8;
#define MAX_DEPTH 50

#define ISRED(c) ( (c & Checkers::RED) >0)
#define ISBLACK(c) ((c & Checkers::BLACK) >0)
#define ISEMPTY(c) (c == Checkers::EMPTY)
#define ISKING(c) ( (c & Checkers::KING) >0)
#define SIGN(x) (x < 0) ? -1 : (x > 0)

	inline std::string StrFormat(const char *fmt, ...) {
		va_list argptr;
		va_start(argptr, fmt);
		int m;
		int n = (int) strlen(fmt) + 1028;
		std::string tmp(n, '0');
		while ((m = vsnprintf(&tmp[0], n - 1, fmt, argptr)) < 0) {
			n = n + 1028;
			tmp.resize(n, '0');
		}
		va_end(argptr);
		return tmp.substr(0, m);
	}
	inline void ReplaceAll(std::string& tInput, std::string tFind, std::string tReplace) {
		size_t uPos = 0;
		size_t uFindLen = tFind.length();
		size_t uReplaceLen = tReplace.length();
		if (uFindLen == 0) {
			return;
		}

		for (; (uPos = tInput.find(tFind, uPos)) != std::string::npos;) {
			tInput.replace(uPos, uFindLen, tReplace);
			uPos += uReplaceLen;
		}

	}
	struct Move {
		/*!
		*\brief Move constructor, no player, jump, doublejump and row=col=0.
		*/
		Move() ;

		/*!
		*\brief Move constructor, no player, but row and col and jump set.
		* \param row is row of  checker position
		* \param col is column of  checker position
		* \param bJump flag to determine if jump move.
		*/
		Move(int row, int col, bool bJump = false) ;

		/*!
		*\brief Start move - assign player, and checker row/col position.
		* \param player is red or black player.
		* \param row is row of  checker position
		* \param col is column of  checker position
		*/
		void Start(int player, int row, int col) ;
        Move Diff(Move start) {
            int rdiff = (row - start.row) / 2;
            int cdiff = (col - start.col) / 2;
            return Move(start.row + rdiff, start.col + cdiff);

        }
		friend bool operator<(const Move &left, const Move &other) {
			return ( (left.row * COLS + left.col)< (other.row * COLS + other.col));
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
	//protected:
		int row; /**<  move row */
		int col; /**<  move column */
		int player; /**<  player making move  */
		bool bJump; /**<  true if move a jump */
		bool bDoubleJumps; /**<  true if move has double jump */
		std::vector<Move> doublejumps;  /**<  list of double jumps */
		int srow;  /**<  starting row */
		int scol; /**<  starting column */
		double score; /**<  maximum score attached to this move*/

	};

	struct BoardType : std::vector<std::vector<int> > {
		/*!
		*\brief Constructor which setups up empty board.
		*/
		BoardType() ;
	};

	class CheckersGame {
		BoardType aboard; /**<  master board */
		std::vector<Move> allmoves; /**< all moves in game so far */
	public:
		/*!
		*\brief Reference to the current "master" game board.
		*/
		BoardType &Board() ;

		/*!
		*\brief Constructor which setups up new game.
		*/
		CheckersGame() ;

		/*!
		*\brief Setup checkers board with pieces laid out for new game.
		*/
		void NewGame();
		/*!
		*\brief Read input file for list of moves to store.
		* \param filename of input file.
		* \param vector of moves to store into.
		*/
		void Restore(std::string filename, std::vector<Move> & moves) ;
		/*!
		*\brief Save current board to file.
		* \param filename  of file to store board.
		*/
		void Save(std::string filename) ;
		/*!
		*\brief Text dump of move - color, and from/to board positions.
		* \param str stream to dump text.
		* \param move to dump.
		*/
		void Dump(std::ostream &str, const Move move);

		/*!
		*\brief Text dump of moves - color, and from/to board positions.
		* \param str stream to dump text.
		* \param vector of moves to dump.
		*/
		void Dump(std::ostream &str, const std::vector<Move> &moves);
		/*!
		*\brief Read input file for list of moves to store.
		* \param player (red or black piece).
		* \param m is move or destination (if at end of opponent board, then king)
		* \return bool true if king, false is not.
		*/
		bool IsKing(int player, Move m) ;
		/*!
		*\brief Is row a legal value?
		* \param n for row number.
		* \return bool true if on board, false if outside board rows.
		*/
		bool LegalRow(int n) ;

		/*!
		*\brief Transform checkerboard symbol into char symbol.
		* For example, red piece returns 'r'. King red piece returns 'R';
		* \param i for piece type.
		* \return char representing piece, blank if empty.
		*/
		char value2symbol(int i) ;
		/*!
		*\brief Print out board as fancy text to string .
		* \param inboard is the board to print.
		* \return string containing text print out of board.
		*/
		std::string printDisplayFancy(BoardType inboard) ;
		/*!
		*\brief Transform checkerboard char symbol into checker int.
		* For example, 'r' is red, 'R' is red and King red piece 
		* \param c for char type.
		* \return int representing piece, 1 if empty, 0 if undefined.
		*/
		int symbol2value(char c) ;

		/*!
		*\brief Take input stream and deserialize into board.
		* \param s_in for input stream.
		* \param board_out reference to board to store deserialization.
		*/
		void Deserialize(std::istream& s_in, BoardType &board_out);

		/*!
		*\brief King any board pieces that are kings given board.
		* \param inboard checkerboard with pieces.
		*/
		void KingMe(BoardType & inboard);

		/*!
		*\brief Update board by exchanging checker position, check if kinged.
		* Fixme: why is this just a swap?
		* \param inboard checkerboard with pieces.
		* \param i is row of original checker position
		* \param j is col of original checker position
		* \param move contains destination checker position.
		*/
		void Jump(BoardType & inboard, int i, int j, Move move);

		/*!
		*\brief Check given player has won given checker board?
		* Fixme: if opponent has no move, its a win. Not checkers.
		* \param inboard checkerboard with pieces.
		* \param player is red or black player.
		* \return bool true if win false if not win.
		*/
		bool IsWin(BoardType & inboard, int player) ;

		/*!
		*\brief Build a list of moves given board, player, current position, and whether a continuation of existing move (jump).
		* \param inboard checkerboard with pieces.
		* \param player is red or black player.
		* \param move contains row/col of from checker position.
		* \param bJumpOnly is true if continuation of previous jump and must be a jump not just a move to empty space.
		* \return vector of legal moves.
		*/
		std::vector<Move> BuildMoves(BoardType inboard, int player, Move from, bool bJumpOnly = false) ;

		/*!
		*\brief Build a list of all possible moves given board, player.
		* \param inboard checkerboard with pieces.
		* \param player is red or black player.
		* \return map of current position containing vector of legal moves.
		*/
		std::map<Move, std::vector<Move>> GenerateMoveList(BoardType inboard, int player) ;

		/*!
		*\brief Dumps a list of all possible moves given board, player.
		* \param moves is map of current position containing vector of legal moves.
		* \return string describing legal moves.
		*/
		std::string DumpLegalMoves(std::map<Move, std::vector<Move>> &moves) ;

		/*!
		*\brief Checks to see whether checker move is valid.
		* \param inboard checkerboard with pieces.
		* \param player is red or black player.
		* \param i is row of original checker position
		* \param j is col of original checker position
		* \param move contains destination checker position.
		* \return string describing error, or empty if valid move.
		*/
		std::string LegalMove(const BoardType & inboard, int player, int i, int j, Move m);

		/*!
		*\brief Moves checker piece and updates board.
		* \param inboard checkerboard with pieces.
		* \param player is red or black player.
		* \param i is row of original checker position
		* \param j is col of original checker position
		* \param move contains destination checker position.
		* \return updated board after move.
		*/
		BoardType MakeMove(BoardType inboard, int player, int i, int j, Move m);

		/*!
		*\brief Evaluates board position for given player.
		* \param inboard checkerboard with pieces.
		* \param player is red or black player.
		* \return value of new board for player.
		*/
		double Eval(BoardType &inBoard, int player) ;

		/*!
		*\brief Selects random move from large list of potential moves.
		* \params map of current position containing vector of legal moves.
		* \param m1 reference to original move piece position.
		* \param m2 reference to destination move piece position.
		* \return true if random move found.
		*/
		bool RandomMove(std::map<Move, std::vector<Move>> &moves, Move &m1, Move &m2) ;

		/*!
		*\brief Evaluates higher if move is double jump. Will recursively evalute double jumps.
		* \param move contains destination checker position.
		* \return value of new board for player.
		*/
		double DoubleJumpEval(Move &m) ;

		/*!
		*\brief Minmax evaluation of checker play. 
		* \param depth of searching for best move.
		* \param curBoard checkerboard with pieces.
		* \param player is red or black player.
		* \param signFactor +1 for player, -1 for opponent.
		* \return value of "new" board for player.
		*/
		double MinMaxEval(int depth, int player, BoardType curBoard, double signFactor) ;

		/*!
		*\brief Minmax best move evaluation of all potential checker moves. 
		* \params moves of current position containing vector of legal moves.
		* \param curBoard checkerboard with pieces.
		* \param player is red or black player.
		* \param m1 reference to store original move piece position.
		* \param m2 reference to store destination move piece position.
		* \return true if "best" move found.
		*/
		bool MinMaxBestMove(std::map<Move, std::vector<Move>> &moves, BoardType curBoard, int player, Move &m1, Move &m2) ;

		/*!
		*\brief String containing test board for evaluation.. 
		* \return string of printy print board.
		*/
		std::string TestBoard();
	};
};
