
#include "confetti.h"
CMath Math;

#if 0


struct Piece {
protected:
    int type;
    double d;
    double dx;
    double dy;
    double dz;
    size_t tx;
    size_t ty;
    double m;
    size_t x;
    size_t y;
    

public:

    std::vector<std::vector<double> > values;
    double value;
    std_msgs::ColorRGBA color;
    Piece(size_t x, size_t y) {
        this->x=x; this->y=y;
        type = Math.floor(Math.random() * 5);
        d = .1;
        dx = 0;
        dy = 0;
        dz = 0;
       tx = x;
        ty = y;
        m = 0;
    }

    void draw() {
        size_t i = Math.floor(Math.random() * Piece.values.size());
        color.r = values[i][0];
        color.g = values[i][1];
        color.b = values[i][2];
        color.a = (100.0 - m) / 100;
    }

    void launch(size_t x, size_t y) {

        double a = Math.random() * Math.PI * 2;

        m = 0.0;
        tx = x;
        ty = y;
        double s;

        switch (type) {

            case 1:

                s = Math.random() * 2;
                break;

            case 2:

                s = 2;
                break;

            case 3:

                s = (Math.PI * 2) - a - Math.random();
                break;

            case 4:

                s = a - Math.random();
                break;

            default:
                s = Math.random() * 2;

                if (Math.random() > .6) {
                    s = 1.5;
                }
        }

        dx = (s + 4) * Math.sin(a);
        dy = (s + 4) * Math.cos(a) - 2;

        draw();

    }

    function move() {

        dy += d;
        tx += dx;
        ty += dy;
        m++;

        draw();
    }
    // m is the # moves
    static bool relinquish() { return m==100; }
};

Piece.values = [
        [59, 4, 10],
        [122, 27, 35],
        [228, 174, 92],
        [248, 229, 182],
        [9, 10, 9],
        [47, 45, 47]
        ];
#endif

struct Pieces {
    std::vector<Piece> pieces;
    size_t maxx, maxy;

    Pieces(size_t x, size_t y) : maxx(x), maxy(y) {
    }

    void advance(size_t new_pieces) {

        var index;
        Piece piece;

        for (size_t index = pieces.size() - 1; index >= 0; index--) {

            piece = pieces[index];
            piece.move();

            if (piece.relinquish()) {
                piece.launch();
                new_pieces -= 1;
            }

        }

        while (new_pieces > 0) {
            pieces.push(new Piece(context, x, y));
            new_pieces -= 1;
        }

    }

    void update(time) {

        size_t new_pieces = (Math.random() * 2) | 0;
        // FIXME: erase or move old pieces
        advance(new_pieces);
        // FIXME: draw new pieces

    }

};
