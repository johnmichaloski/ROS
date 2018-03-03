

//http://wallacecompany.com/tmp/ttt/truetype-tracer-4.0-1-kw/ttt.c 
// https://chromium.googlesource.com/chromium/src/third_party/freetype2/+/VER-2-BETA2/include/fterrors.h

// sudo apt-get install libfreetype6
// http://freetype.sourceforge.net/index2.html
// https://musescore.org/en/node/60016
// sudo apt-get install libfreetype6-dev

// Cmake
// http://stackoverflow.com/questions/23888274/linking-freetype-with-cmake
/**
Q: Where is #include <ft2build.h>? PROBLEM SOLVED!

A: I have had to set FREETYPE in the environment variables as BOOST too and everything is now working properly! Here is how is mine:

export BOOST=/var/lib/jenkins/workspace/boost_1_59_0
export FREETYPE=/var/lib/jenkins/workspace/freetype-2.6.3
 */

/*
This file is part of TTT.  TTT is free software; you can redistribute
it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.  TTT is distributed in
the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with TTT; if not, write to the Free Software Foundation, Inc.,
59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

Copyright © 2004, 2005, 2006, 2007, 2008 
Chris Radek <chris@timeguy.com>

This source gets compiled into 2 executables. The first, ttt generates
G-code from a true type font and a text string. The second executable
is ttt_dxf which generates an autocad dxf file from a true type font
and a text string.

example commandline:
ttt "Hello World" > hw.ngc
ttt_dxf "Hello World" > hw.dxf

MANY THANKS go to Lawrence Glaister <ve7it@shaw.ca> for updates based
on the new FreeType FT_Outline API and .pfb support!
 */
#include "ttt.h"
#include <boost/bind.hpp>

#include <stdio.h>
#include <ctype.h>
#include <getopt.h>
#include <locale.h>
#include <math.h>
#if 0
#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H

#undef __FTERRORS_H__
#define FT_ERRORDEF( e, v, s ) { e, s },
#define FT_ERROR_START_LIST {
#define FT_ERROR_END_LIST { 0, 0 } };

// Seee why this is like it is: https://www.freetype.org/freetype2/docs/reference/ft2-error_enumerations.html

const struct ftError {
    int err_code;
    const char* err_msg;
} ft_errors[] =
#include FT_ERRORS_H
#endif
        static letters lo;

// define the number of linear segments we use to approximate beziers
// in the gcode and the number of polyline control points for dxf code.
int csteps = 10;
// define the subdivision of curves into arcs: approximate curve length
// in font coordinates to get one arc pair (minimum of two arc pairs
// per curve)
double dsteps = 200;

#define NEQ(a,b) ((a).x != (b).x || (a).y != (b).y)
#define SQ(a) ((a)*(a))
#define CUBE(a) ((a)*(a)*(a))

typedef struct {
    double x, y;
} P;

static double max(double a, double b) {
    if (a < b) return b;
    else return a;
}

static P ft2p(const FT_Vector *v) {
    P r = {v->x, v->y};
    return r;
}

static double dot(P a, P b) {
    return a.x * b.x + a.y * b.y;
}

static double mag(P a) {
    return sqrt(dot(a, a));
}

static P scale(P a, double b) {
    P r = {a.x*b, a.y * b};
    return r;
}

static P add(P a, P b) {
    P r = {a.x + b.x, a.y + b.y};
    return r;
}

static P add3(P a, P b, P c) {
    P r = {a.x + b.x + c.x, a.y + b.y + c.y};
    return r;
}

static P add4(P a, P b, P c, P d) {
    P r = {a.x + b.x + c.x + d.x, a.y + b.y + c.y + d.y};
    return r;
}

static P sub(P a, P b) {
    P r = {a.x - b.x, a.y - b.y};
    return r;
}

static P unit(P a) {
    double m = mag(a);
    if (m) {
        P r = {a.x / m, a.y / m};
        return r;
    } else {
        P r = {0, 0};
        return r;
    }
}

void line(P p) {
#ifdef DXF
    printf("  0\nVERTEX\n  8\n0\n 10\n%.4f\n 20\n%.4f\n 30\n0.0\n",
            p.x, p.y);
#else
    printf("G01 X [%.4f*#3+#5] Y [%.4f*#3+#6] (lineto)\n", p.x, p.y);
#endif
}

void arc(P p1, P p2, P d) {
    d = unit(d);
    P p = sub(p2, p1);
    double den = 2 * (p.y * d.x - p.x * d.y);

    if (fabs(den) < 1e-10) {
        printf("G1 X[%.4f*#3+#5] Y[%.4f*#3+#6]\n", p2.x, p2.y);
        return;
    }

    double r = -dot(p, p) / den;

    double i = d.y*r;
    double j = -d.x*r;

    P c = {p1.x + i, p1.y + j};
    double st = atan2(p1.y - c.y, p1.x - c.x);
    double en = atan2(p2.y - c.y, p2.x - c.x);

    if (r < 0)
        while (en <= st) en += 2 * M_PI;
    else
        while (en >= st) en -= 2 * M_PI;

#ifdef DXF
    double bulge = tan(fabs(en - st) / 4);
    if (r > 0) bulge = -bulge;
    printf("  42\n%.4f\n  70\n1\n"
            "  0\nVERTEX\n  8\n0\n  10\n%.4f\n  20\n%.4f\n  30\n0.0\n",
            bulge, p2.x, p2.y);
#else
    double gr = (en - st) < M_PI ? fabs(r) : -fabs(r);
    if (r < 0)
        printf("G3 X[%.4f*#3+#5] Y[%.4f*#3+#6] R[%.4f*#3]\n",
            p2.x, p2.y, gr);
    else
        printf("G2 X[%.4f*#3+#5] Y[%.4f*#3+#6] R[%.4f*#3]\n",
            p2.x, p2.y, gr);
#endif
}

void biarc(P p0, P ts, P p4, P te, double r) {
    ts = unit(ts);
    te = unit(te);

    P v = sub(p0, p4);

    double c = dot(v, v);
    double b = 2 * dot(v, add(scale(ts, r), te));
    double a = 2 * r * (dot(ts, te) - 1);

    double disc = b * b - 4 * a*c;

    if (a == 0 || disc < 0) {
        line(p4);
        return;
    }

    double disq = sqrt(disc);
    double beta1 = (-b - disq) / 2 / a;
    double beta2 = (-b + disq) / 2 / a;
    double beta = max(beta1, beta2);

    if (beta <= 0) {
        line(p4);
        return;
    }

    double alpha = beta*r;
    double ab = alpha + beta;
    P p1 = add(p0, scale(ts, alpha));
    P p3 = add(p4, scale(te, -beta));
    P p2 = add(scale(p1, beta / ab), scale(p3, alpha / ab));
    P tm = sub(p3, p2);

    arc(p0, p2, ts);
    arc(p2, p4, tm);
}

// this is the default font used if not specified on commandline
#define TTFONT "/usr/share/fonts/truetype/freefont/FreeSerifBoldItalic.ttf"

#ifdef DXF
static int bootstrap = 1;
#endif
static FT_Vector last_point;
static int debug = 0;

struct extents {
    long int minx;
    long int maxx;
    long int miny;
    long int maxy;
} glyph_extents, line_extents;

static FT_Vector advance;

// routine to print out hopefully-useful error messages

void handle_ft_error(char *where, int f, int x) {
    const struct ftError *e = &ft_errors[0];
    for (; e->err_msg && e->err_code != f; e++);
    if (e->err_msg) {
        fprintf(stderr, "Fatal error in %s: %s (%d) at line:%d\n", where, e->err_msg, f, x);
    } else {
        fprintf(stderr, "Fatal error in %s: %d at line:%d\n", where, f, x);
    }
    exit(x);
}

// resets extents struct members min and max to +big and -big respectively
// next call to extents_add_point(point) will set them to that point

void extents_reset(struct extents *e) {
    e->maxx = -2000000000;
    e->maxy = -2000000000;
    e->minx = 2000000000;
    e->miny = 2000000000;
}

// updates extents struct to include the point

void extents_add_point(struct extents *e, const FT_Vector *point) {
    if (point->x > e->maxx) e->maxx = point->x;
    if (point->y > e->maxy) e->maxy = point->y;
    if (point->x < e->minx) e->minx = point->x;
    if (point->y < e->miny) e->miny = point->y;
}


// updates extents struct e1 to include all of e2

void extents_add_extents(struct extents *e1, struct extents *e2) {
    if (e2->maxx > e1->maxx) e1->maxx = e2->maxx;
    if (e2->maxy > e1->maxy) e1->maxy = e2->maxy;
    if (e2->minx < e1->minx) e1->minx = e2->minx;
    if (e2->miny < e1->miny) e1->miny = e2->miny;
}

int letters::bezier_move_to(const FT_Vector* to, void* user) {
    char *blockdelete = user ? "/" : "";
    last_point = *to;
    extents_add_point(&glyph_extents, to);
    return 0;

}

// plot with pen down to a new endpoint drawing a line segment 
// Linear Bézier curves (a line)
// B(t)=(1-t)P0 + tP1,	t in [0,1]. 

int letters::bezier_line_to(const FT_Vector* to, void* user) {
    cspline cs;
    cs.type = 'L';
    cs.pts[0] = vec3(last_point.x, last_point.y);
    cs.pts[1] = vec3(to->x, to->y);
    cs.pts[2] = vec3(last_point.x, last_point.y);
    cs.pts[3] = vec3(to->x, to->y);
    lo.bsplines.back().push_back(cs);
    last_point = *to;
    extents_add_point(&glyph_extents, to);
    return 0;
}
// draw a second order curve from current pos to 'to' using control
// Quadratic Bézier curves (a curve)
// B(t) = (1 - t)^2A + 2t(1 - t)B + t^2C,  t in [0,1]. 

int letters::bezier_conic_to(const FT_Vector* control, const FT_Vector* to, void* user) {
    int t;
    double x, y;
    double len = 0;

    vec3 point0(last_point);
    vec3 point1(*control);
    vec3 point2(*to);

    cspline cs;
    cs.type = 'B';

    cs.pts[0] = point0;
    cs.pts[1].x() = point0.x() + 2. * (point1.x() - point0.x()) / 3.;
    cs.pts[1].y() = point0.y() + 2. * (point1.y() - point0.y()) / 3.;

    cs.pts[2].x() = point1.x() + 1. * (point2.x() - point1.x()) / 3.;
    cs.pts[2].y() = point1.y() + 1. * (point2.y() - point1.y()) / 3.;
    cs.pts[3] = point2;
    lo.bsplines.back().push_back(cs);

    last_point = *to;
    return 0;

}

// draw a cubic spline from current pos to 'to' using control1,2
// Cubic Bézier curves ( a compound curve )
// B(t)=A(1-t)^3 + 3Bt(1-t)^2 + 3Ct^2(1-t) + Dt^3 , t in [0,1]. 

int letters::bezier_cubic_to(const FT_Vector* control1, const FT_Vector* control2,
        const FT_Vector *to, void* user) {
    int t;
    double x, y;
    FT_Vector point0 = last_point;
    double len = 0;

    cspline cs;
    cs.type = 'B';
    cs.pts[0] = vec3(point0.x, point0.y);
    cs.pts[1] = vec3(control1->x, control1->y);
    cs.pts[2] = vec3(control2->x, control2->y);
    cs.pts[3] = vec3(to->x, to->y);
    lo.bsplines.back().push_back(cs);
    return 0;
}

void letters::bezier_draw_bitmap(FT_Bitmap *b, FT_Int x, FT_Int y, int linescale) {
    assert(0);
}
// move with 'pen up' to a new position and then put 'pen down' 

int my_move_to(const FT_Vector* to, void* user) {
    char *blockdelete = user ? "/" : "";
#ifdef DXF
    /* every move but the first one means we are starting a new polyline */
    /* make sure we terminate previous polyline with a seqend */
    if (bootstrap == 0) printf("  0\nSEQEND\n");
    bootstrap = 0;
    printf("  0\nPOLYLINE\n  8\n0\n 66\n     1\n 10\n0.0\n 20\n0.0\n 30\n0.0\n");
    printf("  0\nVERTEX\n  8\n0\n 10\n%ld.000\n 20\n%ld.000\n 30\n0.0\n",
            to->x, to->y);
#else
    if (debug) printf("(moveto %ld,%ld)\n", to->x, to->y);
    printf("%sG00 Z #1\n", blockdelete);
    printf("%sG00 X [%ld*#3+#5] Y [%ld*#3+#6] (moveto)\n", blockdelete, to->x, to->y);
    printf("%sG01 Z [0-#2] F#4\n", blockdelete);
#endif
    last_point = *to;
    extents_add_point(&glyph_extents, to);

    return 0;
}

static void my_draw_bitmap(FT_Bitmap *b, FT_Int x, FT_Int y, int linescale) {
    FT_Int i, j;
    static int oldbit;
    FT_Vector oldv = {99999, 0};
    FT_Vector vbuf[100]; //freetype says no more than 32 ever?
    int spans = 0;
    int pitch = abs(b->pitch);
    static int odd = 0;
    for (j = 0; j < b->rows; j++) {
        FT_Vector v;
        oldbit = 0;
        spans = 0;
        for (i = 0; i < pitch; i++) {
            unsigned char byte = b->buffer[j * pitch + i], mask, bits;
            for (bits = 0, mask = 0x80; mask; bits++, mask >>= 1) {
                unsigned char bit = byte & mask;
                v.x = i * 8 + bits + x;
                v.y = (y - j)*64 * 64 / linescale - 64 * 32 / linescale;
                if (!oldbit && bit) {
                    v.x += 8;
                    oldv = v;
                    vbuf[spans++] = v;
                }
                if (oldbit && !bit) {
                    v.x -= 8;
                    if (oldv.x < v.x) {
                        vbuf[spans++] = v;
                    } else spans--;
                }
                oldbit = bit;
            }
        }
        if (oldbit) {
            v.x -= 8;
            vbuf[spans++] = v;
        }
        odd = !odd;
        spans /= 2;
        if (odd) {
            for (int i = spans - 1; i >= 0; i--) {
                // my_move_to(vbuf + 1 + (i * 2), (void*) 1);
                // my_line_to(vbuf + (i * 2), (void*) 1);
            }
        } else {
            for (int i = 0; i < spans; i++) {
                // my_move_to(vbuf + (i * 2), (void*) 1);
                // my_line_to(vbuf + 1 + (i * 2), (void*) 1);
            }
        }
    }
}

// lookup glyph and extract all the shapes required to draw the outline

static long int render_char(FT_Face face, wchar_t c, long int offset, int linescale) {
    int error;
    int glyph_index;
    FT_Outline outline;
    FT_Outline_Funcs func_interface;

    error = FT_Set_Pixel_Sizes(face, 4096, linescale ? linescale : 64);
    if (error) handle_ft_error("FT_Set_Pixel_Sizes", error, __LINE__);

    /* lookup glyph */
    glyph_index = FT_Get_Char_Index(face, (FT_ULong) c);
    if (!glyph_index) handle_ft_error("FT_Get_Char_Index", 0, __LINE__);

    /* load glyph */
    error = FT_Load_Glyph(face, glyph_index, FT_LOAD_NO_BITMAP |
            FT_LOAD_NO_HINTING);
    if (error) handle_ft_error("FT_Load_Glyph", error, __LINE__);
    error = FT_Render_Glyph(face->glyph, FT_RENDER_MODE_MONO);
    if (error) handle_ft_error("FT_Render_Glyph", error, __LINE__);

    if (linescale > 0)
        my_draw_bitmap(&face->glyph->bitmap,
            face->glyph->bitmap_left + offset,
            face->glyph->bitmap_top,
            linescale);


    error = FT_Set_Pixel_Sizes(face, 0, 64);
    if (error) handle_ft_error("FT_Set_Pixel_Sizes", error, __LINE__);
    error = FT_Load_Glyph(face, glyph_index, FT_LOAD_NO_BITMAP |
            FT_LOAD_NO_HINTING);
    if (error) handle_ft_error("FT_Load_Glyph", error, __LINE__);

    /* shortcut to the outline for our desired character */
    outline = face->glyph->outline;

    /* set up entries in the interface used by FT_Outline_Decompose() */
    func_interface.shift = 0;
    func_interface.delta = 0;

#if 0
    func_interface.move_to = my_move_to;
    func_interface.line_to = my_line_to;
    func_interface.conic_to = my_conic_to;
    func_interface.cubic_to = my_cubic_to;
#endif
    lo.bsplines.push_back(letter());
    func_interface.move_to = &letters::bezier_move_to;
    func_interface.line_to = &letters::bezier_line_to;
    func_interface.conic_to = &letters::bezier_conic_to;
    func_interface.cubic_to = &letters::bezier_cubic_to; // boost::bind(&letters::bezier_cubic_to, &lo, _1 , _2,_3,_4);

    /* offset the outline to the correct position in x */
    FT_Outline_Translate(&outline, offset, 0L);

    /* plot the current character */
    error = FT_Outline_Decompose(&outline, &func_interface, NULL);
    if (error) handle_ft_error("FT_Outline_Decompose", error, __LINE__);

    /* save advance in a global */
    advance.x = face->glyph->advance.x;
    advance.y = face->glyph->advance.y;

    /* offset will get bumped up by the x size of the char just plotted */
    return face->glyph->advance.x;
}

std::vector<letter> letters::makescript(std::string fontname, std::string text, unsigned int fontsize) {
    FT_Library library;
    FT_Face face;
    int error;
    long int offset;
    char *s = "Hello world.";
    int i, l = strlen(s);
    char *ttfont = TTFONT;
    double scale = 0.0003;
    int linescale = 0;
    double xoff = 0;
    double yoff = 0;

    error = FT_Init_FreeType(&library);
    if (error) handle_ft_error("FT_Init_FreeType", error, __LINE__);

    error = FT_New_Face(library, ttfont, 0, &face);
    if (error) handle_ft_error("FT_New_Face", error, __LINE__);
    error = FT_Set_Pixel_Sizes(face, 0, fontsize);
    if (error) handle_ft_error("FT_Set_Pixel_Sizes", error, __LINE__);

    if (text.size() > 0) {
        s = text.c_str();
        l = text.size();
    }
    printf("(font: %s)\n", ttfont);
    printf("(text: ");
    for (i = 0; i < l; i++)
        if (isalnum(s[i]))
            printf("%c", s[i]);
        else
            printf("*");
    printf(")\n");
    printf("#3=%f  (XY Scale)\n", scale);
    printf("#5=%f  (X offset)\n", xoff);
    printf("#6=%f  (Y offset)\n", yoff);

    extents_reset(&line_extents);
    offset = 0;

    /* loop through rendering all the chars in the string */
    while (*s) {
        wchar_t wc;
        int r = mbtowc(&wc, s, l);
        if (r == -1) {
            s++;
            continue;
        }

        /* comment gcode at start of each letter		   */
        /* to keep out of trouble, only print numbers/letters, */
        /* The rest get hex representation			   */
        if (isalnum(*s))
            printf("(start of symbol %c)\n", wc);
        else

            /* comment with offset info */
            printf("(starting X offset: %ld)\n", offset);

        extents_reset(&glyph_extents);
        offset += render_char(face, wc, offset, linescale);
        extents_add_extents(&line_extents, &glyph_extents);
        s += r;
        l -= r;

        lo.bsplines.back().minx = glyph_extents.minx;
        lo.bsplines.back().miny = glyph_extents.miny;
        lo.bsplines.back().maxx = glyph_extents.maxx;
        lo.bsplines.back().maxy = glyph_extents.maxy;

        /* comment with the extents of each letter		 */
        if (glyph_extents.maxx > glyph_extents.minx) {
            printf("(symbol extents: X = %ld to %ld, Y = %ld to %ld)\n",
                    glyph_extents.minx, glyph_extents.maxx, glyph_extents.miny, glyph_extents.maxy);
        }
        printf("(symbol advance: X = %ld, Y = %ld)\n", advance.x, advance.y);

        std::string outline = lo.bsplines.back().plot_letter();
        std::cout << outline;
        //Globals.WriteFile("/home/isd/michalos/src/ROS/spline.txt", outline);
    }
    std::string outline;
    for (size_t n = 0; n < lo.bsplines.size(); n++) {

        outline += lo.bsplines[n].plot_letter();

    }
    Globals.WriteFile("/home/isd/michalos/src/ROS/spline.txt", outline);
    /* write out the post amble stuff */

    printf("(final X offset: %ld)\n", offset);
    if (line_extents.maxx > line_extents.minx) {
        printf("(overall extents: X = %ld to %ld, Y = %ld to %ld)\n",
                line_extents.minx, line_extents.maxx, line_extents.miny, line_extents.maxy);
    }

    extentminx = line_extents.minx;
    extentminy = line_extents.miny;
    extentmaxx = line_extents.maxx;
    extentmaxy = line_extents.maxy;
    bsplines = lo.bsplines;
    return lo.bsplines;
}

std::vector<vec3> letter::letter_pts(double inc) {
    std::vector<vec3> pts;
    maxx = FLT_MIN;
    maxy = FLT_MIN;
    minx = FLT_MAX;
    miny = FLT_MAX;
    for (size_t i = 0; i < size(); i++) {
        for (double k = 0; k <= 1.0; k += inc) {
            vec3 v = this->at(i).compute(k);
            if (v.x() < minx) minx = v.x();
            if (v.y() < miny) miny = v.y();
            if (v.x() > maxx) maxx = v.x();
            if (v.y() > maxy) maxy = v.y();
            pts.push_back(v);
        }
    }
    return pts;
}

std::vector<vec3> letter::letter_gap_pts(double gap) {
    std::vector<vec3> pts;
    maxx = DBL_MIN;
    maxy = DBL_MIN;
    minx = DBL_MAX;
    miny = DBL_MAX;
    for (size_t j = 0; j< this->size(); j++) {
        std::vector<vec3> bpts = this->at(j).buildpts(this->at(j), gap);
        for (size_t k = 0; k < bpts.size(); k++) {
            vec3 v = bpts[k];
            if (v.x() < minx) minx = v.x();
            if (v.y() < miny) miny = v.y();
            if (v.x() > maxx) maxx = v.x();
            if (v.y() > maxy) maxy = v.y();
            pts.push_back(v);
        }
    }
    return pts;
}

std::string letter::dump_cubic_splines() {
    std::string str;
    for (size_t i = 0; i < size(); i++) {
        cspline & s(this->at(i));
        str += Globals.StrFormat("%d= %c (%4.2f %4.2f)  (%4.2f %4.2f) (%4.2f %4.2f) (%4.2f %4.2f)\n", i, s.type,
                s.pts[0].x(), s.pts[0].y(), s.pts[1].x(), s.pts[1].y(),
                s.pts[2].x(), s.pts[2].y(), s.pts[3].x(), s.pts[3].y());
    }
    return str;
}

std::string letter::plot_letter() {
    letter & s(*this);
    std::string str;
    for (size_t i = 0; i < s.size(); i++) {
        for (double k = 0; k <= 1.0; k += .1) {
            vec3 v = s[i].compute(k);
            str += Globals.StrFormat("%4.2f %4.2f\n", v.x(), v.y());
        }
    }
    return str;
}

void letter::computeMaxMin() {
    minx = DBL_MAX;
    miny = DBL_MAX;
    maxx = DBL_MIN;
    maxy = DBL_MIN;
    for (size_t i = 0; i < size(); i++)
        this->at(i).maxmin(minx, miny, maxx, maxy);
}

static bool cmp(vec3 a, vec3 b) {
    return a.x() < b.x();
}


struct VecCmp {
    VecCmp( vec3 v ) : a(v) { }
    bool operator() (const vec3 &b)
        { return (a-b).norm() < .01; }
    vec3 a;
};

void letter::horizontal_fill(double gap) {
    letter & l(*this);
    //std::string str = l.dump_cubic_splines();
    //Globals.WriteFile(Globals.StrFormat("%s/cs.txt", Globals.ExeDirectory.c_str()), str);
    std::stringstream str;
    for (double t = miny + 1.0; t < maxy; t += 1.) {
        str << "t= " << t << "\n";
        str << "Line" << vec3(0.0, t).dump() << "==" << vec3(maxx, t).dump() << "\n";
        std::cout << "Line" << vec3(0.0, t).dump() << "==" << vec3(maxx, t).dump() << "\n";
        for (size_t i = 0; i < l.size(); i++) {
            // std::cout << "CSpline" << l[i].Dump();
            vec3 a = vec3(0.0, t);
            vec3 b = vec3(maxx, t);

            if (l[i].type == 'L') {
                double x, y;
                if (linesIntersect(l[i].pts[0], l[i].pts[3], a, b, x, y)) {
                    str << "On bezier line " << i << " t=" << t  << " " << vec3(x, y).dump() << "\n";
                    if (std::find_if(moves[t].begin(), moves[t].end(), VecCmp(vec3(x, y)))!= moves[t].end()) {
                        str << "DUPLICATE\n";
                        std::cout << "DUPLICATE\n";
                        continue;
                    }
                    moves[t].push_back(vec3(x, y));
                }
            } else {
                if(!l[i].IntersectBB(a,b))
                    continue;
                std::vector<vec3> pts= l[i].findintersection(a, b, l[i], gap);
                if (pts.size() > 0) {
                    str << "On bezier curve i=" << i << "t=" << t << " " << pts[0].dump() << "\n";
                    if (std::find_if(moves[t].begin(), moves[t].end(), VecCmp(pts[0]))!= moves[t].end()) {
                        str << "DUPLICATE\n";
                        std::cout << "DUPLICATE\n";
                        continue;
                    }
                    moves[t].push_back(pts[0]);
                }

            }
        }
        sort(moves[t].begin(), moves[t].end(), cmp);
    }
    std::string s = str.str();
    Globals.WriteFile(Globals.StrFormat("%s/fillit.txt", Globals.ExeDirectory.c_str()), s);

}

std::string letter::dump_moves() {
    std::string tmp;
    std::map<double, std::vector < vec3>>::iterator it = moves.begin();
    for (; it != moves.end(); it++) {
        tmp += Globals.StrFormat("At %4.2f\n", (*it).first);
        std::vector<vec3> pts = (*it).second;
        for (size_t i = 0; i < pts.size(); i++)
            tmp += Globals.StrFormat("  %4.2f  %4.2f\n", pts[i].x(), pts[i].y());
    }
    return tmp;
}

/**
Converting FUnits to pixels
 Values in the em square are converted to values in the pixel coordinate system by multiplying them by a scale. This scale is:
    pointSize * resolution / ( 72 points per inch * units_per_em )
 *  where pointSize is the size at which the glyph is to be displayed, and 
 * resolution is the resolution of the output device. 
 * The 72 in the denominator reflects the number of points per inch.
 * 
 * For example, assume that a glyph feature is 550 FUnits in length 
 * on a 72 dpi screen at 18 point. 
 * There are 2048 units per em in true type, 1000 units per em in postscript type 1 font. 
 * The following calculation reveals that the feature is 4.83 pixels long.
 *  550 * 18 * 72 / ( 72 * 2048 ) = 4.83
 * * BELOW IS WRONG BUT IDEA GETTING THERE
 */
double letter::funits_to_inch(double maxd, double point_size, double units_per_em, double dpi) {
    return (maxd * point_size) / (dpi * units_per_em);
}

double letter::funits_to_meter(double maxd, double point_size, double units_per_em, double dpi) {
    return (maxd * point_size) / (dpi * units_per_em * 25.4 * 1000.0);
}