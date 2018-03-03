
#pragma once

#include <math.h>
#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H
#include FT_GLYPH_H

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

#include <cfloat>

#include <vector>
#include <string>
#include <assert.h>     /* assert */
#include "Globals.h"
#include "Bezier.h"

        using namespace bezier;

struct letter : public std::vector<cspline> {
    std::vector<vec3> letter_pts(double inc = .1);
    std::vector<vec3> letter_gap_pts(double gap = 5.);

    range_result GetRange() {
        return range_result(minx, miny, maxx, maxy);
    }
    void horizontal_fill(double gap = 0.1);
    void computeMaxMin();
    std::string dump_moves();

    std::string dump_cubic_splines();
    std::string plot_letter();
    double funits_to_inch(double maxd, double point_size, double units_per_em, double dpi = 72.0);
    double funits_to_meter(double maxd, double point_size, double units_per_em, double dpi = 72.0);
    /////////////////////////////////////////
    double minx;
    double miny;
    double maxx;
    double maxy;
    std::map<double, std::vector<vec3>> moves; // from down to up horizontally 
    std::vector<vec3> path;

    // or maybe determine where each line intersects bezier - but in or out?

};

struct letters : public std::vector<letter> {
    void bezier_draw_bitmap(FT_Bitmap *b, FT_Int x, FT_Int y, int linescale);
    std::vector<letter> makescript(std::string fontname,
            std::string text,
            unsigned int fontsize = 64);

    /////////////////////////////////////////
    static int bezier_line_to(const FT_Vector* to, void* user);
    static int bezier_move_to(const FT_Vector* to, void* user);
    static int bezier_conic_to(const FT_Vector* control, const FT_Vector* to, void* user);
    static int bezier_cubic_to(const FT_Vector* control1, const FT_Vector* control2,
            const FT_Vector *to, void* user);
    /////////////////////////////////////////
    std::vector<letter> bsplines;
    double extentminx;
    double extentminy;
    double extentmaxx;
    double extentmaxy;

};

struct FreeType {
    FT_Library library; /* handle to library     */
    FT_Face face; /* handle to face object */
    int _height; /* in points */
    std::string _fontfilename;
    FT_Glyph glyph; /* a handle to the glyph image */

    FreeType() {
        library = NULL;
        face = NULL;
        glyph = NULL;
    }

    void Exit() {
        if (face != NULL)
            FT_Done_Face(face);
        if (library != NULL)
            FT_Done_FreeType(library);
        if(glyph!=NULL)
            FT_Done_Glyph( glyph ); 
    }

    void Init() {

        int error = FT_Init_FreeType(&library);
        if (error) {
            throw ft_errors[error];
        }
    }

    /*!
     * \brief  Create a new face object by calling FT_New_Face. 
     * A face describes a given typeface and style. For example, �Times New Roman Regular� and �Times New Roman Italic� correspond to two different faces.
     * \param fontfilename the font file pathname (a standard C string).
     * \param face_index
    Certain font formats allow several font faces to be embedded in a single file.
    This index tells which face you want to load. An error is returned if its value is too large.
    Index 0 always works, though.
     */
    int SetFace(std::string fontfilename = "/usr/share/fonts/truetype/arial.ttf", int face_index = 0) {
        if (library == NULL)
            throw std::string("Call init first\n");
        _fontfilename = fontfilename;
        int error = FT_New_Face(library,
                fontfilename.c_str(),
                0,
                &face);

        if (error == FT_Err_Unknown_File_Format) {
            // the font file could be opened and read, but it appears that its font format is unsupported
            return error;

        } else if (error) {
            // another error code means that the font file could not  be opened or read, or simply that it is broken...
            return error;
        }
        return 0;
    }

    /*! \brief To know how many faces a given font file contains, 
    load its first face (this is, face_index should be set to zero), 
    then check the value of face->num_faces, which indicates how many faces are embedded in the font file.
     */
    int NumFaces() {
        if (face == NULL)
            return -1;
        return face->num_faces;
    }

    /*! \brief true indicates that the face's font format is scalable and that glyph images can be rendered for all character pixel sizes
     */
    bool IsScalable() {
        if (face == NULL)
            return false;
        return face->face_flags & FT_FACE_FLAG_SCALABLE;
    }

    /*!
     * \brief It indicates the number of font units covered by the EM.
     *  This field is only valid for scalable formats (it is set to 0 otherwise). 
     * Note, however, that �units_per_EM� is a rather abstract value which bears no relation to the actual size of the glyphs in a font.
     */
    int UnitsPerEm() {
        if (face == NULL)
            return -1;
        if (!IsScalable())
            return false;
        return face->units_per_EM;
    }

    /*!
    FreeType 2 uses size objects to model all information related to a given character size for a given face. 
    For example, a size object holds the value of certain metrics like the ascender or text height, 
    expressed in 1/64th of a pixel, for a character size of 12 points.

    This function computes the character pixel size that corresponds to the character width and height and device resolutions. 
    However, if you want to specify the pixel sizes yourself, you can call FT_Set_Pixel_Sizes.

    \param char_height (and width) in points, e.g., 12 point where a Point = 1/72 Inch
    \param horiz  horizotal device resolution. If both values are zero, 72 dpi is used for both dimensions.
    \param vert vertical device resolution  
     */
    int SetCharSize(int height, int horiz = 300, int vert = 300) {
        if (face == NULL)
            return -1;
        _height = height;
        return FT_Set_Char_Size(
                face, /* handle to face object           */
                0, /* char_width in 1/64th of points  */
                height * 64, /* char_height in 1/64th of points */
                horiz, /* horizontal device resolution    */
                vert); /* vertical device resolution      */
    }
    /*
    while (*s) {
    wchar_t wc;
    int r = mbtowc(&wc, s, l);
    if (r == -1) {
    s++;
    continue;
    }
    width	The desired width, given as a 26.6 fractional point value (with 72pt = 1in).

     */

    /*!
     * width expressed in 26.6 fractional pixel format.
     * if the flag FT_LOAD_NO_SCALE has been used while loading the glyph, values are expressed in font units instead.
     * FreeType2 26.6 size convention to pixel convention - pixel_conv_value = ((double)ft26_conv_value) / 64.0;
     *
     */
    FT_BitmapGlyph GlyphBitmap(char c) {
        if (face == NULL)
            return -1;
        int error;
        wchar_t wc;
        size_t l=1;
        mbtowc(&wc, c, l);
        int glyph_index = FT_Get_Char_Index(face, (FT_ULong) wc);
        if (glyph_index == 0)
            return -1;
        
        // extract glyph image
        if (error = FT_Load_Glyph(face, glyph_index, FT_LOAD_NO_BITMAP |
                FT_LOAD_NO_HINTING))
            return error;

        if (error = FT_Get_Glyph(face->glyph, &glyph))
            return error;
        //FT_Pos width = glyph->;
        // convert to a bitmap (default render mode + destroying old)
        if (glyph->format != FT_GLYPH_FORMAT_BITMAP) {
            if (error = FT_Glyph_To_Bitmap(&glyph, FT_RENDER_MODE_NORMAL,
                    0, 1))
                return error;
        }

        // access bitmap content by typecasting
        FT_BitmapGlyph glyph_bitmap = (FT_BitmapGlyph) glyph;
        return glyph_bitmap;
        //return width.y;
    }
    
    
};