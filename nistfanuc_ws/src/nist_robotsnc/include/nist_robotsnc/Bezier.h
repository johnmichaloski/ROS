/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Bezier.h
 * Author: michalos
 *
 * Created on April 6, 2017, 3:20 PM
 */

#ifndef BEZIER_H
#define BEZIER_H
#include <math.h>
#include <cfloat>
#include <vector>
#include <string>
#include "boost/tuple/tuple.hpp"
#include "Globals.h"
#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H

#ifndef SQ 
#define SQ(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))
#endif

namespace bezier {
    typedef boost::tuple<float, float, float, float> range_result;

    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    struct vec3 {

        vec3() {
            _x  = _y  = _z  = 0.0;
        }

        vec3(double _x_, double _y_, double _z_ = 0.0) {   
         _x =_x_;
         _y =_y_;
         _z =_z_;
        }

        vec3(FT_Vector v) {
            _x = (float) v.x;
            _y  = (float) v.y;
            _z  = 0.0;
        }
        // Eigen also provides the norm() method, which returns the square root of squaredNorm() .
        // http://eigen.tuxfamily.org/dox/group__TutorialReductionsVisitorsBroadcasting.html#title1http://eigen.tuxfamily.org/dox/group__TutorialReductionsVisitorsBroadcasting.html#title1

        double norm() {
            return sqrt(SQ(_x) + SQ(_y) + SQ(_z));
        }

//        double length() {
//            return sqrt(SQ(_x ) + SQ(_y ) + SQ(_z ));
//        }

        double dot(vec3 &v) {
            return _x  * v._x  + _y  * v._y  + _z  * v._z ;
        }

        bool operator==(const vec3& v) {
            return _x  == v._x  && _y  == v._y  && v._z  == _z ;
        }

        friend vec3 operator+(vec3 lhs, // passing lhs by value helps optimize chained a+b+c
                const vec3& rhs) // otherwise, both parameters may be const references
        {
            lhs._x  += rhs._x ;
            lhs._y  += rhs._y ;
            lhs._z  += rhs._z ; // reuse compound assignment
            return lhs; // return the result by value (uses move constructor)
        }

        friend vec3 operator-(vec3 lhs, // passing lhs by value helps optimize chained a+b+c
                const vec3& rhs) // otherwise, both parameters may be const references
        {
            lhs._x   -= rhs._x ;
            lhs._y  -= rhs._y ;
            lhs._z  -= rhs._z ; // reuse compound assignment
            return lhs; // return the result by value (uses move constructor)
        }

        friend vec3 operator-(vec3 lhs) {
            lhs._x  *= -1;
            lhs._y  *= -1;
            lhs._z  *= -1; // reuse compound assignment
            return lhs; // return the result by value (uses move constructor)
        }

        friend vec3 operator*(const double s, vec3 lhs) // passing lhs by value helps optimize chained a+b+c
        {
            lhs._x  *= s;
            lhs._y  *= s;
            lhs._z  *= s; // reuse compound assignment
            return lhs; // return the result by value (uses move constructor)
        }
        friend vec3 operator/(vec3 lhs, // passing lhs by value helps optimize chained a+b+c
                const vec3& rhs) // passing lhs by value helps optimize chained a+b+c
        {
            if (rhs._x == 0.0)
                lhs._x = 0.0;
            else
                lhs._x /= rhs._x;
            if (rhs._y == 0.0)
                lhs._y = 0.0;
            else
                lhs._y /= rhs._y;
            if (rhs._z == 0.0)
                lhs._z = 0.0;
            else
                lhs._z /= rhs._z; // reuse compound assignment
            return lhs; // return the result by value (uses move constructor)
        }
        friend vec3 operator*(vec3 lhs, // passing lhs by value helps optimize chained a+b+c
                const double s) // otherwise, both parameters may be const references
        {
            lhs._x  *= s;
            lhs._y  *= s;
            lhs._z  *= s; // reuse compound assignment
            return lhs; // return the result by value (uses move constructor)
        }

        friend vec3 operator/(vec3 lhs, // passing lhs by value helps optimize chained a+b+c
                const double s) // otherwise, both parameters may be const references
        {
            lhs._x  /= s;
            lhs._y  /= s;
            lhs._z  /= s; // reuse compound assignment
            return lhs; // return the result by value (uses move constructor)
        }
        std::string dump(){
            return Globals.StrFormat("%4.2f %4.2f %4.2f", x(), y(), z());
        }
        /////////////////////////////////////////////////
        double _x, _y, _z;
        double & x() { return _x; }
        double & y() { return _y; }
        double & z() { return _z; }
    };

    template<typename T>
    struct csplineT {
        char type;
        std::vector<T> pts;

        ///////////////////////////////////////////

        csplineT() {
            init();
        }

        void init() {
            pts.resize(4, vec3(0.0, 0.0));
        }

        csplineT(T pt1, T pt2, T pt3, T pt4) {
            pts.clear();
            pts.push_back(pt1);
            pts.push_back(pt2);
            pts.push_back(pt3);
            pts.push_back(pt4);
        }

        T compute(double t) {
            T v;
            v.x() = CUBE(1. - t) * pts[0].x() + 3 * t * SQ(1. - t) * pts[1].x() + 3 * SQ(t) * (1. - t) * pts[2].x() + CUBE(t) * pts[3].x();
            v.y() = CUBE(1. - t) * pts[0].y() + 3 * t * SQ(1. - t) * pts[1].y() + 3 * SQ(t) * (1. - t) * pts[2].y() + CUBE(t) * pts[3].y();
            return v;

        }
        T compute(double t, csplineT<T> cs) {
            T v;
            v.x() = CUBE(1. - t) * cs.pts[0].x() + 3 * t * SQ(1. - t) * cs.pts[1].x() + 3 * SQ(t) * (1. - t) * cs.pts[2].x() + CUBE(t) * cs.pts[3].x();
            v.y() = CUBE(1. - t) * cs.pts[0].y() + 3 * t * SQ(1. - t) * cs.pts[1].y() + 3 * SQ(t) * (1. - t) * cs.pts[2].y() + CUBE(t) * cs.pts[3].y();
            return v;

        }
        bool split(double t, csplineT<T> &orig, csplineT<T> &firsthalf, csplineT<T> &secondhalf) {
            T p0 = orig.pts[0], p1 = orig.pts[1], p2 = orig.pts[2], p3 = orig.pts[3];
            T p4 = lerp(p0, p1, t);
            T p5 = lerp(p1, p2, t);
            T p6 = lerp(p2, p3, t);
            T p7 = lerp(p4, p5, t);
            T p8 = lerp(p5, p6, t);
            T p9 = lerp(p7, p8, t);

            firsthalf = csplineT<T>(p0, p4, p7, p9);
            secondhalf = csplineT<T>(p9, p8, p6, p3);
            return true;
        }

        std::vector<T> buildpts(csplineT<T> & orig, double &gap) {
            std::vector<T> pts;
            csplineT<T> firsthalf;
            csplineT<T> secondhalf;
            if (T(orig.pts[0] - orig.pts[3]).norm() < gap) {
                T g = compute(.5, orig);
                pts.push_back(g);
                return pts;
            }

            this->split(0.5, orig, firsthalf, secondhalf);


            std::vector<T> fpts = buildpts(firsthalf, gap);
            pts.insert(pts.begin(), fpts.begin(), fpts.end());

            std::vector<T> spts = buildpts(secondhalf, gap);
            pts.insert(pts.end(), spts.begin(), spts.end());

            return pts;
        }

        int IntersectBB(vec3 A, vec3 B) {
            csplineT<T> b(A, B, A, B); // line as bspline
            // Compute bounding box for a
            double minax, maxax, minay, maxay;
            
            if (pts[0].x() > pts[3].x()) // These are the most likely to be extremal
                minax = pts[3].x(), maxax = pts[0].x();
            else
                minax = pts[0].x(), maxax = pts[3].x();
            if (pts[2].x() < minax)
                minax = pts[2].x();
            else if (pts[2].x() > maxax)
                maxax = pts[2].x();
            if (pts[1].x() < minax)
                minax = pts[1].x();
            else if (pts[1].x() > maxax)
                maxax = pts[1].x();
            if (pts[0].y() > pts[3].y())
                minay = pts[3].y(), maxay = pts[0].y();
            else
                minay = pts[0].y(), maxay = pts[3].y();
            if (pts[2].y() < minay)
                minay = pts[2].y();
            else if (pts[2].y() > maxay)
                maxay = pts[2].y();
            if (pts[1].y() < minay)
                minay = pts[1].y();
            else if (pts[1].y() > maxay)
                maxay = pts[1].y();

            // Compute bounding box for b
            double minbx, maxbx, minby, maxby;
            if (b.pts[0].x() > b.pts[3].x())
                minbx = b.pts[3].x(), maxbx = b.pts[0].x();
            else
                minbx = b.pts[0].x(), maxbx = b.pts[3].x();
            if (b.pts[2].x() < minbx)
                minbx = b.pts[2].x();
            else if (b.pts[2].x() > maxbx)
                maxbx = b.pts[2].x();
            if (b.pts[1].x() < minbx)
                minbx = b.pts[1].x();
            else if (b.pts[1].x() > maxbx)
                maxbx = b.pts[1].x();
            if (b.pts[0].y() > b.pts[3].y())
                minby = b.pts[3].y(), maxby = b.pts[0].y();
            else
                minby = b.pts[0].y(), maxby = b.pts[3].y();
            if (b.pts[2].y() < minby)
                minby = b.pts[2].y();
            else if (b.pts[2].y() > maxby)
                maxby = b.pts[2].y();
            if (b.pts[1].y() < minby)
                minby = b.pts[1].y();
            else if (b.pts[1].y() > maxby)
                maxby = b.pts[1].y();
            // Test bounding box of b against bounding box of a
            if ((minax > maxbx) || (minay > maxby) // Not >= : need boundary case
                    || (minbx > maxax) || (minby > maxay))
                return 0; // they don't intersect
            else
                return 1; // they intersect
        }
        
        // assumes 1 cross per bezier cubic.
        std::vector<T> findintersection(vec3 &A, vec3 & B, csplineT<T> & orig, double &gap) {
            std::vector<T> pts;
            csplineT<T> firsthalf;
            csplineT<T> secondhalf;
            if (T(orig.pts[0] - orig.pts[3]).norm() < gap) {
                double x,y;
                if(linesIntersect(orig.pts[0], orig.pts[3], A, B, x,y, 0.01)){
                     std::cout << "x=" << x << " y=" << y << "\n";
                     pts.push_back(vec3(x,y,0.));
                }
                return pts;
            }

            this->split(0.5, orig, firsthalf, secondhalf);


            std::vector<T> fpts = findintersection(A, B, firsthalf, gap);
            if( fpts.size()>0)
                return fpts; 

            std::vector<T> spts = findintersection(A, B, secondhalf, gap);
             if( spts.size()>0)
                return spts; 

            return pts;
        }
        void maxmin(double & minx, double & miny, double & maxx, double & maxy) {

            for (double t = 0.0; t <= 1.0; t += 0.1) {
                T v = compute(t);
                if (v.x() < minx) minx = v.x();
                if (v.y() < miny) miny = v.y();
                if (v.x() > maxx) maxx = v.x();
                if (v.y() > maxy) maxy = v.y();
            }
        }

        static void save2D(std::string filename, std::vector<T> &pts) {
            std::string str;
            for (size_t i = 0; i < pts.size(); i++) {
                    str += Globals.StrFormat("%4.2f %4.2f\n", pts[i].x(), pts[i].y());
            }
            Globals.WriteFile(filename, str);
        }
        std::string Dump(){
         return Globals.StrFormat("%c (%4.2f %4.2f)  (%4.2f %4.2f) (%4.2f %4.2f) (%4.2f %4.2f)\n", type,
                pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y(),
                pts[2].x(), pts[2].y(), pts[3].x(), pts[3].y());
        }
    };
    typedef csplineT<vec3> cspline;
    
    
    vec3 lerp(vec3 a, vec3 b, double t);
    bool split(cspline b, double t, cspline &firsthalf, cspline &secondhalf);

    double angle(vec3 pt1, vec3 pt2, vec3 pt3);
    double getClosestPointToCubicBezier(double fx, double fy, int slices, cspline cs);
    
      double bezLength3(const cspline b);
     
    // Use: getClosestPointToCubicBezier(iterations, fx, fy, 0., 1., slices, x0, y0, x1, y1, x2, y2, x3, y3);
    double getClosestPointToCubicBezier(int iterations, double fx, double fy, double start, double end, int slices, cspline cs);
    std::vector<double> cubicRoots(std::vector<double> pts);
    /*computes intersection between a cubic spline and a line segment*/
    std::vector<vec3> computeIntersections(cspline cs, vec3 pt1, vec3 pt2);
    bool isPointOnLine(vec3 p1, vec3 p2, vec3 p3, double tolerance = 0.001);
    static bool isBetween(double x, double bound1, double bound2, double tolerance);

    bool linesIntersect(vec3 P11, vec3 P12, vec3 P21, vec3 P22, double & x, double &y, double tolerance = 0.0001);
    int GetIntersection(double fDst1, double fDst2, vec3 P1, vec3 P2, vec3 &Hit);
    int InBox(vec3 Hit, vec3 B1, vec3 B2, const int Axis);

    // returns true if line (L1, L2) intersects with the box (B1, B2)
    // returns intersection point in Hit
    int CheckLineBox(vec3 B1, vec3 B2, vec3 L1, vec3 L2, vec3 &Hit);
    int CheckLineInCubicBezierBox(cspline cs, vec3 L1, vec3 L2, vec3 &Hit);
    double getIntersectToCubicBezier(long slices, cspline cs, vec3 &a, vec3 &b) ;

};
#endif /* BEZIER_H */

