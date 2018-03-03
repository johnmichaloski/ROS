/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


/* 
 * File:   Bezier.h
 * Author: michaloski
 *
 * Created on April 6, 2017, 3:20 PM
 */

#include "Bezier.h"

namespace bezier {
    //http://stackoverflow.com/questions/18655135/divide-bezier-curve-into-two-equal-halves 

    vec3 lerp(vec3 a, vec3 b, double t) {
        double s = 1 - t;
        vec3 c = b - a;
        return vec3(a.x() + t * c.x(), a.y() + t * c.y());
        //       return vec3(a.x() * s + b.x()*t,
        //              a.y() * s + b.y()*t, 0.0);
    }

    /**
    9
    7 8
    4 5 6
    0 1 2 3

    You would feed the algorithm 0 1 2 3 as control points, then you would index the two perfectly 
    subdivided curves at 0, 4, 7, 9 and 9, 8, 6, 3. Take special note to see that these 
    curves start and end at the same point. and the final index 9 which is the point on the 
    curve is used as the other new anchor point. Given this you can perfectly subdivide a bezier curve.
     */
    bool split(cspline b, double t, cspline &firsthalf, cspline &secondhalf) {
        vec3 p0 = b.pts[0], p1 = b.pts[1], p2 = b.pts[2], p3 = b.pts[3];
        vec3 p4 = lerp(p0, p1, t);
        vec3 p5 = lerp(p1, p2, t);
        vec3 p6 = lerp(p2, p3, t);
        vec3 p7 = lerp(p4, p5, t);
        vec3 p8 = lerp(p5, p6, t);
        vec3 p9 = lerp(p7, p8, t);

        firsthalf = cspline(p0, p4, p7, p9);
        secondhalf = cspline(p9, p8, p6, p3);
        return true;
    }
    // http://stackoverflow.com/questions/1211212/how-to-calculate-an-angle-from-three-points
    /*!
     *\brief Compute angle between three points of which pt2 is the "middle" point.
     * \param pt1 is frist vector
     * \param pt2 is second vector
     * \param pt3 is third vector.
     * \return double angle in radians
     */
    double angle(vec3 pt1, vec3 pt2, vec3 pt3) {
        double P12 = sqrt(SQ(pt1.x() - pt2.x()) + SQ(pt1.y() - pt2.y()));
        double P13 = sqrt(SQ(pt1.x() - pt3.x()) + SQ(pt1.y() - pt3.y()));
        double P23 = sqrt(SQ(pt2.x() - pt3.x()) + SQ(pt2.y() - pt3.y()));
        return acos((SQ(P12) + SQ(P13) - SQ(P23)) / (2 * P12 * P13));

    }

    double getClosestPointToCubicBezier(double fx, double fy, int slices, cspline cs) {
        double x0 = cs.pts[0].x();
        double y0 = cs.pts[0].y();
        double x1 = cs.pts[1].x();
        double y1 = cs.pts[1].y();
        double x2 = cs.pts[2].x();
        double y2 = cs.pts[2].y();
        double x3 = cs.pts[3].x();
        double y3 = cs.pts[3].y();
        double tick = 1. / (double) slices;
        double x;
        double y;
        double t;
        double best = 0;
        double bestDistance = DBL_MAX;
        ;
        double currentDistance;
        for (int i = 0; i <= slices; i++) {
            t = i * tick;
            //B(t) = (1-t)**3 p0 + 3(1 - t)**2 t P1 + 3(1-t)t**2 P2 + t**3 P3
            x = (1 - t) * (1 - t) * (1 - t) * x0 + 3 * (1 - t) * (1 - t) * t * x1 + 3 * (1 - t) * t * t * x2 + t * t * t * x3;
            y = (1 - t) * (1 - t) * (1 - t) * y0 + 3 * (1 - t) * (1 - t) * t * y1 + 3 * (1 - t) * t * t * y2 + t * t * t * y3;

            currentDistance = sqrt(SQ(x - fx) + SQ(y - fy)); // Point.distanceSq(x,y,fx,fy);
            if (currentDistance < bestDistance) {
                bestDistance = currentDistance;
                best = t;
            }
        }
        return best;
    }

    bool on_segment(vec3 p, vec3 a, vec3 b, double epsilon = 0.001) {
        // ensure points are collinear
        double zero = (b.x() - a.x()) * (p.y() - a.y()) - (p.x() - a.x()) * (b.y() - a.y());
        if (zero > epsilon || zero < -epsilon)
            return false;

        // check if x-coordinates are not equal
        if (a.x() - b.x() > epsilon || b.x() - a.x() > epsilon)
            // ensure x is between a.x & b.x (use tolerance)
            return a.x() > b.x()
            ? p.x() + epsilon > b.x() && p.x() - epsilon < a.x()
            : p.x() + epsilon > a.x() && p.x() - epsilon < b.x();

        // ensure y is between a.y & b.y (use tolerance)
        return a.y() > b.y()
                ? p.y() + epsilon > b.y() && p.y() - epsilon < a.y()
                : p.y() + epsilon > a.y() && p.y() - epsilon < b.y();
    }

    double getIntersectToCubicBezier(long slices, cspline cs, vec3 &a, vec3 &b) {
        double x0 = cs.pts[0].x();
        double y0 = cs.pts[0].y();
        double x1 = cs.pts[1].x();
        double y1 = cs.pts[1].y();
        double x2 = cs.pts[2].x();
        double y2 = cs.pts[2].y();
        double x3 = cs.pts[3].x();
        double y3 = cs.pts[3].y();
        double tick = 1. / (double) slices;
        double x;
        double y;
        double t;
        double best = -1.0;
        double bestDistance = DBL_MAX;
        double currentDistance;
        
        // Slices should depend on length of curve
        for (int i = 0; i <= slices; i++) {
            t = i * tick;
            //B(t) = (1-t)**3 p0 + 3(1 - t)**2 t P1 + 3(1-t)t**2 P2 + t**3 P3
            x = (1 - t) * (1 - t) * (1 - t) * x0 + 3 * (1 - t) * (1 - t) * t * x1 + 3 * (1 - t) * t * t * x2 + t * t * t * x3;
            y = (1 - t) * (1 - t) * (1 - t) * y0 + 3 * (1 - t) * (1 - t) * t * y1 + 3 * (1 - t) * t * t * y2 + t * t * t * y3;

            // could use last x,y with this x,y to form a line to see if intersect?
            if( on_segment(vec3(x,y), a, b, 0.1)) {
                best=t;
                return t;
            }
//                currentDistance = sqrt(SQ(x - fx) + SQ(y - fy)); // Point.distanceSq(x,y,fx,fy);
//                if (currentDistance < bestDistance) {
//                    bestDistance = currentDistance;
//                    best = t;
//            }
        }
        return best;
    }

    // https://www.gamedev.net/topic/313018-calculating-the-length-of-a-bezier-curve/

    double bezLength3(const cspline b) {
        vec3 p0 = (b.pts[0] - b.pts[1]);
        vec3 p1 = (b.pts[2] - b.pts[1]);
        vec3 p2;
        vec3 p3 = (b.pts[3] - b.pts[2]);
        double l0 = p0.norm();
        double l1 = p1.norm();
        double l3 = p3.norm();
        if (l0 > 0.f) p0 = p0 / l0;
        if (l1 > 0.f) p1 = p1 / l1;
        if (l3 > 0.f) p3 = p3 / l3;
        p2 = -p1;
        double a = fabs(p0.dot(p1)) + fabs(p2.dot(p3));
        if ((a > 1.98f) || ((l0 + l1 + l3) < ((4.f - a)*8.f))) {
            return l0 + l1 + l3;
        } // if
        cspline bl, br;
        bl.pts[0] = b.pts[0];
        bl.pts[1] = (b.pts[0] + b.pts[1])*.5f;
        vec3 mid = (b.pts[1] + b.pts[2])*.5f;
        bl.pts[2] = (bl.pts[1] + mid)*.5f;
        br.pts[3] = b.pts[3];
        br.pts[2] = (b.pts[2] + b.pts[3])*.5f;
        br.pts[1] = (br.pts[2] + mid)*.5f;
        br.pts[0] = (br.pts[1] + bl.pts[2])*.5f;
        bl.pts[3] = br.pts[0];
        return bezLength3(bl) + bezLength3(br);
    } // bezLength3
    // Use: getClosestPointToCubicBezier(iterations, fx, fy, 0., 1., slices, x0, y0, x1, y1, x2, y2, x3, y3);

    double getClosestPointToCubicBezier(int iterations, double fx, double fy, double start, double end, int slices, cspline cs) {
        double x0 = cs.pts[0].x();
        double y0 = cs.pts[0].y();
        double x1 = cs.pts[1].x();
        double y1 = cs.pts[1].y();
        double x2 = cs.pts[2].x();
        double y2 = cs.pts[2].y();
        double x3 = cs.pts[3].x();
        double y3 = cs.pts[3].y();
        if (iterations <= 0)
            return (start + end) / 2;
        double tick = (end - start) / (double) slices;
        double x, y, dx, dy;
        double best = 0;
        double bestDistance = DBL_MAX;
        double currentDistance;
        double t = start;
        while (t <= end) {
            //B(t) = (1-t)**3 p0 + 3(1 - t)**2 t P1 + 3(1-t)t**2 P2 + t**3 P3
            x = (1 - t) * (1 - t) * (1 - t) * x0 + 3 * (1 - t) * (1 - t) * t * x1 + 3 * (1 - t) * t * t * x2 + t * t * t * x3;
            y = (1 - t) * (1 - t) * (1 - t) * y0 + 3 * (1 - t) * (1 - t) * t * y1 + 3 * (1 - t) * t * t * y2 + t * t * t * y3;

            dx = SQ(x - fx);
            dy = SQ(y - fy);
            currentDistance = dx + dy;
            if (currentDistance < bestDistance) {
                bestDistance = currentDistance;
                best = t;
            }
            t += tick;
        }
        return getClosestPointToCubicBezier(iterations - 1, fx, fy, std::max(best - tick, 0.), std::max(best + tick, 1.), slices, cs);
    }

    std::vector<double> cubicRoots(std::vector<double> pts) {
        double a = pts[0];
        double b = pts[1];
        double c = pts[2];
        double d = pts[3];

        double A = b / a;
        double B = c / a;
        double C = d / a;

        double S, T, Im;

        double Q = (3 * B - pow(A, 2)) / 9;
        double R = (9 * A * B - 27 * C - 2 * pow(A, 3)) / 54;
        double D = pow(Q, 3) + pow(R, 2); // polynomial discriminant

        std::vector<double> t(3, -1.0); //  t=Array();

        if (D >= 0) // complex or duplicate roots
        {
            double S = sgn(R + sqrt(D)) * pow(fabs(R + sqrt(D)), (1. / 3.));
            double T = sgn(R - sqrt(D)) * pow(fabs(R - sqrt(D)), (1. / 3.));

            t[0] = -A / 3 + (S + T); // real root
            t[1] = -A / 3 - (S + T) / 2; // real part of complex root
            t[2] = -A / 3 - (S + T) / 2; // real part of complex root
            Im = fabs(sqrt(3.)*(S - T) / 2.); // complex part of root pair   

            /*discard complex roots*/
            if (Im != 0) {
                t[1] = -1.;
                t[2] = -1.;
            }

        } else // distinct real roots
        {
            double th = acos(R / sqrt(-pow(Q, 3)));

            t[0] = 2 * sqrt(-Q) * cos(th / 3) - A / 3.;
            t[1] = 2 * sqrt(-Q) * cos((th + 2 * M_PI) / 3.) - A / 3.;
            t[2] = 2 * sqrt(-Q) * cos((th + 4 * M_PI) / 3.) - A / 3.;
            Im = 0.0;
        }

        /*discard out of spec roots*/
        for (size_t i = 0; i < 3; i++)
            if (t[i] < 0. || t[i] > 1.0) t[i] = -1.;

        /*sort but place -1 at the end*/
        //t=sortSpecial(t);

        //AtlTrace("%.2f %.2f %.2f\n", t[0],t[1],t[2]);
        return t;
    }

    /*computes intersection between a cubic spline and a line segment*/
    std::vector<vec3> computeIntersections(cspline cs, vec3 pt1, vec3 pt2) // std::vector<double> px, std::vector<double> py,std::vector<double> lx,std::vector<double> ly)
    {
        std::vector<double> X(2);

        double A = pt2.y() - pt1.y(); // ly[1]-ly[0];	    //A=y2-y1
        double B = pt1.x() - pt2.x(); // lx[0]-lx[1];	    //B=x1-x2
        double C = pt1.x() * (pt1.y() - pt2.y()) + // lx[0]*(ly[0]-ly[1]) + 
                pt1.y() * (pt2.x() - pt1.x()); // ly[0]*(lx[1]-lx[0]);	//C=x1*(y1-y2)+y1*(x2-x1)

        std::vector<double> bx(4);
        std::vector<double> bby(4, 0.0);

        bx[0] = cs.pts[0].x();
        bx[1] = cs.pts[1].x();
        bx[2] = cs.pts[2].x();
        bx[3] = cs.pts[3].x(); //var bx = bezierCoeffs(px[0],px[1],px[2],px[3]);
        // by appears to be a keyword of some kind
        bby[0] = cs.pts[0].y();
        bby[1] = cs.pts[1].y();
        bby[2] = cs.pts[2].y();
        bby[3] = cs.pts[3].y(); // var by = bezierCoeffs(py[0],py[1],py[2],py[3])


        std::vector<double> P(4, 0.0);

        P[0] = A * bx[0] + B * bby[0]; /*t^3*/
        P[1] = A * bx[1] + B * bby[1]; /*t^2*/
        P[2] = A * bx[2] + B * bby[2]; /*t*/
        P[3] = A * bx[3] + B * bby[3] + C; /*1*/

        std::vector<double> r = cubicRoots(P);

        std::vector<vec3> sol;
        /*verify the roots are in bounds of the linear segment*/
        for (size_t i = 0; i < 3; i++) {
            double t = r[i];

            X[0] = bx[0] * t * t * t + bx[1] * t * t + bx[2] * t + bx[3];
            X[1] = bby[0] * t * t * t + bby[1] * t * t + bby[2] * t + bby[3];

            /*above is intersection point assuming infinitely long line segment,
            make sure we are also in bounds of the line*/
            double s;
            if ((pt2.x() - pt1.x()) != 0)
                s = (X[0] - pt1.x()) / (pt2.x() - pt1.x());
            else
                s = (X[1] - pt1.y()) / (pt2.y() - pt1.y());
            //if ((lx[1]-lx[0])!=0)           /*if not vertical line*/
            //	s=(X[0]-lx[0])/(lx[1]-lx[0]);
            //else
            //	s=(X[1]-ly[0])/(ly[1]-ly[0]);

            /*in bounds?*/
            if (t < 0 || t > 1.0 || s < 0 || s > 1.0) {
                X[0] = -100; /*move off screen*/
                X[1] = -100;
            } else {
                sol.push_back(vec3(X[0], X[1]));
            }

            /*move intersection point*/
            //I[i].setAttributeNS(null,"cx",X[0]);
            //I[i].setAttributeNS(null,"cy",X[1]);
        }
        return sol;
    }

    bool isPointOnLine(vec3 p1, vec3 p2, vec3 p3, double tolerance) // returns true if p3 is on line p1, p2
    {
        vec3 va = p1 - p2;
        vec3 vb = p3 - p2;
        double area = va.x() * vb.y() - va.y() * vb.x();
        if (fabs(area) < tolerance)
            return true;
        return false;
    }

    static bool isBetween(double x, double bound1, double bound2, double tolerance) {
        // Handles cases when 'bound1' is greater than 'bound2' and when
        // 'bound2' is greater than 'bound1'.
        return (((x >= (bound1 - tolerance)) && (x <= (bound2 + tolerance))) ||
                ((x >= (bound2 - tolerance)) && (x <= (bound1 + tolerance))));
    }

    // https://www.topcoder.com/community/data-science/data-science-tutorials/geometry-concepts-line-intersection-and-its-applications/
    // Assuming you have two lines of the form Ax + By = C, you can find it pretty easily
    // Tested.

    bool linesIntersect(vec3 P11, vec3 P12, vec3 P21, vec3 P22, double & x, double &y, double tolerance) {

        double A1 = P12.y() - P11.y(); // y2-y1
        double B1 = P11.x() - P12.x(); // x1-x2
        double C1 = A1 * P11.x() + B1 * P11.y(); // C = A*x1+B*y1

        double A2 = P22.y() - P21.y(); // y2-y1
        double B2 = P21.x() - P22.x(); // x1-x2
        double C2 = A2 * P21.x() + B2 * P21.y(); // C = A*x1+B*y1

        double delta = A1 * B2 - A2*B1;
        // check perpendicularity
        if (fabs(delta) < tolerance)
            return false;
        x = (B2 * C1 - B1 * C2) / delta;
        y = (A1 * C2 - A2 * C1) / delta;

        //// Now check if intersection point on line segments.
        if (!isBetween(x, std::min(P11.x(), P12.x()), std::max(P11.x(), P12.x()), tolerance))
            return false;
        if (!isBetween(x, std::min(P21.x(), P22.x()), std::max(P21.x(), P22.x()), tolerance))
            return false;
        if (!isBetween(y, std::min(P11.y(), P12.y()), std::max(P11.y(), P12.y()), tolerance))
            return false;
        if (!isBetween(y, std::min(P21.y(), P22.y()), std::max(P21.y(), P22.y()), tolerance))
            return false;
        return true;
    }

    int inline GetIntersection(double fDst1, double fDst2, vec3 P1, vec3 P2, vec3 &Hit) {
        if ((fDst1 * fDst2) >= 0.0f) return 0;
        if (fDst1 == fDst2) return 0;
        Hit = P1 + (P2 - P1) * (-fDst1 / (fDst2 - fDst1));
        return 1;
    }

    int inline InBox(vec3 Hit, vec3 B1, vec3 B2, const int Axis) {
        if (Axis == 1 && Hit.z() > B1.z() && Hit.z() < B2.z() && Hit.y() > B1.y() && Hit.y() < B2.y()) return 1;
        if (Axis == 2 && Hit.z() > B1.z() && Hit.z() < B2.z() && Hit.x() > B1.x() && Hit.x() < B2.x()) return 1;
        if (Axis == 3 && Hit.x() > B1.x() && Hit.x() < B2.x() && Hit.y() > B1.y() && Hit.y() < B2.y()) return 1;
        return 0;
    }

    // returns true if line (L1, L2) intersects with the box (B1, B2)
    // returns intersection point in Hit

    int CheckLineBox(vec3 B1, vec3 B2, vec3 L1, vec3 L2, vec3 &Hit) {
        if (L2.x() < B1.x() && L1.x() < B1.x()) return false;
        if (L2.x() > B2.x() && L1.x() > B2.x()) return false;
        if (L2.y() < B1.y() && L1.y() < B1.y()) return false;
        if (L2.y() > B2.y() && L1.y() > B2.y()) return false;
        if (L2.z() < B1.z() && L1.z() < B1.z()) return false;
        if (L2.z() > B2.z() && L1.z() > B2.z()) return false;
        if (L1.x() > B1.x() && L1.x() < B2.x() &&
                L1.y() > B1.y() && L1.y() < B2.y() &&
                L1.z() > B1.z() && L1.z() < B2.z()) {
            Hit = L1;
            return true;
        }
        if ((GetIntersection(L1.x() - B1.x(), L2.x() - B1.x(), L1, L2, Hit) && InBox(Hit, B1, B2, 1))
                || (GetIntersection(L1.y() - B1.y(), L2.y() - B1.y(), L1, L2, Hit) && InBox(Hit, B1, B2, 2))
                || (GetIntersection(L1.z() - B1.z(), L2.z() - B1.z(), L1, L2, Hit) && InBox(Hit, B1, B2, 3))
                || (GetIntersection(L1.x() - B2.x(), L2.x() - B2.x(), L1, L2, Hit) && InBox(Hit, B1, B2, 1))
                || (GetIntersection(L1.y() - B2.y(), L2.y() - B2.y(), L1, L2, Hit) && InBox(Hit, B1, B2, 2))
                || (GetIntersection(L1.z() - B2.z(), L2.z() - B2.z(), L1, L2, Hit) && InBox(Hit, B1, B2, 3)))
            return true;

        return false;
    }

    int CheckLineInCubicBezierBox(cspline cs, vec3 L1, vec3 L2, vec3 &Hit) {
        double minx = DBL_MAX, miny = DBL_MAX, maxx = -DBL_MAX, maxy = -DBL_MAX;
        for (size_t i = 0; i < 3; i++) {
            if (cs.pts[i].x() < minx) minx = cs.pts[i].x();
            if (cs.pts[i].x() > maxx) maxx = cs.pts[i].x();
            if (cs.pts[i].y() < minx) miny = cs.pts[i].y();
            if (cs.pts[i].y() > maxy) maxy = cs.pts[i].y();
        }

        return CheckLineBox(vec3(minx, miny), vec3(maxx, maxy), L1, L2, Hit);
    }
};
