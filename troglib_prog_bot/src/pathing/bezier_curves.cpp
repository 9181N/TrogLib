#include "vex.h"
#include <vector>
#include <iostream>
#include "drive_movement/pathing/bezier_curves.h"
#include "drive_movement/pathing/pps.h"

// #include "odometry.h"
// #include <iostream>
// P = (1−t)3P1 + 3(1−t)2tP2 +3(1−t)t2P3 + t3P4
// x = (1−t)3x1 + 3(1−t)2tx2 +3(1−t)t2x3 + t3x4
// y = (1−t)3y1 + 3(1−t)2ty2 +3(1−t)t2y3 + t3y4

// x = (pow((1-t), 3) * p0) + (3 * pow((1-t),2) * p1) + (3 * pow((1-t),2) * p2) + (pow(t, 3) * p3)
// y = (pow((1-t), 3) * y1) + (3 * pow((1-t),2) * y2) + (3 * pow((1-t),2) * y3) + (pow(t, 3) * y4)

struct path path1;

float cubicAtT(float p0, float p1, float p2, float p3, float t)
{ // interpolates between the 4 given points
    float p_t = (pow((1 - t), 3) * p0) + (3 * pow((1 - t), 2) * t * p1) + (3 * (1 - t) * t * t * p2) + (t * t * t * p3);
    return p_t;
}

// calculate the curve points and save them as an array
void interpolateCubicX(float p0, float p1, float p2, float p3, int fidelity)
{
    float t = 0;
    for (int i = 0; i <= fidelity; i++)
    {
        t = (float)i / fidelity;
        if (t <= 1)
        {
            path1.x[i] = cubicAtT(p0, p1, p2, p3, t);
        }
        else
        {
            path1.x[i] = p3;
        }
    }
}

// calculate the curve points and save them as an array
void interpolateCubicY(float p0, float p1, float p2, float p3, int fidelity)
{
    float t = 0;
    for (int i = 0; i <= fidelity; i++)
    {
        t = (float)i / fidelity;
        if (t < 1)
        {
            path1.y[i] = cubicAtT(p0, p1, p2, p3, t);
        }
        else
        {
            path1.y[i] = p3;
        }
    }
}

// calculate the curve points and save them as an array
float cubic_bezier_2nd_deriv(float t, float p0, float p1, float p2, float p3)
{
    wait(1, vex::msec);
    //printf("(t: %.2f, p0: %.2f, p1: %.2f, p2: %.2f, p3: %.2f, )\n", t, p0, p1, p2, p3);
    double term1 = 6 * (1 - t) * p0;
    double term2 = (-12 + 18 * t) * p1;
    double term3 =  ((6 - 18 * t) * p2);
    double term4 = (6 * t * p3);
    double output = term1 + term2 + term3 + term4;
    //double wikiDef = 6*(1-t)*(p2-2*p1+p0) + 6*t*(p3-2*p2+p1);
    //printf("t1: %.2f, t2: %.2f, t3: %.2f, t4: %.2f, o: %.2f w: %.2f\n", term1, term2, term3, term4, output, wikiDef);

    //printf("output: %.2f\n", output);
    return output;
}
void interpolateCubicD2X(float p0, float p1, float p2, float p3, int fidelity)
{
    float t = 0;
    for (int i = 0; i <= fidelity; i++)
    {
        t = (float)i / fidelity;

        if (t <= 1)
        {
            path1.d2x[i] = cubic_bezier_2nd_deriv(t, p0, p1, p2, p3);
        }
        else
        {
            path1.d2x[i] = 0;
        }
    }
}

void interpolateCubicD2Y(float p0, float p1, float p2, float p3, int fidelity)
{
    float t = 0;
    for (int i = 0; i <= fidelity; i++)
    {
        t = (float)i / fidelity;
        if (t <= 1)
        {
            path1.d2y[i] = cubic_bezier_2nd_deriv(t, p0, p1, p2, p3);
        }
        else
        {
            path1.d2y[i] = 0;
        }
    }
}

double bezierTangent(float t, float a, float b, float c, float d)
{
    // note that abcd are aka x0 x1 x2 x3
    // the four coefficients ..

    // A = x3 - 3 * x2 + 3 * x1 - x0
    // B = 3 * x2 - 6 * x1 + 3 * x0
    // C = 3 * x1 - 3 * x0
    // D = x0
    // and then...
    // Vx = 3At2 + 2Bt + C

    // first calcuate what are usually known as the coeffients,
    // they are trivial based on the four control points:

    float C1 = (d - (3.0 * c) + (3.0 * b) - a);
    float C2 = ((3.0 * c) - (6.0 * b) + (3.0 * a));
    float C3 = ((3.0 * b) - (3.0 * a));
    // float C4 = ( a );  // (not needed for this calculation)

    // finally it is easy to calculate the slope element,
    // using those coefficients:
    // printf("slope at t = x: %.2f", ( ( 3.0 * C1 * t* t ) + ( 2.0 * C2 * t ) + C3 ));
    return ((3.0 * C1 * t * t) + (2.0 * C2 * t) + C3);

    // note that this routine works for both the x and y side;
    // simply run this routine twice, once for x once for y
    // note that there are sometimes said to be 8 (not 4) coefficients,
    // these are simply the four for x and four for y,
    // calculated as above in each case.
}

float tangent_at(float t, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3)
{
    float rise = bezierTangent(t, y0, y1, y2, y3);
    float run = bezierTangent(t, x0, x1, x2, x3);
    // printf("slope at t: %.2f ", rise / run);
    return rise / run;
}
float tangent_at_degrees(float t, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3)
{
    float rise = bezierTangent(t, y0, y1, y2, y3);
    float run = bezierTangent(t, x0, x1, x2, x3);
    float theta = atan2(run, rise);
    float theta_deg = (theta * (180 / M_PI));
    // printf("slope at t = %.2f: %.2f ", t, theta_deg);
    return theta_deg;
}

float jerk_calc(float t, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3)
{
    float rise = cubic_bezier_2nd_deriv(t, y0, y1, y2, y3);
    float run = cubic_bezier_2nd_deriv(t, x0, x1, x2, x3);
    // printf("slope at t: %.2f ", rise / run);
    return rise / run;
}

// calculate the curve headings and save them as an array (bounded from +-180)
void interpolateCubicH(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, int fidelity)
{
    float t = 0;
    for (int i = 0; i <= fidelity; i++)
    {
        t = (float)i / fidelity;
        if (t < 1)
        {
            path1.h[i] = tangent_at_degrees(t, x0, y0, x1, y1, x2, y2, x3, y3);
        }
        else
        {
            path1.h[i] = tangent_at_degrees(t, x0, y0, x1, y1, x2, y2, x3, y3);
        }
    }
}

double hypotenuse(double x, double y)
{
    return sqrt(x * x + y * y);
}

void pathLength()
{
    path1.length = 0;
    for (int i = 0; i < path1.fidelity; i++)
    {
        double dx = path1.x[i + 1] - path1.x[i];
        double dy = path1.y[i + 1] - path1.y[i];
        path1.length += hypotenuse(dx, dy);
    }
}

void generate_cubic_values(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, float fidelity)
{
    path1.fidelity = fidelity;
    get_line.last_found_index = 1;
    float t = 0;
    for (int i = 0; i <= fidelity; i++)
    {
        t = (float)i / fidelity;
        if (t <= 1)
        {
            path1.t[i] = t;
            path1.x[i] = cubicAtT(x0, x1, x2, x3, t);
            path1.y[i] = cubicAtT(y0, y1, y2, y3, t);
            path1.d1rise[i] = bezierTangent(t, y0, y1, y2, y3);
            path1.d1run[i] = bezierTangent(t, x0, x1, x2, x3);
            path1.h[i] = tangent_at_degrees(t, x0, y0, x1, y1, x2, y2, x3, y3);
            path1.d1slope[i] = tangent_at(t, x0, y0, x1, y1, x2, y2, x3, y3);

            path1.d2x[i] = cubic_bezier_2nd_deriv(t, x0, x1, x2, x3);
            path1.d2y[i] = cubic_bezier_2nd_deriv(t, y0, y1, y2, y3);
            path1.d2slope[i] = path1.d2y[i] / path1.d2x[i];
        }
        else
        {
            path1.x[i] = path1.x[path1.fidelity];
            path1.y[i] = path1.y[path1.fidelity];
            path1.h[i] = path1.h[path1.fidelity];
            path1.d2x[i] = path1.d2x[path1.fidelity];
            path1.d2y[i] = path1.d2y[path1.fidelity];
        }
    }

    pathLength();
}

void print_cubic(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, int fidelity)
{
    printf("\n\n");
    printf("(%.2f, %.2f),(%.2f, %.2f),(%.2f, %.2f),(%.2f, %.2f),\n", x0, y0, x1, y1, x2, y2, x3, y3);
    //wait(1500, vex::msec);
    std::cout << "" << std::flush;

    generate_cubic_values(x0, y0, x1, y1, x2, y2, x3, y3, fidelity);
    int p = 0;
    for (int i = 0; i <= fidelity; i++)
    {
        // printf("(%.2f),", path1.d2x[i]);
        printf("(%.2f, %.2f),", path1.x[i], path1.y[i]);
        //printf("(%.2f),", path1.d2y[i]/path1.d2x[i]); // slope
        //printf("(%.2f, %.2f),", path1.d2x[i], path1.d2y[i]);
        //printf("(%.2f,%.2f),", path1.t[i], path1.d2slope[i]);

        //printf("(%.2f),", path1.h[i]);
        if (p > 4)
        {
            p = 0;
            printf("\n");
        }
        p++;
    }
    printf("\n");
    //printf("curve Length: %.2f", path1.length);
    std::cout << "" << std::flush;
}
