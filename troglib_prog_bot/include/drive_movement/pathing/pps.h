#pragma once
#include "vex.h"
#include "drive_movement/pathing/bezier_curves.h"
#include "sensor_data.h"
#include <iostream>
class line_intersection
{

    int signum(float input)
    { // returns the sign of an input
        if (input > 0.0)
            return 1;
        if (input < 0.0)
            return -1;
        return 0;
    }

    float min(float first, float second)
    {
        if (first < second)
        {
            return first;
        }
        else
        {
            return second;
        }
    }

    float max(float first, float second)
    {
        if (first > second)
        {
            return first;
        }
        else
        {
            return second;
        }
    }

    float hypLength(float x1, float y1, float x2, float y2)
    { // finds the length of a line drawn between two points
        float X = x1 - x2;
        float Y = y1 - y2;
        float dist = sqrt(X * X + Y * Y);
        return dist;
    }

public:
    double goal_x = 0, goal_y = 0;
    int last_found_index = 1, starting_index = 0;
    double look_ahead_dist = 10;
    void searchForIntersect()
    {
        // HEAVILY INSPIRED BY: blrs wiki and Sarah Xiang from VRC team 97963A and VEXU team ILLINI
        // output (intersections found) should be stored in arrays sol1 and sol2 in the form of sol1 = [sol1_x, sol1_y]
        // if two solutions are the same, store the same values in both sol1 and sol2
        /*
            if discriminant < 0 : no intersection
            if discriminant = 0 : tangent
            if discriminant > 0 : intersection
        */
        double sol1_x{0};
        double sol1_y{0};
        double sol2_x{0};
        double sol2_y{0};
        //goal_x = 0, goal_y = 0;

        // boolean variable to keep track of if intersections are found
        bool intersectFound = false;
        double currentX = bot.x;
        double currentY = bot.y;
        starting_index = last_found_index;
        for (int i = starting_index; i < path1.fidelity - 1; i++)
        {
            wait(0, vex::msec);
            // beginning of line-circle intersection code
            // subtract currentX and currentY from [x1, y1] and [x2, y2] to offset the system to origin
            double x1 = path1.x[i] - currentX;
            double y1 = path1.y[i] - currentY;
            double x2 = path1.x[i + 1] - currentX;
            double y2 = path1.y[i + 1] - currentY;
            // printf("(%.2f,%.2f),", x1, y1);
            // printf("(%.2f,%.2f)\n", x2, y2);

            // printf("(%.2f,%.2f),(%.2f,%.2f),(%.2f,%.2f),(%.2f,%.2f)\n", x1, x2, y1, y2, x1_offset, x2_offset, y1_offset, y2_offset);
            //  output (intersections found) should be stored in arrays sol1 and sol2 in the form of sol1 = [sol1_x, sol1_y]
            //  if two solutions are the same, store the same values in both sol1 and sol2
            double r = look_ahead_dist;
            double dx = x2 - x1;
            double dy = y2 - y1;
            // printf("(%.2f,%.2f)\n", dx,dy);
            double dr = sqrt(dx * dx + dy * dy);
            // printf("%.2f\n", dr);
            double D = x1 * y2 - x2 * y1;
            double discriminant = (r * r) * (dr * dr) - (D * D);

            if (discriminant >= 0)
            {
                double rootD = sqrt(discriminant);
                // calculate the solutions
                sol1_x = (D * dy + signum(dy) * dx * rootD) / (dr * dr);
                sol2_x = (D * dy - signum(dy) * dx * rootD) / (dr * dr);
                sol1_y = (-D * dx + fabs(dy) * rootD) / (dr * dr);
                sol2_y = (-D * dx - fabs(dy) * rootD) / (dr * dr);

                // add currentX and currentY back to the solutions, offset the system back to its original position
                sol1_x += currentX;
                sol1_y += currentY;
                sol2_x += currentX;
                sol2_y += currentY;
                // end of line-circle intersection code

                // find min and max x y values
                double minX = min(path1.x[i], path1.x[i + 1]);
                double maxX = max(path1.x[i], path1.x[i + 1]);
                double minY = min(path1.y[i], path1.y[i + 1]);
                double maxY = max(path1.y[i], path1.y[i + 1]);
                // printf("Domain(%.2f,%.2f),", minX, maxX);
                // printf("Range(%.2f,%.2f)\n", minY, maxY);

                // check to see if any of the two solution points are within the correct range
                // for a solution point to be considered valid, its x value needs to be within minX and maxX AND its y value needs to be between minY and maxY
                // if sol1 OR sol2 are within the range, intersection is found

                // TBH its this line thats broken, Good Luck! :D
                bool sol1_x_domain, sol1_y_range, sol2_x_domain, sol2_y_range;
                if (minX <= sol1_x && sol1_x <= maxX)
                    sol1_x_domain = true;
                else
                    sol1_x_domain = false;
                if (minX <= sol2_x && sol2_x <= maxX)
                    sol2_x_domain = true;
                else
                    sol2_x_domain = false;
                if (minY <= sol1_y && sol1_y <= maxY)
                    sol1_y_range = true;
                else
                    sol1_y_range = false;
                if (minY <= sol2_y && sol2_y <= maxY)
                    sol2_y_range = true;
                else
                    sol2_y_range = false;

                bool sol1, sol2;
                if (sol1_x_domain && sol1_y_range)
                    sol1 = true;
                else
                    sol1 = false;
                if (sol2_x_domain && sol2_y_range)
                    sol2 = true;
                else
                    sol2 = false;

                if (sol1 || sol2)
                {
                    intersectFound = true;

                    // if both solutions are in range, check which one is better
                    if (sol1 && sol2)
                    {
                        // make the decision by comparing the distance between the intersections and the next point in path
                        if (hypLength(sol1_x, sol1_y, path1.x[i + 1], path1.y[i + 1]) < hypLength(sol2_x, sol2_y, path1.x[i + 1], path1.y[i + 1]))
                        {
                            // printf("sol1 better\n");
                            goal_x = sol1_x, goal_y = sol1_y;
                        }
                        else
                        {
                            // printf("sol2 better\n");
                            goal_x = sol2_x, goal_y = sol2_y;
                        }
                    }
                    // if not both solutions are in range, take the one that's in range
                    else
                    {
                        // if solution pt1 is in range, set that as goal point
                        if (sol1)
                        {
                            // printf("just sol1\n");
                            goal_x = sol1_x, goal_y = sol1_y;
                            // printf("(%.2f,%.2f)\n", goal_x, goal_y);
                        }
                        else
                        {
                            // printf("just sol2\n");
                            goal_x = sol2_x, goal_y = sol2_y;
                            // printf("(%.2f,%.2f)\n", goal_x, goal_y);
                        }
                    }

                    // only exit loop if the solution pt found is closer to the next pt in path than the current pos
                    if (hypLength(goal_x, goal_y, path1.x[i + 1], path1.y[i + 1]) < hypLength(currentX, currentY, path1.x[i + 1], path1.y[i + 1]))
                    {
                        // printf("sol better\n");
                        //   update lastFoundIndex and exit
                        bot.x_target = goal_x;
                        bot.y_target = goal_y;
                        last_found_index = i;
                        // printf("(%.2f,%.2f)\n", path1.x[last_found_index], path1.y[last_found_index]);
                        // printf("(%.2f,%.2f)\n", path1.x[last_found_index + 1], path1.y[last_found_index + 1]);

                        break;
                    }
                    else
                    {
                        // in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
                        // printf("no intersect\n");
                        last_found_index = i + 1;
                        goal_x = path1.x[last_found_index];
                        goal_y = path1.y[last_found_index];
                    }
                }
                // if no solutions are in range
            }
            else
            {
                intersectFound = false;
                // no new intersection found, potentially deviated from the path
                // follow path[lastFoundIndex]
                goal_x = path1.x[last_found_index], goal_y = path1.y[last_found_index];
            }
            //std::cout << std::flush;
        }
        //printf("(%.2f,%.2f)\n", goal_x, goal_y);
    }


    void ppsFollowPath() {
            if (bot.point_distance(bot.x, bot.y, path1.x[path1.fidelity], path1.y[path1.fidelity]) > look_ahead_dist)
        {
            searchForIntersect();
            bot.x_target = goal_x, bot.y_target = goal_y;
        }
        else
        {
            bot.x_target = path1.x[path1.fidelity], bot.y_target = path1.y[path1.fidelity];
        }
    }


};

extern line_intersection get_line;