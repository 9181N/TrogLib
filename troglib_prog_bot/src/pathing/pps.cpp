#include "vex.h"
#include "drive_movement/pathing/bezier_curves.h"
#include "sensor_data.h"
class line_intersection {

  int last_found_index = 0;
  float goal_x = 0, goal_y = 0;

int signum(float input) { // returns the sign of an input
  if (input > 0.0) return 1;
  if (input < 0.0) return -1;
  return 0;
}

float min(float first, float second) {
  if (first < second) { 
    return first; 
      } else {
    return second;
  }
}

float max(float first, float second) {
  if (first > second) { 
    return first; 
      } else {
    return second;
  }
}

float hypLength(float x1, float y1, float x2, float y2) { // finds the length of a line drawn between two points
float X = x1 - x2;
float Y = y1 - y2;
float dist = sqrt(X * X + Y * Y);
return dist;
}

  void goal_pt_search(float look_ahead_dist) {
    float x1, y1, x2, y2, dx, dy, dr, D, discriminant;
    float sol_x1, sol_y1, sol_x2, sol_y2, sol_pt1_x, sol_pt1_y, sol_pt2_x, sol_pt2_y, min_x, min_y, max_x, max_y;

    bool intersect_found = false;
    int starting_index = last_found_index;

    for (int i = starting_index; starting_index < i < (path1.fidelity - 1); i++) {
      //beginning of line-circle intersection code
        x1 = path1.x[i] - bot.x;
        y1 = path1.y[i] - bot.y;
        x2 = path1.x[i+1] - bot.x;
        y2 = path1.y[i+1] - bot.y;
        dx = x2 - x1;
        dy = y2 - y1;
        dr = sqrt(dx*dx + dy*dy);
        D = x1*y2 - x2*y1;
        discriminant = (look_ahead_dist*look_ahead_dist) * (dr*dr) - D*D;

         if (discriminant >= 0) {
            sol_x1 = (D * dy + signum(dy) * dx * sqrt(discriminant)) / dr*dr;
            sol_x2 = (D * dy - signum(dy) * dx * sqrt(discriminant)) / dr*dr;
            sol_y1 = (- D * dx + fabs(dy) * sqrt(discriminant)) / dr*dr;
            sol_y2 = (- D * dx - fabs(dy) * sqrt(discriminant)) / dr*dr;
            
            sol_pt1_x  =sol_x1 + bot.x;
            sol_pt1_y  =sol_y1 + bot.y;
            sol_pt2_x  =sol_x2 + bot.x;
            sol_pt2_y  =sol_y2 + bot.y;
            // # end of line-circle intersection code
            
            min_x = min(path1.x[i], path1.x[i+1]);
            min_y = min(path1.y[i], path1.y[i+1]);
            max_x = max(path1.x[i], path1.x[i+1]);
            max_y = max(path1.y[i], path1.y[i+1]);
         

             //# if one or both of the solutions are in range
            if (((min_x <= sol_pt1_x <= max_x) and (min_y <= sol_pt1_y <= max_y)) or ((min_x <= sol_pt2_x <= max_x) and (max_y <= sol_pt2_y <= max_y))) {
            
                intersect_found = true;
            
                //# if both solutions are in range, check which one is better
                if (((min_x <= sol_pt1_x <= max_x) and (min_y <= sol_pt1_y <= max_y)) and ((min_x <= sol_pt2_x <= max_x) and (max_y <= sol_pt2_y <= max_y))) {
                    //# make the decision by comparing the distance between the intersections and the next point in path
                    if (hypLength(sol_pt1_x, sol_pt1_y, path1.x[i+1], path1.y[i+1]) < hypLength(sol_pt2_x, sol_pt2_y, path1.x[i+1], path1.y[i+1])) {
                        goal_x = sol_pt1_x;
                        goal_y = sol_pt1_y;
                   } else {
                        goal_x = sol_pt2_x;
                        goal_y = sol_pt2_y;
                   }
               // # if not both solutions are in range, take the one that's in range
               } else {
                   // # if solution pt1 is in range, set that as goal point
                    if ((min_x <= sol_pt1_x <= max_x) and (min_y <= sol_pt1_y <= max_y)) {
                        goal_x = sol_pt1_x;
                        goal_y = sol_pt1_y;
                  } else {
                        goal_x = sol_pt2_x;
                        goal_y = sol_pt2_y;
                    }
                }

                   //# only exit loop if the solution pt found is closer to the next pt in path than the current pos
                if ((hypLength(goal_x, goal_y, path1.x[i+1], path1.y[i+1])) < (hypLength(bot.x, bot.y, path1.x[i+1], path1.y[i+1]))) {
              //# update lastFoundIndex and exit
                    last_found_index = i;
                    break;
                }
      
                else {
                   // # in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
                    last_found_index = i+1;
                }     

                    
            //# if no solutions are in range
             } else {
                    intersect_found = false;
                    //# no new intersection found, potentially deviated from the path
                    //# follow path[lastFoundIndex]
                    int test =last_found_index;
                    goal_x = path1.x[test];
                    goal_y = path1.y[last_found_index];

            }
        }
    }
  }
};

extern line_intersection get_line;