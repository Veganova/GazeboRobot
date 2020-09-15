
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

const double detection_angle = 0.7;
const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;

double prev_x, prev_y, prev_heading = 0;

double
get_heading(Robot* robot)
{
    double dx_heading = robot->pos_x - prev_x;
    double dy_heading = robot->pos_y - prev_y;
    // no change in previous and current state
    if (dx_heading == 0 && dy_heading == 0) return prev_heading;

    double dx_goal = goal_x - robot->pos_x;
    double dy_goal = goal_y - robot->pos_y;

    prev_heading = atan2(dx_heading * dy_goal - dy_heading * dx_goal, dx_heading * dx_goal + dy_goal * dy_heading);
    return prev_heading;
}

void
callback(Robot* robot)
{
    double dx = goal_x - robot->pos_x;
    double dy = goal_y - robot->pos_y;
    if (abs(dx) < 0.75 && abs(dy) < 0.75) {
        cout << "we win!" << endl;
        robot->set_vel(0.0);
        robot->set_turn(0.0);
        robot->done();
        return;
    }

    bool reverse = false;
    double obstacle_boundary_left = 0;
    double obstacle_boundary_right = 0;

    for (LaserHit hit : robot->hits) {
        if (hit.range < 3) {
            if (abs(hit.angle) < detection_angle) {
                if (hit.angle > obstacle_boundary_left) {
                    obstacle_boundary_left = hit.angle;
                } else if (hit.angle < obstacle_boundary_right) {
                    obstacle_boundary_right = hit.angle;
                }

                if (hit.range < 0.3) {
                    reverse = true;
                }
            }
        }
    }

    robot->set_vel(10);

    if (abs(obstacle_boundary_left) < 0.00001 && abs(obstacle_boundary_right) < 0.00001) {
        // no obstacle
        robot->set_turn(-get_heading(robot)/5.0);
    } else if (abs(obstacle_boundary_left) < abs(obstacle_boundary_right)) {
        robot->set_turn(-0.2);
    } else {
        robot->set_turn(0.2);
    }


    cout << "heading to target: " << get_heading(robot) << endl;
    cout << "ob left: " <<  obstacle_boundary_left << endl;
    cout << "ob right: " << obstacle_boundary_right << endl;
    cout << "--------------" << endl;

    prev_x = robot->pos_x;
    prev_y = robot->pos_y;
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    prev_x = robot.pos_x;
    prev_y = robot.pos_y;
    robot.do_stuff();

    return 0;
}
