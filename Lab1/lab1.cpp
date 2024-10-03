#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include "utils.h"
#include "render.h"
//===== Main parameters =====
const int width {800}, height {800}; //Width and height of the environment
const int radius {10}; //Radius of the robot's circular body
const int min_obj_size {50}; //Maximum object dimension
const int max_obj_size {100}; //Maximum object dimension
const int goal_width {100}; //Goal width
const int goal_height {100}; //Goal heigth
const int robot_tol {200}; //Tolerance for robot spawn point
const int occupancy_tol {50}; //Minimum distance between all objects that spawn
const int goal_tol {100}; //Minimum distance in x,y between robot and goal
const int robot_y_min {500}; //Minimum robot y position
const int goal_y_max {300}; //Maximum goal y position
int obj_x, obj_y, obj_width, obj_height; //Parameters for object position/size
int num_objects {0}; //Number of obstacles in environment
// Grid utility class. Students will not use this for lab 1
grid_util grid(width, height, min_obj_size, max_obj_size);
// Random generator to spawn robot and goal
random_generator rand_gen;
// Vector of robot positions to pass to renderer code. Update this after each time step!
std::vector<std::vector<int>> robot_pos;
// Did mission succeed? Update this to make sure it succeeds if robot reaches goal, failure if it hits wall.
bool succeed;


// Function definitions - Task 1
void detectBoundaryCrossing(const Object& robot, int width, int height, int radius, bool& succeed) {
    std::cout << "Checking boundary: Robot at (" << robot.x << ", " << robot.y << "), "
              << "Radius: " << radius << ", "
              << "Bounds: (" << width << ", " << height << ")" << std::endl;

    if (robot.x - radius < 0) {
        std::cout << "Error: Robot crossed left boundary! x = " << robot.x << std::endl;
        succeed = false;
    } else if (robot.x + radius > width) {
        std::cout << "Error: Robot crossed right boundary! x = " << robot.x << std::endl;
        succeed = false;
    } else if (robot.y - radius < 0) {
        std::cout << "Error: Robot crossed top boundary! y = " << robot.y << std::endl;
        succeed = false;
    } else if (robot.y + radius > height) {
        std::cout << "Error: Robot crossed bottom boundary! y = " << robot.y << std::endl;
        succeed = false;
    } else {
        std::cout << "Robot is within boundaries." << std::endl;
        succeed = true;
    }
}

/*
//Task 2 v1 -- DO NOT MARK
void detectGoalReached(const Object& robot, const Object& goal, int radius, bool& succeed) {
    int robotCenterX = robot.x + radius;
    int robotCenterY = robot.y + radius;

    int goalLeft = goal.x;
    int goalRight = goal.x + goal.width;
    int goalTop = goal.y;
    int goalBottom = goal.y + goal.height;

    std::cout << "Robot center: (" << robotCenterX << "," << robotCenterY << ")" << std::endl;
    std::cout << "Goal bounds: (" << goalLeft << "," << goalTop << ") to (" 
              << goalRight << "," << goalBottom << ")" << std::endl;

    if (robotCenterX >= goalLeft && robotCenterX <= goalRight &&
        robotCenterY >= goalTop && robotCenterY <= goalBottom) {
        //std::cout << "Success: Robot reached the goal!" << std::endl;
        succeed = true;
    } else {
        succeed = false;
        std::cout << "Robot has not reached the goal yet." << std::endl;
    }
}
*/

// Task 2 v2 - Mark
void detectGoalReached(const Object& robot, const Object& goal, int radius, bool& succeed) {
    // Calculate robot corners
    int robotLeft = robot.x - radius;
    int robotRight = robot.x + radius;
    int robotTop = robot.y - radius;
    int robotBottom = robot.y + radius;

    // Calculate goal boundaries
    int goalLeft = goal.x;
    int goalRight = goal.x + goal.width;
    int goalTop = goal.y;
    int goalBottom = goal.y + goal.height;

    // Debug output
    std::cout << "Robot bounds: (" << robotLeft << "," << robotTop << ") to ("
              << robotRight << "," << robotBottom << ")" << std::endl;
    std::cout << "Goal bounds: (" << goalLeft << "," << goalTop << ") to ("
              << goalRight << "," << goalBottom << ")" << std::endl;

    // Check for overlap in both x and y directions
    bool overlapX = (robotLeft <= goalRight) && (robotRight >= goalLeft);
    bool overlapY = (robotTop <= goalBottom) && (robotBottom >= goalTop);

    if (overlapX && overlapY) {
        std::cout << "Success: Robot has reached the goal!" << std::endl;
        succeed = true;
    } else {
        succeed = false;
        std::cout << "Robot has not reached the goal yet." << std::endl;
    }
}

// Task3
void moveRobotTask3(Object& robot, const Object& goal) {
    int target_x = goal.x;
    int target_y = goal.y;
    int dx = 0, dy = 0;

    // Move in x direction first
    if (robot.x < target_x) {
        dx = 1;
    } else if (robot.x > target_x) {
        dx = -1;
    }
    // If x is aligned, move in y direction
    else if (robot.y > target_y) {
        dy = -1;
    } else if (robot.y < target_y) {
        dy = 1;
    }

    // Move robot
    robot.x += dx;
    robot.y += dy;

    std::cout << "Robot moved to (" << robot.x << ", " << robot.y << ")" << std::endl;
    std::cout << "Target: (" << target_x << ", " << target_y << ")" << std::endl;
    std::cout << "Movement: dx=" << dx << ", dy=" << dy << std::endl;
}

//Task 4
void moveRobotTask4(Object& robot, const Object& goal) {
    int target_x = goal.x;
    int target_y = goal.y;
    int dx = 0, dy = 0;

    // Move in y direction first
    if (robot.y > target_y) {
        dy = -1;
    } else if (robot.y < target_y) {
        dy = 1;
    }
    // If y is aligned, move in x direction
    else if (robot.x < target_x) {
        dx = 1;
    } else if (robot.x > target_x) {
        dx = -1;
    }

    // Move robot
    robot.x += dx;
    robot.y += dy;

    std::cout << "Robot moved to (" << robot.x << ", " << robot.y << ")" << std::endl;
    std::cout << "Target: (" << target_x << ", " << target_y << ")" << std::endl;
    std::cout << "Movement: dx=" << dx << ", dy=" << dy << std::endl;
}

void moveRobotTask5(Object& robot, const Object& goal) {
    int dx = goal.x - robot.x;
    int dy = goal.y - robot.y;
    
    // Determine the direction of movement
    int move_x = (dx != 0) ? ((dx > 0) ? 1 : -1) : 0;
    int move_y = (dy != 0) ? ((dy > 0) ? 1 : -1) : 0;
    
    // Move the robot
    robot.x += move_x;
    robot.y += move_y;

    std::cout << "Robot moved to (" << robot.x << ", " << robot.y << ")" << std::endl;
    std::cout << "Goal position: (" << goal.x << ", " << goal.y << ")" << std::endl;
    std::cout << "Distance to goal: dx=" << dx << ", dy=" << dy << std::endl;
}

void moveRobotTask6(Object& robot, const Object& goal) {
    double centerX = goal.x + goal.width / 2.0;
    double centerY = goal.y + goal.height / 2.0;
    double dx = centerX - robot.x;
    double dy = centerY - robot.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    robot.x += std::round(dx / distance);
    robot.y += std::round(dy / distance);
}


std::pair<int, int> findClosestCorner(const Object& robot, const Object& goal) {
    std::vector<std::pair<int, int>> corners = {
        {goal.x, goal.y},
        {goal.x + goal.width, goal.y},
        {goal.x, goal.y + goal.height},
        {goal.x + goal.width, goal.y + goal.height}
    };

    std::pair<int, int> closest = corners[0];
    double minDist = std::numeric_limits<double>::max();

    for (const auto& corner : corners) {
        double dist = std::sqrt(std::pow(corner.first - robot.x, 2) + std::pow(corner.second - robot.y, 2));
        if (dist < minDist) {
            minDist = dist;
            closest = corner;
        }
    }

    return closest;
}

void moveRobotTask7(Object& robot, const Object& goal) {
    static std::pair<int, int> targetCorner = findClosestCorner(robot, goal);
    double dx = targetCorner.first - robot.x;
    double dy = targetCorner.second - robot.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    robot.x += std::round(dx / distance);
    robot.y += std::round(dy / distance);
}

int main(int argc, char const argv[])
{

//==========CREATE ROBOT, GOAL, OBJECTS==========
// create robot
Object robot = grid.create_object(grid, rand_gen, robot_tol, 2*radius, 2*radius, robot_y_min, height-radius, 1, "robot");
// create the goal
Object goal = grid.create_object(grid, rand_gen, goal_tol, goal_width, goal_height, 0, goal_y_max, 3, "goal");
// create the objects
std::vector<Object> objects = grid.create_objects(rand_gen, occupancy_tol, num_objects);
// create copies of robot and goal with their initial positions for purpose of render functions
Object robot_init = robot;
Object goal_init = goal;


// uncomment this line to write the grid to csv to see the grid as a csv
// grid.writeGridToCSV("grid.csv");

// place the first robot position to robot_pos
robot_pos.push_back({robot.x, robot.y});

// Calculate initial direction vector - helps with task 3
double dx = goal.x - robot.x;
double dy = goal.y - robot.y;
double distance = std::sqrt(dx*dx + dy*dy);
double vx = dx / distance;
double vy = dy / distance;



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++WRITE YOUR CODE HERE++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


bool needToOvershoot = false;
bool overshot = false;

 while (true)
    {
        std::cout << "Before move - Robot: (" << robot.x << ", " << robot.y << ")" << std::endl;
        std::cout << "Goal: (" << goal.x << ", " << goal.y << ")" << std::endl;

        moveRobotTask4(robot, goal);  // or whichever task you're testing

        std::cout << "After move - Robot: (" << robot.x << ", " << robot.y << ")" << std::endl;

        
        //moveRobotTask3(robot, goal); // Task 3
        moveRobotTask4(robot, goal);  // Task 4
        //moveRobotTask5(robot, goal);  // Task 5
        //moveRobotTask6(robot, goal);  // Task 6
        //moveRobotTask7(robot, goal);  // Task 7

        // Task 1: Detect boundary crossing
        detectBoundaryCrossing(robot, width, height, radius, succeed);
        if (!succeed) {
            break;
        }

        // Task 2: Detect if robot reached the goal
        detectGoalReached(robot, goal, radius, succeed);
        if (succeed) {
            //std::cout << "Success: Robot reached the goal!" << std::endl;
            break;

        }

    
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++END YOUR CODE HERE++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 // place the current robot position at the time step to robot_pos
robot_pos.push_back({robot.x, robot.y});
 }
 
 // send the results of the code to the renderer
render_window(robot_pos, objects, robot_init, goal_init, width, height, succeed);
return 0;
}