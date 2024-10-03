#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include "utils.h"
#include "render.h"

//===== Main parameters =====
const int width {800}, height {800};        // Width and height of the environment
const int radius {10};                      // Radius of the robot's circular body
const int min_obj_size {50};                // Minimum object dimension
const int max_obj_size {100};               // Maximum object dimension
const int goal_width {100};                 // Goal width
const int goal_height {100};                // Goal height
const int robot_tol {200};                  // Tolerance for robot spawn point
const int occupancy_tol {50};               // Minimum distance between all objects that spawn
const int goal_tol {100};                   // Minimum distance in x,y between robot and goal
const int robot_y_min {500};                // Minimum robot y position
const int goal_y_max {300};                 // Maximum goal y position
int num_objects {15};                       // Number of objects in environment

// Grid utility class
grid_util grid(width, height, min_obj_size, max_obj_size);

// Random generator
random_generator rand_gen;

// Vector of velocity commands
std::vector<std::vector<int>> robot_pos;

// Did mission succeed?
bool succeed = false;

// Check to see if it collides with the goal
bool is_goal_detected(const Object& robot, const Object& goal) {
    return !(robot.x + robot.width <= goal.x ||
             robot.x >= goal.x + goal_width ||
             robot.y + robot.height <= goal.y ||
             robot.y >= goal.y + goal_height);
}

// Assuming each grid cell represents 1 pixel
const int grid_cell_width = 1;
const int grid_cell_height = 1;

// This function checks for collisions by looking at the robot's corners and edges on the grid
bool is_collision(Object robot, grid_util &grid) {
    int robot_width = robot.width;
    int robot_height = robot.height;

    // Convert robot's position to grid coordinates
    int grid_top_left_x = robot.x / grid_cell_width;
    int grid_top_left_y = robot.y / grid_cell_height;
    
    int grid_top_right_x = (robot.x + robot_width) / grid_cell_width;
    int grid_top_right_y = grid_top_left_y;
    
    int grid_bottom_left_x = grid_top_left_x;
    int grid_bottom_left_y = (robot.y + robot_height) / grid_cell_height;
    
    int grid_bottom_right_x = (robot.x + robot_width) / grid_cell_width;
    int grid_bottom_right_y = grid_bottom_left_y;

    // Check multiple points along the edges of the robot's bounding box
    for (int x = grid_top_left_x; x <= grid_top_right_x; ++x) {
        // Check the top and bottom edges
        if (grid.grid[x][grid_top_left_y] == 2 || grid.grid[x][grid_bottom_left_y] == 2) {
            std::cout << "Collision detected at (" << x << ", " << grid_top_left_y << ") or (" << x << ", " << grid_bottom_left_y << ")" << std::endl;
            return true;
        }
    }

    for (int y = grid_top_left_y; y <= grid_bottom_left_y; ++y) {
        // Check the left and right edges
        if (grid.grid[grid_top_left_x][y] == 2 || grid.grid[grid_top_right_x][y] == 2) {
            std::cout << "Collision detected at (" << grid_top_left_x << ", " << y << ") or (" << grid_top_right_x << ", " << y << ")" << std::endl;
            return true;
        }
    }

    // If no collision detected, return false
    return false;
}

// Obstacle avoidance function (Task 2) with smaller step sizes
void obstacle_avoidance(Object& robot, const Object& goal, grid_util& grid, bool moving_x) {
    bool obstacle_cleared = false;
    
    // Move perpendicular to current movement direction until the robot clears the obstacle
    while (is_collision(robot, grid)) {
        // Move in smaller steps to avoid skipping over obstacles
        int step_size = 1;

        if (moving_x) {
            // If moving in x, shift in y to clear the obstacle
            robot.y += (goal.y > robot.y) ? step_size : -step_size;
        } else {
            // If moving in y, shift in x to clear the obstacle
            robot.x += (goal.x > robot.x) ? step_size : -step_size;
        }

        // Update the robot's position vector for rendering
        robot_pos.push_back({robot.x, robot.y});

        // If no more collisions are detected, mark the obstacle as cleared
        if (!is_collision(robot, grid)) {
            obstacle_cleared = true;
        }
    }

    // Ensure the robot has moved sufficiently away from the obstacle before recalculating the path
    if (obstacle_cleared) {
        std::cout << "Obstacle cleared! Recalculating path towards the goal." << std::endl;
        robot_pos.push_back({robot.x, robot.y});  // Force update after obstacle clearance
    }
}

// Task 3 movement logic (x direction first)
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

    // Update the robot's position after movement
    robot_pos.push_back({robot.x, robot.y});
}

// Task 4 movement logic (y direction first)
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

    // Update the robot's position after movement
    robot_pos.push_back({robot.x, robot.y});
}

int main(int argc, char const *argv[])
{
    // Create robot, goal, and objects
    Object robot = grid.create_object(grid, rand_gen, robot_tol, 2*radius, 2*radius, robot_y_min, height-radius, 1, "robot");
    Object goal = grid.create_object(grid, rand_gen, goal_tol, goal_width, goal_height, 0, goal_y_max, 3, "goal");
    std::vector<Object> objects = grid.create_objects(rand_gen, occupancy_tol, num_objects);

    Object robot_init = robot;
    Object goal_init = goal;

    robot_pos.push_back({robot.x, robot.y});

    std::cout << "Starting main loop" << std::endl;

    int max_count = 0;

    // Main loop using Task 3 logic and improved obstacle avoidance
    while (true) {
        // Move the robot using Task 3 logic (x direction first)
        moveRobotTask3(robot, goal);
        
        // Check for collision after each movement
        if (is_collision(robot, grid)) {
            std::cout << "Collision detected! Avoiding obstacle." << std::endl;
            obstacle_avoidance(robot, goal, grid, true);  // Pass the goal to obstacle_avoidance
        }

        // Add the robot's new position to robot_positions
        robot_pos.push_back({robot.x, robot.y});

        // Check if the robot has reached the goal
        if (is_goal_detected(robot, goal)) {
            succeed = true;
            std::cout << "Success! Goal reached!" << std::endl;
            break;
        }

        max_count++;
        if (max_count >= 3600) {
            std::cout << "=====1 minute reached with no solution=====" << std::endl;
            break;
        }

        if (max_count % 100 == 0) {
            std::cout << "Iteration " << max_count << ": Robot at (" << robot.x << ", " << robot.y << ")" << std::endl;
        }
    }

    // Render and complete
    render_window(robot_pos, objects, robot_init, goal_init, width, height, succeed);

    return 0;
}
