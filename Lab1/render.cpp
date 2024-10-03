#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

// #include "drawobjects.h"
#include "utils.h"

// Text colours
const std::string RED = "\033[31m";   // Red text
const std::string GREEN = "\033[32m"; // Green text
const std::string RESET = "\033[0m";  // Reset to default

sf::RectangleShape draw_object(int x, int y, int width, int height) {
    sf::RectangleShape obj(sf::Vector2f(width, height));
    sf::Vector2f linePosition(x, y);
    obj.setPosition(linePosition);
    return obj;  
}

void render_window(
    std::vector<std::vector<int>> robot_pos,
    std::vector<Object> objects, 
    Object robot, 
    Object goal, 
    int width, 
    int height,
    bool succeed)
{
    int del_x, del_y, vel_x, vel_y;

    //==========SFML==========

    // retrieve screen resolution to center the window
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();

    int top_left_x = (desktop.width/2) - (width/2);
    int top_left_y = (desktop.height/2) - (height/2);

    // create the window
    sf::RenderWindow window(sf::VideoMode(width, height), "MTE301 Lab 1");
    window.setFramerateLimit(60);
    window.setPosition(sf::Vector2i(top_left_x, top_left_y));

    // create the robot
    sf::CircleShape robot_draw(robot.width/2);
    robot_draw.setFillColor(sf::Color::Blue);
    sf::Vector2f robotPosition(robot.x, robot.y);
    robot_draw.setPosition(robotPosition);

    sf::RectangleShape goal_draw(sf::Vector2f(goal.width, goal.height));
    sf::Vector2f goalPosition(goal.x, goal.y);
    
    goal_draw.setPosition(goalPosition);
    goal_draw.setFillColor(sf::Color::Green);    

    std::vector<sf::RectangleShape> objects_draw;
    // Now spawn the objects in SFML. 
    // For some reason size() returns vector size * # struct members, so divide by # struct members (4)
    for (int i = 0; i < objects.size(); i++) {
        objects_draw.push_back(draw_object(objects[i].x, objects[i].y, objects[i].width, objects[i].height));
    }

    std::vector<sf::RectangleShape>::iterator i;
    int count = 0;
    bool isPaused = false; // State to track whether the game is paused

    // run the program as long as the window is open
    while (window.isOpen())
    {

        // Error handling for index out of bounds events
        if (count < robot_pos.size() && robot_pos[count].size() >= 2) {
            robotPosition.x = robot_pos[count][0]; // Safe access
            robotPosition.y = robot_pos[count][1]; // Safe access
        } else {
            if (count < 200) {
                std::cerr << "Error: Accessing out of bounds for robot_pos at count: " << count << std::endl;
                // Handle error appropriately (e.g., skip iteration, set defaults, etc.)
            }
        }

        // check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        while (window.pollEvent(event))
        {
            // "close requested" event: we close the window
            if(event.type == sf::Event::Closed){
                window.close();
            }
        }

        // clear the window
        window.clear();

        // Drawing operations
        robot_draw.setPosition(robotPosition);
        window.draw(robot_draw);
        window.draw(goal_draw);
        // window.draw(line);
        for (i = objects_draw.begin(); i != objects_draw.end(); ++i){
            window.draw(*i);
        }

        if ((count >= robot_pos.size()-1) && succeed) {
            std::cout << GREEN << "Success! Goal reached!" << RESET << std::endl;
            window.close();
        }
        if ((count >= robot_pos.size()-1) && !succeed) {
            std::cout << RED << "Failure! Collision!" << RESET << std::endl;
            window.close();
        }

        // end the current frame
        window.display();
        count++;
        
    }
}
