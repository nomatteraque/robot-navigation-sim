#pragma once

#include <vector>
#include <SFML/Graphics.hpp>
#include "common.hpp"

// Verifies if a direct line segment between two points is collision-free (outside warning zones).
bool HasLineOfSight(sf::Vector2f p1, sf::Vector2f p2, const std::vector<Obstacle>& obstacles);

// Prunes redundant waypoints from a path if there is a direct line-of-sight shortcut.
std::vector<sf::Vector2f> PrunePath(const std::vector<sf::Vector2f>& path, const std::vector<Obstacle>& obstacles);

// Rounds off sharp polygonal corners using a local Bezier-like Bezier approximation.
std::vector<sf::Vector2f> SmoothPath(const std::vector<sf::Vector2f>& path, const std::vector<Obstacle>& obstacles);

// Plans a path using A* search over a grid, incorporating soft costs for warning zones and obstacles.
std::vector<sf::Vector2f> PlanAStarPath(
    double start_x, double start_y, 
    double goal_x, double goal_y, 
    const std::vector<Obstacle>& obstacles);
