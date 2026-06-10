#pragma once

#include <vector>
#include <SFML/Graphics.hpp>
#include "common.hpp"

// Computes the active lookahead point on the path for the Pure Pursuit tracking controller.
sf::Vector2f GetLookaheadPoint(
    const std::vector<sf::Vector2f>& path, 
    const Robot& robot, 
    double L, 
    size_t& last_index);

// Executes the physics and PD controller update for the Reactive Robot (Green).
void UpdateReactiveRobot(
    Robot& robot,
    double target_x, double target_y,
    std::vector<Obstacle>& obstacles,
    double dt,
    double& integral_error_x, double& integral_error_y,
    double& filtered_d_error_x, double& filtered_d_error_y,
    double& last_noisy_vx, double& last_noisy_vy);

// Executes the physics, Pure Pursuit, and velocity control update for the Planned Robot (Blue).
void UpdatePlannedRobot(
    Robot& planned_robot,
    double target_x, double target_y,
    const std::vector<sf::Vector2f>& astar_path,
    double dt,
    size_t& lookahead_index,
    sf::Vector2f& planned_lookahead_pt,
    double& filtered_planned_vx, double& filtered_planned_vy,
    double& filtered_target_vx, double& filtered_target_vy);
