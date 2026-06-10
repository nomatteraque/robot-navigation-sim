#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <optional>
#include <algorithm>
#include "common.hpp"
#include "pathfinder.hpp"
#include "controller.hpp"

int main() {
    // Window configuration (1200x900)
    const unsigned int screen_width = 1200;
    const unsigned int screen_height = 900;
    sf::RenderWindow window(sf::VideoMode({screen_width, screen_height}), "Robot Control Simulation - Modular (SFML)");
    window.setFramerateLimit(60);

    // Font loading for HUD
    sf::Font font;
    if (!font.openFromFile("C:/Windows/Fonts/arial.ttf")) {
        std::cerr << "Warning: Could not load Windows default Arial font. Text HUD will be disabled." << std::endl;
    }

    // Kinematic states (Reactive Robot 1 = Green, Planned Robot 2 = Blue)
    Robot robot{ 0.0, 0.0, 0.0, 0.0 };
    Robot planned_robot{ 0.0, 0.0, 0.0, 0.0 };

    double target_x = 18.0;
    double target_y = 8.0;

    // Scattered obstacle layout
    std::vector<Obstacle> obstacles = {
        { 5.0, 4.0, 2.2, 0 },
        { 9.0, 8.5, 2.0, 0 },
        { 10.0, 3.0, 2.0, 0 },
        { 14.0, 10.0, 2.2, 0 },
        { 15.0, 4.5, 2.0, 0 }
    };

    double dt = 0.1;

    // Reactive Robot controller states
    double integral_error_x = 0.0;
    double integral_error_y = 0.0;
    double filtered_d_error_x = 0.0;
    double filtered_d_error_y = 0.0;
    double last_noisy_vx = 0.0;
    double last_noisy_vy = 0.0;

    // Planned Robot velocity loop and smoothing states
    double filtered_planned_vx = 0.0;
    double filtered_planned_vy = 0.0;
    double filtered_target_vx = 0.0;
    double filtered_target_vy = 0.0;

    // Trail trackers
    std::vector<sf::Vector2f> path_history;
    path_history.push_back(sf::Vector2f(0.f, 0.f));

    std::vector<sf::Vector2f> planned_path_history;
    planned_path_history.push_back(sf::Vector2f(0.f, 0.f));

    // Pathfinder setup
    std::vector<sf::Vector2f> astar_path = PlanAStarPath(planned_robot.x, planned_robot.y, target_x, target_y, obstacles);
    size_t lookahead_index = 0;
    sf::Vector2f planned_lookahead_pt = GetLookaheadPoint(astar_path, planned_robot, 1.2, lookahead_index);

    // Screen-to-Simulation coordinate mapping settings
    const double scale = 50.0;
    const double origin_x = 100.0;
    const double origin_y = 800.0;

    auto SimToScreen = [&](double x, double y) -> sf::Vector2f {
        return sf::Vector2f(
            (float)(origin_x + x * scale),
            (float)(origin_y - y * scale) // Invert Y-axis
        );
    };

    auto ScreenToSim = [&](float px, float py) -> sf::Vector2f {
        return sf::Vector2f(
            (float)((px - origin_x) / scale),
            (float)((origin_y - py) / scale)
        );
    };

    // Right-click drag-and-drop obstacle select tracker
    Obstacle* selected_obstacle = nullptr;

    while (window.isOpen()) {
        while (const std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            // Left-click to assign target, Right-click to grab obstacle
            if (const auto* mouseButton = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (mouseButton->button == sf::Mouse::Button::Left) {
                    sf::Vector2f click_sim = ScreenToSim((float)mouseButton->position.x, (float)mouseButton->position.y);
                    sf::Vector2f safe_target = ProjectTargetToSafe(click_sim.x, click_sim.y, obstacles);
                    target_x = safe_target.x;
                    target_y = safe_target.y;

                    // Reset robot race kinematics
                    robot = Robot{ 0.0, 0.0, 0.0, 0.0 };
                    planned_robot = Robot{ 0.0, 0.0, 0.0, 0.0 };

                    path_history.clear();
                    path_history.push_back(sf::Vector2f(0.f, 0.f));
                    planned_path_history.clear();
                    planned_path_history.push_back(sf::Vector2f(0.f, 0.f));

                    integral_error_x = 0.0;
                    integral_error_y = 0.0;
                    filtered_d_error_x = 0.0;
                    filtered_d_error_y = 0.0;
                    last_noisy_vx = 0.0;
                    last_noisy_vy = 0.0;

                    filtered_planned_vx = 0.0;
                    filtered_planned_vy = 0.0;
                    filtered_target_vx = 0.0;
                    filtered_target_vy = 0.0;

                    for (auto& obs : obstacles) {
                        obs.vortex_direction = 0;
                    }

                    astar_path = PlanAStarPath(planned_robot.x, planned_robot.y, target_x, target_y, obstacles);
                    lookahead_index = 0;
                    planned_lookahead_pt = GetLookaheadPoint(astar_path, planned_robot, 1.2, lookahead_index);

                } else if (mouseButton->button == sf::Mouse::Button::Right) {
                    sf::Vector2f click_sim = ScreenToSim((float)mouseButton->position.x, (float)mouseButton->position.y);
                    for (auto& obs : obstacles) {
                        if (distance(click_sim.x, click_sim.y, obs.x, obs.y) < obs.radius) {
                            selected_obstacle = &obs;
                            break;
                        }
                    }
                }
            }

            // Right-click release to release obstacle
            if (const auto* mouseButton = event->getIf<sf::Event::MouseButtonReleased>()) {
                if (mouseButton->button == sf::Mouse::Button::Right) {
                    selected_obstacle = nullptr;
                }
            }

            // Mouse movement drag update and dynamic pathfinding replan
            if (const auto* mouseMove = event->getIf<sf::Event::MouseMoved>()) {
                if (selected_obstacle != nullptr) {
                    sf::Vector2f new_pos = ScreenToSim((float)mouseMove->position.x, (float)mouseMove->position.y);
                    selected_obstacle->x = std::clamp((double)new_pos.x, 0.0, 20.0);
                    selected_obstacle->y = std::clamp((double)new_pos.y, 0.0, 14.0);

                    // Adjust target coordinates dynamically if an obstacle is dragged over the current target
                    sf::Vector2f safe_target = ProjectTargetToSafe(target_x, target_y, obstacles);
                    target_x = safe_target.x;
                    target_y = safe_target.y;

                    astar_path = PlanAStarPath(planned_robot.x, planned_robot.y, target_x, target_y, obstacles);
                    lookahead_index = 0;
                    planned_lookahead_pt = GetLookaheadPoint(astar_path, planned_robot, 1.2, lookahead_index);
                }
            }
        }

        // 1. Controller and Kinematics Updates
        UpdateReactiveRobot(
            robot, target_x, target_y, obstacles, dt,
            integral_error_x, integral_error_y,
            filtered_d_error_x, filtered_d_error_y,
            last_noisy_vx, last_noisy_vy
        );

        UpdatePlannedRobot(
            planned_robot, target_x, target_y, astar_path, dt,
            lookahead_index, planned_lookahead_pt,
            filtered_planned_vx, filtered_planned_vy,
            filtered_target_vx, filtered_target_vy
        );

        // Record coordinates for paths
        sf::Vector2f last_r_pos = path_history.back();
        if (distance(robot.x, robot.y, last_r_pos.x, last_r_pos.y) > 0.05) {
            path_history.push_back(sf::Vector2f((float)robot.x, (float)robot.y));
        }

        sf::Vector2f last_p_pos = planned_path_history.back();
        if (distance(planned_robot.x, planned_robot.y, last_p_pos.x, last_p_pos.y) > 0.05) {
            planned_path_history.push_back(sf::Vector2f((float)planned_robot.x, (float)planned_robot.y));
        }

        // 2. SFML Screen Rendering
        window.clear(sf::Color(15, 15, 35));

        // Draw coordinate grid lines (20x14 cells)
        for (int x = 0; x <= 20; ++x) {
            sf::Vertex line[] = {
                sf::Vertex{SimToScreen(x, 0), sf::Color(50, 50, 100)},
                sf::Vertex{SimToScreen(x, 14), sf::Color(50, 50, 100)}
            };
            window.draw(line, 2, sf::PrimitiveType::Lines);
        }
        for (int y = 0; y <= 14; ++y) {
            sf::Vertex line[] = {
                sf::Vertex{SimToScreen(0, y), sf::Color(50, 50, 100)},
                sf::Vertex{SimToScreen(20, y), sf::Color(50, 50, 100)}
            };
            window.draw(line, 2, sf::PrimitiveType::Lines);
        }

        // Draw obstacle zones
        for (const auto& obs : obstacles) {
            sf::CircleShape warningZone((float)(obs.radius * scale));
            warningZone.setFillColor(sf::Color::Transparent);
            warningZone.setOutlineColor(sf::Color(150, 50, 50));
            warningZone.setOutlineThickness(1.0f);
            warningZone.setOrigin({(float)(obs.radius * scale), (float)(obs.radius * scale)});
            warningZone.setPosition(SimToScreen(obs.x, obs.y));
            window.draw(warningZone);

            sf::CircleShape obsCore(8.f);
            obsCore.setFillColor(sf::Color::Red);
            obsCore.setOrigin({8.f, 8.f});
            obsCore.setPosition(SimToScreen(obs.x, obs.y));
            window.draw(obsCore);
        }

        // Draw target point
        sf::CircleShape targetCircle(8.f);
        targetCircle.setFillColor(sf::Color::Blue);
        targetCircle.setOrigin({8.f, 8.f});
        targetCircle.setPosition(SimToScreen(target_x, target_y));
        window.draw(targetCircle);

        // Draw A* path line
        if (astar_path.size() > 1) {
            sf::VertexArray astarLine(sf::PrimitiveType::LineStrip, astar_path.size());
            for (size_t i = 0; i < astar_path.size(); ++i) {
                astarLine[i].position = SimToScreen(astar_path[i].x, astar_path[i].y);
                astarLine[i].color = sf::Color(0, 255, 255, 100);
            }
            window.draw(astarLine);
        }

        // Draw Pure Pursuit lookahead point
        double planned_dist = distance(planned_robot.x, planned_robot.y, target_x, target_y);
        if (!astar_path.empty() && planned_dist > 0.1) {
            sf::CircleShape lookaheadCircle(4.f);
            lookaheadCircle.setFillColor(sf::Color::Yellow);
            lookaheadCircle.setOrigin({4.f, 4.f});
            lookaheadCircle.setPosition(SimToScreen(planned_lookahead_pt.x, planned_lookahead_pt.y));
            window.draw(lookaheadCircle);
        }

        // Draw trails
        if (path_history.size() > 1) {
            sf::VertexArray lineStrip(sf::PrimitiveType::LineStrip, path_history.size());
            for (size_t i = 0; i < path_history.size(); ++i) {
                lineStrip[i].position = SimToScreen(path_history[i].x, path_history[i].y);
                lineStrip[i].color = sf::Color(100, 255, 100, 150);
            }
            window.draw(lineStrip);
        }

        if (planned_path_history.size() > 1) {
            sf::VertexArray lineStrip(sf::PrimitiveType::LineStrip, planned_path_history.size());
            for (size_t i = 0; i < planned_path_history.size(); ++i) {
                lineStrip[i].position = SimToScreen(planned_path_history[i].x, planned_path_history[i].y);
                lineStrip[i].color = sf::Color(255, 100, 255);
            }
            window.draw(lineStrip);
        }

        // Draw robots
        sf::CircleShape robotCircle(8.f);
        robotCircle.setFillColor(sf::Color::Green);
        robotCircle.setOrigin({8.f, 8.f});
        robotCircle.setPosition(SimToScreen(robot.x, robot.y));
        window.draw(robotCircle);

        sf::CircleShape plannedRobotCircle(8.f);
        plannedRobotCircle.setFillColor(sf::Color(100, 150, 255));
        plannedRobotCircle.setOrigin({8.f, 8.f});
        plannedRobotCircle.setPosition(SimToScreen(planned_robot.x, planned_robot.y));
        window.draw(plannedRobotCircle);

        // Draw HUD status outputs
        sf::Text text(font);
        text.setCharacterSize(16);
        text.setFillColor(sf::Color::White);

        text.setString("Robot Control Sim (PID - SFML)");
        text.setPosition({10.f, 10.f});
        window.draw(text);

        text.setString("Left-click targets | Right-click drag obstacles");
        text.setFillColor(sf::Color::Cyan);
        text.setPosition({10.f, 25.f});
        window.draw(text);

        // Left HUD: Reactive (Green)
        text.setFillColor(sf::Color::Green);
        text.setString("REACTIVE ROBOT (Green)");
        text.setPosition({10.f, 50.f});
        window.draw(text);

        text.setFillColor(sf::Color::White);
        text.setString("Pos: (" + std::to_string(robot.x).substr(0, 4) + ", " + std::to_string(robot.y).substr(0, 4) + ")");
        text.setPosition({10.f, 65.f});
        window.draw(text);

        text.setString("Vel: (" + std::to_string(robot.vx).substr(0, 4) + ", " + std::to_string(robot.vy).substr(0, 4) + ")");
        text.setPosition({10.f, 80.f});
        window.draw(text);

        double dist = distance(robot.x, robot.y, target_x, target_y);
        text.setString("Dist to Target: " + std::to_string(dist).substr(0, 4));
        text.setPosition({10.f, 95.f});
        window.draw(text);

        if (dist <= 0.1) {
            text.setString("STATUS: ARRIVED!");
            text.setFillColor(sf::Color::Green);
        } else {
            text.setString("STATUS: MOVING...");
            text.setFillColor(sf::Color::Yellow);
        }
        text.setPosition({10.f, 110.f});
        window.draw(text);

        // Right HUD: Planned (Blue)
        text.setFillColor(sf::Color(100, 150, 255));
        text.setString("PLANNED ROBOT (Blue)");
        text.setPosition({250.f, 50.f});
        window.draw(text);

        text.setFillColor(sf::Color::White);
        text.setString("Pos: (" + std::to_string(planned_robot.x).substr(0, 4) + ", " + std::to_string(planned_robot.y).substr(0, 4) + ")");
        text.setPosition({250.f, 65.f});
        window.draw(text);

        text.setString("Vel: (" + std::to_string(planned_robot.vx).substr(0, 4) + ", " + std::to_string(planned_robot.vy).substr(0, 4) + ")");
        text.setPosition({250.f, 80.f});
        window.draw(text);

        text.setString("Dist to Target: " + std::to_string(planned_dist).substr(0, 4));
        text.setPosition({250.f, 95.f});
        window.draw(text);

        if (planned_dist <= 0.1) {
            text.setString("STATUS: ARRIVED!");
            text.setFillColor(sf::Color::Green);
        } else {
            text.setString("STATUS: MOVING...");
            text.setFillColor(sf::Color::Yellow);
        }
        text.setPosition({250.f, 110.f});
        window.draw(text);

        window.display();
    }

    return 0;
}
