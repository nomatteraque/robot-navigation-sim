#include "controller.hpp"
#include <cmath>
#include <algorithm>
#include <cstdlib>

sf::Vector2f GetLookaheadPoint(
    const std::vector<sf::Vector2f>& path, 
    const Robot& robot, 
    double L, 
    size_t& last_index) 
{
    if (path.empty()) return sf::Vector2f(static_cast<float>(robot.x), static_cast<float>(robot.y));
    if (path.size() == 1) return path[0];

    // If close to the target, lock onto the goal directly
    float dist_to_end = distance(robot.x, robot.y, path.back().x, path.back().y);
    if (dist_to_end <= L) {
        last_index = path.size() - 1;
        return path.back();
    }

    sf::Vector2f r_pos(static_cast<float>(robot.x), static_cast<float>(robot.y));
    sf::Vector2f lookahead = path.back();
    bool found = false;

    // Search segments backwards to find the furthest valid intersection
    size_t start_idx = last_index;
    for (size_t iPlusOne = path.size() - 1; iPlusOne > start_idx; --iPlusOne) {
        size_t i = iPlusOne - 1;
        sf::Vector2f p1 = path[i];
        sf::Vector2f p2 = path[i + 1];

        sf::Vector2f d = p2 - p1;
        sf::Vector2f f = p1 - r_pos;

        float a = d.x * d.x + d.y * d.y;
        float b = 2.0f * (f.x * d.x + f.y * d.y);
        float c = (f.x * f.x + f.y * f.y) - static_cast<float>(L * L);

        float discriminant = b * b - 4.0f * a * c;
        if (discriminant >= 0) {
            discriminant = std::sqrt(discriminant);
            float t1 = (-b - discriminant) / (2.0f * a);
            float t2 = (-b + discriminant) / (2.0f * a);

            // Choose the forward-most intersection
            if (t2 >= 0.f && t2 <= 1.f) {
                lookahead = p1 + d * t2;
                last_index = i;
                found = true;
                break;
            }
            if (t1 >= 0.f && t1 <= 1.f) {
                lookahead = p1 + d * t1;
                last_index = i;
                found = true;
                break;
            }
        }
    }

    if (!found) {
        // Fallback: search for the closest waypoint forward
        float min_dist = 1e9f;
        size_t best_idx = last_index;
        for (size_t i = last_index; i < path.size(); ++i) {
            float d = distance(robot.x, robot.y, path[i].x, path[i].y);
            if (d < min_dist) {
                min_dist = d;
                best_idx = i;
            }
        }
        last_index = best_idx;
        lookahead = path[best_idx];
    }

    return lookahead;
}

void UpdateReactiveRobot(
    Robot& robot,
    double target_x, double target_y,
    std::vector<Obstacle>& obstacles,
    double dt,
    double& integral_error_x, double& integral_error_y,
    double& filtered_d_error_x, double& filtered_d_error_y,
    double& last_noisy_vx, double& last_noisy_vy)
{
    double error_x = target_x - robot.x;
    double error_y = target_y - robot.y;
    double dist = std::sqrt(error_x * error_x + error_y * error_y);

    double repulse_x = 0.0;
    double repulse_y = 0.0;

    // Potential fields: radial repulsion and vortex tangential fields
    for (auto& obs : obstacles) {
        double obstacle_dist = distance(robot.x, robot.y, obs.x, obs.y);
        if (obstacle_dist < obs.radius) {
            double radial_x = (robot.x - obs.x) / obstacle_dist;
            double radial_y = (robot.y - obs.y) / obstacle_dist;

            double tangent_x = -radial_y;
            double tangent_y = radial_x;

            // Decouple CCW/CW direction choice on entry to avoid oscillation
            if (obs.vortex_direction == 0) {
                double dot_product = (tangent_x * error_x) + (tangent_y * error_y);
                obs.vortex_direction = (dot_product >= 0.0) ? 1 : -1;
            }

            if (obs.vortex_direction == -1) {
                tangent_x = -tangent_x;
                tangent_y = -tangent_y;
            }

            double strength = 5.0 * (obs.radius - obstacle_dist);
            
            // Decelerate potential force near the target to prevent orbiting
            double target_scale = std::min(1.0, dist / 2.0);

            const double vortex_factor = 1.2;
            repulse_x += ((radial_x * strength) + (tangent_x * strength * vortex_factor)) * target_scale;
            repulse_y += ((radial_y * strength) + (tangent_y * strength * vortex_factor)) * target_scale;
        } else {
            obs.vortex_direction = 0;
        }
    }

    if (dist > 0.1) {
        const double Kp = 0.5;
        const double Ki = 0.01;
        const double Kd = 1.2;

        // Velocity dependent sensor noise modeling
        const double noise_level = 1.5;
        double speed = std::sqrt(robot.vx * robot.vx + robot.vy * robot.vy);
        double active_noise = noise_level * (speed * 0.1 + 0.05);

        double noise_x = ((double)rand() / RAND_MAX - 0.5) * active_noise;
        double noise_y = ((double)rand() / RAND_MAX - 0.5) * active_noise;

        double noisy_vx = robot.vx + noise_x;
        double noisy_vy = robot.vy + noise_y;

        last_noisy_vx = noisy_vx;
        last_noisy_vy = noisy_vy;

        // Anti-windup integral clamping
        integral_error_x = std::clamp(integral_error_x + error_x * dt, -2.0, 2.0);
        integral_error_y = std::clamp(integral_error_y + error_y * dt, -2.0, 2.0);

        double raw_d_error_x = -noisy_vx;
        double raw_d_error_y = -noisy_vy;

        // Low-pass noise filter (alpha = 0.35)
        const double alpha = 0.35;
        filtered_d_error_x = (alpha * raw_d_error_x) + ((1.0 - alpha) * filtered_d_error_x);
        filtered_d_error_y = (alpha * raw_d_error_y) + ((1.0 - alpha) * filtered_d_error_y);

        double goal_ax = (Kp * error_x) + (Ki * integral_error_x) + (Kd * filtered_d_error_x);
        double goal_ay = (Kp * error_y) + (Ki * integral_error_y) + (Kd * filtered_d_error_y);

        robot.vx += (goal_ax + repulse_x) * dt;
        robot.vy += (goal_ay + repulse_y) * dt;
        robot.x += robot.vx * dt;
        robot.y += robot.vy * dt;
    } else {
        robot.vx = 0.0;
        robot.vy = 0.0;
        last_noisy_vx = 0.0;
        last_noisy_vy = 0.0;
    }
}

void UpdatePlannedRobot(
    Robot& planned_robot,
    double target_x, double target_y,
    const std::vector<sf::Vector2f>& astar_path,
    double dt,
    size_t& lookahead_index,
    sf::Vector2f& planned_lookahead_pt,
    double& filtered_planned_vx, double& filtered_planned_vy,
    double& filtered_target_vx, double& filtered_target_vy)
{
    double planned_dist = distance(planned_robot.x, planned_robot.y, target_x, target_y);
    
    if (planned_dist > 0.1) {
        double L = 1.2;
        planned_lookahead_pt = GetLookaheadPoint(astar_path, planned_robot, L, lookahead_index);

        double dx = planned_lookahead_pt.x - planned_robot.x;
        double dy = planned_lookahead_pt.y - planned_robot.y;
        double len = std::sqrt(dx * dx + dy * dy);
        double dir_x = (len > 0.0) ? (dx / len) : 0.0;
        double dir_y = (len > 0.0) ? (dy / len) : 0.0;

        // Cruise target speed (cruise at 3.0, scale down smoothly inside 2.0 unit threshold)
        double target_speed = 3.0;
        if (planned_dist < 2.0) {
            target_speed = 3.0 * (planned_dist / 2.0);
        }

        double target_vx = target_speed * dir_x;
        double target_vy = target_speed * dir_y;

        // Velocity noise injection
        const double noise_level = 1.5;
        double planned_speed = std::sqrt(planned_robot.vx * planned_robot.vx + planned_robot.vy * planned_robot.vy);
        double active_noise = noise_level * (planned_speed * 0.1 + 0.05);

        double noise_x = ((double)rand() / RAND_MAX - 0.5) * active_noise;
        double noise_y = ((double)rand() / RAND_MAX - 0.5) * active_noise;

        double planned_noisy_vx = planned_robot.vx + noise_x;
        double planned_noisy_vy = planned_robot.vy + noise_y;

        // Low-pass filter actual velocity feedback
        const double alpha = 0.35;
        filtered_planned_vx = (alpha * planned_noisy_vx) + ((1.0 - alpha) * filtered_planned_vx);
        filtered_planned_vy = (alpha * planned_noisy_vy) + ((1.0 - alpha) * filtered_planned_vy);

        // Command smoothing / Input shaping reference tracking
        const double target_alpha = 0.15;
        filtered_target_vx = (target_alpha * target_vx) + ((1.0 - target_alpha) * filtered_target_vx);
        filtered_target_vy = (target_alpha * target_vy) + ((1.0 - target_alpha) * filtered_target_vy);

        // Tracking loop feedback with high gain Kv
        const double Kv = 8.0;
        double planned_ax = Kv * (filtered_target_vx - filtered_planned_vx);
        double planned_ay = Kv * (filtered_target_vy - filtered_planned_vy);

        planned_robot.vx += planned_ax * dt;
        planned_robot.vy += planned_ay * dt;
        planned_robot.x += planned_robot.vx * dt;
        planned_robot.y += planned_robot.vy * dt;
    } else {
        planned_robot.vx = 0.0;
        planned_robot.vy = 0.0;
    }
}
