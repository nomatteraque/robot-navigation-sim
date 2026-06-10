#pragma once

#include <cmath>
#include <SFML/Graphics.hpp>
#include <algorithm>

struct Robot {
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
};

struct Obstacle {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    int vortex_direction = 0; // 0: undecided, 1: CCW, -1: CW
};

// Computes the Euclidean distance between two 2D coordinates.
inline double distance(double x1, double y1, double x2, double y2) {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

// Projects a target coordinate to the nearest safety boundary if it lies within an obstacle zone.
inline sf::Vector2f ProjectTargetToSafe(double tx, double ty, const std::vector<Obstacle>& obstacles) {
    sf::Vector2f projected(static_cast<float>(tx), static_cast<float>(ty));
    for (int iter = 0; iter < 3; ++iter) {
        bool adjusted = false;
        for (const auto& obs : obstacles) {
            const double dist = distance(projected.x, projected.y, obs.x, obs.y);
            const double safe_radius = obs.radius + 0.35;
            if (dist < safe_radius) {
                adjusted = true;
                if (dist < 0.01) {
                    projected.x = static_cast<float>(obs.x);
                    projected.y = static_cast<float>(obs.y + safe_radius);
                } else {
                    const double dx = (projected.x - obs.x) / dist;
                    const double dy = (projected.y - obs.y) / dist;
                    projected.x = static_cast<float>(obs.x + dx * safe_radius);
                    projected.y = static_cast<float>(obs.y + dy * safe_radius);
                }
            }
        }
        if (!adjusted) break;
    }
    return projected;
}
