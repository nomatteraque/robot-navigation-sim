#include "pathfinder.hpp"
#include <algorithm>
#include <cmath>

bool HasLineOfSight(sf::Vector2f p1, sf::Vector2f p2, const std::vector<Obstacle>& obstacles) {
    const float dist = distance(p1.x, p1.y, p2.x, p2.y);
    if (dist < 0.1f) return true;

    // Sample points along the segment to detect collisions
    const int steps = static_cast<int>(dist / 0.1f);
    for (int i = 1; i < steps; ++i) {
        const float t = static_cast<float>(i) / steps;
        const float sx = p1.x + t * (p2.x - p1.x);
        const float sy = p1.y + t * (p2.y - p1.y);
        for (const auto& obs : obstacles) {
            if (distance(sx, sy, obs.x, obs.y) < (obs.radius + 0.35)) {
                return false;
            }
        }
    }
    return true;
}

std::vector<sf::Vector2f> PrunePath(const std::vector<sf::Vector2f>& path, const std::vector<Obstacle>& obstacles) {
    if (path.size() <= 2) return path;

    std::vector<sf::Vector2f> pruned;
    pruned.push_back(path.front());

    size_t curr = 0;
    while (curr < path.size() - 1) {
        size_t best_next = curr + 1;
        for (size_t next = curr + 2; next < path.size(); ++next) {
            if (HasLineOfSight(pruned.back(), path[next], obstacles)) {
                best_next = next;
            }
        }
        pruned.push_back(path[best_next]);
        curr = best_next;
    }
    return pruned;
}

std::vector<sf::Vector2f> SmoothPath(const std::vector<sf::Vector2f>& path, const std::vector<Obstacle>& obstacles) {
    if (path.size() <= 2) return path;

    std::vector<sf::Vector2f> smoothed;
    smoothed.push_back(path.front());

    for (size_t i = 1; i < path.size() - 1; ++i) {
        const sf::Vector2f p_prev = path[i - 1];
        const sf::Vector2f p_curr = path[i];
        const sf::Vector2f p_next = path[i + 1];

        const sf::Vector2f v1 = p_prev - p_curr;
        const sf::Vector2f v2 = p_next - p_curr;

        const float len1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
        const float len2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);

        if (len1 < 0.05f || len2 < 0.05f) {
            smoothed.push_back(p_curr);
            continue;
        }

        const sf::Vector2f u1 = v1 / len1;
        const sf::Vector2f u2 = v2 / len2;

        // Choose smooth distance (up to 0.6 units, or 40% of segment length)
        const float d_smooth = std::min({0.6f, 0.4f * len1, 0.4f * len2});

        const sf::Vector2f A = p_curr + d_smooth * u1;
        const sf::Vector2f C = p_curr + 0.5f * d_smooth * (u1 + u2);
        const sf::Vector2f B = p_curr + d_smooth * u2;

        // Safety verification: Ensure the smoothed corner point C does not clip any obstacle warning/core zones
        bool safe = true;
        for (const auto& obs : obstacles) {
            if (distance(C.x, C.y, obs.x, obs.y) < (obs.radius + 0.25)) {
                safe = false;
                break;
            }
        }

        if (safe) {
            smoothed.push_back(A);
            smoothed.push_back(C);
            smoothed.push_back(B);
        } else {
            smoothed.push_back(p_curr); // Keep original corner
        }
    }

    smoothed.push_back(path.back());
    return smoothed;
}

std::vector<sf::Vector2f> PlanAStarPath(
    double start_x, double start_y, 
    double goal_x, double goal_y, 
    const std::vector<Obstacle>& obstacles) 
{
    const double cell_size = 0.25;
    const int grid_cols = 81; // 20 units / 0.25 + 1
    const int grid_rows = 57; // 14 units / 0.25 + 1

    auto SimToGrid = [&](double x, double y) -> std::pair<int, int> {
        int col = std::clamp(static_cast<int>(std::round(x / cell_size)), 0, grid_cols - 1);
        int row = std::clamp(static_cast<int>(std::round(y / cell_size)), 0, grid_rows - 1);
        return {col, row};
    };

    auto GridToSim = [&](int col, int row) -> sf::Vector2f {
        return sf::Vector2f(
            static_cast<float>(col * cell_size),
            static_cast<float>(row * cell_size)
        );
    };

    auto GetCellCost = [&](int col, int row) -> double {
        sf::Vector2f pos = GridToSim(col, row);
        double cost = 1.0;
        for (const auto& obs : obstacles) {
            double dist = distance(pos.x, pos.y, obs.x, obs.y);
            if (dist < obs.radius) {
                cost = std::max(cost, 100000.0);
            } else if (dist < (obs.radius + 0.35)) {
                double penalty = 1.0 + 1000.0 * (1.0 - (dist - obs.radius) / 0.35);
                cost = std::max(cost, penalty);
            }
        }
        return cost;
    };

    auto grid_dist = [](int c1, int r1, int c2, int r2) -> double {
        double dc = c2 - c1;
        double dr = r2 - r1;
        return std::sqrt(dc * dc + dr * dr);
    };

    struct Node {
        int col, row;
        double g, h;
        Node* parent = nullptr;
        double f() const { return g + h; }
    };

    auto start_coords = SimToGrid(start_x, start_y);
    auto goal_coords = SimToGrid(goal_x, goal_y);
    double start_cost = GetCellCost(start_coords.first, start_coords.second);

    std::vector<std::vector<Node>> node_grid(grid_cols, std::vector<Node>(grid_rows));
    for (int c = 0; c < grid_cols; ++c) {
        for (int r = 0; r < grid_rows; ++r) {
            node_grid[c][r].col = c;
            node_grid[c][r].row = r;
            node_grid[c][r].g = 1e9;
            node_grid[c][r].h = grid_dist(c, r, goal_coords.first, goal_coords.second);
            node_grid[c][r].parent = nullptr;
        }
    }

    std::vector<Node*> open_set;
    std::vector<std::vector<bool>> closed_set(grid_cols, std::vector<bool>(grid_rows, false));

    Node* start_node = &node_grid[start_coords.first][start_coords.second];
    start_node->g = 0.0;
    open_set.push_back(start_node);

    Node* target_node = nullptr;

    while (!open_set.empty()) {
        auto min_it = std::min_element(open_set.begin(), open_set.end(), [](Node* a, Node* b) {
            return a->f() < b->f();
        });

        Node* current = *min_it;
        open_set.erase(min_it);

        closed_set[current->col][current->row] = true;

        if (current->col == goal_coords.first && current->row == goal_coords.second) {
            target_node = current;
            break;
        }

        for (int dc = -1; dc <= 1; ++dc) {
            for (int dr = -1; dr <= 1; ++dr) {
                if (dc == 0 && dr == 0) continue;

                int nc = current->col + dc;
                int nr = current->row + dr;

                if (nc < 0 || nc >= grid_cols || nr < 0 || nr >= grid_rows) continue;
                if (closed_set[nc][nr]) continue;

                double cell_cost = GetCellCost(nc, nr);
                // Allow search to traverse blocked cells if start cell itself is inside an obstacle
                if (cell_cost >= 100000.0 && start_cost < 100000.0) {
                    if (!(nc == goal_coords.first && nr == goal_coords.second)) {
                        continue;
                    }
                }

                Node* neighbor = &node_grid[nc][nr];
                double step_cost = std::sqrt(dc * dc + dr * dr) * cell_cost;
                double tentative_g = current->g + step_cost;

                if (tentative_g < neighbor->g) {
                    neighbor->parent = current;
                    neighbor->g = tentative_g;

                    if (std::find(open_set.begin(), open_set.end(), neighbor) == open_set.end()) {
                        open_set.push_back(neighbor);
                    }
                }
            }
        }
    }

    std::vector<sf::Vector2f> path;
    if (target_node != nullptr) {
        Node* curr = target_node;
        while (curr != nullptr) {
            path.push_back(GridToSim(curr->col, curr->row));
            curr = curr->parent;
        }
        std::reverse(path.begin(), path.end());
    } else {
        path.push_back(sf::Vector2f(static_cast<float>(start_x), static_cast<float>(start_y)));
        path.push_back(sf::Vector2f(static_cast<float>(goal_x), static_cast<float>(goal_y)));
    }

    return SmoothPath(PrunePath(path, obstacles), obstacles);
}
