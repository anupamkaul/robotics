#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <memory>
#include <sstream>

// Emscripten header tells the compiler which functions to export to JavaScript
#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

constexpr double PI = 3.14159265358979323846;
constexpr double EARTH_RADIUS_MILES = 3958.8;

struct LatLng {
    double lat;
    double lng;
};

struct H3Cell {
    int q;
    int r;
    bool operator==(const H3Cell& other) const { return q == other.q && r == other.r; }
};

namespace std {
    template <>
    struct hash<H3Cell> {
        size_t operator()(const H3Cell& cell) const {
            return hash<int>()(cell.q) ^ (hash<int>()(cell.r) << 1);
        }
    };
}

H3Cell LatLngToH3(const LatLng& coord, double resolution_scale) {
    double x = coord.lng * cos(coord.lat * PI / 180.0) * resolution_scale;
    double y = coord.lat * resolution_scale;
    double q_frac = (sqrt(3.0) / 3.0 * x - 1.0 / 3.0 * y);
    double r_frac = (2.0 / 3.0 * y);
    int q = std::round(q_frac);
    int r = std::round(r_frac);
    double s = -q - r;
    double q_diff = std::abs(q - q_frac);
    double r_diff = std::abs(r - r_frac);
    double s_diff = std::abs(s - (-q_frac - r_frac));
    if (q_diff > r_diff && q_diff > s_diff) q = -r - std::round(-q_frac - r_frac);
    else if (r_diff > s_diff) r = -q - std::round(-q_frac - r_frac);
    return H3Cell{q, r};
}

LatLng H3ToLatLng(const H3Cell& cell, double resolution_scale) {
    double y = (3.0 / 2.0) * cell.r / resolution_scale;
    double x = (sqrt(3.0) * cell.q + sqrt(3.0) / 2.0 * cell.r) / resolution_scale;
    double lat = y;
    double lng = x / cos(lat * PI / 180.0);
    return LatLng{lat, lng};
}

double CalculateHaversineDistance(const LatLng& p1, const LatLng& p2) {
    double dLat = (p2.lat - p1.lat) * PI / 180.0;
    double dLng = (p2.lng - p1.lng) * PI / 180.0;
    double rLat1 = p1.lat * PI / 180.0;
    double rLat2 = p2.lat * PI / 180.0;
    double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
               std::sin(dLng / 2.0) * std::sin(dLng / 2.0) * std::cos(rLat1) * std::cos(rLat2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return EARTH_RADIUS_MILES * c;
}

struct Obstacle {
    LatLng center;
    double radius_miles;
};

bool IsCellObstructed(const H3Cell& cell, const std::vector<Obstacle>& obstacles, double resolution_scale) {
    LatLng cell_center = H3ToLatLng(cell, resolution_scale);
    for (const auto& obs : obstacles) {
        if (CalculateHaversineDistance(cell_center, obs.center) <= obs.radius_miles) return true;
    }
    return false;
}

std::vector<H3Cell> GetH3Neighbors(const H3Cell& cell) {
    return {
        {cell.q + 1, cell.r}, {cell.q - 1, cell.r},
        {cell.q, cell.r + 1}, {cell.q, cell.r - 1},
        {cell.q + 1, cell.r - 1}, {cell.q - 1, cell.r + 1}
    };
}

struct AStarNode {
    H3Cell cell;
    double g_cost;
    double h_cost;
    double f_cost() const { return g_cost + h_cost; }
};

struct CompareNode {
    bool operator()(const AStarNode& n1, const AStarNode& n2) { return n1.f_cost() > n2.f_cost(); }
};

// Global output string buffer to safely send data back to JavaScript
std::string cpp_output_buffer = "";

// The primary planning interface exposed directly to JavaScript
extern "C" {
    EMSCRIPTEN_KEEPALIVE
    const char* RunWasmPlanner(double startLat, double startLng, double goalLat, double goalLng, const double* obstacleData, int obstacleCount) {
        LatLng start{startLat, startLng};
        LatLng goal{goalLat, goalLng};
        
        std::vector<Obstacle> obstacles;
        for (int i = 0; i < obstacleCount; ++i) {
            obstacles.push_back({ {obstacleData[i*3], obstacleData[i*3+1]}, obstacleData[i*3+2] });
        }

        double distance = CalculateHaversineDistance(start, goal);
        // Adaptive resolution scale matching the scale of up to 10,000 miles
        double resolution_scale = (distance > 4000.0) ? 0.04 : 0.12; 

        H3Cell start_cell = LatLngToH3(start, resolution_scale);
        H3Cell goal_cell = LatLngToH3(goal, resolution_scale);

        std::priority_queue<AStarNode, std::vector<AStarNode>, CompareNode> open_set;
        std::unordered_map<H3Cell, double> g_costs;
        std::unordered_map<H3Cell, H3Cell> came_from;
        std::unordered_set<H3Cell> closed_set;

        open_set.push({start_cell, 0.0, CalculateHaversineDistance(start, goal)});
        g_costs[start_cell] = 0.0;

        bool found = false;
        int safety_break = 5000; // Protect loop limits over massive spans

        while (!open_set.empty() && safety_break-- > 0) {
            AStarNode current = open_set.top();
            open_set.pop();

            if (current.cell == goal_cell) { found = true; break; }
            if (closed_set.count(current.cell)) continue;
            closed_set.insert(current.cell);

            for (const auto& neighbor : GetH3Neighbors(current.cell)) {
                if (closed_set.count(neighbor) || IsCellObstructed(neighbor, obstacles, resolution_scale)) continue;

                LatLng current_pos = H3ToLatLng(current.cell, resolution_scale);
                LatLng neighbor_pos = H3ToLatLng(neighbor, resolution_scale);
                
                if (std::abs(neighbor_pos.lat) > 85 || std::abs(neighbor_pos.lng) > 180) continue;

                double step_cost = CalculateHaversineDistance(current_pos, neighbor_pos);
                double tentative_g = g_costs[current.cell] + step_cost;

                if (!g_costs.count(neighbor) || tentative_g < g_costs[neighbor]) {
                    g_costs[neighbor] = tentative_g;
                    came_from[neighbor] = current.cell;
                    double h = CalculateHaversineDistance(neighbor_pos, goal);
                    open_set.push({neighbor, tentative_g, h});
                }
            }
        }

        std::stringstream ss;
        if (found) {
            std::vector<LatLng> path;
            H3Cell curr = goal_cell;
            while (!(curr == start_cell)) {
                path.push_back(H3ToLatLng(curr, resolution_scale));
                curr = came_from[curr];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());

            for (size_t i = 0; i < path.size(); ++i) {
                ss << path[i].lat << "," << path[i].lng << (i + 1 < path.size() ? "|" : "");
            }
        } else {
            ss << "ERROR";
        }

        cpp_output_buffer = ss.str();
        return cpp_output_buffer.c_str();
    }
}

int main() { return 0; }
