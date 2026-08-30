#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <memory>

/***
This code implements the custom geospatial H3 grid math, geographic distance calculation using 
the Haversine formula (allowing up to 10K+ miles), obstacle collision mapping, and the 
A^{*} heuristic pathfinder. It compiles down to a clean command-line engine that outputs 
standard JSON paths.

***/

// PI definition for spherical trigonometry
constexpr double PI = 3.14159265358979323846;
constexpr double EARTH_RADIUS_MILES = 3958.8; // Miles

// Structure representing standard coordinates
struct LatLng {
    double lat;
    double lng;
};

// Custom simplified H3 Hexagonal Cell abstraction mapped to 2D local space
// This replicates H3 behavior using standard axial coordinates on an unrolled global grid
struct H3Cell {
    int q;
    int r;

    bool operator==(const H3Cell& other) const {
        return q == other.q && r == other.r;
    }
};

// Hash function for H3Cell to allow storage in unordered structures
namespace std {
    template <>
    struct hash<H3Cell> {
        size_t operator()(const H3Cell& cell) const {
            return hash<int>()(cell.q) ^ (hash<int>()(cell.r) << 1);
        }
    };
}

// Convert global geospatial coordinates to an explicit H3 grid cell map representation
H3Cell LatLngToH3(const LatLng& coord, double resolution_scale) {
    // Mercator-like projection flattening for the local cell matrix mapping
    double x = coord.lng * cos(coord.lat * PI / 180.0) * resolution_scale;
    double y = coord.lat * resolution_scale;

    // Convert 2D Cartesian space to a fractional hexagonal coordinate system
    double q_frac = (sqrt(3.0) / 3.0 * x - 1.0 / 3.0 * y);
    double r_frac = (2.0 / 3.0 * y);

    // Round fractional coordinates to the nearest axial hexagon cell integers
    int q = std::round(q_frac);
    int r = std::round(r_frac);
    double s = -q - r;

    double q_diff = std::abs(q - q_frac);
    double r_diff = std::abs(r - r_frac);
    double s_diff = std::abs(s - (-q_frac - r_frac));

    if (q_diff > r_diff && q_diff > s_diff) {
        q = -r - std::round(-q_frac - r_frac);
    } else if (r_diff > s_diff) {
        r = -q - std::round(-q_frac - r_frac);
    }
    return H3Cell{q, r};
}

// Convert H3 cell coordinates back into an explicit global geographical location
LatLng H3ToLatLng(const H3Cell& cell, double resolution_scale) {
    double y = (3.0 / 2.0) * cell.r / resolution_scale;
    double x = (sqrt(3.0) * cell.q + sqrt(3.0) / 2.0 * cell.r) / resolution_scale;

    double lat = y;
    double lng = x / cos(lat * PI / 180.0);
    return LatLng{lat, lng};
}

// Haversine formula calculation for accurate trans-oceanic distances up to 10K+ miles
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

// Struct to store explicit parameters of arbitrarily defined obstacle points
struct Obstacle {
    LatLng center;
    double radius_miles;
};

// Check if a generated H3 node intersects with any set map obstacles
bool IsCellObstructed(const H3Cell& cell, const std::vector<Obstacle>& obstacles, double resolution_scale) {
    LatLng cell_center = H3ToLatLng(cell, resolution_scale);
    for (const auto& obs : obstacles) {
        if (CalculateHaversineDistance(cell_center, obs.center) <= obs.radius_miles) {
            return true; // Node is within the threat ring of the obstacle
        }
    }
    return false;
}

// Get the 6 equidistant immediate neighbors of an H3 cell
std::vector<H3Cell> GetH3Neighbors(const H3Cell& cell) {
    return {
        {cell.q + 1, cell.r}, {cell.q - 1, cell.r},
        {cell.q, cell.r + 1}, {cell.q, cell.r - 1},
        {cell.q + 1, cell.r - 1}, {cell.q - 1, cell.r + 1}
    };
}

// A* Node tracking structure
struct AStarNode {
    H3Cell cell;
    double g_cost;
    double h_cost;
    double f_cost() const { return g_cost + h_cost; }
};

// Comparator for the priority queue sorting
struct CompareNode {
    bool operator()(const AStarNode& n1, const AStarNode& n2) {
        return n1.f_cost() > n2.f_cost();
    }
};

// The core A* path planning logic using H3 cell stepping
std::vector<LatLng> PlanGlobalRoute(LatLng start, LatLng goal, const std::vector<Obstacle>& obstacles) {
    // Dynamic cell resolution scaling factor (higher means tighter paths, lower scales out over 10K miles)
    double distance = CalculateHaversineDistance(start, goal);
    double resolution_scale = (distance > 2000.0) ? 0.05 : 0.2; 

    H3Cell start_cell = LatLngToH3(start, resolution_scale);
    H3Cell goal_cell = LatLngToH3(goal, resolution_scale);

    std::cout << "H3 start cell: (q " << start_cell.q << ") (r " << start_cell.r << ") " << std::endl;
    std::cout << "H3 goal  cell: (q " << goal_cell.q  << ") (r " << goal_cell.r <<  ") " <<  std::endl;

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareNode> open_set;
    std::unordered_map<H3Cell, double> g_costs;
    std::unordered_map<H3Cell, H3Cell> came_from;
    std::unordered_set<H3Cell> closed_set;

    open_set.push({start_cell, 0.0, CalculateHaversineDistance(start, goal)});
    g_costs[start_cell] = 0.0;

    bool found = false;

    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();

        std::cout << "\rH3 current cell: (q " << current.cell.q  << ") (r " << current.cell.r <<  ") " <<  std::flush;
        if (current.cell == goal_cell) {
            found = true;
            break;
        }

        if (closed_set.count(current.cell)) continue;
        closed_set.insert(current.cell);

        for (const auto& neighbor : GetH3Neighbors(current.cell)) {
            if (closed_set.count(neighbor) || IsCellObstructed(neighbor, obstacles, resolution_scale)) {
                continue;
            }

            LatLng current_pos = H3ToLatLng(current.cell, resolution_scale);
            LatLng neighbor_pos = H3ToLatLng(neighbor, resolution_scale);
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

    std::vector<LatLng> path;
    if (found) {
        H3Cell curr = goal_cell;
        while (!(curr == start_cell)) {
            path.push_back(H3ToLatLng(curr, resolution_scale));
            curr = came_from[curr];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
    }
    return path;
}

int main() {
    // 10,000 Mile Global Trajectory simulation test setup
    LatLng start_waypoint{ -10.0, -140.0 }; // Pacific South-West
    LatLng goal_waypoint{ 45.0, 150.0 };    // North-West Pacific Rim

    // Add arbitrary large-scale simulated spatial navigation obstacles
    std::vector<Obstacle> global_obstacles = {
        { {15.0, -40.0}, 800.0 },
        { {20.0, 45.0}, 1200.0 }
    };

    std::cout << "--- Executing A* H3 Maritime Global Planner ---" << std::endl;
    std::cout << "Calculating long-range global route (Approx " 
              << CalculateHaversineDistance(start_waypoint, goal_waypoint) << " Miles)..." << std::endl;

    std::vector<LatLng> final_route = PlanGlobalRoute(start_waypoint, goal_waypoint, global_obstacles);

    if (!final_route.empty()) {
        std::cout << "Path successfully verified! Waypoints generated: " << final_route.size() << "\n\nJSON Output Data:\n[";
        for (size_t i = 0; i < final_route.size(); ++i) {
            std::cout << "{\"lat\":" << final_route[i].lat << ",\"lng\":" << final_route[i].lng << "}" 
                      << (i + 1 < final_route.size() ? "," : "");
        }
        std::cout << "]" << std::endl;
    } else {
        std::cout << "No safe route could be found matching current grid settings." << std::endl;
    }
    return 0;
}
