#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

/***

Given a set of traversed and non-traversed waypoints
find the most reasonable path to goal

(use eucledean distances)

supplement later with grid based search based on cost heuristics

**/

struct Point2D {
    double x;
    double y;
};

struct Waypoint {
    int id;
    Point2D position;
    bool completed;
};

double squaredDistance(
    const Point2D& a,
    const Point2D& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;

    return dx * dx + dy * dy;
}

std::optional<Waypoint> findNearestUncompleted(
    const Point2D& robot,
    const std::vector<Waypoint>& waypoints)
{
    double best_distance_sq =
        std::numeric_limits<double>::infinity();

    const Waypoint* best = nullptr;

    for (const auto& waypoint : waypoints) {

        if (waypoint.completed) {
            continue;
        }

        const double distance_sq =
            squaredDistance(robot, waypoint.position);

        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            best = &waypoint;
        }
    }

    if (best == nullptr) {
        return std::nullopt;
    }

    return *best;
}

int main()
{
    Point2D robot {0.0, 0.0};

    std::vector<Waypoint> waypoints {
        {1, {1.0, 1.0}, true},
        {2, {5.0, 4.0}, false},
        {3, {2.0, 2.0}, true},
        {4, {3.0, 1.0}, false},
        {5, {8.0, 8.0}, false}
    };

    auto nearest =
        findNearestUncompleted(robot, waypoints);

    if (nearest) {
        std::cout
            << "Nearest waypoint = "
            << nearest->id
            << " at ("
            << nearest->position.x
            << ", "
            << nearest->position.y
            << ")\n";
    } else {
        std::cout << "No remaining waypoints\n";
    }

    return 0;
}
