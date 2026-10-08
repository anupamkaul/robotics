#include <cmath>
#include <iomanip>
#include <iostream>

/***

Robot pose in the world (xr, yr, theta)
Point measured in robot frame: pr = (x,y)

Find the same frame in world coordinates
(Robot Point x,y --> World Point x,y)

Do the opposite also
(World Point x,y --> Robot Point x, y)

***/

struct Pose2D {
    double x;
    double y;
    double theta;   // radians
};

struct Point2D {
    double x;
    double y;
};

Point2D transformToWorld(
    const Pose2D& robot_pose,
    const Point2D& robot_point)
{
    const double c = std::cos(robot_pose.theta);
    const double s = std::sin(robot_pose.theta);

    Point2D world_point;

    world_point.x =
        robot_pose.x
        + c * robot_point.x
        - s * robot_point.y;

    world_point.y =
        robot_pose.y
        + s * robot_point.x
        + c * robot_point.y;

    return world_point;
}

Point2D transformToRobot(
    const Pose2D& robot_pose,
    const Point2D& world_point)
{
    // First remove translation.
    const double dx = world_point.x - robot_pose.x;
    const double dy = world_point.y - robot_pose.y;

    const double c = std::cos(robot_pose.theta);
    const double s = std::sin(robot_pose.theta);

    // Apply inverse rotation R^T.
    Point2D robot_point;

    robot_point.x = c * dx + s * dy;
    robot_point.y = -s * dx + c * dy;

    return robot_point;
}

int main()
{
    constexpr double PI = 3.14159265358979323846;

    Pose2D robot {
        10.0,
        5.0,
        PI / 2.0        // 90 degrees
    };

    Point2D point_in_robot {
        2.0,
        0.0
    };

    Point2D world =
        transformToWorld(robot, point_in_robot);

    std::cout << std::fixed << std::setprecision(3);

    std::cout << "World point: "
              << world.x << ", "
              << world.y << '\n';

    Point2D recovered =
        transformToRobot(robot, world);

    std::cout << "Recovered robot point: "
              << recovered.x << ", "
              << recovered.y << '\n';

    return 0;
}
