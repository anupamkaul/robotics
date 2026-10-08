#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>

/***

Simple code: 

Given a robot with 4 joint arm and a random ball in space:

(1) calculate the forward kinematics of the end effector
(2) use inverse kinematics to orient the arm to touch the ball

***/

const double PI = 3.141592653589793;

// A simple structure to hold 2D coordinates
struct Point {
    double x, y;
};

// Robot Configuration :
const int NUM_JOINTS = 4;
const double LINK_LENGTHS[NUM_JOINTS] = {2.5, 2.0, 1.5, 1.0}; // Total maximum reach = 7.0 units

// 1. FORWARD KINEMATICS (FK)
// Calculates the positions of all joints based on relative angles (in radians)

void calculateFK(const std::vector<double>& angles, std::vector<Point>& jointPositions) {
    jointPositions[0] = {0.0, 0.0}; // The base of the robot is at the origin (0,0)
    
    double globalAngle = 0.0;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        globalAngle += angles[i]; // Each joint angle is relative to the previous link
        jointPositions[i + 1].x = jointPositions[i].x + LINK_LENGTHS[i] * cos(globalAngle);
        jointPositions[i + 1].y = jointPositions[i].y + LINK_LENGTHS[i] * sin(globalAngle);
    }
}

// 2. INVERSE KINEMATICS (IK)
// Uses Cyclic Coordinate Descent (CCD) to iteratively move joints closer to the target ball

void calculateIK(Point ball, std::vector<double>& angles) {
    std::vector<Point> joints(NUM_JOINTS + 1);

    int maxIterations = 100; // Limit loops to prevent infinite calculations
    double tolerance = 0.01;  // Stop when end-effector is within 0.01 units of the ball

    for (int iter = 0; iter < maxIterations; ++iter) {

        // Step A: Find out where the end-effector currently is
        calculateFK(angles, joints);
        Point endEffector = joints[NUM_JOINTS]; // endEffector is the (x,y) of last link's joint

        // Check if we are close enough to the ball
        double error = sqrt(pow(ball.x - endEffector.x, 2) + pow(ball.y - endEffector.y, 2));
        if (error < tolerance) {
            std::cout << ">> Target reached in " << iter << " iterations!\n";
            return;
        }

        // Step B: Work backward from the last joint to the first joint
        for (int i = NUM_JOINTS - 1; i >= 0; --i) {

            // Update joint positions layout
            calculateFK(angles, joints);
            endEffector = joints[NUM_JOINTS];

            // Vector from current joint to the End Effector
            double vectorEE_x = endEffector.x - joints[i].x;
            double vectorEE_y = endEffector.y - joints[i].y;

            // Vector from current joint to the Red Ball
            double vectorBall_x = ball.x - joints[i].x;
            double vectorBall_y = ball.y - joints[i].y;

            // Calculate the angle of both vectors relative to the world
            double angleToEE = atan2(vectorEE_y, vectorEE_x);
            double angleToBall = atan2(vectorBall_y, vectorBall_x);

            // The correction angle is the difference between them
            double deltaAngle = angleToBall - angleToEE;

            // Keep the angle adjustments clean between -PI and +PI
            if (deltaAngle > PI)  deltaAngle -= 2 * PI;
            if (deltaAngle < -PI) deltaAngle += 2 * PI;

            // Apply the correction to this joint
            angles[i] += deltaAngle;
        }
    } // for loop close

    std::cout << ">> Could not perfectly reach the target (possibly out of range).\n";
} // IK CCD function

int main() {

    srand(time(0)); // Seed for random number generation

    // Initial joint angles set to 0.0 radians (straight arm pointing right)
    std::vector<double> jointAngles(NUM_JOINTS, 0.0);
    std::vector<Point> jointPositions(NUM_JOINTS + 1);

    // 1. Place a red ball at a random location within the arm's reach (max radius 7)
    // Random angle between 0 and 2*PI, random distance between 2 and 6

    double randomAngle = (rand() % 360) * (PI / 180.0);
    double randomDist = 2.0 + (rand() % 40) / 10.0; 
    Point redBall = { randomDist * cos(randomAngle), randomDist * sin(randomAngle) };

    std::cout << "--- ROBOT ARM SIMULATION ---\n";
    std::cout << "Red Ball placed at: (" << redBall.x << ", " << redBall.y << ")\n\n";

    // 2. Run Forward Kinematics to see starting position

    calculateFK(jointAngles, jointPositions);
    std::cout << "Initial End-Effector Position: (" 
              << jointPositions[NUM_JOINTS].x << ", " << jointPositions[NUM_JOINTS].y << ")\n";

    // 3. Run Inverse Kinematics to target the ball

    std::cout << "Calculating IK...\n";
    calculateIK(redBall, jointAngles);

    // 4. Verify results with Forward Kinematics using our new angles

    calculateFK(jointAngles, jointPositions);
    std::cout << "\nFinal Joint Angles (Degrees):\n";
    for(int i = 0; i < NUM_JOINTS; ++i) {
        std::cout << " Joint " << i + 1 << ": " << jointAngles[i] * (180.0 / PI) << " deg\n";
    }

    std::cout << "Final End-Effector Position: (" 
              << jointPositions[NUM_JOINTS].x << ", " << jointPositions[NUM_JOINTS].y << ")\n";

    return 0;
}
