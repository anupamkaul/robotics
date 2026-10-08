#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <thread> // For timing/pausing
#include <chrono> // For milliseconds

/***
Simple code: 
Given a robot with 4 joint arm and a random ball in space:
(1) calculate the forward kinematics of the end effector
(2) use inverse kinematics to orient the arm to touch the ball
With text-based visual rendering and animation pauses!
***/

const double PI = 3.141592653589793;

// A simple structure to hold 2D coordinates
struct Point {
    double x, y;
};

// Robot Configuration:
const int NUM_JOINTS = 4;
const double LINK_LENGTHS[NUM_JOINTS] = {2.5, 2.0, 1.5, 1.0}; // Total maximum reach = 7.0 units

// 1. FORWARD KINEMATICS (FK)
void calculateFK(const std::vector<double>& angles, std::vector<Point>& jointPositions) {
    jointPositions[0] = {0.0, 0.0}; // The base of the robot is at the origin (0,0)
    
    double globalAngle = 0.0;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        globalAngle += angles[i]; // Each joint angle is relative to the previous link
        jointPositions[i + 1].x = jointPositions[i].x + LINK_LENGTHS[i] * cos(globalAngle);
        jointPositions[i + 1].y = jointPositions[i].y + LINK_LENGTHS[i] * sin(globalAngle);
    }
}

// VISUAL RENDERING FUNCTION
// Draws a 2D ASCII map in the console representing the current state of the arm and ball
void renderFrame(Point ball, const std::vector<Point>& joints, int currentIteration) {
    // Canvas dimensions (Rows x Columns)
    const int WIDTH = 51;
    const int HEIGHT = 25;
    
    // Create an empty canvas filled with spaces
    std::vector<std::string> canvas(HEIGHT, std::string(WIDTH, ' '));
    
    // Scale factor to map robot coordinates (max reach 7) to terminal dimensions
    // X goes from -8 to +8 (span of 16). Y goes from -8 to +8 (span of 16)
    auto mapToGrid = [](Point p) -> std::pair<int, int> {
        int col = static_cast<int>((p.x + 8.0) / 16.0 * (WIDTH - 1));
        int row = static_cast<int>((8.0 - p.y) / 16.0 * (HEIGHT - 1)); // Invert Y because terminal prints top-to-bottom
        
        // Keep inside boundaries
        if (col < 0) col = 0; if (col >= WIDTH) col = WIDTH - 1;
        if (row < 0) row = 0; if (row >= HEIGHT) row = HEIGHT - 1;
        return {row, col};
    };

    // 1. Place the grid background / center origin marker
    auto [baseRow, baseCol] = mapToGrid({0.0, 0.0});
    canvas[baseRow][baseCol] = '+'; // Base of the robot arm

    // 2. Draw the segments connecting the joints
    for (int i = 0; i < NUM_JOINTS; ++i) {
        auto [r1, c1] = mapToGrid(joints[i]);
        auto [r2, c2] = mapToGrid(joints[i+1]);
        
        // Simple linear interpolation to draw a crude line between joints
        int steps = std::max(std::abs(r2 - r1), std::abs(c2 - c1)) * 2;
        for (int s = 0; s <= steps; ++s) {
            double t = (steps == 0) ? 0.0 : static_cast<double>(s) / steps;
            int r = static_cast<int>(r1 + t * (r2 - r1));
            int c = static_cast<int>(c1 + t * (c2 - c1));
            canvas[r][c] = '.'; // '.' represents the arm structure
        }
    }

    // 3. Mark the joint nodes and the end effector
    for (int i = 0; i <= NUM_JOINTS; ++i) {
        auto [r, c] = mapToGrid(joints[i]);
        if (i == 0) canvas[r][c] = 'O';      // Robot Base
        else if (i == NUM_JOINTS) canvas[r][c] = 'E'; // End Effector
        else canvas[r][c] = '0' + i;         // Joints labeled 1, 2, 3
    }

    // 4. Place the target ball on top (so it shows over the arm if caught)
    auto [ballRow, ballCol] = mapToGrid(ball);
    canvas[ballRow][ballCol] = 'X'; // 'X' represents the target Ball

    // Clear the console frame using ANSI escape codes for smooth flickering-free updates
    std::cout << "\033[H\033[J"; 
    
    // Display metadata header
    std::cout << "===================================================\n";
    std::cout << " ROBOT IK ANIMATION  | Iteration: " << currentIteration << "\n";
    std::cout << " Legend: O=Base, 1-3=Joints, E=EndEffector, X=Ball\n";
    std::cout << " Target Ball Position: (" << ball.x << ", " << ball.y << ")\n";
    std::cout << "===================================================\n";

    // Print out the complete canvas grid
    for (int r = 0; r < HEIGHT; ++r) {
        std::cout << "|" << canvas[r] << "|\n";
    }
    std::cout << "===================================================\n";
}

// 2. INVERSE KINEMATICS (IK)
void calculateIK(Point ball, std::vector<double>& angles, int pauseMs) {
    std::vector<Point> joints(NUM_JOINTS + 1);

    int maxIterations = 100; // Limit loops to prevent infinite calculations
    double tolerance = 0.01;  // Stop when end-effector is within 0.01 units of the ball

    for (int iter = 0; iter < maxIterations; ++iter) {

        // Step A: Find out where the end-effector currently is
        calculateFK(angles, joints);
        Point endEffector = joints[NUM_JOINTS]; 

        // RENDER & ANIMATE: Show the frame *before* we make an optimization iteration step
        renderFrame(ball, joints, iter);
        std::this_thread::sleep_for(std::chrono::milliseconds(pauseMs)); // Delay so you can view it

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

    // Render final position if out of loops
    calculateFK(angles, joints);
    renderFrame(ball, joints, maxIterations);
    std::cout << ">> Could not perfectly reach the target (possibly out of range).\n";
}

int main() {
    srand(time(0)); // Seed for random number generation

    // Adjustable Variable for Animation Delay
    int animationSpeedMs = 150; // Change this to alter step speed (e.g. 500 for slower, 50 for faster)

    // Initial joint angles set to 0.0 radians (straight arm pointing right)
    std::vector<double> jointAngles(NUM_JOINTS, 0.0);
    std::vector<Point> jointPositions(NUM_JOINTS + 1);

    // 1. Place a red ball at a random location within the arm's reach (max radius 7)
    double randomAngle = (rand() % 360) * (PI / 180.0);
    double randomDist = 2.0 + (rand() % 40) / 10.0; 
    Point redBall = { randomDist * cos(randomAngle), randomDist * sin(randomAngle) };

    // 2. Run Inverse Kinematics to target the ball (which handles its own visualization loop)
    calculateIK(redBall, jointAngles, animationSpeedMs);

    // 3. Verify final statistics below the rendered map
    calculateFK(jointAngles, jointPositions);
    std::cout << "\nFinal Joint Angles (Degrees):\n";
    for(int i = 0; i < NUM_JOINTS; ++i) {
        std::cout << " Joint " << i + 1 << ": " << jointAngles[i] * (180.0 / PI) << " deg\n";
    }
    std::cout << "Final End-Effector Position: (" 
              << jointPositions[NUM_JOINTS].x << ", " << jointPositions[NUM_JOINTS].y << ")\n";

    return 0;
}
