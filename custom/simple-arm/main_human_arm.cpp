#include <iostream>
#include <cmath>
#include "mujoco/mujoco.h"
#include "mujoco/mjrender.h" 
#include "GLFW/glfw3.h"

mjModel* m = nullptr; mjData* d = nullptr;
mjvCamera cam; mjvOption opt; mjvScene scn; mjrContext con;
GLFWwindow* window = nullptr;
double lastx = 0; double lasty = 0;

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    double dx = xpos - lastx; double dy = ypos - lasty;
    lastx = xpos; lasty = ypos;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        mjv_moveCamera(m, mjMOUSE_ROTATE_V, dx/900.0, dy/900.0, &cam);
    }
}

int main() {
    char error[1000] = "Could not load XML model";
    m = mj_loadXML("../models/arm2d_shoulder_elbow.xml", nullptr, error, 1000);
    if (!m) { std::cerr << error << std::endl; return 1; }
    d = mj_makeData(m);

    if (!glfwInit()) return 1;
    window = glfwCreateWindow(1200, 900, "Human Arm Simulation: Shoulder & Elbow Control", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, mouse_move);

    mjv_defaultCamera(&cam); mjv_defaultOption(&opt); mjv_defaultScene(&scn); mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);       

    // Blue background clearcolor initialization parameters
    cam.type = mjCAMERA_FREE; 
    cam.distance = 2.2; 
    cam.lookat[0] = 0.0; 
    cam.lookat[1] = 0.0; 
    cam.lookat[2] = 0.4;

    // Pull distinct internal body array indices for precise coordinate matrix evaluations
    int shoulder_id = mj_name2id(m, mjOBJ_BODY, "link1");
    int elbow_id    = mj_name2id(m, mjOBJ_BODY, "link2");

    std::cout << "Hierarchical Human Arm Simulator Ready. Compiling cycles..." << std::endl;

    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {

            // =================================================================
            // PROGRAMMATIC CONTROL ZONE: DEFINE YOUR ANATOMICAL ANGLES HERE
            // =================================================================
            // Angles are in Radians. 
            // Let's make the shoulder sway wide, and the elbow flex tightly.

            // Values that make shoulder stationary and elbow move like a pendulum

            //double target_shoulder = 0.6 * std::sin(d->time * 2.0);
            //double target_elbow    = 0.9 * std::cos(d->time * 4.5);

 
            // Adding forces so it "breaks" the shoulder joint and causes it to move as well        
            double target_shoulder =  9 * 0.6 * std::sin(d->time * 2.0);
            double target_elbow    = 10 * 0.6 * std::sin(d->time * 2.0);

            // Direct input matrix coordinate register assignments
            d->ctrl[0] = target_shoulder;
            d->ctrl[1] = target_elbow;  // this is the top


            // =================================================================

            mj_step(m, d);

            // --- MULTI-LINK ORIENTATION TELEMETRY ---
            // Extract the 3x3 orientation frame buffers (9 elements row-major)
            double* R_shoulder = &d->xmat[shoulder_id * 9];
            double* R_elbow    = &d->xmat[elbow_id * 9];

            if (std::fmod(d->time, 0.5) < m->opt.timestep) {
                std::printf("\n=== Real-Time Arm Telemetry (Time: %.2f) ===\n", d->time);
                
                // Track spatial kinematics directly from the matrices
                double current_shoulder_angle = std::atan2(-R_shoulder[2], R_shoulder[0]);
                double current_elbow_angle    = std::atan2(-R_elbow[2], R_elbow[0]);
                
                std::printf("Shoulder Angle (Matrix Derived): %6.3f Rad | Target: %6.3f Rad\n", current_shoulder_angle, target_shoulder);
                std::printf("Elbow Angle    (Matrix Derived): %6.3f Rad | Target: %6.3f Rad\n", current_elbow_angle, target_elbow);
            }
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window); glfwPollEvents();
    }
    mjv_freeScene(&scn); mjr_freeContext(&con); mj_deleteData(d); mj_deleteModel(m); glfwTerminate();
    return 0;
}
