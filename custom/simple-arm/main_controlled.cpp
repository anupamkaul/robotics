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
    m = mj_loadXML("../models/arm2d_controlled.xml", nullptr, error, 1000);
    if (!m) { std::cerr << error << std::endl; return 1; }
    d = mj_makeData(m);

    if (!glfwInit()) return 1;
    window = glfwCreateWindow(1200, 900, "Version 2: Programmable Rotation Control", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, mouse_move);

    mjv_defaultCamera(&cam); mjv_defaultOption(&opt); mjv_defaultScene(&scn); mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);       

    cam.type = mjCAMERA_FREE; cam.distance = 2.0; cam.lookat[2] = 0.4;

    // Look up the exact internal body IDs for tracking rotation matrices
    int link1_id = mj_name2id(m, mjOBJ_BODY, "link1");
    int link2_id = mj_name2id(m, mjOBJ_BODY, "link2");

    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {

            // =================================================================
            // PROGRAMMATIC CONTROL ZONE: I CAN FIDDLE WITH MY DESIRED TARGET ANGLES HERE!
            // =================================================================
            // Let's create a smooth time-varying path (Angles are in Radians)
            double theta1_target = 0.7 * std::sin(d->time * 2.0); // Moves Link 1 back & forth
            double theta2_target = 1.0 * std::cos(d->time * 3.0); // Moves Link 2 back & forth

            // Direct input injection command matching the actuator index slots
            d->ctrl[0] = theta1_target;
            d->ctrl[1] = theta2_target;
            // =================================================================

            mj_step(m, d);

            // --- EXPLICIT ROTATION MATRIX MONITORING ---
            // d->xmat stores the global 3x3 orientation matrix of each link (9 flat values row-major)
            // It maps exactly how the limb's local coordinate frame is rotated relative to the world.
            double* R1 = &d->xmat[link1_id * 9];
            double* R2 = &d->xmat[link2_id * 9];

            // Print the rotation matrices every 0.5 seconds to track structural changes
            if (std::fmod(d->time, 0.5) < m->opt.timestep) {
                std::printf("\n--- Time: %.2f ---\n", d->time);
                std::printf("Link 1 Rotation Matrix (Row 1): [ %5.2f  %5.2f  %5.2f ]\n", R1[0], R1[1], R1[2]);
                std::printf("Link 1 Rotation Matrix (Row 2): [ %5.2f  %5.2f  %5.2f ]\n", R1[3], R1[4], R1[5]);
                std::printf("Link 1 Rotation Matrix (Row 3): [ %5.2f  %5.2f  %5.2f ]\n", R1[6], R1[7], R1[8]);
                
                // You can extract the true kinematic angle mathematically back out from this matrix:
                double computed_angle1 = std::atan2(-R1[2], R1[0]);
                std::printf("Extracted Kinematic Angle from Matrix 1: %.3f Radians\n", computed_angle1);
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
