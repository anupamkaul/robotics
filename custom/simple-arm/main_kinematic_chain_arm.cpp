#include <iostream>
#include <cmath>
#include "mujoco/mujoco.h"
#include "mujoco/mjrender.h" 
#include "GLFW/glfw3.h"

/**
This code tracks all three actuators by their specific array indices. 
It extracts the global 3D rotation matrices for the 
Shoulder (link1), Elbow (link2), and Wrist (link3) bodies to print live 
telemetry.
**/

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
    m = mj_loadXML("../models/arm3d_kinematic_chain.xml", nullptr, error, 1000);
    if (!m) { std::cerr << error << std::endl; return 1; }
    d = mj_makeData(m);

    if (!glfwInit()) return 1;
    window = glfwCreateWindow(1200, 900, "3-Link Chain Arm Controller", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, mouse_move);

    mjv_defaultCamera(&cam); mjv_defaultOption(&opt); mjv_defaultScene(&scn); mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);       

    // Blue background clearcolor initialization
    cam.type = mjCAMERA_FREE; cam.distance = 2.5; 
    cam.lookat[0] = 0.0; cam.lookat[1] = 0.0; cam.lookat[2] = 0.4;

    // Pull distinct internal body array IDs for matrix evaluation
    int link1_id = mj_name2id(m, mjOBJ_BODY, "link1");
    int link2_id = mj_name2id(m, mjOBJ_BODY, "link2");
    int link3_id = mj_name2id(m, mjOBJ_BODY, "link3");

    std::cout << "3-Joint Arm Simulation Active. Monitoring control array entries..." << std::endl;

    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {

            // =================================================================
            // PROGRAMMATIC CONTROL ZONE: THREE DISTINCT TARGET WAVES
            // =================================================================
            // Angles are in Radians. Different frequencies force them to unsync.
            double target_shoulder = 0.5 * std::sin(d->time * 2.0); // Green Link
            double target_elbow    = 0.7 * std::cos(d->time * 3.5); // Blue Link
            double target_wrist    = 0.9 * std::sin(d->time * 5.0); // Red Link

            // Inject targets into the 3 compiled position actuator slots explicitly
            if (m->nu >= 3) {
                d->ctrl[0] = target_shoulder;
                d->ctrl[1] = target_elbow;
                d->ctrl[2] = target_wrist;
            }
            // =================================================================

            mj_step(m, d);

            // --- MULTI-LINK ORIENTATION TELEMETRY ---
            double* R1 = &d->xmat[link1_id * 9];
            double* R2 = &d->xmat[link2_id * 9];
            double* R3 = &d->xmat[link3_id * 9];

            if (std::fmod(d->time, 0.5) < m->opt.timestep) {
                std::printf("\n=== Real-Time 3-Link Telemetry (Time: %.2f) ===\n", d->time);
                double angle1 = std::atan2(-R1[2], R1[0]);
                double angle2 = std::atan2(-R2[2], R2[0]);
                double angle3 = std::atan2(-R3[2], R3[0]);
                
                std::printf("Shoulder (Green) Angle: %6.3f Rad | Target: %6.3f Rad\n", angle1, target_shoulder);
                std::printf("Elbow    (Blue)  Angle: %6.3f Rad | Target: %6.3f Rad\n", angle2, target_elbow);
                std::printf("Wrist    (Red)   Angle: %6.3f Rad | Target: %6.3f Rad\n", angle3, target_wrist);
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
