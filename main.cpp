#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <chrono>
#include "haply_client.h"
#include "coordinate_transform.h"
#include "inverse_kinematics.h"

// MuJoCo model and data
mjModel* model = nullptr;
mjData* data = nullptr;

// MuJoCo visualization
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// Mouse interaction state
bool button_left = false;
bool button_right = false;
double lastx = 0, lasty = 0;

// Keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// Mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)  == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

// Mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    if (!button_left && !button_right) return;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    mjtMouse action;
    if (button_right)
        action = GLFW_MOD_SHIFT ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else
        action = mjMOUSE_ROTATE_V;

    mjv_moveCamera(model, action, dx/height, dy/height, &scn, &cam);
}

// Scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

// Target joint positions
double target_qpos[7] = {
    0.0,    // joint 1
   -0.785,  // joint 2
    0.0,    // joint 3
   -2.356,  // joint 4
    0.0,    // joint 5
    1.571,  // joint 6
    0.785   // joint 7
};

// Control callback - PD controller + gravity compensation
void control(const mjModel* m, mjData* d) {
    double kp = 200.0;  // position gain
    double kd = 20.0;   // damping gain

    for (int i = 0; i < 7; ++i) {
        double pos_error = target_qpos[i] - d->qpos[i];
        double vel_error = -d->qvel[i];
        d->ctrl[i] = kp * pos_error + kd * vel_error + d->qfrc_bias[i];
    }
}

int main() {
    // Load model
    char error[1000];
    model = mj_loadXML("../models/panda.xml", nullptr, error, 1000);
    if (!model) {
        std::cerr << "Failed to load model: " << error << std::endl;
        return 1;
    }
    data = mj_makeData(model);

    // Set a natural starting pose (joint angles in radians)
    data->qpos[0] =  0.0;    // joint 1
    data->qpos[1] = -0.785;  // joint 2 (-45 degrees)
    data->qpos[2] =  0.0;    // joint 3
    data->qpos[3] = -2.356;  // joint 4 (-135 degrees)
    data->qpos[4] =  0.0;    // joint 5
    data->qpos[5] =  1.571;  // joint 6 (90 degrees)
    data->qpos[6] =  0.785;  // joint 7 (45 degrees)

    int ee_body = mj_name2id(model, mjOBJ_BODY, "hand");

    // Forward kinematics to update the pose immediately
    mj_forward(model, data);

    // Init IKSolver
    IKSolver* ik_solver = new IKSolver(model, "hand");

    // Init GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(1200, 900, "Franka Panda Simulation", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Set callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);

    // Init MuJoCo visualization
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);
    mjcb_control = control;

    // Haply client
    HaplyClient haply_client;
    CoordinateTransformation coor_trans;
    haply_client.connect();

    double haply_px = 0.0, haply_py = 0.0, haply_pz = 0.0;
    double haply_fx = 0.0, haply_fy = 0.0, haply_fz = 0.0;
    double haply_ox = 0.0, haply_oy = 0.0, haply_oz = 0.0, haply_ow = 1.0;
    double franka_x = 0.0, franka_y = 0.0, franka_z = 0.0;

    // TODO: THESE ARE TEMP VARS, REMOVE THEM WHEN INVERSE3 IS WORKING
    double franka_home_x = 0.304;
    double franka_home_y = 0.000;
    double franka_home_z = 0.644;

    // Time control var
    std::chrono::steady_clock::time_point prev_time = std::chrono::steady_clock::now();
    int time_interval_ms = 500;

    // Run simulation loop
    while (!glfwWindowShouldClose(window)) {
        mj_step(model, data);

        coor_trans.inverseTransform(
            data->cfrc_ext[ee_body * 6 + 0],
            data->cfrc_ext[ee_body * 6 + 1],
            data->cfrc_ext[ee_body * 6 + 2],
            haply_fx, haply_fy, haply_fz
        );
        haply_client.sendForce(haply_fx, haply_fy, haply_fz);

        //haply_client.getPosition(haply_px, haply_py, haply_pz);
        haply_client.getOrientation(haply_ox, haply_oy, haply_oz, haply_ow);

        if (haply_client.isConnected()) {
            //coor_trans.transform(haply_px, haply_py, haply_pz, franka_x, franka_y, franka_z);
            ik_solver->solve(data, franka_home_x, franka_home_y, franka_home_z, // TODO: REPLACE franka_home VARS WITH franka_x, ... WHEN INVERSE3 WORKS
                             haply_ox, haply_oy, haply_oz, haply_ow, target_qpos);
        }

        std::chrono::steady_clock::time_point cur_time = std::chrono::steady_clock::now();
        int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - prev_time).count();
        if (elapsed >= time_interval_ms) {
            prev_time = cur_time;
            if (ee_body >= 0) {
                std::cout << "End-effector position: "
                        << data->xpos[ee_body * 3 + 0] << " "
                        << data->xpos[ee_body * 3 + 1] << " "
                        << data->xpos[ee_body * 3 + 2] << std::endl;
            } else {
                std::cout << "Body 'hand' not found!" << std::endl;
            }
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(data);
    mj_deleteModel(model);
    glfwTerminate();
    return 0;
}