#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <string>
#include <array>
#include <cmath>
#include <algorithm>
#include "franka_ik_He.hpp"

class IKSolver {
public:
    IKSolver(mjModel* model, const std::string& body_name)
        : model(model) {
        body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    }

    void solve(mjData* data, double target_x, double target_y, double target_z,
               double target_qx, double target_qy, double target_qz, double target_qw,
               double* target_qpos) {

        // Build 4x4 homogeneous transform from target position and orientation
        Eigen::Quaterniond q(target_qw, target_qx, target_qy, target_qz);
        q.normalize();
        Eigen::Matrix3d R = q.toRotationMatrix();

        // O_T_EE as column-major 4x4 array (MuJoCo/libfranka convention)
        std::array<double, 16> O_T_EE = {
            R(0,0), R(1,0), R(2,0), 0,
            R(0,1), R(1,1), R(2,1), 0,
            R(0,2), R(1,2), R(2,2), 0,
            target_x, target_y, target_z, 1
        };

        // Current joint angles
        std::array<double, 7> q_actual;
        for (int i = 0; i < 7; i++) {
            q_actual[i] = data->qpos[i];
        }

        // Use q7 (last joint) from current configuration as redundancy parameter
        double q7 = data->qpos[6];

        // Solve IK - case consistent to avoid jumps
        std::array<double, 7> result = franka_IK_EE_CC(O_T_EE, q7, q_actual);

        // Check for valid solution (NaN means no solution found)
        for (int i = 0; i < 7; i++) {
            if (!std::isnan(result[i])) {
                target_qpos[i] = result[i];
            }
            // If NaN, keep previous target_qpos unchanged
        }
    }

private:
    mjModel* model;
    int body_id;
};

#endif