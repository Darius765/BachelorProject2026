#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <string>

class IKSolver {
public:
    IKSolver(mjModel* model, const std::string& body_name)
        : model(model) {
        body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
        initialized = false;
        lock_x = 0.0;
        lock_y = 0.0;
        lock_z = 0.0;
    }

    // Returns corrected joint positions via target_qpos
    void solve(mjData* data, double target_x, double target_y, double target_z,
               double target_qx, double target_qy, double target_qz, double target_qw,
               double* target_qpos, double dt = 0.1) {

        int nv = model->nv;

        // Get current EE position
        double cur_pos_x = data->xpos[body_id * 3 + 0];
        double cur_pos_y = data->xpos[body_id * 3 + 1];
        double cur_pos_z = data->xpos[body_id * 3 + 2];

        if (!initialized) {
            lock_x = cur_pos_x;
            lock_y = cur_pos_y;
            lock_z = cur_pos_z;
            initialized = true;
        }

        // Compute position error
        Eigen::Vector3d pos_error;
        pos_error(0) = lock_x - cur_pos_x;
        pos_error(1) = lock_y - cur_pos_y;
        pos_error(2) = lock_z - cur_pos_z;
        pos_error(0) = target_x - cur_pos_x;
        pos_error(1) = target_y - cur_pos_y;
        pos_error(2) = target_z - cur_pos_z;


        // Get current EE orientation
        double cur_ori_x = data->xquat[body_id * 4 + 1];
        double cur_ori_y = data->xquat[body_id * 4 + 2];
        double cur_ori_z = data->xquat[body_id * 4 + 3];
        double cur_ori_w = data->xquat[body_id * 4 + 0];
        
        // Compute quaterion error
        Eigen::Quaterniond q_cur(cur_ori_w, cur_ori_x, cur_ori_y, cur_ori_z);
        Eigen::Quaterniond q_tgt(target_qw, target_qx, target_qy, target_qz);

        Eigen::Quaterniond q_err = q_tgt * q_cur.inverse();

        Eigen::Vector3d ori_error = q_err.vec();

        // Use shortest rotation path of the two possible paths
        if (q_err.w() < 0) ori_error = -ori_error;

        double ori_error_norm = ori_error.norm();
        if (ori_error_norm > 0.5) {
            ori_error *= (0.5 / ori_error_norm);
        }

        // Create 6-dimensional error vector
        Eigen::VectorXd error(6);
        error.head(3) = pos_error * 10;
        error.tail(3) = ori_error * 0.1;

        // Get the full Jacobian
        Eigen::MatrixXd jac_pos = Eigen::MatrixXd::Zero(3, nv);
        Eigen::MatrixXd jac_ori = Eigen::MatrixXd::Zero(3, nv);
        mj_jacBody(model, data, jac_pos.data(), jac_ori.data(), body_id);

        // Stack into 6 x nv Jacobian
        Eigen::MatrixXd J(6, nv);
        J.topRows(3) = jac_pos;
        J.bottomRows(3) = jac_ori;

        // Calculate damped psuedoinverse
        double damping = 0.01;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

        Eigen::VectorXd s = svd.singularValues();
        Eigen::MatrixXd sigma_inv = Eigen::MatrixXd::Zero(6, 6);
        for (int i = 0; i < s.size(); i++) {
            double si = s(i);
            sigma_inv(i, i) = si / (si * si + damping * damping);
        }
        Eigen::MatrixXd J_pinv = svd.matrixV() * sigma_inv * svd.matrixU().transpose();

        // Compute joint corrections
        Eigen::VectorXd dq = J_pinv * error;

        // Update target joint positions
        for (int i = 0; i < 7; i++) {
            target_qpos[i] = data->qpos[i] + dq(i) * dt;
        }
    }

private:
    mjModel* model;
    int body_id;
    bool initialized;
    double lock_x, lock_y, lock_z;
};

#endif