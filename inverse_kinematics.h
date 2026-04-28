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
    }

    // Returns corrected joint positions via target_qpos
    void solve(mjData* data, double target_x, double target_y, double target_z,
               double* target_qpos, double dt = 0.002) {

        // Get current EE position
        double current_x = data->xpos[body_id * 3 + 0];
        double current_y = data->xpos[body_id * 3 + 1];
        double current_z = data->xpos[body_id * 3 + 2];

        // Compute position error
        Eigen::Vector3d error;
        error(0) = target_x - current_x;
        error(1) = target_y - current_y;
        error(2) = target_z - current_z;

        // Get Jacobian
        int nv = model->nv;
        Eigen::MatrixXd jacp = Eigen::MatrixXd::Zero(3, nv);
        Eigen::MatrixXd jacr = Eigen::MatrixXd::Zero(3, nv);
        mj_jacBody(model, data, jacp.data(), jacr.data(), body_id);

        // Compute pseudoinverse using SVD
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacp, 
            Eigen::ComputeThinU | Eigen::ComputeThinV);
        
        // Damped pseudoinverse for stability near singularities
        double damping = 0.01;
        Eigen::MatrixXd sigma = svd.singularValues().asDiagonal();
        Eigen::MatrixXd sigma_inv = Eigen::MatrixXd::Zero(3, 3);
        for (int i = 0; i < svd.singularValues().size(); i++) {
            double s = svd.singularValues()(i);
            sigma_inv(i, i) = s / (s * s + damping * damping);
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
};

#endif