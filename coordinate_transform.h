#ifndef COORDINATE_TRANSFORM_H
#define COORDINATE_TRANSFORM_H

class CoordinateTransformation {
    public:
        float home_x = 0.304;
        float home_y = 0.000;
        float home_z = 0.644;
        double scale = 2.0;

        // Scale Haply coordinates to Franka workspace
        void transform(double haply_x, double haply_y, double haply_z,
               double& franka_x, double& franka_y, double& franka_z) {
            franka_x = home_x + (haply_x * scale);
            franka_y = home_y + (haply_y * scale);
            franka_z = home_z + (haply_z * scale);
        }

        // Scale Franka forces and coordinates back to Haply workspace
        void inverseTransform(double franka_fx, double franka_fy, double franka_fz,
                      double& haply_fx, double& haply_fy, double& haply_fz) {
            haply_fx = franka_fx / scale;
            haply_fy = franka_fy / scale;
            haply_fz = franka_fz / scale;
        }
};

#endif