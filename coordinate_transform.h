#ifndef COORDINATE_TRANSFORM_H
#define COORDINATE_TRANSFORM_H

class CoordinateTransformation {
public:
    CoordinateTransformation() {
        // Franka home position
        home_x = 0.304;
        home_y = 0.000;
        home_z = 0.644;

        // Scale factor
        scale = 0.5;

        // Neutral not set yet
        neutral_set = false;
        haply_neutral_x = 0.0;
        haply_neutral_y = 0.0;
        haply_neutral_z = 0.0;
    }

    // Call this once at startup with the first position reading
    void setNeutral(double hx, double hy, double hz) {
        haply_neutral_x = hx;
        haply_neutral_y = hy;
        haply_neutral_z = hz;
        neutral_set = true;
        std::cout << "Neutral position set: "
                  << hx << " " << hy << " " << hz << std::endl;
    }

    bool isNeutralSet() { return neutral_set; }

    void transform(double hx, double hy, double hz,
                   double& fx, double& fy, double& fz) {
        // Relative movement from neutral
        double rel_x = hx - haply_neutral_x;
        double rel_y = hy - haply_neutral_y;
        double rel_z = hz - haply_neutral_z;

        // Apply axis mapping and scaling
        fx = home_x + (-rel_x * scale);
        fy = home_y + (-rel_y * scale);
        fz = home_z + ( rel_z * scale);
    }

    void inverseTransform(double ffx, double ffy, double ffz,
                          double& hfx, double& hfy, double& hfz) {
        hfx = -ffx / scale;
        hfy = -ffy / scale;
        hfz =  ffz / scale;
    }

private:
    double home_x, home_y, home_z;
    double haply_neutral_x, haply_neutral_y, haply_neutral_z;
    double scale;
    bool neutral_set;
};

#endif
