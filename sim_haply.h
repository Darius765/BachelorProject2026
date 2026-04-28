#ifndef SIM_HAPLY_H
#define SIM_HAPLY_H

#include "haply.h"
#include <cmath>
#include <iostream>

class SimHaply: public Haply {
    public:
        void connect() {
            std::cout << "Connect was called" << std::endl;
        }
        void disconnect() { 
            std::cout << "Disconnect was called" << std::endl;
        }
        void getPosition(double& x, double& y, double& z) {
            std::cout << "getPosition was called" << std::endl;
            time += 0.01;
            x = cos(time);
            y = sin(time);
            z = 0.1 * sin(time);
        }
        void sendForce(double x_force, double y_force, double z_force) {
            std::cout << "SendForce was called with:" << std::endl;
            std::cout << "x = " << x_force << "  y = " << y_force << "  z = " << z_force << std::endl;
        }

    private:
        double time = 0.0;
};

#endif