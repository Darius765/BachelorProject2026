#ifndef HAPLY_H
#define HAPLY_H

class Haply {
public:
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void getPosition(double& x, double& y, double& z) = 0;
    virtual void sendForce(double x_force, double y_force, double z_force) = 0;

    virtual ~Haply() = default;
};

#endif