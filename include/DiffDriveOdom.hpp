#include "Core/ViewPortRenderable.hpp"
#include "Eigen/Dense"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

inline Eigen::Vector2d wheelVelFromGoal(double x, double y, double theta, double targetX, double targetY); 

class DiffDriveOdom : public ViewPortRenderable
{
private:
    float encoderA = 0;
    float encoderB = 0;
    float chassisWidth = 0.173f;
    float wheelRadius = 0.03f;

    Eigen::Vector3d state;

public:
    DiffDriveOdom()
    {
        state.setZero(3);
    }

    ~DiffDriveOdom()
    {

    }

    Eigen::Vector3d getState()
    {
        return state;
    }

    void update(float newEncoderA, float newEncoderB)
    {
        float dL = (newEncoderA - encoderA) * wheelRadius;
        float dR = (newEncoderB - encoderB) * wheelRadius;
        encoderA = newEncoderA;
        encoderB = newEncoderB;

        float d = (dL + dR) / 2.0f;
        float dTheta = (dR - dL) / (chassisWidth);
        
        state.x() += d * cos(state.z() + dTheta / 2.0f);
        state.y() += d * sin(state.z() + dTheta / 2.0f);
        state.z() += dTheta;
    }

    void render() override 
    {
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().robotTexture, state.head(2), {chassisWidth, chassisWidth}, -state.z() + M_PI / 2.0, WHITE, SDL_ALPHA_OPAQUE);
    }
};

inline Eigen::Vector2d wheelVelFromGoal(double x, double y, double theta, double targetX, double targetY) 
{

    const double width = 0.173; 

    double targetTheta = atan2(targetY - y, targetX - x);
    double error = targetTheta - theta;

    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;

    double omega = 1.0 * error;

    double vForwards = 0.05;  
    double vL = vForwards - (width / 2.0) * omega;
    double vR = vForwards + (width / 2.0) * omega;

    double omegaL = vL / 0.03;
    double omegaR = vR / 0.03;

    return {omegaL, omegaR}; 
}