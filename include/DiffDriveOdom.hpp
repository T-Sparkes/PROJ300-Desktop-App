#include "ViewPortRenderable.hpp"
#include "Eigen/Dense"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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


