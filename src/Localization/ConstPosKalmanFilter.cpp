

#define _USE_MATH_DEFINES
#include <math.h>
#include <random>
#include "ConstPosKalmanFilter.hpp"

ConstPosKalmanFilter::ConstPosKalmanFilter(const Eigen::Vector2d initialState, double processNoise, double measurementNoise)
    : processNoise(processNoise), measurementNoise(measurementNoise)
{
    x = initialState;

    P.setIdentity();
    Q.setIdentity();
    R.setIdentity();

    Q *= processNoise;
    R *= measurementNoise;
}

void ConstPosKalmanFilter::setAnchors(const Eigen::Vector2d& anchorA, const Eigen::Vector2d& anchorB) 
{
    this->anchorA = anchorA;
    this->anchorB = anchorB;
}

void ConstPosKalmanFilter::predict(const Eigen::Vector2d& U, double dt) 
{
    // identity state transition
    F.setIdentity();
    P = F * P * F.transpose() + Q;
}

void ConstPosKalmanFilter::update(const Eigen::Vector2d& measurement, double dt) 
{
    // Measurement noise covariance matrix (2x2)
    R.setIdentity();
    R *= measurementNoise;

    // State vector (2x1)
    double x_pos = x(0);
    double y_pos = x(1);

    // Expected measurement
    double r_a = std::sqrt(std::pow(x_pos - anchorA(0), 2) + std::pow(y_pos - anchorA(1), 2));
    double r_b = std::sqrt(std::pow(x_pos - anchorB(0), 2) + std::pow(y_pos - anchorB(1), 2));

    // Jacobian matrix (2x2)
    double d_rA_dx = (x_pos - anchorA(0)) / r_a;
    double d_rA_dy = (y_pos - anchorA(1)) / r_a;
    double d_rB_dx = (x_pos - anchorB(0)) / r_b;
    double d_rB_dy = (y_pos - anchorB(1)) / r_b;

    H << d_rA_dx, d_rA_dy,
         d_rB_dx, d_rB_dy;

    // Kalman gain (2x2)
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Update state estimate (2x1)
    x = x + K * (measurement - h(x));

    // Update error covariance matrix (2x2)
    P = (Eigen::Matrix2d::Identity() - K * H) * P;
}

void ConstPosKalmanFilter::render() 
{
    ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, x, {0.1, 0.1}, 0.0, YELLOW, 255);
    ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, {x.x(), -x.y()}, {0.1, 0.1}, 0.0, YELLOW, 255);
}

Eigen::Vector2d ConstPosKalmanFilter::getStateEstimate() 
{
    return x;
}

double& ConstPosKalmanFilter::GetProcessNoiseRef() 
{
    return processNoise;
}

double& ConstPosKalmanFilter::GetProcessNoiseThetaRef() 
{
    return processNoiseTheta;
}

double& ConstPosKalmanFilter::GetMeasureNoiseRef() 
{
    return measurementNoise;
}

Eigen::Vector2d ConstPosKalmanFilter::h(const Eigen::Vector2d& state) 
{
    double r_a = sqrt((state(0) - anchorA(0)) * (state(0) - anchorA(0)) + (state(1) - anchorA(1)) * (state(1) - anchorA(1)));
    double r_b = sqrt((state(0) - anchorB(0)) * (state(0) - anchorB(0)) + (state(1) - anchorB(1)) * (state(1) - anchorB(1)));

    Eigen::Vector2d measurement;
    measurement << r_a, r_b;
    return measurement;
}