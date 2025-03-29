#pragma once

#include <Eigen/Dense>
#include <iostream>
#include "Core/ViewPortRenderable.hpp"

class ConstPosKalmanFilter : ViewPortRenderable
{
public:
    ConstPosKalmanFilter(const Eigen::Vector2d initialState, double processNoise, double measurementNoise);

    void setAnchors(const Eigen::Vector2d& anchorA, const Eigen::Vector2d& anchorB);
    void predict(const Eigen::Vector2d &U, double dt);
    void update(const Eigen::Vector2d& measurement, double dt);
    void render() override;
    
    Eigen::Vector2d getStateEstimate();
    double& GetProcessNoiseRef();
    double& GetProcessNoiseThetaRef();
    double& GetMeasureNoiseRef();

public:
    Eigen::Vector2d x;
    Eigen::Matrix2d P;
    Eigen::Matrix2d Q;
    Eigen::Matrix2d R;
    Eigen::Matrix2d F;
    Eigen::Matrix2d H;
    Eigen::Matrix2d K;

    Eigen::Vector2d x_pred;
    Eigen::Matrix2d P_pred;

    Eigen::Vector2d anchorA;
    Eigen::Vector2d anchorB;

    double processNoise;
    double processNoiseTheta = 1e-6;
    double measurementNoise;

    Eigen::Vector2d h(const Eigen::Vector2d& state);
};
