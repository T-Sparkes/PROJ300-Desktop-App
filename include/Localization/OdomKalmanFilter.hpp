#pragma once
#include <Eigen/Dense>
#include "ViewPortRenderable.hpp"

class OdomKalmanFilter : public ViewPortRenderable
{
public:
    Eigen::Vector3d x;  // State vector: [x, y, theta]
    Eigen::Matrix3d P;  // State covariance matrix
    Eigen::Matrix3d F;  // State transition matrix 
    Eigen::Matrix<double, 2, 3> H; // Measurement matrix (linearized)
    Eigen::Matrix3d Q; // Process noise covariance matrix
    Eigen::Matrix2d R; // Measurement noise covariance matrix
    Eigen::Matrix<double, 3, 2> K; // Kalman gain matrix
    
    Eigen::Vector2d anchorA;
    Eigen::Vector2d anchorB;

    double processNoise;
    double measurementNoise;

    float encoderA = 0;
    float encoderB = 0;

    Eigen::Vector2d h(const Eigen::Vector3d& state);

public:
    OdomKalmanFilter(Eigen::Vector3d initialState, double processNoise, double measurementNoise);
    void setAnchors(const Eigen::Vector2d& anchorA, const Eigen::Vector2d& anchorB);
    void predict(const Eigen::Vector2d &U, double dt);
    void update(const Eigen::Vector2d& measurement, double dt);
    void updateLandmark(char landmark, Eigen::Vector2d landmarkPos, double measurement);
    void setPoseEstimate(Eigen::Vector3d initialState);
    void render() override;
};
