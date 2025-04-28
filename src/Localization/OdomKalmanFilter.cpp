#define _USE_MATH_DEFINES
#include <math.h>
#include "OdomKalmanFilter.hpp"

OdomKalmanFilter::OdomKalmanFilter(Eigen::Vector3d initialState, double processNoise, double measurementNoise)
{
    x = initialState;
    this->processNoise = processNoise;
    this->measurementNoise = measurementNoise;

    P.setIdentity();
    Q.setIdentity();
    R.setIdentity();

    Q *= processNoise;
    R *= measurementNoise;
}

void OdomKalmanFilter::setAnchors(const Eigen::Vector2d& anchorA, const Eigen::Vector2d& anchorB) 
{
    this->anchorA = anchorA;
    this->anchorB = anchorB;
}

// Take in wheel encoder data and dt
void OdomKalmanFilter::predict(const Eigen::Vector2d& U, double dt)
{
    Q.setIdentity();
    Q *= processNoise;
    Q(Q.rows() - 1, Q.cols() - 1) = 1e-4; // 1e-6

    const float chassisWidth = 0.173f;
    const float wheelRadius = 0.03f;

    float dL = static_cast<float>((U[0] - encoderA) * wheelRadius); //New encoder - old encoder value
    float dR = static_cast<float>((U[1] - encoderB) * wheelRadius);

    // Convert encoder values to distance traveled
    float d = (dL + dR) / 2.0f;
    float dTheta = (dR - dL) / (chassisWidth);

    // Save for next prediction step
    encoderA = static_cast<float>(U[0]); 
    encoderB = static_cast<float>(U[1]);

    // Jacobian of the motion model
    F << 1, 0, -d * sin(x.z() + dTheta / 2.0f),
         0, 1,  d * cos(x.z() + dTheta / 2.0f),
         0, 0,  1;

    // nonlinear motion model (differential drive)
    x.x() += d * cos(x.z() + dTheta / 2.0f);
    x.y() += d * sin(x.z() + dTheta / 2.0f);
    x.z() += dTheta;

    P = F * P * F.transpose() + Q;
}

void OdomKalmanFilter::batchUpdate(const Eigen::Vector2d& measurement,  double dt) 
{
    R.setIdentity();
    R *= measurementNoise;

    // current state
    double x_pos = x(0);  
    double y_pos = x(1); 
    double theta = x(2); 

    // distances to the anchors, predicted ranges
    double r_a = std::sqrt(std::pow(x_pos - anchorA(0), 2) + std::pow(y_pos - anchorA(1), 2));
    double r_b = std::sqrt(std::pow(x_pos - anchorB(0), 2) + std::pow(y_pos - anchorB(1), 2));

    // partial derivatives for r_a and r_b wrt x and y
    double d_rA_dx = (x_pos - anchorA(0)) / r_a;
    double d_rA_dy = (y_pos - anchorA(1)) / r_a;
    double d_rB_dx = (x_pos - anchorB(0)) / r_b;
    double d_rB_dy = (y_pos - anchorB(1)) / r_b;

    // Jacobian matrix H
    H << d_rA_dx, d_rA_dy, 0,
         d_rB_dx, d_rB_dy, 0;

    // Kalman gain           
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // State update
    x = x + K * (measurement - h(x));

    // Covariance update
    P = (Eigen::MatrixXd::Identity(3, 3) - K * H) * P;
}

void OdomKalmanFilter::updateLandmark(char landmark, Eigen::Vector2d landmarkPos,  double measurement)
{
    // Local R for scalar measurement
    Eigen::MatrixXd R_(1,1);
    R_ << measurementNoise;

    // Current state
    double x_pos = x(0);  
    double y_pos = x(1); 

    // Expected measurement
    double dx = x_pos - landmarkPos(0);
    double dy = y_pos - landmarkPos(1);
    double h_x = std::sqrt(dx * dx + dy * dy);
    if (h_x < 1e-6) h_x = 1e-6;

    // Jacobian matrix H for one landmark
    Eigen::MatrixXd H_(1, 3); 
    H_ << dx / h_x, dy / h_x, 0;

    // Kalman gain (3x1)
    Eigen::Vector3d K_ = P * H_.transpose() * (H_ * P * H_.transpose() + R_).inverse();

    // State update
    x = x + K_ * (measurement - h_x);

    // Covariance update
    P = (Eigen::MatrixXd::Identity(3, 3) - K_ * H_) * P;

    // Update full K matrix for visualization
    if (landmark == 'A') K.col(0) = K_;
    else if (landmark == 'B') K.col(1) = K_;
}

void OdomKalmanFilter::setPoseEstimate(Eigen::Vector3d initialState)
{
    x = initialState;
}

Eigen::Vector2d OdomKalmanFilter::h(const Eigen::Vector3d& state) 
{
    double r_a = sqrt((state(0) - anchorA(0)) * (state(0) - anchorA(0)) + (state(1) - anchorA(1)) * (state(1) - anchorA(1)));
    double r_b = sqrt((state(0) - anchorB(0)) * (state(0) - anchorB(0)) + (state(1) - anchorB(1)) * (state(1) - anchorB(1)));

    Eigen::Vector2d measurement;
    measurement << r_a, r_b;
    return measurement;
}

void OdomKalmanFilter::render() 
{
    Eigen::Matrix2d covariance = P.block<2,2>(0, 0); // 2x2 part of the covariance matrix for x and y
    
    // Validate covariance matrix
    if (covariance.allFinite()) 
    {
        Eigen::EigenSolver<Eigen::Matrix2d> solver(covariance);

        Eigen::Vector2d eigenvalues = solver.eigenvalues().real();
        Eigen::Matrix2d eigenvectors = solver.eigenvectors().real();

        double std_dev_x = std::sqrt(eigenvalues(0)); // Standard deviation for the x-axis
        double std_dev_y = std::sqrt(eigenvalues(1)); // Standard deviation for the y-axis

        // rotation of the ellipse from first eigenvector
        double angle = std::atan2(eigenvectors(1, 0), eigenvectors(0, 0));
        
        // Use standard deviations as the ellipse size
        double ellipse_width = 2 * std_dev_x;  
        double ellipse_height = 2 * std_dev_y; 
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, x.head(2), {ellipse_width, ellipse_height}, -angle, YELLOW, 50);
    }

    else
    {
        printf("KALMAN ERROR: Covariance matrix invalid\n");
    }
    
    ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().robotTexture, x.head(2), {0.173, 0.173}, -x.z() + M_PI_2, WHITE, 255);
}