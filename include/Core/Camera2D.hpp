#pragma once
#include "Eigen/Dense"

class Camera2D {
private:
    int scale = 100; // Pixels per meter
    Eigen::Vector2d position; // Camera2D position (world coordinates)
    Eigen::Vector2d screenSize;
    Eigen::Vector2d fPanStart;

public:
    double rotation = 0.0;
    Eigen::Affine2d transform; // Affine transformation (scale + translation)

public:
    Camera2D(const Eigen::Vector2d& screenSize, const Eigen::Vector2d& position = {0, 0}, int scale = 100);
    Camera2D() = default;

    void setScreenSize(Eigen::Vector2d newSize);
    void setScale(int newScale);
    void setPosition(const Eigen::Vector2d& newPosition);
    void setRotation(double angle);
    void followObject(Eigen::Affine2d objTransform);
    void updateZoom(Eigen::Vector2d zoomOrigin, float MouseWheelChange);
    void setPanStart(Eigen::Vector2d& fMousePosWorld);
    void panCamera(Eigen::Vector2d& fMousePosWorld);
    void updateTransform();

    inline int getScale() const { return scale; };
    inline Eigen::Vector2d getPosition() const { return position; };
};


