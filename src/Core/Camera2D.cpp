#include "Core/Camera2D.hpp"

Camera2D::Camera2D(const Eigen::Vector2d& screenSize, const Eigen::Vector2d& position, int scale)
    : screenSize(screenSize), position(position), scale(scale)
{
    updateTransform();
}

void Camera2D::setScreenSize(Eigen::Vector2d newSize)
{
    screenSize = newSize;
    updateTransform();
}

void Camera2D::setScale(int newScale) 
{
    scale = newScale;
    updateTransform();
}

void Camera2D::setPosition(const Eigen::Vector2d& newPosition) 
{
    position = newPosition;
    updateTransform();
}

void Camera2D::setRotation(double angle)
{
    rotation = angle;
    updateTransform();
}

void Camera2D::followObject(Eigen::Affine2d objTransform)
{
    position = objTransform.translation();
    rotation = atan2(objTransform.matrix()(1, 0), objTransform.matrix()(0, 0));
    updateTransform();
}

void Camera2D::updateZoom(Eigen::Vector2d zoomOrigin, float MouseWheelChange)
{
    Eigen::Vector2d zoomOriginPixel = transform * zoomOrigin;
    scale = static_cast<int>(scale * ((MouseWheelChange / abs(MouseWheelChange) > 0) ? 1.25 : 0.75));
    scale = (scale > 3779) ? 3779 : scale;
    scale = (scale < 5) ? 5 : scale;
    updateTransform();

    Eigen::Vector2d zoomEnd = transform.inverse() * zoomOriginPixel;
    position -= zoomEnd - zoomOrigin;
    updateTransform();
}

void Camera2D::setPanStart(Eigen::Vector2d& fMousePosWorld)
{
    fPanStart = fMousePosWorld;
}

void Camera2D::panCamera(Eigen::Vector2d& fMousePosWorld)
{
    position -= (fMousePosWorld - fPanStart);
    updateTransform();
}

void Camera2D::updateTransform() 
{
    Eigen::Vector2d worldOffset = Eigen::Scaling(1/(double)scale, -1/(double)scale) * (screenSize / 2.0);

    transform.setIdentity();
    transform.scale(Eigen::Vector2d(scale, -scale));  // Apply the scaling
    transform.translate(worldOffset);
    transform.rotate(-rotation);
    transform.translate(-this->position);  // Apply the translation 
}
