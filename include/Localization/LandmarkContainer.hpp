#pragma once

#include <Eigen/Dense>
#include <random>
#include <vector>

#include "Core/ViewPortRenderable.hpp"

#define LANDMARK_A_CALIBRATION 0.75f + 0.375f
#define LANDMARK_B_CALIBRATION 0.75f + 0.3f

class LandmarkContainer : public ViewPortRenderable
{
private:
    Eigen::Vector2d m_LandmarkPosA;
    Eigen::Vector2d m_LandmarkPosB;
    Eigen::Vector2d m_posEstimateA;
    Eigen::Vector2d m_posEstimateB;

public:
    double rangeA = 0;
    double rangeB = 0;

    int rangeAlpha = 20;

    bool bDrawRange = true;
    bool bDrawLandmarks = true;
    bool bDrawRawPos = false;

    LandmarkContainer() = default;
    LandmarkContainer(Eigen::Vector2d m_LandmarkPosA_, Eigen::Vector2d LandmarkPosB_);

    void OnNewPacket(LandmarkPacket *packet);
    void calculatePos(bool printDebug = false);
    void updateRange(double _rangeA, double _rangeB);
    void simulateRange(Eigen::Vector2d realPosition, double sttdev);
    void SetLandmarkPos(char Landmark, Eigen::Vector2d newPos);
    
    Eigen::Vector2d getPosEstimate(bool printDebug = false);
    Eigen::Vector2d getLandmarkPos(char Landmark);
    double getLandmarkRange(char Landmark);

    void render();
};

inline LandmarkContainer::LandmarkContainer(Eigen::Vector2d m_LandmarkPosA_, Eigen::Vector2d LandmarkPosB_) 
    : m_LandmarkPosA(m_LandmarkPosA_), m_LandmarkPosB(LandmarkPosB_) {}

inline Eigen::Vector2d LandmarkContainer::getLandmarkPos(char Landmark)
{
    if(Landmark == 'A'){return m_LandmarkPosA;}
    else if(Landmark == 'B'){return m_LandmarkPosB;}
    else return {0, 0};
}

inline double LandmarkContainer::getLandmarkRange(char Landmark)
{
    if(Landmark =='A'){return rangeA;}
    else if(Landmark == 'B'){return rangeB;}
    else return 0;
}

inline void LandmarkContainer::SetLandmarkPos(char Landmark, Eigen::Vector2d newPos)
{
    if(Landmark =='A')
    {
        m_LandmarkPosA = newPos;
    }
    else if(Landmark == 'B')
    {
        m_LandmarkPosB = newPos;
    }
}

inline void LandmarkContainer::render()
{
    calculatePos();
    if(bDrawRange)
    {
        //Draw Landmark Range
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_LandmarkPosA, Eigen::Vector2d(2.0 * rangeA, 2.0 * rangeA), 0, RED, rangeAlpha);
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_LandmarkPosB, Eigen::Vector2d(2.0 * rangeB, 2.0 * rangeB), 0, GREEN, rangeAlpha);
    }

    if (bDrawLandmarks)
    {
        //Draw Landmarks
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_LandmarkPosA, Eigen::Vector2d(0.1, 0.1), 0, RED, SDL_ALPHA_OPAQUE);
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_LandmarkPosB, Eigen::Vector2d(0.1, 0.1), 0, GREEN, SDL_ALPHA_OPAQUE);
    }

    if (bDrawRawPos)
    {
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_posEstimateA, Eigen::Vector2d(0.1, 0.1), 0, BLUE, SDL_ALPHA_OPAQUE);
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_posEstimateB, Eigen::Vector2d(0.1, 0.1), 0, BLUE, SDL_ALPHA_OPAQUE);
    }    
}

inline void LandmarkContainer::simulateRange(Eigen::Vector2d realPosition, double sttdev)
{
    // random number generator
    static std::random_device rd;
    static std::mt19937 gen(rd()); 

    std::normal_distribution<double> gaussianDist(0, sttdev);

    rangeA = (realPosition - m_LandmarkPosA).norm();
    rangeB = (realPosition - m_LandmarkPosB).norm();

    rangeA += gaussianDist(gen);
    rangeB += gaussianDist(gen);
}

inline void LandmarkContainer::updateRange(double _rangeA, double _rangeB)
{
    rangeA = _rangeA;
    rangeB = _rangeB;
}

inline void LandmarkContainer::OnNewPacket(LandmarkPacket *packet)
{
    if (packet->LandmarkID == 'A')
    {   
        // Calculate horizontal range if anchor is 75cm above the ground
        float correctedRange = sqrtf(powf(packet->range * LANDMARK_A_CALIBRATION, 2) - powf(0.0f, 2));
        if (correctedRange > 0 && correctedRange < 10)
        {
            rangeA = correctedRange;
        }
    }

    else if (packet->LandmarkID == 'B')
    {   
        // Calculate horizontal range if anchor is 75cm above the ground
        float correctedRange = sqrtf(powf(packet->range * LANDMARK_B_CALIBRATION, 2) - powf(0.0f, 2));
        if (correctedRange > 0 && correctedRange < 10)
        {
            rangeB = correctedRange;
        }
    }
}
inline void LandmarkContainer::calculatePos(bool printDebug)
{
    double d = sqrt(pow((m_LandmarkPosB.x() - m_LandmarkPosA.x()), 2.0) + pow((m_LandmarkPosB.y() - m_LandmarkPosA.y()), 2.0));
    double a = (pow(rangeA, 2.0) - pow(rangeB, 2.0) + pow(d, 2.0)) / (2.0 * d);
    double h = sqrt(pow(rangeA, 2.0) - pow(a, 2.0));

    double midPointX = m_LandmarkPosA.x() + a * (m_LandmarkPosB.x() - m_LandmarkPosA.x()) / d;
    double midPointY = m_LandmarkPosA.y() + a * (m_LandmarkPosB.y() - m_LandmarkPosA.y()) / d;

    double posXA = midPointX - h * (m_LandmarkPosB.y() - m_LandmarkPosA.y()) / d;
    double posYA = midPointY + h * (m_LandmarkPosB.x() - m_LandmarkPosA.x()) / d;

    double posXB = midPointX + h * (m_LandmarkPosB.y() - m_LandmarkPosA.y()) / d;
    double posYB = midPointY - h * (m_LandmarkPosB.x() - m_LandmarkPosA.x()) / d;

    if(printDebug)
    {
        printf("d: %f, a: %f, h: %f X: %f, Y:, %f\n", d, a, h, midPointX, midPointY);
    }

    m_posEstimateA = {posXA, posYA};
    m_posEstimateB = {posXB, posYB};
}

inline Eigen::Vector2d LandmarkContainer::getPosEstimate(bool printDebug)
{
    calculatePos(printDebug);
    return m_posEstimateA;
}
