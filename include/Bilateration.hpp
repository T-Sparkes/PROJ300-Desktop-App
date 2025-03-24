#pragma once

#include <Eigen/Dense>
#include <random>
#include <vector>

#include "Core/ViewPortRenderable.hpp"

class Bilateration : public ViewPortRenderable
{
private:
    Eigen::Vector2d m_anchorPosA;
    Eigen::Vector2d m_anchorPosB;
    Eigen::Vector2d m_posEstimateA;
    Eigen::Vector2d m_posEstimateB;

public:
    double rangeA = 0;
    double rangeB = 0;

    int rangeAlpha = 5;

    bool bDrawRange = true;
    bool bDrawAnchors = true;
    bool bDrawRawPos = false;

    Bilateration() = default;
    Bilateration(Eigen::Vector2d m_anchorPosA_, Eigen::Vector2d anchorPosB_);
    ~Bilateration();
    void calculatePos(bool printDebug = false);
    Eigen::Vector2d getPosEstimate(bool printDebug = false);
    void updateRange(double _rangeA, double _rangeB);
    void simulateRange(Eigen::Vector2d realPosition, double sttdev);
    Eigen::Vector2d getAnchorPos(char anchor);
    double getAnchorRange(char anchor);
    void SetAnchorPos(char anchor, Eigen::Vector2d newPos);

    void render();
};

inline Bilateration::Bilateration(Eigen::Vector2d m_anchorPosA_, Eigen::Vector2d anchorPosB_) 
    : m_anchorPosA(m_anchorPosA_), m_anchorPosB(anchorPosB_) {}

inline Bilateration::~Bilateration(){}

inline Eigen::Vector2d Bilateration::getAnchorPos(char anchor)
{
    if(anchor == 'A'){return m_anchorPosA;}
    else if(anchor == 'B'){return m_anchorPosB;}
    else return {0, 0};
}

inline double Bilateration::getAnchorRange(char anchor)
{
    if(anchor =='A'){return rangeA;}
    else if(anchor == 'B'){return rangeB;}
    else return 0;
}

inline void Bilateration::SetAnchorPos(char anchor, Eigen::Vector2d newPos)
{
    if(anchor =='A')
    {
        m_anchorPosA = newPos;
    }
    else if(anchor == 'B')
    {
        m_anchorPosB = newPos;
    }
}

inline void Bilateration::render()
{
    calculatePos();
    if(bDrawRange)
    {
        //Draw Anchor Range
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_anchorPosA, Eigen::Vector2d(2.0 * rangeA, 2.0 * rangeA), 0, RED, rangeAlpha);
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_anchorPosB, Eigen::Vector2d(2.0 * rangeB, 2.0 * rangeB), 0, GREEN, rangeAlpha);
    }

    if (bDrawAnchors)
    {
        //Draw Anchors
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_anchorPosA, Eigen::Vector2d(0.1, 0.1), 0, RED, SDL_ALPHA_OPAQUE);
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_anchorPosB, Eigen::Vector2d(0.1, 0.1), 0, GREEN, SDL_ALPHA_OPAQUE);
    }

    if (bDrawRawPos)
    {
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_posEstimateA, Eigen::Vector2d(0.1, 0.1), 0, BLUE, SDL_ALPHA_OPAQUE);
        ViewPort::GetInstance().RenderTexture(ViewPort::GetInstance().circleTexture, m_posEstimateB, Eigen::Vector2d(0.1, 0.1), 0, BLUE, SDL_ALPHA_OPAQUE);
    }    
}

inline void Bilateration::simulateRange(Eigen::Vector2d realPosition, double sttdev)
{
    // random number generator
    static std::random_device rd;
    static std::mt19937 gen(rd()); 

    std::normal_distribution<double> gaussianDist(0, sttdev);

    rangeA = (realPosition - m_anchorPosA).norm();
    rangeB = (realPosition - m_anchorPosB).norm();

    rangeA += gaussianDist(gen);
    rangeB += gaussianDist(gen);
}

inline void Bilateration::updateRange(double _rangeA, double _rangeB)
{
    rangeA = _rangeA;
    rangeB = _rangeB;
}

inline void Bilateration::calculatePos(bool printDebug)
{
    double d = sqrt(pow((m_anchorPosB.x() - m_anchorPosA.x()), 2.0) + pow((m_anchorPosB.y() - m_anchorPosA.y()), 2.0));
    double a = (pow(rangeA, 2.0) - pow(rangeB, 2.0) + pow(d, 2.0)) / (2.0 * d);
    double h = sqrt(pow(rangeA, 2.0) - pow(a, 2.0));

    double midPointX = m_anchorPosA.x() + a * (m_anchorPosB.x() - m_anchorPosA.x()) / d;
    double midPointY = m_anchorPosA.y() + a * (m_anchorPosB.y() - m_anchorPosA.y()) / d;

    double posXA = midPointX - h * (m_anchorPosB.y() - m_anchorPosA.y()) / d;
    double posYA = midPointY + h * (m_anchorPosB.x() - m_anchorPosA.x()) / d;

    double posXB = midPointX + h * (m_anchorPosB.y() - m_anchorPosA.y()) / d;
    double posYB = midPointY - h * (m_anchorPosB.x() - m_anchorPosA.x()) / d;

    if(printDebug)
    {
        printf("d: %f, a: %f, h: %f X: %f, Y:, %f\n", d, a, h, midPointX, midPointY);
    }

    m_posEstimateA = {posXA, posYA};
    m_posEstimateB = {posXB, posYB};
}

inline Eigen::Vector2d Bilateration::getPosEstimate(bool printDebug)
{
    calculatePos(printDebug);
    return m_posEstimateA;
}
