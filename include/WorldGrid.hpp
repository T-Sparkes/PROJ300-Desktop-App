#pragma once
#include <SDL3/SDL.h>
#include <Eigen/Dense>

#include "ViewPort.hpp"
#include "ViewPortRenderable.hpp"

class GridRenderer : public ViewPortRenderable
{
private:
    
public:
    Eigen::Vector2d gridCenter;
    Eigen::Vector2d gridSize;
    float gridStep = 1;
    bool bRender = true;
    GridRenderer(ViewPort* targetViewPort, Eigen::Vector2d gridCenter = {0, 0}, Eigen::Vector2d gridSize = {10, 10}) : gridCenter(gridCenter), gridSize(gridSize), ViewPortRenderable(targetViewPort) {}

    // Method to render the grid within the specified world space bounds
    inline void render() override
    {
        if(!bRender) { return; };

        Eigen::Vector2d lineStart;
        Eigen::Vector2d lineEnd;
        Eigen::Vector2d gridStart = gridCenter - (gridSize / 2.0);
        Eigen::Vector2d gridEnd = gridCenter + (gridSize / 2.0);

        SDL_SetRenderDrawColor(m_targetViewPort->GetSdlRenderer(), 128, 128, 128, 255);  // GREY color for grid lines

        // Render vertical grid lines
        for (double gridX = gridStart.x(); gridX <= gridEnd.x(); gridX += gridStep)
        {
            lineStart = m_targetViewPort->GetCamera().transform * Eigen::Vector2d(gridX, gridStart.y());
            lineEnd = m_targetViewPort->GetCamera().transform * Eigen::Vector2d(gridX, gridEnd.y());

            SDL_RenderLine(m_targetViewPort->GetSdlRenderer(), (float)lineStart.x(), (float)lineStart.y(), (float)lineEnd.x(), (float)lineEnd.y());
        }

        // Render horizontal grid lines
        for (double gridY = gridStart.y(); gridY <= gridEnd.y(); gridY += gridStep)
        {
            lineStart = m_targetViewPort->GetCamera().transform * Eigen::Vector2d(gridStart.x(), gridY);
            lineEnd = m_targetViewPort->GetCamera().transform * Eigen::Vector2d(gridEnd.x(), gridY);

            SDL_RenderLine(m_targetViewPort->GetSdlRenderer(), (float)lineStart.x(), (float)lineStart.y(), (float)lineEnd.x(), (float)lineEnd.y());
        }
    }
};
