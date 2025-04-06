#pragma once

#include <deque>
#include <Eigen/Dense>
#include "Core/ViewPortRenderable.hpp"

// Class to handle placing waypoints and the selection of the path to follow
class PathController : public ViewPortRenderable
{
private:
    std::deque<Eigen::Vector2d> m_Waypoints; // List of waypoints to follow
    int m_CurrentWaypointIndex = 0; // Index of the current waypoint

public:
    PathController(/* args */)
    {

    }

    ~PathController()
    {

    }

    Eigen::Vector2d getNextWaypoint()
    {
        if (m_CurrentWaypointIndex < m_Waypoints.size() && m_Waypoints.size() > 0)
        {
            return m_Waypoints[m_CurrentWaypointIndex++];
        }
        else if (m_CurrentWaypointIndex == m_Waypoints.size() && m_Waypoints.size() > 0)
        {
            m_CurrentWaypointIndex = 0; // Reset the index to loop through waypoints again
            return m_Waypoints[m_CurrentWaypointIndex++];
        }
        else
        {
            return Eigen::Vector2d(0, -1); // Return a default value if no waypoints are available
        }
    }

    void addWaypoint(const Eigen::Vector2d& waypoint)
    {
        m_Waypoints.push_back(waypoint);
        printf("Waypoint added at: %.2f, %.2f\n", waypoint.x(), waypoint.y());
    }

    void removeWaypointNear(Eigen::Vector2d& position)
    {
        for (auto it = m_Waypoints.begin(); it != m_Waypoints.end(); ++it)
        {
            if ((it->head(2) - position).norm() < 0.1) // Check if the waypoint is within a certain distance
            {
                m_Waypoints.erase(it);
                printf("Waypoint removed at: %.2f, %.2f\n", position.x(), position.y());
                break;
            }
        }
    }
    
    bool isMouseOverWaypoint(Eigen::Vector2d waypoint)
    {
        Eigen::Vector2d mousePosWorld = ViewPort::GetInstance().GetCamera().transform.inverse() * ViewPort::GetInstance().GetViewPortMousePos();
        return (waypoint.head(2) - mousePosWorld).norm() < 0.1; // Check if the mouse is over a waypoint
    }

    void render() override
    {
        ViewPort& viewport = ViewPort::GetInstance();

        //Draw lines between waypoints
        if (m_Waypoints.size() > 1)
        {
            for (size_t i = 0; i < m_Waypoints.size() - 1; ++i)
            {
                viewport.RenderLineTexture(m_Waypoints[i], m_Waypoints[i + 1], 0.025, YELLOW, 100); // Render the line between waypoints
            }

            // Draw line between last a first waypoint
            viewport.RenderLineTexture(m_Waypoints.back(), m_Waypoints.front(), 0.025, YELLOW, 100); // Render the line between last and first waypoint
        }

        // Render the waypoints as circles
        for (const auto& waypoint : m_Waypoints)
        {
            // Render each waypoint as a circle
            if (isMouseOverWaypoint(waypoint))
            {
                viewport.RenderTexture(viewport.circleTexture, waypoint, {0.05, 0.05}, 0, WHITE, 255);
            }
            else
            {
                viewport.RenderTexture(viewport.circleTexture, waypoint, {0.05, 0.05}, 0, YELLOW, 255);
            }    
        }
    }
};


