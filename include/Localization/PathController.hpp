#pragma once

#include <deque>
#include <Eigen/Dense>
#include "Core/ViewPortRenderable.hpp"
#include <fstream>

// Class to handle placing waypoints and the selection of the path to follow
class PathController : public ViewPortRenderable
{
private:
    int m_CurrentWaypointIndex = 0; 

public:
    std::deque<Eigen::Vector2d> waypoints; 

    Eigen::Vector2d getNextWaypoint()
    {
        if (m_CurrentWaypointIndex < waypoints.size() && waypoints.size() > 0)
        {
            return waypoints[m_CurrentWaypointIndex++];
        }
        else if (m_CurrentWaypointIndex == waypoints.size() && waypoints.size() > 0)
        {
            m_CurrentWaypointIndex = 0; // Reset index to path start
            return waypoints[m_CurrentWaypointIndex++];
        }
        else
        {
            return Eigen::Vector2d(0, -1); // Return a default value if no waypoints are available
        }
    }

    void moveWaypoint(int index, const Eigen::Vector2d& newPos)
    {
        if (index >= 0 && index < waypoints.size())
        {
            waypoints[index] = newPos;
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PATHCONTROL INFO: Waypoint moved to: %.2f, %.2f\n", newPos.x(), newPos.y());
        }
    }

    void addWaypoint(const Eigen::Vector2d& waypoint)
    {
        waypoints.push_back(waypoint);
        printf("Waypoint added at: %.2f, %.2f\n", waypoint.x(), waypoint.y());
    }

    void removeWaypointNear(Eigen::Vector2d& position)
    {
        for (auto it = waypoints.begin(); it != waypoints.end(); ++it)
        {
            if ((it->head(2) - position).norm() < 0.1) // Check if the waypoint is within 10cm
            {
                waypoints.erase(it);
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PATHCONTROL INFO: Waypoint removed at: %.2f, %.2f\n", position.x(), position.y());
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
        if (waypoints.size() > 1)
        {
            for (size_t i = 0; i < waypoints.size() - 1; ++i)
            {
                viewport.RenderLineTexture(waypoints[i], waypoints[i + 1], 0.025, YELLOW, 100); // Render the line between waypoints
            }

            // Draw line between last a first waypoint
            viewport.RenderLineTexture(waypoints.back(), waypoints.front(), 0.025, YELLOW, 100); // Render the line between last and first waypoint
        }

        // Render the waypoints as circles
        for (const auto& waypoint : waypoints)
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

    void savePath(const std::string& pathName)
    {
        // Save to binary file
        std::ofstream file(pathName, std::ios::binary);
        if (file.is_open())
        {
            size_t waypointCount = waypoints.size();
            file.write(reinterpret_cast<const char*>(&waypointCount), sizeof(waypointCount)); // Write the number of waypoints

            for (const auto& waypoint : waypoints)
            {
                file.write(reinterpret_cast<const char*>(&waypoint[0]), sizeof(double)); 
                file.write(reinterpret_cast<const char*>(&waypoint[1]), sizeof(double)); 
            }

            file.close();
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PATHCONTROL INFO: Path saved to %s\n", pathName.c_str());
        }
        else
        {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PATHCONTROL ERROR: Unable to open file %s for writing\n", pathName.c_str());
        }
    }

    void loadPath(const std::string& pathName)
    {
        // Load waypoints from file
        std::ifstream file(pathName, std::ios::binary);
        if (file.is_open())
        {
            waypoints.clear(); // Clear existing waypoints
            size_t waypointCount;
            file.read(reinterpret_cast<char*>(&waypointCount), sizeof(waypointCount)); // Read number of waypoints

            for (size_t i = 0; i < waypointCount; ++i)
            {
                Eigen::Vector2d waypoint;
                file.read(reinterpret_cast<char*>(&waypoint[0]), sizeof(double)); 
                file.read(reinterpret_cast<char*>(&waypoint[1]), sizeof(double)); 
                waypoints.push_back(waypoint);
            }

            file.close();
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PATHCONTROL INFO: Path loaded from %s\n", pathName.c_str());
        }
        else
        {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PATHCONTROL ERROR: Unable to open file %s for reading\n", pathName.c_str());
        }
    }
};


