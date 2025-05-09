#pragma once
#include <imgui.h>
#include <Eigen/Dense>  
#include "Core/ViewPort.hpp"
#include "UI/UIwindow.hpp"
#include "WorldGrid.hpp"
#include "Localization/LandmarkContainer.hpp"
#include "Localization/OdomKalmanFilter.hpp"
#include "Localization/PathController.hpp"

class ConfigWindow : public UIwindow
{
private:
    GridRenderer& m_WorldGrid;
    LandmarkContainer& m_Landmarks;
    OdomKalmanFilter& m_KalmanFilter;
    PathController& m_PathController;

public:
    ConfigWindow::ConfigWindow
    (
        GridRenderer& worldGrid, 
        LandmarkContainer& landmarks, 
        OdomKalmanFilter& kalmanFilter, 
        PathController& pathController
    ) 
        : m_WorldGrid(worldGrid), 
        m_Landmarks(landmarks), 
        m_KalmanFilter(kalmanFilter), 
        m_PathController(pathController)
    {}

    void ConfigWindow::OnUpdate()
    {
        ViewPort& viewPort = ViewPort::GetInstance();
        Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * ViewPort::GetInstance().GetViewPortMousePos();

        ImGui::Begin("Config");
        {
            // Options for adjusting the world grid visualisation
            if (ImGui::CollapsingHeader("World Grid", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Checkbox("Enable Grid", &m_WorldGrid.bRender);
                int gridSizeInt[2] = {static_cast<int>(m_WorldGrid.gridSize.x()), static_cast<int>(m_WorldGrid.gridSize.y())};
                ImGui::InputInt2("Grid Size", gridSizeInt);
                m_WorldGrid.gridSize = Eigen::Vector2d(gridSizeInt[0], gridSizeInt[1]);
                ImGui::InputFloat("Cell Size", &m_WorldGrid.gridStep, 0.1f, 1.0f);
            }

            // Options fo setting Landmark pos & and adjusting visualisation
            if (ImGui::CollapsingHeader("Landmark Options", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Text("Landmark A Position: %.3f, %.3f", m_Landmarks.getLandmarkPos('A').x(), m_Landmarks.getLandmarkPos('A').y());
                ImGui::Text("Landmark B Position: %.3f, %.3f", m_Landmarks.getLandmarkPos('B').x(), m_Landmarks.getLandmarkPos('B').y());

                ImGui::Text("Landmark A Range: %.3f", m_Landmarks.getLandmarkRange('A'));
                ImGui::Text("Landmark B Range: %.3f", m_Landmarks.getLandmarkRange('B'));

                ImGui::Separator();

                static bool setLandmarkA = false;
                if(ImGui::Button("Set Pos A")) setLandmarkA = true;

                if (setLandmarkA && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                {
                    setLandmarkA = false;
                    m_Landmarks.SetLandmarkPos('A', mousePosWorld);
                    m_KalmanFilter.setAnchors(m_Landmarks.getLandmarkPos('A'), m_Landmarks.getLandmarkPos('B'));
                }

                ImGui::SameLine();
                static bool setLandmarkB = false;
                if(ImGui::Button("Set Pos B")) setLandmarkB = true;

                if (setLandmarkB && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                {
                    setLandmarkB = false;
                    m_Landmarks.SetLandmarkPos('B', mousePosWorld);
                    m_KalmanFilter.setAnchors(m_Landmarks.getLandmarkPos('A'), m_Landmarks.getLandmarkPos('B'));
                }

                ImGui::Separator();

                ImGui::Checkbox("Show Landmarks", &m_Landmarks.bDrawLandmarks);
                ImGui::SameLine();
                ImGui::Checkbox("Show Ranges", &m_Landmarks.bDrawRange);
                ImGui::SameLine();
                ImGui::Checkbox("Show Raw Pos", &m_Landmarks.bDrawRawPos);

                ImGui::Separator();

                ImGui::SliderInt("Range Alpha", static_cast<int*>(&m_Landmarks.rangeAlpha), 0, 255);
            }

            if (ImGui::CollapsingHeader("Kalman Filter", ImGuiTreeNodeFlags_DefaultOpen))
            {  
                ImGui::Text("Pose Estimate: %.3f, %.3f, %.3f", m_KalmanFilter.x.x(), m_KalmanFilter.x.y(), m_KalmanFilter.x.z());
                ImGui::InputDouble("Process Noise", &m_KalmanFilter.processNoise, 0.01f, 0.1f, "%.3e");
                ImGui::InputDouble("Measurement Noise", &m_KalmanFilter.measurementNoise, 0.01f, 0.1f, "%.3e");
            }

            if (ImGui::CollapsingHeader("Waypoint Options", ImGuiTreeNodeFlags_DefaultOpen))
            {
                static char fileName[64] = "DefaultPath";
                ImGui::InputText("File Name", fileName, sizeof(fileName));
        
                if (ImGui::Button("Save Path"))
                {
                    m_PathController.savePath(std::string(fileName) + ".bin");
                }

                ImGui::SameLine();
        
                if (ImGui::Button("Load Path"))
                {
                    m_PathController.loadPath(std::string(fileName) + ".bin");
                }

                ImGui::SameLine();
        
                if (ImGui::Button("Clear Path"))
                {
                    m_PathController.waypoints.clear();
                }
            }

        }
        ImGui::End();
    }
};

