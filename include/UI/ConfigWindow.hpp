#pragma once
#include <imgui.h>
#include <Eigen/Dense>  
#include "Core/ViewPort.hpp"
#include "UI/UIwindow.hpp"
#include "WorldGrid.hpp"
#include "Bilateration.hpp"
#include "ConstPosKalmanFilter.hpp"

class ConfigWindow : public UIwindow
{
private:
    GridRenderer& m_WorldGrid;
    Bilateration& biLat;
    ConstPosKalmanFilter& m_KalmanFilter;

public:
    ConfigWindow(GridRenderer& worldGrid, Bilateration& biLat, ConstPosKalmanFilter& kalmanFilter);
    ~ConfigWindow();
    void OnUpdate() override;
};

inline ConfigWindow::ConfigWindow(GridRenderer& worldGrid, Bilateration& biLat, ConstPosKalmanFilter& kalmanFilter) 
: m_WorldGrid(worldGrid), biLat(biLat), m_KalmanFilter(kalmanFilter)
{

}

inline ConfigWindow::~ConfigWindow()
{

}

inline void ConfigWindow::OnUpdate()
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
        
        // Options fo setting Anchor pos & and adjusting visualisation
        if (ImGui::CollapsingHeader("Anchor Options", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Checkbox("Show Anchors", &biLat.bDrawAnchors);
            ImGui::Checkbox("Show Ranges", &biLat.bDrawRange);
            ImGui::Checkbox("Show Raw Pos", &biLat.bDrawRawPos);
            ImGui::SliderInt("Range Alpha", (int*)&biLat.rangeAlpha, 0, 255);
            ImGui::Text("Anchor A Position: %f, %f", biLat.getAnchorPos('A').x(), biLat.getAnchorPos('A').y());
            ImGui::Text("Anchor B Position: %f, %f", biLat.getAnchorPos('B').x(), biLat.getAnchorPos('B').y());
            
            static bool setAnchorA = false;
            if(ImGui::Button("Set Pos A")) setAnchorA = true;

            if (setAnchorA && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            {
                setAnchorA = false;
                biLat.SetAnchorPos('A', mousePosWorld);
            }

            ImGui::SameLine();
            static bool setAnchorB = false;
            if(ImGui::Button("Set Pos B")) setAnchorB = true;

            if (setAnchorB && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            {
                setAnchorB = false;
                biLat.SetAnchorPos('B', mousePosWorld);
            }
        }

        if (ImGui::CollapsingHeader("Kalman Filter", ImGuiTreeNodeFlags_DefaultOpen))
        {  
            // Todo
        }
    }
    ImGui::End();
}

