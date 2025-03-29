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
    Bilateration& m_biLat;
    ConstPosKalmanFilter& m_KalmanFilter;

public:
    ConfigWindow(GridRenderer& worldGrid, Bilateration& m_biLat, ConstPosKalmanFilter& kalmanFilter);
    ~ConfigWindow();
    void OnUpdate() override;
};

inline ConfigWindow::ConfigWindow(GridRenderer& worldGrid, Bilateration& m_biLat, ConstPosKalmanFilter& kalmanFilter) 
: m_WorldGrid(worldGrid), m_biLat(m_biLat), m_KalmanFilter(kalmanFilter)
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
            ImGui::Checkbox("Show Anchors", &m_biLat.bDrawAnchors);
            ImGui::Checkbox("Show Ranges", &m_biLat.bDrawRange);
            ImGui::Checkbox("Show Raw Pos", &m_biLat.bDrawRawPos);
            ImGui::SliderInt("Range Alpha", (int*)&m_biLat.rangeAlpha, 0, 255);
            ImGui::Text("Anchor A Position: %.3f, %.3f", m_biLat.getAnchorPos('A').x(), m_biLat.getAnchorPos('A').y());
            ImGui::Text("Anchor B Position: %.3f, %.3f", m_biLat.getAnchorPos('B').x(), m_biLat.getAnchorPos('B').y());

            ImGui::Text("Anchor A Range: %.3f", m_biLat.getAnchorRange('A'));
            ImGui::Text("Corrected A Range: %.3f", m_biLat.getAnchorRange('A') * 0.75f + 0.375f);

            ImGui::Text("Anchor B Range: %.3f", m_biLat.getAnchorRange('B'));
            ImGui::Text("Corrected B Range: %.3f", m_biLat.getAnchorRange('B') * 0.75f + 0.375f);
            
            static bool setAnchorA = false;
            if(ImGui::Button("Set Pos A")) setAnchorA = true;

            if (setAnchorA && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            {
                setAnchorA = false;
                m_biLat.SetAnchorPos('A', mousePosWorld);
                m_KalmanFilter.setAnchors(m_biLat.getAnchorPos('A'), m_biLat.getAnchorPos('B'));
            }

            ImGui::SameLine();
            static bool setAnchorB = false;
            if(ImGui::Button("Set Pos B")) setAnchorB = true;

            if (setAnchorB && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            {
                setAnchorB = false;
                m_biLat.SetAnchorPos('B', mousePosWorld);
                m_KalmanFilter.setAnchors(m_biLat.getAnchorPos('A'), m_biLat.getAnchorPos('B'));
            }
        }

        if (ImGui::CollapsingHeader("Kalman Filter", ImGuiTreeNodeFlags_DefaultOpen))
        {  
            // Todo
        }
    }
    ImGui::End();
}

