#include <imgui.h>
#include "UI/UIwindow.hpp"
#include "SerialInterface.hpp"

#define FORWARD 2.0f, 2.0f
#define BACKWARD -2.0f, -2.0f
#define LEFT 0.0f, 2.0f
#define RIGHT 2.0f, 0.0f
#define STOP 0.0f, 0.0f

typedef enum {MANUAL, WAYPOINT, MOUSE} ControlMode_t;

class BotControlWindow : public UIwindow
{
private:
    SerialInterface& m_serial;

public:
ControlMode_t controlMode = MANUAL;

    BotControlWindow(SerialInterface& serial) : m_serial(serial)
    {
        //m_serial = serial;
    }

    void OnUpdate() override
    {
        ImGui::Begin("Robot Controls");
        {
            ImGui::Text("Control Mode:  ");

            ImGui::SameLine();
            if (ImGui::Button("Manual")) controlMode = MANUAL;

            ImGui::SameLine();
            if (ImGui::Button("Waypoint")) controlMode = WAYPOINT;

            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(255, 0, 0, 255));

            if (ImGui::Button("STOP"))
            {
                controlMode = MANUAL;
                m_serial.SetCommandVel(0.0f, 0.0f);
            }  

            ImGui::PopStyleColor();

            if (controlMode == MANUAL)
            {
                if (ImGui::IsKeyDown(ImGuiKey_W))
                {
                    m_serial.SetCommandVel(FORWARD);
                }
                else if (ImGui::IsKeyDown(ImGuiKey_S))
                {
                    m_serial.SetCommandVel(BACKWARD);
                }
                else if (ImGui::IsKeyDown(ImGuiKey_A))
                {
                    m_serial.SetCommandVel(LEFT);
                }
                else if (ImGui::IsKeyDown(ImGuiKey_D))
                {
                    m_serial.SetCommandVel(RIGHT);
                }
                else if (ImGui::IsKeyDown(ImGuiKey_Space))
                {
                    m_serial.SetCommandVel(STOP);
                }
            }
        }
        ImGui::End();
    }
};