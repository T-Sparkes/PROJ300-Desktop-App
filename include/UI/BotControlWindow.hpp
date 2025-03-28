#include <imgui.h>
#include "UI/UIwindow.hpp"
#include "SerialInterface.hpp"

#define FORWARD 2.0f, 2.0f
#define BACKWARD -2.0f, -2.0f
#define LEFT 0.0f, 2.0f
#define RIGHT 2.0f, 0.0f
#define STOP 0.0f, 0.0f

class BotControlWindow : public UIwindow
{
private:
    SerialInterface& m_serial;
public:

    BotControlWindow(SerialInterface& serial) : m_serial(serial)
    {
        //m_serial = serial;
    }

    void OnUpdate() override
    {
        ImGui::Begin("Robot Controls");
        {
            if (ImGui::IsWindowHovered())
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

            if (ImGui::Button("STOP"))
            {
                m_serial.SetCommandVel(0.0f, 0.0f);
            }  
        }
        ImGui::End();
    }
};