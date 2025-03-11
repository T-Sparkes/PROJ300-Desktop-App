#include "imgui.h"
#include "SerialInterface.hpp"

#define SERIAL_LINE_SIZE_BYTES 128
#define SERIAL_HISTORY_SIZE_LINES 64
#define DEFAULT_PORT "COM3"
#define DEFAULT_BAUDRATE 115200

class SerialMonitor
{
private:
    EncoderDataPacket EncPacket;
    AnchorRangePacket AncPacket;
    char portBuff[8] = DEFAULT_PORT;
    int baudInput = DEFAULT_BAUDRATE;
    std::vector<std::string> historyBuffer; // This is gross, i might fix it later

    SerialInterface* serialCom;

public:
    SerialMonitor(SerialInterface* serialCom)
    {
        this->serialCom = serialCom;
    }

    ~SerialMonitor()
    {
        serialCom = nullptr;
    }

    void OnNewFrame()
    {

        ImGui::Begin("Serial Console");
        {
            ImGui::InputText("Port", portBuff, sizeof(portBuff));
            ImGui::InputInt("BaudRate", &baudInput, 0, 0);

            if (ImGui::Button("Connect")) serialCom->OpenPort(portBuff, baudInput);
            
            ImGui::SameLine();
            if (ImGui::Button("Disconnect")) serialCom->ClosePort();
           
            ImGui::SameLine();
            if (ImGui::Button("Clear")) historyBuffer.clear();
            
            static bool encEnable = false;
            ImGui::SameLine();
            ImGui::Checkbox("Encoder Data", &encEnable);
            
            static bool ancEnable = false;
            ImGui::SameLine();
            ImGui::Checkbox("Anchor Data", &ancEnable);

            if (serialCom->getPacket(&EncPacket) && encEnable)
            {
                static char lineBuffer[SERIAL_LINE_SIZE_BYTES];
                sprintf_s(lineBuffer, sizeof(lineBuffer), "PACKET: 0x%02X | 0x%02X | %f | %f | %f | %f |\n", EncPacket.header, EncPacket.packetID, EncPacket.encA, EncPacket.encB, EncPacket.velA, EncPacket.velB);
                historyBuffer.push_back(lineBuffer);
            }

            if (serialCom->getPacket(&AncPacket) && ancEnable)
            {
                static char lineBuffer[SERIAL_LINE_SIZE_BYTES];
                sprintf_s(lineBuffer, sizeof(lineBuffer), "PACKET: 0x%02X | 0x%02X | %c | %f |\n", AncPacket.header, AncPacket.packetID, AncPacket.anchorID, AncPacket.range);
                historyBuffer.push_back(lineBuffer);
            }

            if (historyBuffer.size() > SERIAL_HISTORY_SIZE_LINES)
            {
                historyBuffer.erase(historyBuffer.begin());
            }

            ImGui::BeginChild("Console", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar);
            {
                for (auto line : historyBuffer)
                {
                    ImGui::TextWrapped("%s", line.c_str());
                }
                ImGui::SetScrollHereY(0.0f);
            }
            ImGui::EndChild();
        }
        ImGui::End();
    }
};