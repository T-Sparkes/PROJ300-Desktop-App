#include "imgui.h"
#include "SerialInterface.hpp"
#include "UI/UIwindow.hpp"

#define SERIAL_LINE_SIZE_BYTES 128
#define SERIAL_HISTORY_SIZE_LINES 64
#define DEFAULT_BAUDRATE 115200

class SerialMonitor : public UIwindow
{
public:
    EncoderDataPacket EncPacket;
    AnchorRangePacket AncPacket;
    StatusPacket statPacket;

    int baudInput = DEFAULT_BAUDRATE;
    std::vector<std::string> historyBuffer; // This is gross, i might fix it later

    SerialInterface& serialCom;
    std::vector<std::string> availablePorts;

public:
    SerialMonitor(SerialInterface& serialCom) : serialCom(serialCom)
    {
        
        findAvailablePorts();
    }

    void findAvailablePorts() // test what ports are available by trying to open and close each one
    {
        availablePorts.clear();
        for (int i = 0; i < 32; i++)
        {
            std::string portName = "COM" + std::to_string(i);

            if (serialCom.OpenPort(portName, 115200, false))
            {
                availablePorts.push_back(portName);
                serialCom.ClosePort();
            }
        }
    }

    void OnUpdate() override
    {

        ImGui::Begin("Serial Console");
        {
            if (availablePorts.empty()) availablePorts.push_back("None");
            
            std::string availablePortsChar;
            for (auto& port : availablePorts)
            {
                availablePortsChar += port + '\0';
            }

            // Combo box to select port
            static int selectedIdx = 0;
            ImGui::Combo("SerialPort", &selectedIdx, availablePortsChar.c_str(), (int)availablePortsChar.size());
            ImGui::InputInt("BaudRate", &baudInput, 0, 0);
            
            ImGui::Separator();

            if (ImGui::Button("Connect")) serialCom.OpenPort(availablePorts[selectedIdx], baudInput);
            
            ImGui::SameLine();
            if (ImGui::Button("Disconnect")) serialCom.ClosePort();

            ImGui::SameLine();
            if (ImGui::Button("Refresh")) findAvailablePorts();
           
            ImGui::SameLine();
            if (ImGui::Button("Clear")) historyBuffer.clear();

            ImGui::Separator();
            
            static bool encEnable = false;
            ImGui::Checkbox("Encoder Data", &encEnable);
            
            static bool ancEnable = false;
            ImGui::SameLine();
            ImGui::Checkbox("Anchor Data", &ancEnable);

            static bool statEnable = false;
            ImGui::SameLine();
            ImGui::Checkbox("Status Data", &statEnable);

            ImGui::Separator();

            if (serialCom.getPacket(&EncPacket) && encEnable)
            {
                static char lineBuffer[SERIAL_LINE_SIZE_BYTES];
                sprintf_s(
                    lineBuffer, 
                    sizeof(lineBuffer), 
                    "PACKET: 0x%02X | 0x%02X | %f | %f | %f | %f |\n", 
                    EncPacket.header, 
                    EncPacket.packetID,
                    EncPacket.encA, 
                    EncPacket.encB, 
                    EncPacket.velA, 
                    EncPacket.velB
                );

                historyBuffer.push_back(lineBuffer);
            }

            if (serialCom.getPacket(&AncPacket) && ancEnable)
            {
                static char lineBuffer[SERIAL_LINE_SIZE_BYTES];
                sprintf_s(
                    lineBuffer, 
                    sizeof(lineBuffer), 
                    "PACKET: 0x%02X | 0x%02X | %c | %.2f |\n", 
                    AncPacket.header, 
                    AncPacket.packetID, 
                    AncPacket.anchorID, 
                    abs(AncPacket.range)
                );

                historyBuffer.push_back(lineBuffer);
            }

            if (serialCom.getPacket(&statPacket) && statEnable)
            {
                static char lineBuffer[SERIAL_LINE_SIZE_BYTES];
                sprintf_s(
                    lineBuffer, 
                    sizeof(lineBuffer), 
                    "PACKET: 0x%02X | 0x%02X | %d |\n", 
                    statPacket.header, 
                    statPacket.packetID, 
                    statPacket.connected
                
                );
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