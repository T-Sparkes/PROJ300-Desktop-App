
#pragma once
#include "serialib.h"
#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#pragma pack(push, 1)
struct SerialPacket
{
    uint16_t header;
    uint32_t data1;
    uint32_t data2;
    char message[32];
};
#pragma pack(pop)


class SerialInterface
{
private:
    std::thread* m_Worker = nullptr;
    std::mutex m_Mutex;

    serialib m_SerialPort;
    unsigned int m_Baudrate;
    std::string m_PortName; 

    bool m_RunThread = false;
    bool m_packetReady = false;
    SerialPacket m_LatestPacket;
   
    void readPacket();
    void SerialTask();

public:
    ~SerialInterface();
    bool OpenPort(std::string portName, unsigned int baudrate);
    bool ClosePort();
    void PrintRawPacket(SerialPacket packet);
    bool getPacket(SerialPacket *packet);
};

inline SerialInterface::~SerialInterface()
{
    ClosePort();
}

inline bool SerialInterface::OpenPort(std::string portName, unsigned int baudrate)
{
    m_PortName = portName;
    m_Baudrate = baudrate;

    if (m_SerialPort.isDeviceOpen())
    {
        printf("SERIAL WARN: Port %s is already open\n", m_PortName.c_str());
        return false;
    }
    else if (m_SerialPort.openDevice(m_PortName.c_str(), m_Baudrate) != 1 || m_Worker)
    {
        printf("SERIAL ERROR: Unable to open port %s\n", m_PortName.c_str());
        return false;
    } 
    else
    {
        m_RunThread = true;
        m_Worker = new std::thread(&SerialInterface::SerialTask, this);
        printf("SERIAL INFO: Port %s opened\n", m_PortName.c_str());  
        return true;
    }   
}

inline bool SerialInterface::ClosePort()
{
    m_RunThread = false;  
    if (m_Worker)
    {    
        m_Worker->join(); 
        delete m_Worker;   
        m_Worker = nullptr;
    }
    
    if (m_SerialPort.isDeviceOpen())
    {
        m_SerialPort.closeDevice();
        printf("SERIAL INFO: Port %s closed\n", m_PortName.c_str());  
    }
    return false;    
}

inline void SerialInterface::readPacket()
{
    if (m_SerialPort.isDeviceOpen() && m_SerialPort.available() > 0)
    {
        static SerialPacket rxPacket;
        int bytesRead = m_SerialPort.readBytes(reinterpret_cast<uint8_t*>(&rxPacket), sizeof(rxPacket));
        if (bytesRead == sizeof(rxPacket) && rxPacket.header == 0xAA55) 
        {
            m_SerialPort.writeChar(0x01);

            std::lock_guard<std::mutex> lock(m_Mutex);
            m_LatestPacket = rxPacket;
            m_packetReady = true;
        }
        else m_SerialPort.flushReceiver();
    }
}

inline void SerialInterface::SerialTask()
{
    m_SerialPort.flushReceiver();
    while (m_RunThread)
    {
        if (!m_SerialPort.isDeviceOpen() || m_SerialPort.available() < 0)
        {
            m_SerialPort.closeDevice();
            printf("SERIAL ERROR: Port %s disconnected, attemping to reconnect...\n", m_PortName.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            if(m_SerialPort.openDevice(m_PortName.c_str(), m_Baudrate) == 1)
            {
                printf("SERIAL INFO: Port %s opened\n", m_PortName.c_str());  
            }
        }
        else
        {
            readPacket();
        }
    }
}

inline bool SerialInterface::getPacket(SerialPacket* packet)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    if(m_packetReady)
    {
        *packet = m_LatestPacket;
        m_packetReady = false;
        return true;
    }
    else return false;
}

inline void SerialInterface::PrintRawPacket(SerialPacket packet)
{
    uint8_t* rawBytes = reinterpret_cast<uint8_t*>(&packet);
    printf("Raw Packet: ");
    for (int i = 0; i < sizeof(packet); i++)
    {
        printf("%02X ", rawBytes[i]); // Print hex 
    }
    printf("\n");
}


