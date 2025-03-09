
#pragma once
#include "serialib.h"
#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#define PACKET_HEADER 0xAA55
#define ENCODER_PACKET_ID 0x01
#define COMMAND_PACKET_ID 0x02
#define PACKET_ACK 0x01

#pragma pack(push, 1)
struct GenericPacket // This is a test packet.
{
    uint16_t header;
    uint8_t packetID;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct EncoderDataPacket // This is a test packet.
{
    uint16_t header;    // 2 Bytes
    uint8_t packetID;   // 1 Bytes
    float encA;         // 4 Bytes
    float encB;         // 4 Bytes
    float velA;         // 4 Bytes
    float velB;         // 4 Bytes
};                      // 19 Bytes Total
#pragma pack(pop)

#pragma pack(push, 1)
struct RobotCommandPacket // This is a test packet.
{
    uint16_t header;
    uint8_t packetID;
    float VelA;
    float VelB;
};
#pragma pack(pop)

// Packets to implement:
// Robot command packet to send
// Robot odometry packet to receive
// Anchor range packet, (anchor ID, range), receive


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
    bool m_NewCommandPacket = false;
    EncoderDataPacket m_LatestEncoderPacket;
    RobotCommandPacket m_LatestCommandPacket;
   
    void readPacket();
    void SerialTask();

public:
    ~SerialInterface();
    bool OpenPort(std::string portName, unsigned int baudrate);
    bool ClosePort();
    void PrintRawPacket(EncoderDataPacket packet);
    void writePacket();
    void SetCommandVel(float velA, float velB);
    bool getPacket(EncoderDataPacket *packet);
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

// runs in a separate thread to read packets from the serial port.
inline void SerialInterface::readPacket()
{
    if (m_SerialPort.isDeviceOpen() && m_SerialPort.available() > 0)
    {
        static GenericPacket rxPacket;
        static uint8_t rxBuffer[64];

        int bytesRead = m_SerialPort.readBytes(reinterpret_cast<uint8_t*>(rxBuffer), sizeof(rxBuffer));
        memcpy(&rxPacket, rxBuffer, sizeof(rxPacket));

        if (rxPacket.header == PACKET_HEADER && rxPacket.packetID == ENCODER_PACKET_ID) // Encoder packet
        {
            //m_SerialPort.writeChar(PACKET_ACK);
            std::lock_guard<std::mutex> lock(m_Mutex);
            memcpy(&m_LatestEncoderPacket, rxBuffer, sizeof(m_LatestEncoderPacket));
            m_packetReady = true;
        }

        //else if (rxPacket.header == PACKET_HEADER && rxPacket.packetID == OTHER_PACKET_ID) // Other packet
        //{
        //    m_SerialPort.writeChar(PACKET_ACK);
        //}

        else m_SerialPort.flushReceiver();
    }
}

// serial task that runs in a separate thread.
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
            if (m_NewCommandPacket) // If there is a new command packet to send
            {
                writePacket();
                m_NewCommandPacket = false;
            }
        }
    }
}

// get the latest packet from the serial interface.
inline bool SerialInterface::getPacket(EncoderDataPacket* packet)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    if(m_packetReady)
    {
        *packet = m_LatestEncoderPacket;
        m_packetReady = false;
        return true;
    }
    else return false;
}

inline void SerialInterface::PrintRawPacket(EncoderDataPacket packet)
{
    uint8_t* rawBytes = reinterpret_cast<uint8_t*>(&packet);
    printf("Raw Packet: ");
    for (int i = 0; i < sizeof(packet); i++)
    {
        printf("%02X ", rawBytes[i]); // Print hex 
    }
    printf("\n");
}

inline void SerialInterface::writePacket()
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    uint8_t* rawBytes = reinterpret_cast<uint8_t*>(&m_LatestCommandPacket);
    m_SerialPort.writeBytes(rawBytes, sizeof(m_LatestCommandPacket));
}

inline void SerialInterface::SetCommandVel(float velA, float velB)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_NewCommandPacket = true;
    m_LatestCommandPacket = {PACKET_HEADER, COMMAND_PACKET_ID, velA, velB};
}
