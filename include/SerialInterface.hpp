
#pragma once
#include "serialib.h"
#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <vector>

#define PACKET_ACK 0x01
#define PACKET_HEADER 0xAA55
#define PACKET_SIZE 32

#define ENCODER_PACKET_ID 0x01
#define COMMAND_PACKET_ID 0x02
#define LANDMARK_PACKET_ID 0x03
#define STATUS_PACKET_ID 0x04

#define SERIAL_STATUS_EVENT SDL_EVENT_USER + 0x01
#define SERIAL_LANDMARK_EVENT SDL_EVENT_USER + 0x02
#define SERIAL_ENCODER_EVENT SDL_EVENT_USER + 0x03

#pragma pack(push, 1)
struct GenericPacket // This is a test packet.
{
    uint16_t header;
    uint8_t packetID;
};
#pragma pack(pop)

#pragma pack(push, 1)
// Data packet structures
struct EncoderDataPacket 
{
   uint16_t header = PACKET_HEADER;
   uint8_t packetID = ENCODER_PACKET_ID; // Encoder Data Packet ID
   uint16_t Checksum = 0x00; // checksum placeholder
   float encA = 0.0;
   float encB = 0.0;
   float velA = 0.0;
   float velB = 0.0;
}; // 19 Bytes Total (In theory)
#pragma pack(pop)

#pragma pack(push, 1)
struct RobotCommandPacket // This is a test packet.
{
    uint16_t header = PACKET_HEADER;
    uint8_t packetID = COMMAND_PACKET_ID; // Command Packet ID
    uint16_t Checksum = 0x00; // checksum placeholder
    float VelA = 0.0;   
    float VelB = 0.0;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct LandmarkPacket 
{
    uint16_t header = PACKET_HEADER;
    uint8_t packetID = LANDMARK_PACKET_ID;
    uint16_t Checksum = 0x00; // checksum placeholder
    uint8_t LandmarkID = 0x00; // anchor ID (A or B)
    float range = 0.0; // range in meters
    float rxPower; // new field to store the received power
};
#pragma pack(pop)

#pragma pack(push, 1)
struct StatusPacket
{
    uint16_t header = PACKET_HEADER;
    uint8_t packetID = STATUS_PACKET_ID;
    uint16_t Checksum = 0x00; // checksum placeholder
    bool connected = false;
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

    std::atomic_bool m_RunThread = false; // Prevent undefined behaviour when stopping thread
    volatile bool m_EncoderDataReady = false;
    volatile bool m_RangeDataReady = false;
    volatile bool m_NewCommandPacket = false;
    volatile bool m_StatusDataReady = false;

    EncoderDataPacket m_LatestEncoderPacket;
    LandmarkPacket m_LatestAnchorPacket;
    RobotCommandPacket m_LatestCommandPacket;
    StatusPacket m_LatestStatusPacket;
   
    void m_ReadPacket();
    void m_SerialTask();
    void m_WritePacket();
    uint16_t m_calculateChecksum(uint8_t* data, size_t size); 

public:
    SerialInterface();
    ~SerialInterface();
    bool OpenPort(std::string portName, unsigned int baudrate, bool printDebug = true);
    bool ClosePort();
    void SetCommandVel(float velA, float velB);
    void PrintRawPacket(uint8_t *bytes, size_t numBytes);
    
    template <typename PacketType>
    void dispatchPacketEvent(PacketType *packet, int eventCode);
};