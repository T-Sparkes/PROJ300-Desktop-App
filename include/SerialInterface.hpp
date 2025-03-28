
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
#define RANGE_PACKET_ID 0x03
#define STATUS_PACKET_ID 0x04

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

#pragma pack(push, 1)
struct AnchorRangePacket 
{
    uint16_t header;
    uint8_t packetID;
    uint8_t anchorID;
    float range;
    float rxPower;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct StatusPacket
{
    uint16_t header;
    uint8_t packetID;
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
    AnchorRangePacket m_LatestAnchorPacket;
    RobotCommandPacket m_LatestCommandPacket;
    StatusPacket m_LatestStatusPacket;
   
    void m_ReadPacket();
    void m_SerialTask();
    void m_WritePacket();

public:
    SerialInterface();
    ~SerialInterface();
    bool OpenPort(std::string portName, unsigned int baudrate, bool printDebug = true);
    bool ClosePort();
    void SetCommandVel(float velA, float velB);
    bool getPacket(EncoderDataPacket *packet);
    bool getPacket(AnchorRangePacket *packet);
    bool getPacket(StatusPacket *packet);
    void PrintRawPacket(uint8_t *bytes, size_t numBytes);
};