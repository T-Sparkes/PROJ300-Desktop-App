#include "SerialInterface.hpp"
#include "SDL3/SDL.h"

SerialInterface::SerialInterface()
{

}

SerialInterface::~SerialInterface()
{
    ClosePort();
}

// @brief Open a serial port with the specified name and baudrate.
// @param portName the name of the port to open.
// @param baudrate the baudrate to open the port with.
// @return true if the port was opened successfully, false otherwise.
// @note This function will start a new thread to read packets from the serial port.
bool SerialInterface::OpenPort(std::string portName, unsigned int baudrate, bool printDebug)
{
    m_PortName = portName;
    m_Baudrate = baudrate;

    if (m_SerialPort.isDeviceOpen())
    {
        if (printDebug) printf("SERIAL WARN: Port %s is already open\n", m_PortName.c_str());
        return false;
    }
    else if (m_SerialPort.openDevice(m_PortName.c_str(), m_Baudrate) != 1 || m_Worker)
    {
        if (printDebug) printf("SERIAL ERROR: Unable to open port %s\n", m_PortName.c_str());
        return false;
    } 
    else
    {
        m_RunThread = true;
        m_Worker = new std::thread(&SerialInterface::m_SerialTask, this);
        if (printDebug) printf("SERIAL INFO: Port %s opened\n", m_PortName.c_str());  
        return true;
    }   
}

bool SerialInterface::ClosePort()
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
void SerialInterface::m_ReadPacket()
{
    if (m_SerialPort.isDeviceOpen() && m_SerialPort.available())
    {
        static GenericPacket rxPacket;
        static uint8_t rxBuffer[PACKET_SIZE];

        int bytesRead = m_SerialPort.readBytes((uint8_t*)(rxBuffer), sizeof(rxBuffer));
        memcpy(&rxPacket, rxBuffer, sizeof(rxPacket));

        //PrintRawPacket(rxBuffer, PACKET_SIZE);

        if (rxPacket.header == PACKET_HEADER && rxPacket.packetID == ENCODER_PACKET_ID) // Encoder packet
        {
            m_SerialPort.writeChar(PACKET_ACK);
            std::lock_guard<std::mutex> lock(m_Mutex);

            memcpy(&m_LatestEncoderPacket, rxBuffer, sizeof(m_LatestEncoderPacket));
            uint16_t checksum = m_calculateChecksum((uint8_t*)&m_LatestEncoderPacket, sizeof(EncoderDataPacket));
            
            if (m_LatestEncoderPacket.Checksum == checksum)
            {
                m_EncoderDataReady = true;
                EncoderDataPacket* packet = new EncoderDataPacket;
                memcpy(packet, &m_LatestEncoderPacket, sizeof(EncoderDataPacket));
                dispatchPacketEvent(packet, SERIAL_ENCODER_EVENT);            
            }
            else
            {
                printf("SERIAL ERROR: Encoder packet checksum mismatch\n");
            }
        }

        else if (rxPacket.header == PACKET_HEADER && rxPacket.packetID == LANDMARK_PACKET_ID) // Encoder packet
        {
            m_SerialPort.writeChar(PACKET_ACK);
            std::lock_guard<std::mutex> lock(m_Mutex);

            memcpy(&m_LatestAnchorPacket, rxBuffer, sizeof(m_LatestAnchorPacket));
            uint16_t checksum = m_calculateChecksum((uint8_t*)&m_LatestAnchorPacket, sizeof(LandmarkPacket));

            if (m_LatestAnchorPacket.Checksum == checksum)
            {
                m_RangeDataReady = true;
                LandmarkPacket* packet = new LandmarkPacket;
                memcpy(packet, &m_LatestAnchorPacket, sizeof(LandmarkPacket));
                dispatchPacketEvent(packet, SERIAL_LANDMARK_EVENT);            
            }
            else
            {
                printf("SERIAL ERROR: Encoder packet checksum mismatch\n");
            }
        }

        else if (rxPacket.header == PACKET_HEADER && rxPacket.packetID == STATUS_PACKET_ID) // Encoder packet
        {
            m_SerialPort.writeChar(PACKET_ACK);
            std::lock_guard<std::mutex> lock(m_Mutex);

            memcpy(&m_LatestStatusPacket, rxBuffer, sizeof(m_LatestStatusPacket));
            uint16_t checksum = m_calculateChecksum((uint8_t*)&m_LatestStatusPacket, sizeof(StatusPacket));

            if (m_LatestStatusPacket.Checksum == checksum)
            {
                m_StatusDataReady = true;
                StatusPacket* packet = new StatusPacket;
                memcpy(packet, &m_LatestStatusPacket, sizeof(StatusPacket));
                dispatchPacketEvent(packet, SERIAL_STATUS_EVENT);            
            }
            else
            {
                printf("SERIAL ERROR: Encoder packet checksum mismatch\n");
            }
        }

        else 
        {
            m_SerialPort.flushReceiver();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

template <typename PacketType>
void SerialInterface::dispatchPacketEvent(PacketType* packet, int eventCode)
{
    SDL_Event event;
    event.type = SDL_EVENT_USER;
    event.user.code = eventCode;
    event.user.data1 = packet;
    event.user.data2 = NULL;
    SDL_PushEvent(&event);
}

uint16_t SerialInterface::m_calculateChecksum(uint8_t* data, size_t size) 
{
    uint16_t checksum = 0;
    for (size_t i = 0; i < size; i++) 
    {
        // Exclude the checksum bytes (indices 3 and 4) from the checksum calculation
        if (i != 3 && i != 4)
        {
            checksum += data[i];
        }
    }
    return checksum;
}

// serial task that runs in a separate thread.
void SerialInterface::m_SerialTask()
{
    m_SerialPort.flushReceiver();
    while (m_RunThread)
    {
        if (!m_SerialPort.isDeviceOpen() || m_SerialPort.available() < 0) // Check if the port has closed or had an error
        {
            m_SerialPort.closeDevice(); // Confirm closed
            printf("SERIAL ERROR: Port %s disconnected, attemping to reconnect...\n", m_PortName.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            if(m_SerialPort.openDevice(m_PortName.c_str(), m_Baudrate) == 1) // Attempt to reopen
            {
                printf("SERIAL INFO: Port %s opened\n", m_PortName.c_str());  
            }
        }
        else
        {
            m_ReadPacket(); // Read and Decode Incoming packets
            if (m_NewCommandPacket) // If there is a new command packet to send
            {
                std::lock_guard<std::mutex> lock(m_Mutex);
                m_WritePacket();
                m_NewCommandPacket = false;
            }
        }
    }
}

void SerialInterface::PrintRawPacket(uint8_t* bytes, size_t numBytes)
{
    printf("Raw Packet: ");
    for (int i = 0; i < numBytes; i++)
    {
        printf("%02X ", bytes[i]); // Print hex 
    }
    printf("\n");
}

// @brief Write the latest command packet to the serial interface.
// @note This function is called from the serial task thread.
void SerialInterface::m_WritePacket()
{
    m_LatestCommandPacket.Checksum = m_calculateChecksum((uint8_t*)&m_LatestCommandPacket, sizeof(RobotCommandPacket));
    uint8_t* rawBytes = reinterpret_cast<uint8_t*>(&m_LatestCommandPacket);
    m_SerialPort.writeBytes(rawBytes, sizeof(m_LatestCommandPacket));
}

// runs on main thread, notfies serial thread
void SerialInterface::SetCommandVel(float velA, float velB)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_NewCommandPacket = true;
    m_LatestCommandPacket = {PACKET_HEADER, COMMAND_PACKET_ID, 0x00, velA, velB};
}
