#include "SerialInterface.hpp"

SerialInterface::SerialInterface()
{
    // Test what ports are available by trying to open and close each one
    //for (int i = 0; i < 256; i++)
    //{
    //    std::string portName = "COM" + std::to_string(i);
//
    //    if (m_SerialPort.openDevice(portName.c_str(), 115200) == 1)
    //    {
    //        printf("Port %s is available\n", portName.c_str());
    //        m_SerialPort.closeDevice();
    //    }
    //}
}

SerialInterface::~SerialInterface()
{
    ClosePort();
}

bool SerialInterface::OpenPort(std::string portName, unsigned int baudrate)
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
        m_Worker = new std::thread(&SerialInterface::m_SerialTask, this);
        printf("SERIAL INFO: Port %s opened\n", m_PortName.c_str());  
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
            m_EncoderDataReady = true;
        }

        else if (rxPacket.header == PACKET_HEADER && rxPacket.packetID == RANGE_PACKET_ID) // Encoder packet
        {
            m_SerialPort.writeChar(PACKET_ACK);
            std::lock_guard<std::mutex> lock(m_Mutex);
            memcpy(&m_LatestAnchorPacket, rxBuffer, sizeof(m_LatestAnchorPacket));
            m_RangeDataReady = true;
        }

        else if (rxPacket.header == PACKET_HEADER && rxPacket.packetID == STATUS_PACKET_ID) // Encoder packet
        {
            m_SerialPort.writeChar(PACKET_ACK);
            std::lock_guard<std::mutex> lock(m_Mutex);
            memcpy(&m_LatestStatusPacket, rxBuffer, sizeof(m_LatestStatusPacket));
            m_StatusDataReady = true;
        }

        else 
        {
            m_SerialPort.flushReceiver();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
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

// get the latest packet from the serial interface.
bool SerialInterface::getPacket(EncoderDataPacket* packet)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    if(m_EncoderDataReady)
    {
        *packet = m_LatestEncoderPacket;
        m_EncoderDataReady = false;
        return true;
    }
    else return false;
}

bool SerialInterface::getPacket(AnchorRangePacket* packet)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    if(m_RangeDataReady)
    {
        *packet = m_LatestAnchorPacket;
        m_RangeDataReady = false;
        return true;
    }
    else return false;
}

bool SerialInterface::getPacket(StatusPacket* packet)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    *packet = m_LatestStatusPacket;
    if(m_StatusDataReady)
    {
        m_StatusDataReady = false;
        return true;
    }
    else return false;
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

// Runs on seperate thread
void SerialInterface::m_WritePacket()
{
    uint8_t* rawBytes = reinterpret_cast<uint8_t*>(&m_LatestCommandPacket);
    m_SerialPort.writeBytes(rawBytes, sizeof(m_LatestCommandPacket));
}

// runs on main thread, notfies serial thread
void SerialInterface::SetCommandVel(float velA, float velB)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_NewCommandPacket = true;
    m_LatestCommandPacket = {PACKET_HEADER, COMMAND_PACKET_ID, velA, velB};
}
