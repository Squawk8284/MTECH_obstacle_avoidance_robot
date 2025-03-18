/**
 * @file nex_robot.hpp
 * @author Kartik Sahasrabudhe
 * @brief Header file for nex robot communication
 * @version 0.1
 * @date 2025-03-17
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __NEX_ROBOT_H__
#define __NEX_ROBOT_H__

#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <serial/serial.h> // Requires the serial library
#include <chrono>          // For timing
#include <thread>          // For sleep_for
#include <cstdio>          // For printf
#include <sstream>         // For stringstream

// =======================
// Serial Device Management
// =======================

/**
 * @brief Create and open a serial port; returns pointer or nullptr if failed.
 *
 * @param port Port name (e.g., "/dev/ttyRobot")
 * @param baud Baud rate (e.g., 57600)
 * @return serial::Serial* Pointer to the opened serial port.
 */
serial::Serial *createSerial(const std::string &port, uint32_t baud)
{
    serial::Serial *s = new serial::Serial();
    s->setPort(port);
    s->setBaudrate(baud);
    // Set a 2-second timeout for read and write operations.
    serial::Timeout timeout = serial::Timeout(2000, 2000, 0, 2000, 2000);
    s->setTimeout(timeout);

    try
    {
        s->open();
        if (!s->isOpen())
        {
            std::cerr << "Unable to open port: " << port << std::endl;
            delete s;
            return nullptr;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception opening port " << port << ": " << e.what() << std::endl;
        delete s;
        return nullptr;
    }
    return s;
}

/**
 * @brief Close and free a serial object.
 *
 * @param s Pointer to the serial object.
 */
void clearSerial(serial::Serial *s)
{
    if (s)
    {
        if (s->isOpen())
        {
            s->close();
        }
        delete s;
    }
}

// =======================
// Helper Functions
// =======================

/**
 * @brief Compute checksum by summing all bytes, taking only the last 8 bits, then taking the 1's complement and adding 1.
 *
 * @param data Vector of bytes for which to compute the checksum.
 * @return uint8_t The computed checksum.
 */
uint8_t computeChecksum(const std::vector<uint8_t> &data)
{
    uint16_t sum = 0;
    for (auto byte : data)
    {
        sum += byte;
    }
    uint8_t last8 = sum & 0xFF;
    uint8_t onesComplement = ~last8;
    return onesComplement + 1;
}

/**
 * @brief Send a command using the protocol: header ('N','E','X') + command + data, then append checksum.
 *
 * @param s Pointer to an open serial port.
 * @param command The command byte.
 * @param data Vector of data bytes to send.
 * @return true if the command is sent successfully, false otherwise.
 */
bool sendCommand(serial::Serial *s, uint8_t command, const std::vector<uint8_t> &data = {})
{
    if (!s || !s->isOpen())
    {
        std::cerr << "❌ Serial port not open!" << std::endl;
        return false;
    }

    // Construct packet: 'N', 'E', 'X' + Command + Data
    std::vector<uint8_t> packet = {'N', 'E', 'X', command};
    packet.insert(packet.end(), data.begin(), data.end());

    // Compute and append checksum
    uint8_t checksum = computeChecksum(packet);
    packet.push_back(checksum);

    // Debug: Print the exact sent command in HEX format
    std::cout << "Sent Command: ";
    for (auto byte : packet)
    {
        printf("0x%02X ", byte);
    }
    std::cout << std::endl;

    // Write packet to serial port
    size_t bytes_written = s->write(packet);
    s->flush();  // Ensure the packet is fully transmitted

    if (bytes_written != packet.size())
    {
        std::cerr << "⚠️ Warning: Not all bytes were sent! Expected: " << packet.size()
                  << ", Sent: " << bytes_written << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief Receive response from the serial port. expectedBytes should include the checksum.
 *
 * @param s Pointer to the open serial port.
 * @param expectedBytes Total number of bytes expected in the response.
 * @return std::vector<uint8_t> Vector containing the received bytes.
 */
std::vector<uint8_t> receiveResponse(serial::Serial *s, size_t expectedBytes)
{
    std::vector<uint8_t> response;
    if (!s || !s->isOpen())
    {
        std::cerr << "Serial port not open!" << std::endl;
        return response;
    }
    
    size_t totalRead = 0;
    int attempts = 0;
    // Read until we've accumulated expectedBytes or we've tried a number of times.
    while (totalRead < expectedBytes && attempts < 20)
    {
        size_t available = s->available();
        if (available > 0)
        {
            std::vector<uint8_t> buffer(available);
            size_t bytes_read = s->read(buffer, available);
            response.insert(response.end(), buffer.begin(), buffer.begin() + bytes_read);
            totalRead = response.size();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        attempts++;
    }
    
    // Debug: Print raw response in HEX.
    std::cout << "Raw Response: ";
    for (auto byte : response)
    {
        printf("0x%02X ", byte);
    }
    std::cout << std::endl;
    
    return response;
}

/**
 * @brief Execute a command: send, receive, verify checksum, and return the response in 'respOut' (excluding the checksum).
 *        Returns true if successful (i.e. header is 'S'), false otherwise.
 *
 * @param s Pointer to an open serial port.
 * @param command The command byte.
 * @param data Vector of data bytes to send.
 * @param expectedBytes Total number of bytes expected in the response (including header, command, data, checksum).
 * @param respOut Pointer to a vector to store response data (excluding header, command, and checksum).
 * @return true if the command execution was successful, false otherwise.
 */
bool executeCommand(serial::Serial *s, uint8_t command, const std::vector<uint8_t> &data,
                    size_t expectedBytes, std::vector<uint8_t> *respOut = nullptr)
{
    // Send the command without flushing input (to avoid discarding immediate replies)
    if (!sendCommand(s, command, data))
        return false;

    // A very short delay if needed (adjust if necessary)
    // std::this_thread::sleep_for(std::chrono::milliseconds(10)) ;

    // Wait until the expected number of bytes are available.
    int waitCount = 0;
    while (s->available() < expectedBytes && waitCount < 20)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        waitCount++;
    }

    // Now, read the response.
    std::vector<uint8_t> response = receiveResponse(s, expectedBytes);

    if (response.empty() || response.size() < 3)
    {
        std::cerr << "No valid response received." << std::endl;
        return false;
    }

    // Separate checksum (last byte) from the response.
    uint8_t receivedChecksum = response.back();
    std::vector<uint8_t> withoutChecksum(response.begin(), response.end() - 1);
    uint8_t computed = computeChecksum(withoutChecksum);

    if (receivedChecksum != computed)
    {
        std::cerr << "Checksum verification failed. Received 0x"
                  << std::hex << static_cast<int>(receivedChecksum)
                  << " but computed 0x" << static_cast<int>(computed) << std::endl;
        return false;
    }

    // Check if header indicates success ('S'). (If it's 'F', it's a failure.)
    if (withoutChecksum[0] != 'S')
    {
        std::cerr << "Command execution failed. Received header: 0x" 
                  << std::hex << static_cast<int>(withoutChecksum[0]) << std::endl;
        return false;
    }

    // If caller provided a pointer for response data, return the data (excluding header and command).
    if (respOut)
    {
        if (withoutChecksum.size() > 2)
            respOut->assign(withoutChecksum.begin() + 2, withoutChecksum.end());
        else
            respOut->clear();
    }

    return true;
}

/**
 * @brief Convert a decimal value to a hex string.
 *
 * @param decimal The decimal number.
 * @return std::string Hexadecimal representation.
 */
std::string decimalToHex(unsigned int decimal)
{
    std::stringstream ss;
    ss << std::hex << std::uppercase << decimal;
    return ss.str();
}

#endif // __NEX_ROBOT_H__
