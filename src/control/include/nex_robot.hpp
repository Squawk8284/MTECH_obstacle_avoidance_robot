/**
 * @file nex_robot.hpp
 * @author Kartik Sahasrabudhe (kartik.sahasrabudhe1997@gmail.com)
 * @brief Header file for nex robot
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

// =======================
// Serial Device Management
// =======================

/**
 * @brief Create and open a serial port; returns pointer or nullptr if failed.
 *
 * @param port
 * @param baud
 * @return serial::Serial*
 */
serial::Serial *createSerial(const std::string &port, uint32_t baud)
{
    serial::Serial *s = new serial::Serial();
    s->setPort(port);
    s->setBaudrate(baud);

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
 * @param s
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
 * @param data
 * @return uint8_t
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
 * @param s
 * @param command
 * @param data
 * @return true
 * @return false
 */
bool sendCommand(serial::Serial *s, uint8_t command, const std::vector<uint8_t> &data = {})
{
    if (!s || !s->isOpen())
    {
        std::cerr << "❌ Serial port not open!" << std::endl;
        return false;
    }

    // Construct packet: 'N' 'E' 'X' + Command + Data
    std::vector<uint8_t> packet = {'N', 'E', 'X', command};
    packet.insert(packet.end(), data.begin(), data.end());

    // Compute and append checksum
    uint8_t checksum = computeChecksum(packet);
    packet.push_back(checksum);

    // Debug: Print the exact sent command in HEX format
    std::cout << "Sent Command: ";
    for (auto byte : packet)
    {
        printf("0x%02X ", byte); // Proper hex format with leading zeros
    }
    std::cout << std::endl;

    // Write to serial and ensure all bytes are sent
    size_t bytes_written = s->write(packet);
    s->flush(); // Ensure the packet is fully transmitted

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
 * @param s
 * @param expectedBytes
 * @return std::vector<uint8_t>
 */
std::vector<uint8_t> receiveResponse(serial::Serial *s, size_t expectedBytes)
{
    std::vector<uint8_t> response;
    if (!s || !s->isOpen())
    {
        std::cerr << "Serial port not open!" << std::endl;
        return response;
    }
    try
    {
        response.resize(expectedBytes);
        size_t bytes_read = s->read(response, response.size());
        if (bytes_read > 0)
        {
            response.resize(bytes_read);
            return response;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error reading response: " << e.what() << std::endl;
    }
    return response;
}

/**
 * @brief Execute a command: send, receive, verify checksum, and return the response in 'respOut' (excluding the checksum). The function returns true if successful (i.e. header is 'S').
 *
 * @param s
 * @param command
 * @param data
 * @param expectedBytes
 * @param respOut
 * @return true
 * @return false
 */
bool executeCommand(serial::Serial *s, uint8_t command, const std::vector<uint8_t> &data, size_t expectedBytes, std::vector<uint8_t> *respOut = nullptr)
{
    s->flushInput();
    
    if (!sendCommand(s, command, data))
        return false;

    std::vector<uint8_t> response = receiveResponse(s, expectedBytes);

    // Debug: Print raw response bytes
    std::cout << "Raw Response: ";
    for (auto byte : response)
    {
        std::cout << std::hex << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;

    if (response.empty() || response.size() < 3)
    {
        std::cerr << "No valid response received." << std::endl;
        return false;
    }

    // Separate checksum (last byte) from response data.
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

    // Check if header indicates success ('S')
    if (withoutChecksum[0] != 'S' && withoutChecksum[0] != 'F')
    {
        std::cerr << "Unexpected response header: " << static_cast<int>(withoutChecksum[0]) << std::endl;
        return false;
    }

    if (withoutChecksum[0] == 'F')
    {
        std::cerr << "Command execution failed on robot." << std::endl;
        return false;
    }

    // Return response data (excluding header & command byte)
    if (respOut)
    {
        if (withoutChecksum.size() > 2)
            respOut->assign(withoutChecksum.begin() + 2, withoutChecksum.end());
        else
            respOut->clear();
    }

    return true;
}

#endif // __NEX_ROBOT_H__