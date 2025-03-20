/**
 * @file nex_robot.hpp
 * @author Kartik Sahasrabudhe
 * @brief Header file for NEX robot serial communication
 * @version 0.2
 * @date 2025-03-20
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
 * @param port Serial port name (e.g., /dev/ttyRobot)
 * @param baud Baud rate for communication
 * @return serial::Serial* Pointer to the opened serial object
 */
serial::Serial *createSerial(const std::string &port, uint32_t baud)
{
    serial::Serial *s = new serial::Serial();
    s->setPort(port);
    s->setBaudrate(baud);
    s->setParity(serial::parity_none);
    s->setStopbits(serial::stopbits_one);

    // Adjusted timeout to a higher value (200 ms)
    serial::Timeout timeout = serial::Timeout::simpleTimeout(200);
    s->setTimeout(timeout);

    try
    {
        s->open();
        if (!s->isOpen())
        {
            std::cerr << "❌ Unable to open port: " << port << std::endl;
            delete s;
            return nullptr;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "❌ Exception opening port " << port << ": " << e.what() << std::endl;
        delete s;
        return nullptr;
    }

    return s;
}

/**
 * @brief Close and free a serial object.
 *
 * @param s Pointer to the serial object
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
 * @param data Vector of bytes
 * @return uint8_t Computed checksum
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
 * @param s Pointer to the serial object
 * @param command Command byte
 * @param data Optional data vector (default: empty)
 * @return true if the command is sent successfully
 * @return false if failed to send
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

    // Debug: Print the sent command in HEX format
    std::cout << "Sent Command (HEX): ";
    for (auto byte : packet)
    {
        printf("0x%02X ", byte);
    }
    std::cout << std::endl;

    // Write to serial and ensure all bytes are sent
    s->flushInput(); // Clear any leftover data
    size_t bytes_written = s->write(packet);

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
 * @param s Pointer to the serial object
 * @param expectedBytes Number of bytes expected (including checksum)
 * @return std::vector<uint8_t> Received response bytes
 */
std::vector<uint8_t> receiveResponse(serial::Serial *s, size_t expectedBytes)
{
    std::vector<uint8_t> response;

    if (!s || !s->isOpen())
    {
        std::cerr << "❌ Serial port not open!" << std::endl;
        return response;
    }

    try
    {
        response.resize(expectedBytes);
        size_t bytes_read = s->read(response, expectedBytes);

        std::cout << "Raw Response (HEX): ";
        for (auto byte : response)
        {
            printf("0x%02X ", byte);
        }
        std::cout << std::endl;

        if (bytes_read == expectedBytes)
        {
            return response;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "❌ Error reading response: " << e.what() << std::endl;
    }

    return response;
}

/**
 * @brief Execute a command: send, receive, verify checksum, and return the response in 'respOut'.
 *
 * @param s Pointer to the serial object
 * @param command Command byte
 * @param data Optional data vector
 * @param expectedBytes Number of expected response bytes (including checksum)
 * @param respOut Pointer to a vector for storing the response data (excluding the checksum)
 * @return true if successful, false otherwise
 */
bool executeCommand(serial::Serial *s, uint8_t command, const std::vector<uint8_t> &data, size_t expectedBytes, std::vector<uint8_t> *respOut = nullptr)
{
    if (!sendCommand(s, command, data))
        return false;

    std::vector<uint8_t> response = receiveResponse(s, expectedBytes);

    if (response.empty() || response.size() < 3) // Minimum size: Header + Command + Checksum
    {
        std::cerr << "❌ No valid response received." << std::endl;
        return false;
    }

    uint8_t receivedChecksum = response.back();
    std::vector<uint8_t> withoutChecksum(response.begin(), response.end() - 1);
    uint8_t computedChecksum = computeChecksum(withoutChecksum);

    if (receivedChecksum != computedChecksum)
    {
        std::cerr << "❌ Checksum mismatch! Received: 0x" << std::hex << static_cast<int>(receivedChecksum)
                  << ", Computed: 0x" << static_cast<int>(computedChecksum) << std::endl;
        return false;
    }

    if (respOut)
        respOut->assign(withoutChecksum.begin() + 2, withoutChecksum.end()); // Skip header + command

    return true;
}

#endif // __NEX_ROBOT_H__
