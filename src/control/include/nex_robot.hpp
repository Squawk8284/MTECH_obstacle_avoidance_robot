/**
 * @file nex_robot.hpp
 * @author Kartik Sahasrabudhe
 * @brief Header file for NEX robot serial communication
 * @version 0.3
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
#include <cstddef>
#include <cstdint>
#include <cstring>         // For memcpy
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
 * @brief Compute checksum by summing all bytes (from a raw buffer), taking only the last 8 bits,
 *        then taking the 1's complement and adding 1.
 *
 * @param data Pointer to raw data.
 * @param length Number of bytes to process.
 * @return uint8_t Computed checksum.
 */
uint8_t computeChecksum(const void *data, size_t length)
{
    uint16_t sum = 0;
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    for (size_t i = 0; i < length; i++)
    {
        sum += bytes[i];
    }
    uint8_t last8 = sum & 0xFF;
    uint8_t onesComplement = ~last8;
    return onesComplement + 1;
}

/**
 * @brief Send a command using the protocol: header ('N', 'E', 'X') + command + data, then append checksum.
 *
 * @param s Pointer to the serial object.
 * @param command Command byte.
 * @param data Pointer to a buffer containing optional data (pass nullptr if none).
 * @param dataSize Number of bytes in the data buffer.
 * @return true if the command is sent successfully; false otherwise.
 */
bool sendCommand(serial::Serial *s, uint8_t command, const void *data, size_t dataSize)
{
    if (!s || !s->isOpen())
    {
        std::cerr << "❌ Serial port not open!" << std::endl;
        return false;
    }

    // Construct packet: header ('N','E','X'), command, then data (if any)
    std::vector<uint8_t> packet;
    packet.push_back('N');
    packet.push_back('E');
    packet.push_back('X');
    packet.push_back(command);
    if (data && dataSize > 0)
    {
        const uint8_t *d = static_cast<const uint8_t *>(data);
        packet.insert(packet.end(), d, d + dataSize);
    }

    // Compute and append checksum
    uint8_t checksum = computeChecksum(packet.data(), packet.size());
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
 * @brief Receive response from the serial port into a pre-allocated buffer.
 *        expectedBytes should include the checksum.
 *
 * @param s Pointer to the serial object.
 * @param expectedBytes Number of bytes expected in the response.
 * @param respOut Pointer to the pre-allocated output buffer.
 * @param outSize Size (in bytes) of the output buffer.
 * @return true if the response is received successfully; false otherwise.
 */
bool receiveResponse(serial::Serial *s, size_t expectedBytes, void *respOut, size_t outSize)
{
    if (!s || !s->isOpen())
    {
        std::cerr << "❌ Serial port not open!" << std::endl;
        return false;
    }

    std::vector<uint8_t> response(expectedBytes);
    size_t bytes_read = s->read(response, expectedBytes);

    std::cout << "Raw Response (HEX): ";
    for (auto byte : response)
    {
        printf("0x%02X ", byte);
    }
    std::cout << std::endl;

    if (bytes_read != expectedBytes)
    {
        std::cerr << "❌ Incomplete response. Expected: " << expectedBytes << ", Received: " << bytes_read << std::endl;
        return false;
    }

    if (outSize > response.size())
    {
        std::cerr << "❌ Output buffer size is larger than response size." << std::endl;
        return false;
    }

    memcpy(respOut, response.data(), outSize);
    return true;
}

/**
 * @brief Execute a command: send, receive, verify checksum, and return the response in a raw buffer.
 *
 * The function sends the command with optional data, reads the entire response (including checksum),
 * verifies the checksum, and then copies the response payload (excluding header and command) into the
 * provided output buffer.
 *
 * @param s Pointer to the serial object.
 * @param command Command byte.
 * @param data Pointer to the data to send (pass nullptr if no data).
 * @param dataSize Size (in bytes) of the data to send.
 * @param expectedBytes Total number of bytes expected in the response (including checksum).\n"
 * @param respOut Pointer to the pre-allocated output buffer where the payload will be copied.\n"
 *                (The payload is the response with header and command skipped.)\n"
 * @param outSize Size (in bytes) of the output buffer. Must be ≤ (expectedBytes - 2).\n"
 *                (Here, we assume the first two bytes of the response are header+command.)\n"
 * @return true if successful; false otherwise.
 */
bool executeCommand(serial::Serial *s, uint8_t command, const void *data, size_t dataSize,
                    size_t expectedBytes, void *respOut, size_t outSize)
{
    if (!sendCommand(s, command, data, dataSize))
        return false;

    // Read full response into a temporary vector
    std::vector<uint8_t> response(expectedBytes);
    size_t bytes_read = s->read(response, expectedBytes);

    std::cout << "Raw Response (HEX): ";
    for (auto byte : response)
    {
        printf("0x%02X ", byte);
    }
    std::cout << std::endl;

    if (response.empty() || response.size() < 3) // Minimum: header + command + checksum
    {
        std::cerr << "❌ No valid response received." << std::endl;
        return false;
    }

    uint8_t receivedChecksum = response.back();
    size_t withoutChecksumSize = response.size() - 1;
    // Create a temporary buffer without the checksum
    std::vector<uint8_t> withoutChecksum(response.begin(), response.end() - 1);
    uint8_t computedChecksum = computeChecksum(withoutChecksum.data(), withoutChecksum.size());
    if (receivedChecksum != computedChecksum)
    {
        std::cerr << "❌ Checksum mismatch! Received: 0x" << std::hex << static_cast<int>(receivedChecksum)
                  << ", Computed: 0x" << static_cast<int>(computedChecksum) << std::endl;
        return false;
    }

    // Skip header + command (assume first 2 bytes) to get the payload.
    if (withoutChecksum.size() < 2)
    {
        std::cerr << "❌ Response too short." << std::endl;
        return false;
    }
    size_t available = withoutChecksum.size() - 2;
    if (outSize > available)
    {
        std::cerr << "❌ Requested output size (" << outSize << ") is larger than available payload ("
                  << available << ")." << std::endl;
        return false;
    }

    memcpy(respOut, withoutChecksum.data() + 2, outSize);
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
