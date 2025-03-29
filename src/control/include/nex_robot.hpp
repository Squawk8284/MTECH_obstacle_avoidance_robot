/**
 * @file nex_robot.hpp
 * @author Kartik Sahasrabudhe
 * @brief Header file for NEX robot serial communication with vector-based command support
 * @version 0.5
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

template <typename... Args>
void print(Args &&...args)
{
    using Expander = int[];
    (void)Expander{0, ((std::cout << args), 0)...};
}

// ---------------------------------------------------------------------------
// MACROS
// ---------------------------------------------------------------------------
#define Print(...)          \
    do                      \
    {                       \
        print(__VA_ARGS__); \
    } while (0)
#define PrintLn(...)            \
    do                          \
    {                           \
        print(__VA_ARGS__);     \
        std::cout << std::endl; \
    } while (0)

#define CMD(name, cmd, ...) \
    std::vector<uint8_t> { cmd, ##__VA_ARGS__ }
#define ReturnPayload(payload) (payload + 3)

#ifdef DEBUG
#define _DEBUG(x) x
#else
#define _DEBUG(x)
#endif

#define SUCCESS (true)
#define FAILURE (false)

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
uint8_t computeChecksum(const void *data, std::size_t length)
{
    uint16_t sum = 0;
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    for (std::size_t i = 0; i < length; i++)
    {
        sum += bytes[i];
    }
    uint8_t last8 = sum & 0xFF;
    uint8_t onesComplement = ~last8;
    return onesComplement + 1;
}

/**
 * @brief Construct and send a packet that includes a vector of command bytes and optional data.
 *
 * The packet structure is:
 * Header ("N", "E", "X") + command bytes (vector) + [Optional Data] + Checksum
 *
 * @param s Pointer to the serial object.
 * @param commands Vector of command bytes (this can include subcommands if needed).
 * @param data Pointer to additional data bytes (can be nullptr).
 * @param dataSize Number of additional data bytes.
 * @return true if the packet is sent successfully; false otherwise.
 */
bool sendPacket(serial::Serial *s, const std::vector<uint8_t> &commands, const void *data, std::size_t dataSize)
{
    if (!s || !s->isOpen())
    {
        std::cerr << "❌ Serial port not open!" << std::endl;
        return FAILURE;
    }

    std::vector<uint8_t> packet;
    // Header bytes
    packet.push_back('N');
    packet.push_back('E');
    packet.push_back('X');

    // Append all command bytes from the vector
    for (uint8_t byte : commands)
    {
        packet.push_back(byte);
    }

    // Optional additional data
    if (data && dataSize > 0)
    {
        const uint8_t *d = static_cast<const uint8_t *>(data);
        packet.insert(packet.end(), d, d + dataSize);
    }

    // Compute checksum over the packet so far and append it
    uint8_t checksum = computeChecksum(packet.data(), packet.size());
    packet.push_back(checksum);

    _DEBUG(
        std::cout << "Sent Packet (HEX): ";
        for (auto byte : packet) {
            printf("0x%02X ", byte);
        } std::cout
        << std::endl;)

    s->flushInput();
    std::size_t bytes_written = s->write(packet);
    if (bytes_written != packet.size())
    {
        std::cerr << "⚠️ Warning: Not all bytes were sent! Expected: " << packet.size()
                  << ", Sent: " << bytes_written << std::endl;
        return FAILURE;
    }
    return SUCCESS;
}

/**
 * @brief Receive a response from the serial port.
 *
 * The response packet structure is assumed to be:
 * Header ('S' or 'F') + Command + Data ... + Checksum
 *
 * The function verifies the checksum and copies the payload (after the header and command)
 * into the provided buffer.
 *
 * @param s Pointer to the serial object.
 * @param expectedBytes Total number of bytes expected in the response (including checksum).
 * @param respOut Pointer to the output buffer.
 * @param outSize Size (in bytes) of the output buffer.
 * @return true if the response is received and verified successfully; false otherwise.
 */
bool receiveResponse(serial::Serial *s, std::size_t expectedBytes, void *respOut, std::size_t outSize)
{
    if (!s || !s->isOpen())
    {
        std::cerr << "❌ Serial port not open!" << std::endl;
        return FAILURE;
    }

    std::vector<uint8_t> response;
    std::size_t bytes_read = s->read(response, expectedBytes);

    _DEBUG(
        std::cout << "Raw Response (HEX): ";
        for (auto byte : response) {
            printf("0x%02X ", byte);
        } std::cout
        << std::endl;)

    if (response.empty() || response.size() < 3) // Minimum: header + command + checksum
    {
        std::cerr << "❌ No valid response received." << std::endl;
        return FAILURE;
    }

    uint8_t receivedChecksum = response.back();
    std::vector<uint8_t> withoutChecksum(response.begin(), response.end() - 1);
    uint8_t computedChecksum = computeChecksum(withoutChecksum.data(), withoutChecksum.size());
    if (receivedChecksum != computedChecksum)
    {
        std::cerr << "❌ Checksum mismatch! Received: 0x" << std::hex << static_cast<int>(receivedChecksum)
                  << ", Computed: 0x" << static_cast<int>(computedChecksum) << std::endl;
        return FAILURE;
    }

    // Assume first two bytes (header and command) are not part of the payload
    if (withoutChecksum.size() < 2)
    {
        std::cerr << "❌ Response too short." << std::endl;
        return FAILURE;
    }
    std::size_t available = withoutChecksum.size() - 2;
    if (outSize > available)
    {
        std::cerr << "❌ Requested output size (" << outSize << ") is larger than available payload (" << available << ")." << std::endl;
        return FAILURE;
    }

    std::memcpy(respOut, withoutChecksum.data() + 2, outSize);
    return SUCCESS;
}

/**
 * @brief Execute a packet-based command by sending the packet and then receiving the response.
 *
 * This function uses sendPacket and receiveResponse internally.
 *
 * @param s Pointer to the serial object.
 * @param commands Vector of command bytes (can include subcommands if needed).
 * @param data Pointer to additional data bytes (can be nullptr).
 * @param dataSize Number of additional data bytes.
 * @param expectedBytes Total expected bytes in the response (including checksum).
 * @param respOut Pointer to the pre-allocated output buffer for the payload.
 * @param outSize Size of the output buffer.
 * @return true if the command is executed successfully; false otherwise.
 */
bool executeCommand(serial::Serial *s, const std::vector<uint8_t> &commands, const void *data, std::size_t dataSize,
                    std::size_t expectedBytes, void *respOut, std::size_t outSize)
{
    if (!sendPacket(s, commands, data, dataSize))
        return FAILURE;

    if (!receiveResponse(s, expectedBytes, respOut, outSize))
        return FAILURE;
    return SUCCESS;
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
