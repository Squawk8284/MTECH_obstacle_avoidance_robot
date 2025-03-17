#ifndef __NEX_ROBOT_H__
#define __NEX_ROBOT_H__

#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <serial/serial.h> // Requires the serial library

#define DEC_TO_HEX(decimal) (std::hex << decimal)

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
serial::Serial* createSerial(const std::string &port, uint32_t baud) {
    serial::Serial* s = new serial::Serial();
    s->setPort(port);
    s->setBaudrate(baud);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    s->setTimeout(timeout);
    try {
        s->open();
        if (!s->isOpen()) {
            std::cerr << "Unable to open port: " << port << std::endl;
            delete s;
            return nullptr;
        }
    } catch (const std::exception &e) {
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
void clearSerial(serial::Serial* s) {
    if (s) {
        if (s->isOpen()) {
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
uint8_t computeChecksum(const std::vector<uint8_t>& data) {
    uint16_t sum = 0;
    for (auto byte : data) {
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
bool sendCommand(serial::Serial* s, uint8_t command, const std::vector<uint8_t>& data = {}) {
    if (!s || !s->isOpen()) {
        std::cerr << "Serial port not open!" << std::endl;
        return false;
    }
    std::vector<uint8_t> packet = {'N', 'E', 'X', command};
    packet.insert(packet.end(), data.begin(), data.end());
    packet.push_back(computeChecksum(packet));
    s->write(packet);
    return true;
}

/**
 * @brief Receive response from the serial port. expectedBytes should include the checksum.
 * 
 * @param s 
 * @param expectedBytes 
 * @return std::vector<uint8_t> 
 */
std::vector<uint8_t> receiveResponse(serial::Serial* s, size_t expectedBytes) {
    std::vector<uint8_t> response;
    if (!s || !s->isOpen()) {
        std::cerr << "Serial port not open!" << std::endl;
        return response;
    }
    try {
        response.resize(expectedBytes);
        size_t bytes_read = s->read(response, response.size());
        if (bytes_read > 0) {
            response.resize(bytes_read);
            return response;
        }
    } catch (const std::exception &e) {
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
bool executeCommand(serial::Serial* s, uint8_t command, const std::vector<uint8_t>& data, size_t expectedBytes, std::vector<uint8_t>* respOut = nullptr) 
{
    if (!sendCommand(s, command, data))
        return false;
    std::vector<uint8_t> response = receiveResponse(s, expectedBytes);
    if (response.empty() || response.size() < 3) 
    { // minimal response: header, command, checksum
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
    if (withoutChecksum[0] != 'S') 
    {
        std::cerr << "Command failed. Header received: " << withoutChecksum[0] << std::endl;
        return false;
    }
    // If caller wants the response data, return data bytes (from index 2 onward).
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