/* MCP2210 class - Version 0.4.0
   Copyright (c) 2022 Samuel Lourenço

   This library is free software: you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or (at your
   option) any later version.

   This library is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
   License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this library.  If not, see <https://www.gnu.org/licenses/>.


   Please feel free to contact me via e-mail: samuel.fmlourenco@gmail.com */


#ifndef MCP2210_H
#define MCP2210_H

// Includes
#include <cstdint>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>

class MCP2210
{
private:
    libusb_context *context_;
    libusb_device_handle *handle_;
    bool disconnected_, kernelWasAttached_;

public:
    // Class definitions
    static const uint16_t VID = 0x04D8;    // Default USB vendor ID
    static const uint16_t PID = 0x00DE;    // Default USB product ID
    static const int SUCCESS = 0;          // Returned by open() if successful
    static const int ERROR_INIT = 1;       // Returned by open() in case of a libusb initialization failure
    static const int ERROR_NOT_FOUND = 2;  // Returned by open() if the device was not found
    static const int ERROR_BUSY = 3;       // Returned by open() if the device is already in use

    // Transfer specific definitions
    static const uint8_t EPIN = 0x81;       // Address of endpoint assuming the IN direction
    static const uint8_t EPOUT = 0x01;      // Address of endpoint assuming the OUT direction
    static const size_t COMMAND_SIZE = 64;  // HID command size

    // HID commands
    static const uint8_t GET_CHIP_SETTINGS = 0x20;  // Get GPIO current chip settings
    static const uint8_t SET_CHIP_SETTINGS = 0x21;  // Set current chip settings
    static const uint8_t SET_SPI_SETTINGS = 0x40;   // Set SPI transfer settings
    static const uint8_t GET_SPI_SETTINGS = 0x41;   // Get SPI transfer settings

    // HID command responses
    static const uint8_t COMPLETED = 0x00;      // Command completed successfully
    static const uint8_t BUSY = 0xF7;           // SPI bus not available
    static const uint8_t IN_PROGRESS = 0xF8;    // USB or SPI transfer in progress
    static const uint8_t UNKNOWN = 0xF9;        // Response to unknown command
    static const uint8_t WRITE_FAILURE = 0xFA;  // EEPROM write failure
    static const uint8_t BLOCKED = 0xFB;        // Access not allowed or blocked, or EEPROM is password protected
    static const uint8_t UNDEFINED = 0xFF;      // Undefined response (error)

    // The following values are applicable to SPISettings/configureSPISettings()/getSPISettings()
    static const uint32_t BRT3M = 3000000;  // Value corresponding to a bit rate of 3Mb
    static const uint8_t SPIMODE0 = 0x00;   // Value corresponding to SPI mode 0
    static const uint8_t SPIMODE1 = 0x01;   // Value corresponding to SPI mode 1
    static const uint8_t SPIMODE2 = 0x02;   // Value corresponding to SPI mode 2
    static const uint8_t SPIMODE3 = 0x03;   // Value corresponding to SPI mode 3

    struct SPISettings {
        uint16_t nbytes;    // Number of bytes per SPI transaction
        uint32_t bitrate;   // Bit rate
        uint8_t mode;       // SPI mode (0, 1, 2 or 3)
        uint8_t actcs;      // Active chip select value (CS7 to CS0)
        uint8_t idlcs;      // Idle chip select value (CS7 to CS0)
        uint16_t csdtdly;   // Chip select to data delay (100us units)
        uint16_t dtcsdly;   // Data to chip select (de-asserted) delay (100us units)
        uint16_t itbytdly;  // Inter-byte delay (100us units)

        bool operator ==(const SPISettings &other) const;
        bool operator !=(const SPISettings &other) const;
    };

    MCP2210();
    ~MCP2210();

    bool disconnected() const;
    bool isOpen() const;

    void close();
    uint8_t configureSPISettings(const SPISettings &settings, int &errcnt, std::string &errstr);
    SPISettings getSPISettings(int &errcnt, std::string &errstr);
    std::vector<uint8_t> hidTransfer(const std::vector<uint8_t> &data, int &errcnt, std::string &errstr);
    void interruptTransfer(uint8_t endpointAddr, unsigned char *data, int length, int *transferred, int &errcnt, std::string &errstr);
    int open(uint16_t vid, uint16_t pid, const std::string &serial = std::string());

    static std::vector<std::string> listDevices(uint16_t vid, uint16_t pid, int &errcnt, std::string &errstr);
};

#endif  // MCP2210_H
