/* MCP2210 class - Version 0.2.0
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
#include <list>
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
    static const uint16_t VID = 0x04D8;     // Default USB vendor ID
    static const uint16_t PID = 0x00DE;     // Default USB product ID
    static const uint8_t EPIN = 0x81;       // Address of endpoint assuming the IN direction
    static const uint8_t EPOUT = 0x01;      // Address of endpoint assuming the OUT direction
    static const int SUCCESS = 0;           // Returned by open() if successful
    static const int ERROR_INIT = 1;        // Returned by open() in case of a libusb initialization failure
    static const int ERROR_NOT_FOUND = 2;   // Returned by open() if the device was not found
    static const int ERROR_BUSY = 3;        // Returned by open() if the device is already in use
    static const size_t COMMAND_SIZE = 64;  // HID command size

    MCP2210();
    ~MCP2210();

    bool disconnected() const;
    bool isOpen() const;

    void close();
    std::vector<uint8_t> hidTransfer(const std::vector<uint8_t> &data, int &errcnt, std::string &errstr);
    void interruptTransfer(uint8_t endpointAddr, unsigned char *data, int length, int *transferred, int &errcnt, std::string &errstr);
    int open(uint16_t vid, uint16_t pid, const std::string &serial = std::string());

    static std::list<std::string> listDevices(uint16_t vid, uint16_t pid, int &errcnt, std::string &errstr);
};

#endif  // MCP2210_H
