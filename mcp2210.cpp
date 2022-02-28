/* MCP2210 class - Version 0.4.1
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


// Includes
#include <cstring>
#include <iomanip>
#include <sstream>
#include "mcp2210.h"
extern "C" {
#include "libusb-extra.h"
}

// Definitions
const unsigned int TR_TIMEOUT = 500;  // Transfer timeout in milliseconds

// "Equal to" operator for SPISettings
bool MCP2210::SPISettings::operator ==(const MCP2210::SPISettings &other) const
{
    return nbytes == other.nbytes && bitrate == other.bitrate && mode == other.mode && actcs == other.actcs && idlcs == other.idlcs && csdtdly == other.csdtdly && dtcsdly == other.dtcsdly && itbytdly == other.itbytdly;
}

// "Not equal to" operator for SPISettings
bool MCP2210::SPISettings::operator !=(const MCP2210::SPISettings &other) const
{
    return !(operator ==(other));
}

MCP2210::MCP2210() :
    context_(nullptr),
    handle_(nullptr),
    disconnected_(false),
    kernelWasAttached_(false)
{
}

MCP2210::~MCP2210()
{
    close();  // The destructor is used to close the device, and this is essential so the device can be freed when the parent object is destroyed
}

// Diagnostic function used to verify if the device has been disconnected
bool MCP2210::disconnected() const
{
    return disconnected_;  // Returns true if the device has been disconnected, or false otherwise
}

// Checks if the device is open
bool MCP2210::isOpen() const
{
    return handle_ != nullptr;  // Returns true if the device is open, or false otherwise
}

// Closes the device safely, if open
void MCP2210::close()
{
    if (isOpen()) {  // This condition avoids a segmentation fault if the calling algorithm tries, for some reason, to close the same device twice (e.g., if the device is already closed when the destructor is called)
        libusb_release_interface(handle_, 0);  // Release the interface
        if (kernelWasAttached_) {  // If a kernel driver was attached to the interface before
            libusb_attach_kernel_driver(handle_, 0);  // Reattach the kernel driver
        }
        libusb_close(handle_);  // Close the device
        libusb_exit(context_);  // Deinitialize libusb
        handle_ = nullptr;  // Required to mark the device as closed
    }
}

// Configures SPI transfer settings
uint8_t MCP2210::configureSPISettings(const SPISettings &settings, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command{
        SET_SPI_SETTINGS, 0x00, 0x00, 0x00,            // Header
        static_cast<uint8_t>(settings.bitrate),        // Bit rate
        static_cast<uint8_t>(settings.bitrate >> 8),
        static_cast<uint8_t>(settings.bitrate >> 16),
        static_cast<uint8_t>(settings.bitrate >> 24),
        settings.idlcs, 0x00,                          // Idle chip select
        settings.actcs, 0x00,                          // Active chip select
        static_cast<uint8_t>(settings.csdtdly),        // Chip select to data delay
        static_cast<uint8_t>(settings.csdtdly >> 8),
        static_cast<uint8_t>(settings.dtcsdly),        // Data to chip select delay
        static_cast<uint8_t>(settings.dtcsdly >> 8),
        static_cast<uint8_t>(settings.itbytdly),       // Inter-byte delay
        static_cast<uint8_t>(settings.itbytdly >> 8),
        static_cast<uint8_t>(settings.nbytes),         // Number of bytes per SPI transaction
        static_cast<uint8_t>(settings.nbytes >> 8),
        settings.mode                                  // SPI mode
    };
    int preverrcnt = errcnt;
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    uint8_t retval = UNDEFINED;
    if (errcnt == preverrcnt) {
        if (response[0] != SET_SPI_SETTINGS) {
            ++errcnt;
            errstr += "Received mismatched response to HID command.\n";
        } else {
            retval = response[1];
        }
    }
    return retval;
}

// Returns applied SPI transfer settings
MCP2210::SPISettings MCP2210::getSPISettings(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command{
        GET_SPI_SETTINGS
    };
    int preverrcnt = errcnt;
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    SPISettings settings;
    if (errcnt == preverrcnt) {
        if (response[0] != GET_SPI_SETTINGS) {
            ++errcnt;
            errstr += "Received mismatched response to HID command.\n";
        } else {
            settings.nbytes = static_cast<uint16_t>(response[18] | response[19] << 8);                                         // Number of bytes per SPI transfer corresponds to bytes 18 and 19 (little-endian conversion)
            settings.bitrate = static_cast<uint32_t>(response[4] | response[5] << 8 | response[6] << 16 | response[7] << 24);  // Bit rate corresponds to bytes 4, 5, 6 and 7 (little-endian conversion)
            settings.mode = response[20];                                                                                      // SPI mode corresponds to byte 20
            settings.actcs = response[10];                                                                                     // Active chip select value corresponds to byte 10
            settings.idlcs = response[8];                                                                                      // Idle chip select value corresponds to byte 8
            settings.csdtdly = static_cast<uint16_t>(response[12] | response[13] << 8);                                        // Chip select to data corresponds to bytes 12 and 13 (little-endian conversion)
            settings.dtcsdly = static_cast<uint16_t>(response[14] | response[15] << 8);                                        // Data to chip select delay corresponds to bytes 14 and 15 (little-endian conversion)
            settings.itbytdly = static_cast<uint16_t>(response[16] | response[17] << 8);                                       // Inter-byte delay corresponds to bytes 16 and 17 (little-endian conversion)
        }
    }
    return settings;
}

// Sends a HID command based on the given vector, and returns the response
// The command vector can be shorter or longer than 64 bytes, but the resulting command will either be padded with zeros or truncated in order to fit
std::vector<uint8_t> MCP2210::hidTransfer(const std::vector<uint8_t> &data, int &errcnt, std::string &errstr)
{
    size_t bytesToFill = data.size() > COMMAND_SIZE ? COMMAND_SIZE : data.size();
    unsigned char commandBuffer[COMMAND_SIZE] = {0x00};  // It is important to initialize the array in this manner, so that unused indexes are filled with zeros!
    for (size_t i = 0; i < bytesToFill; ++i) {
        commandBuffer[i] = data[i];
    }
    int preverrcnt = errcnt;
#if LIBUSB_API_VERSION >= 0x01000105
    interruptTransfer(EPOUT, commandBuffer, static_cast<int>(COMMAND_SIZE), nullptr, errcnt, errstr);
#else
    int bytesWritten;
    interruptTransfer(EPOUT, commandBuffer, static_cast<int>(COMMAND_SIZE), &bytesWritten, errcnt, errstr);
#endif
    unsigned char responseBuffer[COMMAND_SIZE];
    int bytesRead = 0;  // Important!
    interruptTransfer(EPIN, responseBuffer, static_cast<int>(COMMAND_SIZE), &bytesRead, errcnt, errstr);
    std::vector<uint8_t> retdata(bytesRead);
    for (int i = 0; i < bytesRead; ++i) {
        retdata[i] = responseBuffer[i];
    }
    if (errcnt == preverrcnt && bytesRead < static_cast<int>(COMMAND_SIZE)) {  // This additional verification only makes sense if the error count does not increase
        ++errcnt;
        errstr += "Received incomplete response to HID command.\n";
    }
    return retdata;
}

// Safe interrupt transfer
void MCP2210::interruptTransfer(uint8_t endpointAddr, unsigned char *data, int length, int *transferred, int &errcnt, std::string &errstr)
{
    if (!isOpen()) {
        ++errcnt;
        errstr += "In interruptTransfer(): device is not open.\n";  // Program logic error
    } else {
        int result = libusb_interrupt_transfer(handle_, endpointAddr, data, length, transferred, TR_TIMEOUT);
        if (result != 0 || (transferred != nullptr && *transferred != length)) {  // The number of transferred bytes is also verified, as long as a valid (non-null) pointer is passed via "transferred"
            ++errcnt;
            std::ostringstream stream;
            if (endpointAddr < 0x80) {
                stream << "Failed interrupt OUT transfer to endpoint "
                       << (0x0F & endpointAddr)
                       << " (address 0x"
                       << std::hex << std::setfill ('0') << std::setw(2) << static_cast<int>(endpointAddr)
                       << ")." << std::endl;
            } else {
                stream << "Failed interrupt IN transfer from endpoint "
                       << (0x0F & endpointAddr)
                       << " (address 0x"
                       << std::hex << std::setfill ('0') << std::setw(2) << static_cast<int>(endpointAddr)
                       << ")." << std::endl;
            }
            errstr += stream.str();
            if (result == LIBUSB_ERROR_NO_DEVICE || result == LIBUSB_ERROR_IO) {  // Note that libusb_interrupt_transfer() may return "LIBUSB_ERROR_IO" [-1] on device disconnect
                disconnected_ = true;  // This reports that the device has been disconnected
            }
        }
    }
}

// Opens the device having the given VID, PID and, optionally, the given serial number, and assigns its handle
int MCP2210::open(uint16_t vid, uint16_t pid, const std::string &serial)
{
    int retval = SUCCESS;
    if (!isOpen()) {  // Just in case the calling algorithm tries to open a device that was already sucessfully open, or tries to open different devices concurrently, all while using (or referencing to) the same object
        if (libusb_init(&context_) != 0) {  // Initialize libusb. In case of failure
            retval = ERROR_INIT;
        } else {  // If libusb is initialized
            if (serial.empty()) {  // Note that serial, by omission, is an empty string
                handle_ = libusb_open_device_with_vid_pid(context_, vid, pid);  // If no serial number is specified, this will open the first device found with matching VID and PID
            } else {
                char *serialcstr = new char[serial.size() + 1];
                std::strcpy(serialcstr, serial.c_str());
                handle_ = libusb_open_device_with_vid_pid_serial(context_, vid, pid, reinterpret_cast<unsigned char *>(serialcstr));
                delete[] serialcstr;
            }
            if (handle_ == nullptr) {  // If the previous operation fails to get a device handle
                libusb_exit(context_);  // Deinitialize libusb
                retval = ERROR_NOT_FOUND;
            } else {  // If the device is successfully opened and a handle obtained
                if (libusb_kernel_driver_active(handle_, 0) == 1) {  // If a kernel driver is active on the interface
                    libusb_detach_kernel_driver(handle_, 0);  // Detach the kernel driver
                    kernelWasAttached_ = true;  // Flag that the kernel driver was attached
                } else {
                    kernelWasAttached_ = false;  // The kernel driver was not attached
                }
                if (libusb_claim_interface(handle_, 0) != 0) {  // Claim the interface. In case of failure
                    if (kernelWasAttached_) {  // If a kernel driver was attached to the interface before
                        libusb_attach_kernel_driver(handle_, 0);  // Reattach the kernel driver
                    }
                    libusb_close(handle_);  // Close the device
                    libusb_exit(context_);  // Deinitialize libusb
                    handle_ = nullptr;  // Required to mark the device as closed
                    retval = ERROR_BUSY;
                } else {
                    disconnected_ = false;  // Note that this flag is never assumed to be true for a device that was never opened - See constructor for details!
                }
            }
        }
    }
    return retval;
}

// Helper function to list devices
std::vector<std::string> MCP2210::listDevices(uint16_t vid, uint16_t pid, int &errcnt, std::string &errstr)
{
    std::vector<std::string> devices;
    libusb_context *context;
    if (libusb_init(&context) != 0) {  // Initialize libusb. In case of failure
        ++errcnt;
        errstr += "Could not initialize libusb.\n";
    } else {  // If libusb is initialized
        libusb_device **devs;
        ssize_t devlist = libusb_get_device_list(context, &devs);  // Get a device list
        if (devlist < 0) {  // If the previous operation fails to get a device list
            ++errcnt;
            errstr += "Failed to retrieve a list of devices.\n";
        } else {
            for (ssize_t i = 0; i < devlist; ++i) {  // Run through all listed devices
                libusb_device_descriptor desc;
                if (libusb_get_device_descriptor(devs[i], &desc) == 0 && desc.idVendor == vid && desc.idProduct == pid) {  // If the device descriptor is retrieved, and both VID and PID correspond to the respective given values
                    libusb_device_handle *handle;
                    if (libusb_open(devs[i], &handle) == 0) {  // Open the listed device. If successfull
                        unsigned char str_desc[256];
                        libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, str_desc, static_cast<int>(sizeof(str_desc)));  // Get the serial number string in ASCII format
                        devices.push_back(reinterpret_cast<char *>(str_desc));  // Add the serial number string to the list
                        libusb_close(handle);  // Close the device
                    }
                }
            }
            libusb_free_device_list(devs, 1);  // Free device list
        }
        libusb_exit(context);  // Deinitialize libusb
    }
    return devices;
}
