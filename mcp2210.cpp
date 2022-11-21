/* MCP2210 class - Version 1.1.0
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
const uint8_t EPIN = 0x81;            // Address of endpoint assuming the IN direction
const uint8_t EPOUT = 0x01;           // Address of endpoint assuming the OUT direction
const unsigned int TR_TIMEOUT = 500;  // Transfer timeout in milliseconds

// Private generic function that is used to get any descriptor
std::u16string MCP2210::getDescGeneric(uint8_t subcomid, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_NVRAM_SETTINGS, subcomid  // Header
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    size_t maxLength = 2 * DESC_MAXLEN + 2;  // Maximum descriptor length in bytes
    size_t length = (response[4] > maxLength ? maxLength : response[4]) - 2;  // Descriptor internal length
    std::u16string descriptor;
    for (size_t i = 0; i < length; i += 2) {
        descriptor += static_cast<char16_t>(response[i + 7] << 8 | response[i + 6]);  // UTF-16LE conversion as per the USB 2.0 specification
    }
    return descriptor;
}

// Private function that is used to perform interrupt transfers
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
                       << (0x0f & endpointAddr)
                       << " (address 0x"
                       << std::hex << std::setfill ('0') << std::setw(2) << static_cast<int>(endpointAddr)
                       << ")." << std::endl;
            } else {
                stream << "Failed interrupt IN transfer from endpoint "
                       << (0x0f & endpointAddr)
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

// Private generic function that is used to write any descriptor
uint8_t MCP2210::writeDescGeneric(const std::u16string &descriptor, uint8_t subcomid, int &errcnt, std::string &errstr)
{
    size_t strLength = descriptor.size();  // Descriptor string length
    std::vector<uint8_t> command(2 * strLength + 6);
    command[0] = SET_NVRAM_SETTINGS;                       // Header
    command[1] = subcomid;
    command[2] = 0x00;
    command[3] = 0x00;
    command[4] = static_cast<uint8_t>(2 * strLength + 2);  // Descriptor length in bytes
    command[5] = 0x03;                                     // USB descriptor constant
    for (size_t i = 0; i < strLength; ++i) {
        command[2 * i + 6] = static_cast<uint8_t>(descriptor[i]);
        command[2 * i + 7] = static_cast<uint8_t>(descriptor[i] >> 8);
    }
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[1];
}

// "Equal to" operator for ChipSettings
bool MCP2210::ChipSettings::operator ==(const MCP2210::ChipSettings &other) const
{
    return gp0 == other.gp0 && gp1 == other.gp1 && gp2 == other.gp2 && gp3 == other.gp3 && gp4 == other.gp4 && gp5 == other.gp5 && gp6 == other.gp6 && gp7 == other.gp7 && gp8 == other.gp8 && gpdir == other.gpdir && gpout == other.gpout && rmwakeup == other.rmwakeup && intmode == other.intmode && nrelspi == other.nrelspi;
}

// "Not equal to" operator for ChipSettings
bool MCP2210::ChipSettings::operator !=(const MCP2210::ChipSettings &other) const
{
    return !(operator ==(other));
}

// "Equal to" operator for ChipStatus
bool MCP2210::ChipStatus::operator ==(const MCP2210::ChipStatus &other) const
{
    return busreq == other.busreq && busowner == other.busowner && pwtries == other.pwtries && pwok == other.pwok;
}

// "Not equal to" operator for ChipStatus
bool MCP2210::ChipStatus::operator !=(const MCP2210::ChipStatus &other) const
{
    return !(operator ==(other));
}

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

// "Equal to" operator for USBParameters
bool MCP2210::USBParameters::operator ==(const MCP2210::USBParameters &other) const
{
    return vid == other.vid && pid == other.pid && maxpow == other.maxpow && powmode == other.powmode && rmwakeup == other.rmwakeup;
}

// "Not equal to" operator for USBParameters
bool MCP2210::USBParameters::operator !=(const MCP2210::USBParameters &other) const
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

// Cancels the ongoing SPI transfer
uint8_t MCP2210::cancelSPITransfer(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        CANCEL_SPI_TRANSFER  // Header
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[1];
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

// Configures volatile chip settings
uint8_t MCP2210::configureChipSettings(const ChipSettings &settings, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        SET_CHIP_SETTINGS, 0x00, 0x00, 0x00,                                                              // Header
        settings.gp0,                                                                                     // GP0 pin configuration
        settings.gp1,                                                                                     // GP1 pin configuration
        settings.gp2,                                                                                     // GP2 pin configuration
        settings.gp3,                                                                                     // GP3 pin configuration
        settings.gp4,                                                                                     // GP4 pin configuration
        settings.gp5,                                                                                     // GP5 pin configuration
        settings.gp6,                                                                                     // GP6 pin configuration
        settings.gp7,                                                                                     // GP7 pin configuration
        settings.gp8,                                                                                     // GP8 pin configuration
        settings.gpout, 0x00,                                                                             // Default GPIO outputs (GPIO7 to GPIO0)
        settings.gpdir, 0x01,                                                                             // Default GPIO directions (GPIO7 to GPIO0)
        static_cast<uint8_t>(settings.rmwakeup << 4 | (0x07 & settings.intmode) << 1 | settings.nrelspi)  // Other chip settings
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[1];
}

// Configures volatile SPI transfer settings
uint8_t MCP2210::configureSPISettings(const SPISettings &settings, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        SET_SPI_SETTINGS, 0x00, 0x00, 0x00,                                                          // Header
        static_cast<uint8_t>(settings.bitrate), static_cast<uint8_t>(settings.bitrate >> 8),         // Bit rate
        static_cast<uint8_t>(settings.bitrate >> 16), static_cast<uint8_t>(settings.bitrate >> 24),
        settings.idlcs, 0x00,                                                                        // Idle chip select (CS7 to CS0)
        settings.actcs, 0x00,                                                                        // Active chip select (CS7 to CS0)
        static_cast<uint8_t>(settings.csdtdly), static_cast<uint8_t>(settings.csdtdly >> 8),         // Chip select to data delay
        static_cast<uint8_t>(settings.dtcsdly), static_cast<uint8_t>(settings.dtcsdly >> 8),         // Data to chip select delay
        static_cast<uint8_t>(settings.itbytdly), static_cast<uint8_t>(settings.itbytdly >> 8),       // Inter-byte delay
        static_cast<uint8_t>(settings.nbytes), static_cast<uint8_t>(settings.nbytes >> 8),           // Number of bytes per SPI transaction
        settings.mode                                                                                // SPI mode
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[1];
}

// Retrieves the access control mode from the MCP2210 NVRAM
uint8_t MCP2210::getAccessControlMode(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_NVRAM_SETTINGS, NV_CHIP_SETTINGS  // Header
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[18];  // Access control mode corresponds to byte 18
}


// Returns applied chip settings
MCP2210::ChipSettings MCP2210::getChipSettings(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_CHIP_SETTINGS  // Header
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    ChipSettings settings;
    settings.gp0 = response[4];                                         // GP0 pin configuration corresponds to byte 4
    settings.gp1 = response[5];                                         // GP1 pin configuration corresponds to byte 5
    settings.gp2 = response[6];                                         // GP2 pin configuration corresponds to byte 6
    settings.gp3 = response[7];                                         // GP3 pin configuration corresponds to byte 7
    settings.gp4 = response[8];                                         // GP4 pin configuration corresponds to byte 8
    settings.gp5 = response[9];                                         // GP5 pin configuration corresponds to byte 9
    settings.gp6 = response[10];                                        // GP6 pin configuration corresponds to byte 10
    settings.gp7 = response[11];                                        // GP7 pin configuration corresponds to byte 11
    settings.gp8 = response[12];                                        // GP8 pin configuration corresponds to byte 12
    settings.gpdir = response[15];                                      // Default GPIO directions (GPIO7 to GPIO0) corresponds to byte 15
    settings.gpout = response[13];                                      // Default GPIO outputs (GPIO7 to GPIO0) corresponds to byte 13
    settings.rmwakeup = (0x10 & response[17]) != 0x00;                  // Remote wakeup corresponds to bit 4 of byte 17
    settings.intmode = static_cast<uint8_t>(0x07 & response[17] >> 1);  // Interrupt counting mode corresponds to bits 3:1 of byte 17
    settings.nrelspi = (0x01 & response[17]) != 0x00;                   // SPI bus release corresponds to bit 0 of byte 17
    return settings;
}

// Returns the current status
MCP2210::ChipStatus MCP2210::getChipStatus(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_CHIP_STATUS  // Header
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    ChipStatus status;
    status.busreq = response[2] != 0x01;  // SPI bus release external request status corresponds to byte 2
    status.busowner = response[3];        // SPI bus current owner corresponds to byte 3
    status.pwtries = response[4];         // Number of NVRAM password tries corresponds to byte 4
    status.pwok = response[5] != 0x00;    // Password validation status corresponds to byte 5
    return status;
}

// Gets the number of events from the interrupt pin
uint16_t MCP2210::getEventCount(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_EVENT_COUNT,  // Header
        0x01              // Do not reset the event counter
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return static_cast<uint16_t>(response[5] << 8 | response[4]);  // Event count corresponds to bytes 4 and 5 (little-endian conversion)
}

// Returns the value of a given GPIO pin on the MCP2210
bool MCP2210::getGPIO(int gpio, int &errcnt, std::string &errstr)
{
    bool value;
    if (gpio < 0 || gpio > 8) {
        ++errcnt;
        errstr += "In getGPIO(): GPIO pin number must be between 0 and 8.\n";  // Program logic error
        value = false;
    } else {
        value = (0x0001 << gpio & getGPIOs(errcnt, errstr)) != 0x0000;
    }
    return value;
}

// Returns the direction of a given GPIO pin on the MCP2210
bool MCP2210::getGPIODirection(int gpio, int &errcnt, std::string &errstr)
{
    bool direction;
    if (gpio < 0 || gpio > 7) {
        ++errcnt;
        errstr += "In getGPIODirection(): GPIO pin number must be between 0 and 7.\n";  // Program logic error
        direction = false;
    } else {
        direction = (0x01 << gpio & getGPIODirections(errcnt, errstr)) != 0x00;
    }
    return direction;
}

// Returns the directions of all GPIO pins on the MCP2210
uint8_t MCP2210::getGPIODirections(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_GPIO_DIRECTIONS  // Header
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[4];  // GPIO directions (GPIO7 to GPIO0) corresponds to byte 4
}

// Returns the values of all GPIO pins on the MCP2210
uint16_t MCP2210::getGPIOs(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_GPIO_VALUES  // Header
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return static_cast<uint16_t>((0x01 & response[5]) << 8 | response[4]);  // GPIO values (GPIO8 to GPIO0) corresponds to bytes 4 and 5
}

// Retrieves the manufacturer descriptor from the MCP2210 NVRAM
std::u16string MCP2210::getManufacturerDesc(int &errcnt, std::string &errstr)
{
    return getDescGeneric(MANUFACTURER_NAME, errcnt, errstr);
}

// Retrieves the power-up (non-volatile) chip settings from the MCP2210 NVRAM
MCP2210::ChipSettings MCP2210::getNVChipSettings(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_NVRAM_SETTINGS, NV_CHIP_SETTINGS  // Header
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    ChipSettings settings;
    settings.gp0 = response[4];                                         // GP0 pin configuration corresponds to byte 4
    settings.gp1 = response[5];                                         // GP1 pin configuration corresponds to byte 5
    settings.gp2 = response[6];                                         // GP2 pin configuration corresponds to byte 6
    settings.gp3 = response[7];                                         // GP3 pin configuration corresponds to byte 7
    settings.gp4 = response[8];                                         // GP4 pin configuration corresponds to byte 8
    settings.gp5 = response[9];                                         // GP5 pin configuration corresponds to byte 9
    settings.gp6 = response[10];                                        // GP6 pin configuration corresponds to byte 10
    settings.gp7 = response[11];                                        // GP7 pin configuration corresponds to byte 11
    settings.gp8 = response[12];                                        // GP8 pin configuration corresponds to byte 12
    settings.gpdir = response[15];                                      // Default GPIO directions (GPIO7 to GPIO0) corresponds to byte 15
    settings.gpout = response[13];                                      // Default GPIO outputs (GPIO7 to GPIO0) corresponds to byte 13
    settings.rmwakeup = (0x10 & response[17]) != 0x00;                  // Remote wakeup corresponds to bit 4 of byte 17
    settings.intmode = static_cast<uint8_t>(0x07 & response[17] >> 1);  // Interrupt counting mode corresponds to bits 3:1 of byte 17
    settings.nrelspi = (0x01 & response[17]) != 0x00;                   // SPI bus release corresponds to bit 0 of byte 17
    return settings;
}

// Retrieves the power-up (non-volatile) SPI transfer settings from the MCP2210 NVRAM
MCP2210::SPISettings MCP2210::getNVSPISettings(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_NVRAM_SETTINGS, NV_SPI_SETTINGS  // Header
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    SPISettings settings;
    settings.nbytes = static_cast<uint16_t>(response[19] << 8 | response[18]);                                         // Number of bytes per SPI transfer corresponds to bytes 18 and 19 (little-endian conversion)
    settings.bitrate = static_cast<uint32_t>(response[7] << 24 | response[6] << 16 | response[5] << 8 | response[4]);  // Bit rate corresponds to bytes 4 to 7 (little-endian conversion)
    settings.mode = response[20];                                                                                      // SPI mode corresponds to byte 20
    settings.actcs = response[10];                                                                                     // Active chip select (CS7 to CS0) corresponds to byte 10
    settings.idlcs = response[8];                                                                                      // Idle chip select (CS7 to CS0) corresponds to byte 8
    settings.csdtdly = static_cast<uint16_t>(response[13] << 8 | response[12]);                                        // Chip select to data corresponds to bytes 12 and 13 (little-endian conversion)
    settings.dtcsdly = static_cast<uint16_t>(response[15] << 8 | response[14]);                                        // Data to chip select delay corresponds to bytes 14 and 15 (little-endian conversion)
    settings.itbytdly = static_cast<uint16_t>(response[17] << 8 | response[16]);                                       // Inter-byte delay corresponds to bytes 16 and 17 (little-endian conversion)
    return settings;
}

// Retrieves the product descriptor from the MCP2210 NVRAM
std::u16string MCP2210::getProductDesc(int &errcnt, std::string &errstr)
{
    return getDescGeneric(PRODUCT_NAME, errcnt, errstr);
}

// Returns applied SPI transfer settings
MCP2210::SPISettings MCP2210::getSPISettings(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_SPI_SETTINGS  // Header
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    SPISettings settings;
    settings.nbytes = static_cast<uint16_t>(response[19] << 8 | response[18]);                                         // Number of bytes per SPI transfer corresponds to bytes 18 and 19 (little-endian conversion)
    settings.bitrate = static_cast<uint32_t>(response[7] << 24 | response[6] << 16 | response[5] << 8 | response[4]);  // Bit rate corresponds to bytes 4 to 7 (little-endian conversion)
    settings.mode = response[20];                                                                                      // SPI mode corresponds to byte 20
    settings.actcs = response[10];                                                                                     // Active chip select (CS7 to CS0) corresponds to byte 10
    settings.idlcs = response[8];                                                                                      // Idle chip select (CS7 to CS0) corresponds to byte 8
    settings.csdtdly = static_cast<uint16_t>(response[13] << 8 | response[12]);                                        // Chip select to data corresponds to bytes 12 and 13 (little-endian conversion)
    settings.dtcsdly = static_cast<uint16_t>(response[15] << 8 | response[14]);                                        // Data to chip select delay corresponds to bytes 14 and 15 (little-endian conversion)
    settings.itbytdly = static_cast<uint16_t>(response[17] << 8 | response[16]);                                       // Inter-byte delay corresponds to bytes 16 and 17 (little-endian conversion)
    return settings;
}

// Gets the USB parameters, namely VID, PID and power settings
MCP2210::USBParameters MCP2210::getUSBParameters(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_NVRAM_SETTINGS, USB_PARAMETERS  // Header
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    USBParameters parameters;
    parameters.vid = static_cast<uint16_t>(response[13] << 8 | response[12]);  // Vendor ID corresponds to bytes 12 and 13 (little-endian conversion)
    parameters.pid = static_cast<uint32_t>(response[15] << 8 | response[14]);  // Product ID corresponds to bytes 14 and 15 (little-endian conversion)
    parameters.maxpow = response[30];                                          // Maximum consumption current corresponds to byte 30
    parameters.powmode = (0x80 & response[29]) != 0x00;                        // Power mode corresponds to bit 7 of byte 29 (bit 6 is redundant)
    parameters.rmwakeup = (0x20 & response[29]) != 0x00;                       // Remote wakeup corresponds to bit 5 of byte 29
    return parameters;
}

// Sends a HID command based on the given vector, and returns the response
// The command vector can be shorter or longer than 64 bytes, but the resulting command will either be padded with zeros or truncated in order to fit
std::vector<uint8_t> MCP2210::hidTransfer(const std::vector<uint8_t> &data, int &errcnt, std::string &errstr)
{
    size_t vecSize = data.size();  // Size of "data" (optimization implemented in version 1.0.2)
    size_t bytesToFill = vecSize > COMMAND_SIZE ? COMMAND_SIZE : vecSize;
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
    std::vector<uint8_t> retdata(COMMAND_SIZE);
    for (int i = 0; i < bytesRead; ++i) {
        retdata[i] = responseBuffer[i];
    }
    if (errcnt == preverrcnt && (bytesRead < static_cast<int>(COMMAND_SIZE) || responseBuffer[0] != commandBuffer[0])) {  // This additional verification only makes sense if the error count does not increase
        ++errcnt;
        errstr += "Received invalid response to HID command.\n";
    }
    return retdata;
}

// Opens the device having the given VID, PID and, optionally, the given serial number, and assigns its handle
int MCP2210::open(uint16_t vid, uint16_t pid, const std::string &serial)
{
    int retval;
    if (isOpen()) {  // Just in case the calling algorithm tries to open a device that was already sucessfully open, or tries to open different devices concurrently, all while using (or referencing to) the same object
        retval = SUCCESS;
    } else if (libusb_init(&context_) != 0) {  // Initialize libusb. In case of failure
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
                retval = SUCCESS;
            }
        }
    }
    return retval;
}

// Reads a byte from the given EEPROM address
uint8_t MCP2210::readEEPROMByte(uint8_t address, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        READ_EEPROM,  // Header
        address       // Address to be read
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[3];
}

// Reads the EEPROM within the specified range, returning a vector
// If an error occurs, the size of the vector will be smaller than expected
std::vector<uint8_t> MCP2210::readEEPROMRange(uint8_t begin, uint8_t end, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> values;
    if (begin > end) {
        ++errcnt;
        errstr += "In readEEPROMRange(): the first address cannot be greater than the last address.\n";  // Program logic error
    } else {
        for (size_t i = begin; i <= end; ++i) {
            int preverrcnt = errcnt;
            uint8_t value = readEEPROMByte(static_cast<uint8_t>(i), errcnt, errstr);
            if (errcnt != preverrcnt) {  // If an error occurs
                break;  // Abort
            }
            values.push_back(value);
        }
    }
    return values;
}

// Resets the interrupt event counter
uint8_t MCP2210::resetEventCounter(int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        GET_EVENT_COUNT,  // Header
        0x00              // Reset the event counter
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[1];
}

// Sets the value of a given GPIO pin on the MCP2210
uint8_t MCP2210::setGPIO(int gpio, bool value, int &errcnt, std::string &errstr)
{
    uint8_t retval;
    if (gpio < 0 || gpio > 7) {
        ++errcnt;
        errstr += "In setGPIO(): GPIO pin number must be between 0 and 7.\n";  // Program logic error
        retval = OTHER_ERROR;
    } else {
        int preverrcnt = errcnt;
        uint16_t values = getGPIOs(errcnt, errstr);
        if (errcnt == preverrcnt) {
            uint16_t mask = static_cast<uint16_t>(0x0001 << gpio);
            if (value) {  // If the selected GPIO pin value is set to be high
                values = static_cast<uint16_t>(mask | values);  // Set pin high
            } else {
                values = static_cast<uint16_t>(~mask & values);  // Set pin low
            }
            retval = setGPIOs(values, errcnt, errstr);
        } else {
            retval = OTHER_ERROR;
        }
    }
    return retval;
}

// Sets the direction of a given GPIO pin on the MCP2210
uint8_t MCP2210::setGPIODirection(int gpio, bool direction, int &errcnt, std::string &errstr)
{
    uint8_t retval;
    if (gpio < 0 || gpio > 7) {
        ++errcnt;
        errstr += "In setGPIODirection(): GPIO pin number must be between 0 and 7.\n";  // Program logic error
        retval = OTHER_ERROR;
    } else {
        int preverrcnt = errcnt;
        uint8_t directions = getGPIODirections(errcnt, errstr);
        if (errcnt == preverrcnt) {
            uint8_t mask = static_cast<uint8_t>(0x01 << gpio);
            if (direction) {  // If the selected GPIO pin is to be used as an input
                directions = static_cast<uint8_t>(mask | directions);  // Set pin as input
            } else {
                directions = static_cast<uint8_t>(~mask & directions);  // Set pin as output
            }
            retval = setGPIODirections(directions, errcnt, errstr);
        } else {
            retval = OTHER_ERROR;
        }
    }
    return retval;
}

// Sets the directions of all GPIO pins on the MCP2210
uint8_t MCP2210::setGPIODirections(uint8_t directions, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        SET_GPIO_DIRECTIONS, 0x00, 0x00, 0x00,  // Header
        directions, 0x01                        // GPIO directions (GPIO7 to GPIO0)
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[1];
}

// Sets the values of all GPIO pins on the MCP2210
uint8_t MCP2210::setGPIOs(uint16_t values, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        SET_GPIO_VALUES, 0x00, 0x00, 0x00,  // Header
        static_cast<uint8_t>(values)        // GPIO values (GPIO7 to GPPIO0 - GPIO8 is an input only pin)
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[1];
}

// Performs a basic SPI transfer
// Note that the variable "status" is used to return either the SPI transfer engine status or, in case of error, the HID command response
std::vector<uint8_t> MCP2210::spiTransfer(const std::vector<uint8_t> &data, uint8_t &status, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> retdata;
    size_t bytesToSend = data.size();
    if (bytesToSend > SPIDATA_MAXSIZE) {
        ++errcnt;
        errstr += "In spiTransfer(): vector size cannot exceed 60 bytes.\n";  // Program logic error
    } else {
        std::vector<uint8_t> command(bytesToSend + 4);
        command[0] = TRANSFER_SPI_DATA;                  // Header
        command[1] = static_cast<uint8_t>(bytesToSend);  // Number of bytes to send
        for (size_t i = 0; i < bytesToSend; ++i) {
            command[i + 4] = data[i];
        }
        std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
        if (response[1] == COMPLETED) {  // If the HID transfer was completed
            status = response[3];  // The returned status corresponds to the obtained SPI transfer engine status
            size_t bytesReceived = response[2];
            retdata.resize(bytesReceived);
            for (size_t i = 0; i < bytesReceived; ++i) {
                retdata[i] = response[i + 4];
            }
        } else {
            status = response[1];  // The returned status corresponds to the obtained HID command response (it can be "BUSY" [0xf7] or "IN_PROGRESS" [0xf8])
        }
    }
    return retdata;
}

// Toggles (inverts the value of) a given GPIO pin on the MCP2210
uint8_t MCP2210::toggleGPIO(int gpio, int &errcnt, std::string &errstr)
{
    uint8_t retval;
    if (gpio < 0 || gpio > 7) {
        ++errcnt;
        errstr += "In toggleGPIO(): GPIO pin number must be between 0 and 7.\n";  // Program logic error
        retval = OTHER_ERROR;
    } else {
        int preverrcnt = errcnt;
        uint16_t values = getGPIOs(errcnt, errstr);
        if (errcnt == preverrcnt) {
            uint16_t mask = static_cast<uint16_t>(0x0001 << gpio);
            if ((mask & values) == 0x0000) {  // If the selected GPIO pin is low
                values = static_cast<uint16_t>(mask | values);  // Toggle pin high
            } else {
                values = static_cast<uint16_t>(~mask & values);  // Toggle pin low
            }
            retval = setGPIOs(values, errcnt, errstr);
        } else {
            retval = OTHER_ERROR;
        }
    }
    return retval;
}

// Writes a byte to a given EEPROM address
uint8_t MCP2210::writeEEPROMByte(uint8_t address, uint8_t value, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        WRITE_EEPROM,  // Header
        address,       // Address to be written
        value          // Value
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[1];
}

// Writes over the EEPROM, within the specified range and based on the given vector
uint8_t MCP2210::writeEEPROMRange(uint8_t begin, uint8_t end, const std::vector<uint8_t> &values, int &errcnt, std::string &errstr)
{
    uint8_t retval;
    if (begin > end) {
        ++errcnt;
        errstr += "In writeEEPROMRange(): the first address cannot be greater than the last address.\n";  // Program logic error
        retval = OTHER_ERROR;
    } else {
        size_t vecSize = values.size();  // Size of "values" (optimization implemented in version 1.0.2)
        if (vecSize != end - begin + 1u) {
            ++errcnt;
            errstr += "In writeEEPROMRange(): vector size does not match range size.\n";  // Program logic error
            retval = OTHER_ERROR;
        } else {
            for (size_t i = 0; i < vecSize; ++i) {
                int preverrcnt = errcnt;
                retval = writeEEPROMByte(static_cast<uint8_t>(begin + i), values[i], errcnt, errstr);
                if (errcnt != preverrcnt || retval != COMPLETED) {  // If an error occurs
                    break;  // Abort
                }
            }
        }
    }
    return retval;
}

// Writes the manufacturer descriptor to the MCP2210 OTP NVRAM
uint8_t MCP2210::writeManufacturerDesc(const std::u16string &manufacturer, int &errcnt, std::string &errstr)
{
    uint8_t retval;
    if (manufacturer.size() > DESC_MAXLEN) {
        ++errcnt;
        errstr += "In writeManufacturerDesc(): manufacturer descriptor string cannot be longer than 28 characters.\n";  // Program logic error
        retval = OTHER_ERROR;
    } else {
        retval = writeDescGeneric(manufacturer, MANUFACTURER_NAME, errcnt, errstr);
    }
    return retval;
}

// Writes the given chip transfer settings to the MCP2210 OTP NVRAM (this overloaded function is functionally equivalent to the implementation of writeNVChipSettings() that is found in versions 1.0.0 to 1.0.2)
// The use of this variant of writeNVChipSettings() sets the access control mode to "ACNONE" [0x00] and clears the password
uint8_t MCP2210::writeNVChipSettings(const ChipSettings &settings, int &errcnt, std::string &errstr)
{
    return writeNVChipSettings(settings, ACNONE, "", errcnt, errstr);
}

// Writes the given chip transfer settings to the MCP2210 OTP NVRAM, while also setting the access control mode and password (expanded in version 1.1.0)
// Note that it is possible to use an empty string for the password
uint8_t MCP2210::writeNVChipSettings(const ChipSettings &settings, uint8_t accessControlMode, const std::string &password, int &errcnt, std::string &errstr)
{
    uint8_t retval;
    if (accessControlMode != ACNONE && accessControlMode != ACPASSWORD && accessControlMode != ACLOCKED) {
        ++errcnt;
        errstr += "In writeNVChipSettings(): the specified access control mode is not supported.\n";  // Program logic error
        retval = OTHER_ERROR;
    } else if (password.size() > PASSWORD_MAXLEN) {
        ++errcnt;
        errstr += "In writeNVChipSettings(): password string cannot be longer than 8 characters.\n";  // Program logic error
        retval = OTHER_ERROR;
    } else {
        std::vector<uint8_t> command = {
            SET_NVRAM_SETTINGS, NV_CHIP_SETTINGS, 0x00, 0x00,                                                  // Header
            settings.gp0,                                                                                      // GP0 pin configuration
            settings.gp1,                                                                                      // GP1 pin configuration
            settings.gp2,                                                                                      // GP2 pin configuration
            settings.gp3,                                                                                      // GP3 pin configuration
            settings.gp4,                                                                                      // GP4 pin configuration
            settings.gp5,                                                                                      // GP5 pin configuration
            settings.gp6,                                                                                      // GP6 pin configuration
            settings.gp7,                                                                                      // GP7 pin configuration
            settings.gp8,                                                                                      // GP8 pin configuration
            settings.gpout, 0x00,                                                                              // Default GPIO outputs (GPIO7 to GPIO0)
            settings.gpdir, 0x01,                                                                              // Default GPIO directions (GPIO7 to GPIO0)
            static_cast<uint8_t>(settings.rmwakeup << 4 | (0x07 & settings.intmode) << 1 | settings.nrelspi),  // Other chip settings
            accessControlMode                                                                                  // Access control mode
        };
        for (size_t i = 0; i < password.size(); ++i) {
            command.push_back(static_cast<uint8_t>(password[i]));
        }
        std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
        retval = response[1];
    }
    return retval;
}

// Writes the given SPI transfer settings to the MCP2210 OTP NVRAM
uint8_t MCP2210::writeNVSPISettings(const SPISettings &settings, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        SET_NVRAM_SETTINGS, NV_SPI_SETTINGS, 0x00, 0x00,                                             // Header
        static_cast<uint8_t>(settings.bitrate), static_cast<uint8_t>(settings.bitrate >> 8),         // Bit rate
        static_cast<uint8_t>(settings.bitrate >> 16), static_cast<uint8_t>(settings.bitrate >> 24),
        settings.idlcs, 0x00,                                                                        // Idle chip select (CS7 to CS0)
        settings.actcs, 0x00,                                                                        // Active chip select (CS7 to CS0)
        static_cast<uint8_t>(settings.csdtdly), static_cast<uint8_t>(settings.csdtdly >> 8),         // Chip select to data delay
        static_cast<uint8_t>(settings.dtcsdly), static_cast<uint8_t>(settings.dtcsdly >> 8),         // Data to chip select delay
        static_cast<uint8_t>(settings.itbytdly), static_cast<uint8_t>(settings.itbytdly >> 8),       // Inter-byte delay
        static_cast<uint8_t>(settings.nbytes), static_cast<uint8_t>(settings.nbytes >> 8),           // Number of bytes per SPI transaction
        settings.mode                                                                                // SPI mode
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[1];
}

// Writes the product descriptor to the MCP2210 OTP NVRAM
uint8_t MCP2210::writeProductDesc(const std::u16string &product, int &errcnt, std::string &errstr)
{
    uint8_t retval;
    if (product.size() > DESC_MAXLEN) {
        ++errcnt;
        errstr += "In writeProductDesc(): product descriptor string cannot be longer than 28 characters.\n";  // Program logic error
        retval = OTHER_ERROR;
    } else {
        retval = writeDescGeneric(product, PRODUCT_NAME, errcnt, errstr);
    }
    return retval;
}

// Writes the USB parameters to the MCP2210 OTP NVRAM
uint8_t MCP2210::writeUSBParameters(const USBParameters &parameters, int &errcnt, std::string &errstr)
{
    std::vector<uint8_t> command = {
        SET_NVRAM_SETTINGS, USB_PARAMETERS, 0x00, 0x00,                                                       // Header
        static_cast<uint8_t>(parameters.vid), static_cast<uint8_t>(parameters.vid >> 8),                      // Vendor ID
        static_cast<uint8_t>(parameters.pid), static_cast<uint8_t>(parameters.pid >> 8),                      // Product ID
        static_cast<uint8_t>(parameters.powmode << 7 | !parameters.powmode << 6 | parameters.rmwakeup << 5),  // Chip power options
        parameters.maxpow                                                                                     // Maximum consumption current
    };
    std::vector<uint8_t> response = hidTransfer(command, errcnt, errstr);
    return response[1];
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
