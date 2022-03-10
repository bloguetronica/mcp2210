/* MCP2210 class - Version 0.13.0
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

    std::u16string getDescGeneric(uint8_t subcomid, int &errcnt, std::string &errstr);
    void interruptTransfer(uint8_t endpointAddr, unsigned char *data, int length, int *transferred, int &errcnt, std::string &errstr);
    uint8_t writeDescGeneric(const std::u16string &descriptor, uint8_t subcomid, int &errcnt, std::string &errstr);

public:
    // Class definitions
    static const uint16_t VID = 0x04D8;     // Default USB vendor ID
    static const uint16_t PID = 0x00DE;     // Default USB product ID
    static const int SUCCESS = 0;           // Returned by open() if successful
    static const int ERROR_INIT = 1;        // Returned by open() in case of a libusb initialization failure
    static const int ERROR_NOT_FOUND = 2;   // Returned by open() if the device was not found
    static const int ERROR_BUSY = 3;        // Returned by open() if the device is already in use
    static const size_t COMMAND_SIZE = 64;  // HID command size

    // Descriptor specific definitions
    static const size_t DESC_MAXLEN = 28;  // Maximum length for any descriptor

    // EEPROM specific definitions
    static const size_t EEPROM_SIZE = 256;     // EEPROM size in bytes
    static const uint8_t EEPROM_BEGIN = 0x00;  // EEPROM first address
    static const uint8_t EEPROM_END = 0xFF;    // EEPROM last address

    // HID command IDs
    static const uint8_t GET_CHIP_SETTINGS = 0x20;   // Get chip settings
    static const uint8_t SET_CHIP_SETTINGS = 0x21;   // Set chip settings
    static const uint8_t SET_SPI_SETTINGS = 0x40;    // Set SPI transfer settings
    static const uint8_t GET_SPI_SETTINGS = 0x41;    // Get SPI transfer settings
    static const uint8_t READ_EEPROM = 0x50;         // Read EEPROM
    static const uint8_t WRITE_EEPROM = 0x51;        // Write EEPROM
    static const uint8_t SET_NVRAM_SETTINGS = 0x60;  // Set NVRAM settings
    static const uint8_t GET_NVRAM_SETTINGS = 0x61;  // Get NVRAM settings

    // NVRAM settings sub-command IDs
    static const uint8_t NV_SPI_SETTINGS = 0x10;    // Power-up (non-volatile) SPI transfer settings
    static const uint8_t NV_CHIP_SETTINGS = 0x20;   // Power-up (non-volatile) chip settings
    static const uint8_t PRODUCT_NAME = 0x40;       // USB product name
    static const uint8_t MANUFACTURER_NAME = 0x50;  // USB manufacturer name

    // HID command responses
    static const uint8_t COMPLETED = 0x00;       // Command completed successfully
    static const uint8_t BUSY = 0xF7;            // SPI bus not available
    static const uint8_t IN_PROGRESS = 0xF8;     // USB or SPI transfer in progress (settings not written)
    static const uint8_t UNKNOWN = 0xF9;         // Response to unknown command
    static const uint8_t WRITE_FAILURE = 0xFA;   // EEPROM write failure
    static const uint8_t BLOCKED = 0xFB;         // Access not allowed or blocked, or EEPROM is password protected
    static const uint8_t REJECTED = 0xFC;        // Access rejected
    static const uint8_t WRONG_PASSWORD = 0xFD;  // Wrong password (number of attempts is still within the limit)
    static const uint8_t OTHER_ERROR = 0xFF;     // Other error (check errcnt and errstr for details)

    // The following values are applicable to SPISettings/configureChipSettings()/getChipSettings()
    static const uint8_t PCGPIO = 0x00;   // Pin configured as GPIO
    static const uint8_t PCCS = 0x01;     // Pin configured as chip select
    static const uint8_t PCFUNC = 0x02;   // Pin configured as a dedicated function pin
    static const uint8_t IMNOCNT = 0x00;  // Interrupt mode disabled (no interrupt counting)
    static const uint8_t IMCNTFE = 0x01;  // Interrupt mode set to count falling edges
    static const uint8_t IMCNTRE = 0x02;  // Interrupt mode set to count rising edges
    static const uint8_t IMCNTLP = 0x03;  // Interrupt mode set to count low pulses
    static const uint8_t IMCNTHP = 0x04;  // Interrupt mode set to count high pulses

    // The following values are applicable to SPISettings/configureSPISettings()/getSPISettings()
    static const uint32_t BRT1K464 = 1464;    // Value corresponding to a bit rate of 1.464Kbps
    static const uint32_t BRT1K5 = 1500;      // Value corresponding to a bit rate of 1.5Kbps
    static const uint32_t BRT1K875 = 1875;    // Value corresponding to a bit rate of 1.875Kbps
    static const uint32_t BRT2K5 = 2500;      // Value corresponding to a bit rate of 2.5Kbps
    static const uint32_t BRT3K = 3000;       // Value corresponding to a bit rate of 3Kbps
    static const uint32_t BRT3K125 = 3125;    // Value corresponding to a bit rate of 3.125Kbps
    static const uint32_t BRT3K75 = 3750;     // Value corresponding to a bit rate of 3.75Kbps
    static const uint32_t BRT5K = 5000;       // Value corresponding to a bit rate of 5Kbps
    static const uint32_t BRT6K = 6000;       // Value corresponding to a bit rate of 6Kbps
    static const uint32_t BRT6K25 = 6250;     // Value corresponding to a bit rate of 6.25Kbps
    static const uint32_t BRT7K5 = 7500;      // Value corresponding to a bit rate of 7.5Kbps
    static const uint32_t BRT9K375 = 9375;    // Value corresponding to a bit rate of 9.375Kbps
    static const uint32_t BRT10K = 10000;     // Value corresponding to a bit rate of 10Kbps
    static const uint32_t BRT12K = 12000;     // Value corresponding to a bit rate of 12Kbps
    static const uint32_t BRT12K5 = 12500;    // Value corresponding to a bit rate of 12.5Kbps
    static const uint32_t BRT15K = 15000;     // Value corresponding to a bit rate of 15Kbps
    static const uint32_t BRT15K625 = 15625;  // Value corresponding to a bit rate of 15.625Kbps
    static const uint32_t BRT18K75 = 18750;   // Value corresponding to a bit rate of 18.750Kbps
    static const uint32_t BRT20K = 20000;     // Value corresponding to a bit rate of 20Kbps
    static const uint32_t BRT24K = 24000;     // Value corresponding to a bit rate of 24Kbps
    static const uint32_t BRT25K = 25000;     // Value corresponding to a bit rate of 25Kbps
    static const uint32_t BRT30K = 30000;     // Value corresponding to a bit rate of 30Kbps
    static const uint32_t BRT31K25 = 31250;   // Value corresponding to a bit rate of 31.25Kbps
    static const uint32_t BRT37K5 = 37500;    // Value corresponding to a bit rate of 37.5Kbps
    static const uint32_t BRT40K = 40000;     // Value corresponding to a bit rate of 40Kbps
    static const uint32_t BRT46K875 = 46875;  // Value corresponding to a bit rate of 46.875Kbps
    static const uint32_t BRT48K = 48000;     // Value corresponding to a bit rate of 48Kbps
    static const uint32_t BRT50K = 50000;     // Value corresponding to a bit rate of 50Kbps
    static const uint32_t BRT60K = 60000;     // Value corresponding to a bit rate of 60Kbps
    static const uint32_t BRT62K5 = 62500;    // Value corresponding to a bit rate of 62.5Kbps
    static const uint32_t BRT75K = 75000;     // Value corresponding to a bit rate of 75Kbps
    static const uint32_t BRT80K = 80000;     // Value corresponding to a bit rate of 80Kbps
    static const uint32_t BRT93K75 = 93750;   // Value corresponding to a bit rate of 93.75Kbps
    static const uint32_t BRT100K = 100000;   // Value corresponding to a bit rate of 100Kbps
    static const uint32_t BRT120K = 120000;   // Value corresponding to a bit rate of 120Kbps
    static const uint32_t BRT125K = 125000;   // Value corresponding to a bit rate of 125Kbps
    static const uint32_t BRT150K = 150000;   // Value corresponding to a bit rate of 150Kbps
    static const uint32_t BRT187K5 = 187500;  // Value corresponding to a bit rate of 187.5Kbps
    static const uint32_t BRT200K = 200000;   // Value corresponding to a bit rate of 200Kbps
    static const uint32_t BRT240K = 240000;   // Value corresponding to a bit rate of 240Kbps
    static const uint32_t BRT250K = 250000;   // Value corresponding to a bit rate of 250Kbps
    static const uint32_t BRT300K = 300000;   // Value corresponding to a bit rate of 300Kbps
    static const uint32_t BRT375K = 375000;   // Value corresponding to a bit rate of 375Kbps
    static const uint32_t BRT400K = 400000;   // Value corresponding to a bit rate of 400Kbps
    static const uint32_t BRT500K = 500000;   // Value corresponding to a bit rate of 500Kbps
    static const uint32_t BRT600K = 600000;   // Value corresponding to a bit rate of 600Kbps
    static const uint32_t BRT750K = 750000;   // Value corresponding to a bit rate of 750Kbps
    static const uint32_t BRT1M = 1000000;    // Value corresponding to a bit rate of 1Mbps
    static const uint32_t BRT1M2 = 1200000;   // Value corresponding to a bit rate of 1.2Mbps
    static const uint32_t BRT1M5 = 1500000;   // Value corresponding to a bit rate of 1.5Mbps
    static const uint32_t BRT2M = 2000000;    // Value corresponding to a bit rate of 2Mbps
    static const uint32_t BRT3M = 3000000;    // Value corresponding to a bit rate of 3Mbps
    static const uint32_t BRT12M = 12000000;  // Value corresponding to a bit rate of 12Mbps
    static const uint8_t SPIMODE0 = 0x00;     // Value corresponding to SPI mode 0
    static const uint8_t SPIMODE1 = 0x01;     // Value corresponding to SPI mode 1
    static const uint8_t SPIMODE2 = 0x02;     // Value corresponding to SPI mode 2
    static const uint8_t SPIMODE3 = 0x03;     // Value corresponding to SPI mode 3

    struct ChipSettings {
        uint8_t gp0;      // GP0 pin configuration
        uint8_t gp1;      // GP1 pin configuration
        uint8_t gp2;      // GP2 pin configuration
        uint8_t gp3;      // GP3 pin configuration
        uint8_t gp4;      // GP4 pin configuration
        uint8_t gp5;      // GP5 pin configuration
        uint8_t gp6;      // GP6 pin configuration
        uint8_t gp7;      // GP7 pin configuration
        uint8_t gp8;      // GP8 pin configuration
        uint8_t gpdir;    // Default GPIO direction (CS7 to CS0)
        uint8_t gpout;    // Default GPIO output (CS7 to CS0)
        bool rmwakeup;    // Remote wake-up
        uint8_t intmode;  // Interrupt counting mode
        bool nrelspi;     // SPI bus release (negated)

        bool operator ==(const ChipSettings &other) const;
        bool operator !=(const ChipSettings &other) const;
    };

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
    uint8_t configureChipSettings(const ChipSettings &settings, int &errcnt, std::string &errstr);
    uint8_t configureSPISettings(const SPISettings &settings, int &errcnt, std::string &errstr);
    ChipSettings getChipSettings(int &errcnt, std::string &errstr);
    std::u16string getManufacturerDesc(int &errcnt, std::string &errstr);
    ChipSettings getNVChipSettings(int &errcnt, std::string &errstr);
    SPISettings getNVSPISettings(int &errcnt, std::string &errstr);
    std::u16string getProductDesc(int &errcnt, std::string &errstr);
    SPISettings getSPISettings(int &errcnt, std::string &errstr);
    std::vector<uint8_t> hidTransfer(const std::vector<uint8_t> &data, int &errcnt, std::string &errstr);
    int open(uint16_t vid, uint16_t pid, const std::string &serial = std::string());
    uint8_t readEEPROMByte(uint8_t address, int &errcnt, std::string &errstr);
    std::vector<uint8_t> readEEPROMRange(uint8_t begin, uint8_t end, int &errcnt, std::string &errstr);
    uint8_t writeEEPROMByte(uint8_t address, uint8_t value, int &errcnt, std::string &errstr);
    uint8_t writeEEPROMRange(uint8_t begin, uint8_t end, const std::vector<uint8_t> &values, int &errcnt, std::string &errstr);
    uint8_t writeNVChipSettings(const ChipSettings &settings, int &errcnt, std::string &errstr);
    uint8_t writeNVSPISettings(const SPISettings &settings, int &errcnt, std::string &errstr);
    uint8_t writeManufacturerDesc(const std::u16string &manufacturer, int &errcnt, std::string &errstr);
    uint8_t writeProductDesc(const std::u16string &product, int &errcnt, std::string &errstr);

    static std::vector<std::string> listDevices(uint16_t vid, uint16_t pid, int &errcnt, std::string &errstr);
};

#endif  // MCP2210_H
