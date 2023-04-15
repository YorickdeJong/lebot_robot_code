#include "ADS1115.h"

ADS1115::ADS1115(uint8_t address) : m_address(address) {}

ADS1115::~ADS1115() {
    // No need to call any function since wiringPi does not require explicit cleanup for I2C
}

// bool ADS1115::init() {
//     if (wiringPiSetup() == -1) {
//         std::cerr << "Failed to initialize wiringPi." << std::endl;
//         return false;
//     }

//     m_fd = wiringPiI2CSetup(m_address);
//     if (m_fd == -1) {
//         std::cerr << "Failed to initialize I2C." << std::endl;
//         return false;
//     }

//     uint8_t config[3] = {ADS1115_REG_CONFIG, 0b11000010, 0b10000111};
//     wiringPiI2CWriteReg8(m_fd, ADS1115_REG_CONFIG, (config[1] << 8) | config[2]);

//     delay(1000);

//     return true;
// }

bool ADS1115::init() {
    if (!bcm2835_init()) {
        std::cerr << "Failed to initialize bcm2835." << std::endl;
        return false;
    }

    if (!bcm2835_i2c_begin()) {
        std::cerr << "Failed to initialize I2C." << std::endl;
        return false;
    }

    bcm2835_i2c_setSlaveAddress(m_address);
    bcm2835_i2c_set_baudrate(100000); // Set I2C baudrate to 100kHz

    uint8_t config[3] = {ADS1115_REG_CONFIG, 0b11000010, 0b10000111};
    bcm2835_i2c_write((const char *)config, 3);

    bcm2835_delay(1000);

    return true;
}


int16_t ADS1115::readADC_SingleEnded(uint8_t channel) {
    m_config = (m_config & ~0x7000) | ((channel & 0x07) << 12);
    writeConfig();

    bcm2835_delay(8);

    char buf[2] = {0};
    char reg_conversion = ADS1115_REG_CONVERSION;
    bcm2835_i2c_read_register_rs(&reg_conversion, buf, 2);
    int16_t raw_adc = (buf[0] << 8) | buf[1];

    return raw_adc;
}

void ADS1115::writeConfig() {
    char config[3] = {ADS1115_REG_CONFIG, static_cast<char>(m_config >> 8), static_cast<char>(m_config & 0xFF)};
    bcm2835_i2c_write(config, 3);
}


void ADS1115::setGain(uint16_t gain) {
    m_config = (m_config & ~0x0E00) | (gain & 0x0E00);
    writeConfig();
}

void ADS1115::setMode(uint16_t mode) {
    m_config = (m_config & ~0x0100) | (mode & 0x0100);
    writeConfig();
}

void ADS1115::setDataRate(uint16_t data_rate) {
    m_config = (m_config & ~0x00E0) | (data_rate & 0x00E0);
    writeConfig();
}

// int16_t ADS1115::readADC_SingleEnded(uint8_t channel) {
//     m_config = (m_config & ~0x7000) | ((channel & 0x07) << 12);
//     writeConfig();

//     delay(8);

//     int16_t raw_adc = wiringPiI2CReadReg16(m_fd, ADS1115_REG_CONVERSION);
//     // Swap bytes since wiringPiI2CReadReg16 returns the data in little-endian format
//     raw_adc = ((raw_adc & 0xFF) << 8) | ((raw_adc >> 8) & 0xFF);

//     return raw_adc;
// }


// void ADS1115::writeConfig() {
//     uint8_t config[3] = {ADS1115_REG_CONFIG, static_cast<uint8_t>(m_config >> 8), static_cast<uint8_t>(m_config & 0xFF)};
//     wiringPiI2CWriteReg8(m_fd, ADS1115_REG_CONFIG, (config[1] << 8) | config[2]);
// }
