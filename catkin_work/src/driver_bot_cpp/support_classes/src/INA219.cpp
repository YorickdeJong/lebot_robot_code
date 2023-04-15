#include "INA219.h"

#define INA219_READ 0x01
#define INA219_REG_CONFIG 0x00
#define INA219_REG_CURRENT 0x04
#define INA219_REG_POWER 0x03

INA219::INA219(uint8_t address) : m_address(address) {
}

INA219::~INA219() {
    if (m_i2c_handle >= 0) {
        i2cClose(m_i2c_handle);
    }
}

bool INA219::init() {
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio." << std::endl;
        return false;
    }

    m_i2c_handle = i2cOpen(1, m_address, 0);
    if (m_i2c_handle < 0) {
        std::cerr << "Failed to open I2C device with error: " << m_i2c_handle << std::endl;
        return false;
    }
    return true;
}

uint16_t INA219::readRegister(uint8_t reg) {
    int16_t value = i2cReadWordData(m_i2c_handle, reg);
    if (value < 0) {
        std::cerr << "Failed to read register: " << value << std::endl;
        return 0;
    }
    return (uint16_t)value;
}

void INA219::writeRegister(uint8_t reg, uint16_t value) {
    int status = i2cWriteWordData(m_i2c_handle, reg, value);
    if (status < 0) {
        std::cerr << "Failed to write register: " << status << std::endl;
    }
}



void INA219::setCalibration_16V_400mA() {
    ina219_calValue = 8192;
    ina219_currentDivider_mA = 20;
    ina219_powerDivider_mW = 1;
    writeRegister(INA219_REG_CONFIG, 0x3FFF);
}

float INA219::getCurrent_mA() {
    uint16_t value = readRegister(INA219_REG_CURRENT);
    float current_mA = value / static_cast<float>(ina219_currentDivider_mA);
    return current_mA;
}

float INA219::getPower_mW() {
    int16_t value = static_cast<int16_t>(readRegister(INA219_REG_POWER));
    float power_mW = value * ina219_powerDivider_mW;
    return power_mW;
}

// bool INA219::init() {
//     // N.B. program must be ran as root

//     if (!bcm2835_init()) {
//         std::cerr << "Failed to initialize bcm2835." << std::endl;
//         return false;
//     }

// 	if (bcm2835_i2c_begin() != 1) {
// 		fprintf(stderr, "bcm2835_i2c_begin() failed\n");
// 		return -3;
// 	}
    
//     bcm2835_i2c_setSlaveAddress(m_address);
//     bcm2835_i2c_set_baudrate(100000); // Set I2C baudrate to 100kHz
//     bcm2835_delay(500);
//     return true;
// }
// uint16_t INA219::readRegister(uint8_t reg) {
//     char buf[2] = {0};
//     int status = bcm2835_i2c_read_register_rs((char *)&reg, buf, 2);
//     if (status != BCM2835_I2C_REASON_OK) {
//         std::cerr << "Failed to read register: " << status << std::endl;
//         return 0;
//     }
//     uint16_t value = (buf[0] << 8) | buf[1];
//     return value;
// } 

// void INA219::writeRegister(uint8_t reg, uint16_t value) {
//     char buf[3] = {reg, static_cast<char>(value >> 8), static_cast<char>(value & 0xFF)};
//     bcm2835_i2c_write(buf, 3);
// }