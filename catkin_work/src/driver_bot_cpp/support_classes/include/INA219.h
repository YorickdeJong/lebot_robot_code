#include <cstdint>
// #include <wiringPi.h>
// #include <bcm2835.h>
#include <pigpio.h>
// #include <wiringPiI2C.h>
#include <iostream>
#include <sys/types.h>

class INA219 {
public:
    INA219(uint8_t address);
    ~INA219();
    bool init();
    void setCalibration_16V_400mA();
    float getCurrent_mA();
    float getPower_mW();

private:
    uint8_t m_address;
    int m_i2c_handle;
    uint32_t ina219_calValue;
    uint32_t ina219_currentDivider_mA;
    uint32_t ina219_powerDivider_mW;

    uint16_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint16_t value);
};
