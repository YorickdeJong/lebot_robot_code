#ifndef ADS1115_H
#define ADS1115_H

#include <cstdint>
// #include <wiringPi.h>
// #include <wiringPiI2C.h>
#include <bcm2835.h>
#include <iostream>

#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG 0x01

#define GAIN_TWOTHIRDS 0x0000
#define MODE_SINGLESHOT 0x0100
#define DATARATE_860SPS 0x0080
#define ADS1115_MAX_VALUE 32767

class ADS1115 {
public:
    ADS1115(uint8_t address = 0x48);
    ~ADS1115();
    bool init();
    void setGain(uint16_t gain);
    void setMode(uint16_t mode);
    void setDataRate(uint16_t data_rate);
    int16_t readADC_SingleEnded(uint8_t channel);

private:
    uint8_t m_address;
    uint16_t m_config;
    int m_fd;
    void writeConfig();
};

#endif // ADS1115_H