/**
 *  pwm.cpp
 *
 *  MIT License
 *
 *  Copyright (c) 2018, Tom Clarke
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include "pwm.h"
#include "util.h"

#include <thread>
#include <chrono>

PWM::PWM (int deviceAddress)
    : device (deviceAddress)
{
    if (device.isValid())
    {
        log::output ("Resetting PCA9685 mode 1 (without sleep) and mode 2");

        setAll (0, 0);
        device.write8 (Registers::kMode2, Bits::kOutDrive);
        device.write8 (Registers::kMode1, Bits::kAllCall);

        // wait for oscillator
        std::this_thread::sleep_for (std::chrono::milliseconds (5));

        int mode = device.read8 (Registers::kMode1);
        // reset sleep
        mode = mode & ~Bits::kSleep;
        device.write8 (Registers::kMode1, mode);

        // wait for oscillator
        std::this_thread::sleep_for (std::chrono::milliseconds (5));
    }
}

void PWM::setFrequency (double frequency)
{
    if (device.isValid())
    {
        // 25Mhz
        double preScaleValue = 25000000.0;
        // to 12-bit
        preScaleValue /= 4096.0;
        preScaleValue /= frequency;
        preScaleValue -= 1.0;

        log::output ("Setting PWM frequency to " + std::to_string (frequency) + "Hz");
        log::output ("Estimated pre-scale: " + std::to_string (preScaleValue));

        const int finalPreScale = static_cast<int> (preScaleValue + 0.5);

        log::output ("Final pre-scale: " + std::to_string (finalPreScale));

        const int oldMode = device.read8 (Registers::kMode1);
        const int newMode = (oldMode & 0x7F) | Bits::kSleep;

        // go to sleep
        device.write8 (Registers::kMode1, newMode);
        // set prescale
        device.write8 (Registers::kPreScale, finalPreScale);
        // wake up
        device.write8 (Registers::kMode1, oldMode);

        std::this_thread::sleep_for (std::chrono::milliseconds (5));

        // restart
        device.write8 (Registers::kMode1, oldMode | Bits::kRestart);
    }
}

void PWM::setChannel (int channel, int on, int off)
{
    if (device.isValid())
    {
        device.write8 (Registers::kLed0OnL + 4 * channel, on & 0xFF);
        device.write8 (Registers::kLed0OnH + 4 * channel, on >> 8);
        device.write8 (Registers::kLed0OffL + 4 * channel, off & 0xFF);
        device.write8 (Registers::kLed0OffH + 4 * channel, off >> 8);
    }
    else
    {
        std::cout << "device is NOT valid" << std::endl;
        std::cout << "Have you tried chmod a+rw /dev/i2c-* in driver_bot_cpp/motor.cpp/ driverbot_pkg/motor.py?" << std::endl;
    }
}

void PWM::setAll (int on, int off)
{
    if (device.isValid())
    {
        device.write8 (Registers::kAllLedOnL, on & 0xFF);
        device.write8 (Registers::kAllLedOnH, on >> 8);
        device.write8 (Registers::kAllLedOffL, off & 0xFF);
        device.write8 (Registers::kAllLedOffH, off >> 8);
    }
}
