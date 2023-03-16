project(driver_bot_cpp)

include_directories(include)


add_library(motordriverlib src/adafruitdcmotor.cpp src/adafruitmotorhat.cpp src/i2cdevice.cpp src/pwm.cpp)
