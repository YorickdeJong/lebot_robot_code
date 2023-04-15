project(driver_bot_cpp)

include_directories(include)

find_library(BCM2835_LIBRARY bcm2835 REQUIRED)
find_library(WIRINGPI_LIB wiringPi)
find_library(PIGPIO_LIBRARY pigpio)


add_library(angledistancelib src/angle_operations.cpp src/distance_operations.cpp src/camera_operations)
add_library(cameralib src/camera_operations/draw_shapes.cpp src/camera_operations/frame_analysis.cpp src/camera_operations/overlap.cpp src/camera_operations/qr_code_detection.cpp)
add_library(powerlib src/ADS1115.cpp src/INA219.cpp)

target_link_libraries(cameralib quirc)
target_link_libraries(powerlib wiringPi pigpio bcm2835)
