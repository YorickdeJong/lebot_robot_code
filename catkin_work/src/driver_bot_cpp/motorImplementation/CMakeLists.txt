project(driver_bot_cpp)

include_directories(include)

add_library(motorlib src/motor2Wheels.cpp src/motor4Wheels.cpp src/motorFactory.cpp)
target_link_libraries(motorlib motordriverlib)