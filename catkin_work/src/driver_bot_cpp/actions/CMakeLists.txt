project(driver_bot_cpp)

include_directories(include)

#add_library(motorlib ../motorImplementation/src/motor2Wheels.cpp ../motorImplementation/src/motor4Wheels.cpp ../motorImplementation/src/motorFactory.cpp)
add_library(actionslib src/actions2Wheels.cpp src/actions4Wheels.cpp src/actionsFactory.cpp)

target_link_libraries(actionslib motorlib)