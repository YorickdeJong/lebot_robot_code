cmake_minimum_required(VERSION 3.0.2)
project(driver_bot_cpp)

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  OpenCV
  sensor_msgs
  cv_bridge
  image_transport 
)

find_library(WIRINGPI_LIB wiringPi)

add_message_files(
  FILES
  distanceVelocity.msg
  lidar.msg
  objectAngle.msg
  qrCode.msg
  images.msg
  distanceData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${WIRINGPI_INCLUDE_DIRS}
)

add_subdirectory(actions)
add_subdirectory(motorDriver)
add_subdirectory(motorImplementation)
add_subdirectory(support_classes)

include_directories(driving_on_command/include)
include_directories(nodes/include)
include_directories(general_interface/include)
include_directories(support_classes/include)

add_executable(camera_node nodes/src/Sensors/camera_node.cpp )
target_link_libraries(camera_node ${OpenCV_LIBS} ${catkin_LIBRARIES}) 
add_dependencies(camera_node driver_bot_cpp_generate_messages_cpp)


add_executable(cameraSub_node nodes/src/Sensors/camera_Subscriber.cpp )
target_link_libraries(cameraSub_node ${OpenCV_LIBS} ${catkin_LIBRARIES}) 
add_dependencies(cameraSub_node driver_bot_cpp_generate_messages_cpp)


add_executable(calibrate nodes/src/Calibrate/calibrate.cpp)
target_link_libraries(calibrate ${catkin_LIBRARIES} angledistancelib actionslib) 
add_dependencies(calibrate driver_bot_cpp_generate_messages_cpp)


add_executable(rplidar_node nodes/src/Sensors/rplidar_node.cpp )
target_link_libraries(rplidar_node ${catkin_LIBRARIES})
add_dependencies(rplidar_node driver_bot_cpp_generate_messages_cpp)


add_executable(angle_node nodes/src/Perceptions/angle_node.cpp )
target_link_libraries(angle_node ${catkin_LIBRARIES} angledistancelib) 
add_dependencies(angle_node driver_bot_cpp_generate_messages_cpp)

add_executable(distance_node nodes/src/Perceptions/distance_node.cpp )
target_link_libraries(distance_node ${catkin_LIBRARIES} angledistancelib) 
add_dependencies(distance_node driver_bot_cpp_generate_messages_cpp)


add_executable(detection_node nodes/src/Sensors/object_detection_node.cpp )
target_link_libraries(detection_node ${OpenCV_LIBS} ${catkin_LIBRARIES} cameralib) 
add_dependencies(detection_node driver_bot_cpp_generate_messages_cpp)


add_executable(qr_code_detection_node nodes/src/Sensors/qr_code_detection_node.cpp )
target_link_libraries(qr_code_detection_node ${OpenCV_LIBS} ${catkin_LIBRARIES} cameralib)
add_dependencies(qr_code_detection_node driver_bot_cpp_generate_messages_cpp)


add_executable(qrSub nodes/src/Sensors/qr_code_detection_subscriber.cpp)
target_link_libraries(qrSub ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(qrSub driver_bot_cpp_generate_messages_cpp)


add_executable(send_round track/client/src/data/send_round.cpp)
target_link_libraries(send_round ${catkin_LIBRARIES} actionslib)
add_dependencies(send_round driver_bot_cpp_generate_messages_cpp)


add_executable(send_powerUp track/client/src/data/send_powerUp.cpp)
target_link_libraries(send_powerUp ${catkin_LIBRARIES} actionslib)
add_dependencies(send_powerUp driver_bot_cpp_generate_messages_cpp)

add_executable(run_node nodes/src/run.cpp )
target_link_libraries(run_node ${catkin_LIBRARIES} actionslib) #hier zou dan iets van actionslib etc. moeten komen
add_dependencies(run_node driver_bot_cpp_generate_messages_cpp)


add_executable(generalInterface general_interface/src/generalInterface.cpp)
target_link_libraries(generalInterface ${catkin_LIBRARIES}) #hier zou dan iets van actionslib etc. moeten komen
add_dependencies(generalInterface driver_bot_cpp_generate_messages_cpp)


####------- key publishers -------#####
add_executable(keys_move driving_on_command/src/keys_move.cpp )
target_link_libraries(keys_move ${catkin_LIBRARIES} actionslib) 
add_dependencies(keys_move driver_bot_cpp_generate_messages_cpp)

add_executable(keys_publisher driving_on_command/src/keys_publisher.cpp )
target_link_libraries(keys_publisher ${catkin_LIBRARIES}) 
add_dependencies(keys_publisher driver_bot_cpp_generate_messages_cpp)

##-------------- Publish data Plots -------------------##
add_executable(plot_data_transformation_node nodes/src/plots/plot_data_transformation_node.cpp )
target_link_libraries(plot_data_transformation_node ${catkin_LIBRARIES} angledistancelib) 
add_dependencies(plot_data_transformation_node driver_bot_cpp_generate_messages_cpp)

##-------------- Encoder data -------------------##
add_executable(encoder_node nodes/src/plots/encoder_node.cpp )
target_link_libraries(encoder_node ${catkin_LIBRARIES} ${WIRINGPI_LIB})
add_dependencies(encoder_node driver_bot_cpp_generate_messages_cpp)