project(driver_bot_cpp)

include_directories(include)

add_library(angledistancelib src/angle_operations.cpp src/distance_operations.cpp src/camera_operations)
add_library(cameralib src/camera_operations/draw_shapes.cpp src/camera_operations/frame_analysis.cpp src/camera_operations/overlap.cpp src/camera_operations/qr_code_detection.cpp)

target_link_libraries(cameralib quirc)