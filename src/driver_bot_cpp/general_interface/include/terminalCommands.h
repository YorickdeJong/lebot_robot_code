#define CALIBRATE "\
#!/bin/bash \n\
gnome-terminal & \n\
roslaunch rplidar_ros rplidar.launch & \n\
sleep 5 \n\
gnome-terminal & \n\
roslaunch driver_bot_cpp calibrate.launch \n\
"

#define CAMLIDAR "\
#!/bin/bash \n\
roslaunch rplidar_ros rplidar.launch & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp camera_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp detection_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp rplidar_node \n\
"

#define CAMDETECTIONLIDAR "\
#!/bin/bash \n\
roslaunch rplidar_ros rplidar.launch & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp camera_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp detection_node & \n\
sleep 5 \n\
gnome-terminal & \n\
cd src/\n\
rosrun driver_bot_cpp rplidar_node \n\
"


#define CAMADJUSTEDLIDAR "\
#!/bin/bash \n\
roslaunch rplidar_ros rplidar.launch & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp camera_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp rplidar_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp angle_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp distance_node & \n\
"

#define CAMDETECTIONADJUSTEDLIDAR "\
#!/bin/bash \n\
roslaunch rplidar_ros rplidar.launch & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp camera_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp detection_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp rplidar_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp angle_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp distance_node & \n\
"

#define CAMQRCODELIDAR "\
#!/bin/bash \n\
roslaunch rplidar_ros rplidar.launch & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp camera_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp qr_code_detection_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp qrRoundPublisher & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp rplidar_node \n\
"

#define RACE "\
#!/bin/bash \n\
roslaunch rplidar_ros rplidar.launch & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp camera_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp qr_code_detection_node & \n\
sleep 5 \n\
gnome-terminal & \n\
rosrun driver_bot_cpp qrRoundPublisher & \n\
sleep 5 \n\
gnome-terminal & \n\
cd src/driver_bot_cpp/track/client \n\
npm run watch\n\
"

#define RACETEST "\
#!/bin/bash \n\
roslaunch driver_bot_cpp camera.launch & \n\
sleep 5 \n\
gnome-terminal & \n\
cd src/driver_bot_cpp/track/client \n\
npm run watch\n\
"


#define DRIVER "\
#!/bin/bash \n\
roslaunch rplidar_ros rplidar.launch & \n\
sleep 5 \n\
gnome-terminal & \n\
roslaunch driver_bot_cpp drivers.launch \n\
"

#define TEST "\
#!/bin/bash \n\
roslaunch rplidar_ros rplidar.launch & \n\
sleep 5 \n\
gnome-terminal & \n\
cd src/driver_bot_cpp/track/client \n\
npm run watch\n\
"