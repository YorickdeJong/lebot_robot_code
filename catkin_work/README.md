In order to make this program work, the quirc and rplidar library have to be installed.

# Installing ROS
sudo bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add 
sudo apt update
sudo apt install ros-noetic-desktop-full -y
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Installing extra libs for quirc
sudo apt-get install libsdl2-dev -y
sudo apt-get install libsdl-gfx1.2-dev -y

# Intalling JavaScript Related libs: Node and npm
sudo apt-get install nodejs -y
sudo apt-get install npm -y

#Installing gnome terminal
sudo apt-get install gnome-terminal -y


------------------Installation quirc------------------

1. Go into the driver_bot_cpp directory (or from catkin_work cd src/driver_bot_cpp)
2. Download the repository: git clone https://github.com/dlbeer/quirc.git 
3. a quirc folder inside driver_bot_cpp is created, cd quirc from driver_bot_cpp
4. type as root: make install
5. The library is now available


------------------RPLidar Library------------------

1. Go into the src directory of catkin_work (or cd src from catkin_work)
2. Download the repository: git clone https://github.com/robopeak/rplidar_ros.git
3. cd .. (into catkin_work)
4. catkin_make 
5. The library is now build


------------------ npm packages ------------------ 

- To install the npm packages, from catkin_work type: 
  1. cd src/driver_bot_cpp/track/client
  2. npm install

------------------ Add wiringPi Library ---------------
1. git clone https://github.com/WiringPi/WiringPi.git
2. cd WirinPi && ./build

------------------ add user to gpio group ------------------
    1. sudo chmod 777 /dev/gpiomem 
    2. sudo chmod 777 /dev/mem 
------------------ Start Script ------------------

- sudo su root 
- source devel/setup.bash
- rosrun driver_bot_cpp generalInterface