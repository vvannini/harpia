### Use of Artificial Neural Network for anomaly detection over a timeseries database

---

Data Acquisition: using PX4 Firmware integrated to Gazebo + QGroundControl GUI:

![image](https://user-images.githubusercontent.com/60454486/190869658-0f86a52b-d46d-407b-a1c0-7789630531a1.png)

---

# Dependencies:

## 1. ROS Installation 🐢

- Install on Ubuntu: http://wiki.ros.org/noetic/Installation/Ubuntu
- `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
- `sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`
- `sudo apt update`
- `sudo apt install ros-noetic-desktop-full`
- `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`
- `source ~/.bashrc`
- `sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`
- `sudo rosdep init`
- `rosdep update`

## 2. External Dependencies 📑
- `sudo apt-get install flex bison python3-opencv python3-matplotlib python3-catkin-tools python3-colcon-common-extensions`
- `sudo apt install python3-prettytable python3-pip`
- `sudo -H pip3 install --upgrade pip`
- `pip3 install toml empy packaging jinja2 rospkg pandas pyproj shapely spicy scikit-learn psutil`

## 3. MavROS Installation 📦
- `sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras`
- `wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh`
- `chmod +x install_geographiclib_datasets.sh`
- `sudo ./install_geographiclib_datasets.sh`

## 4. QGroundControl Installation 🕹
- Download the app: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html
- `sudo usermod -a -G dialout $USER`
- `sudo apt-get remove modemmanager -y`
- `sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y`
- LogOut and Login again
- `wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage`
- `chmod +x ./QGroundControl.AppImage`
- Run `./QGroundControl.AppImage`

## 5. PX4 Firmware 🚁
- `git clone http://github.com/PX4/Firmware`
- `cd Firmware`
- `make`
- `bash ./Tools/setup/ubuntu.sh`
- `sudo make px4_sitl gazebo`
