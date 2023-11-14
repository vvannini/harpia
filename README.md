# This readme file is currently under construction

Please be aware that this readme file is a work in progress and may not be complete. We will be updating it regularly with new information, so please check back later for updates.

# Harpia - A Hybrid System for UAV Missions

## About the project

Harpia is a system for UAV mission and path planning. The project aims to provides a set of tools and libraries for generate UAV autonomous missions in a high-level for the user. 

## The Architecture

![Harpia Architecture.](https://ars.els-cdn.com/content/image/1-s2.0-S2772375523000217-gr003.jpg)

### User Input for Mission Execution

The first step is for the user to provide data for mission execution, including:

- The map on which the mission will take place, containing:
  - Bases for support
  - Areas to be monitored, with photo retrieval by the Unmanned Aerial Vehicle (UAV)
  - No-fly zone where the aircraft can't fly

- Mission details:
  - Information about the region and action to be performed
    
- Hardware Information:
  - including hardware information such as battery power, flight speed, and camera information.

All this information is parsed in the ROS node of the backend, which sends these data as ROS messages to other nodes, such as the Mission Goal Manager.

> All this information are structed as json files and can be found on the json folder. The backend code is in: /src/decision_support/mission_planning/scripts/client.py

### Mission Goal Manager

The Mission Goal Manager node is responsible for high-level mission command. It receives user changes for additions and removals of areas of interest and triggers a new planning process.

> The Mission Goal Manager code is in: /src/decision_support/mission_goal_manager/scripts/mission_goal_manager_server.py

### Mission Planner

The Mission Planner is activated by the Goal Manager and is responsible for invoking all stages of ROSPlan for the creation of an action plan. ROSPlan reads the problem information and domain code, generating a PDDL plan to meet the problem requirements.

> The Mission Planner code is in: /src/decision_support/mission_planning/scripts/mission_planner_server.py

### Mission Manager
The ROS Mission Manager node receives the actions to be executed one by one, containing the necessary code to execute specific system actions.

> The Mission Manager code is in: /src/rosplan_interface_harpia/src/RPHarpiaExecutor.cpp

Before continuing the plan, the Mission Manager sends the current action and the remaining actions of the plan to the Risk Mitigation node, which evaluates whether the current plan is still safe or needs replanning.

### Risk Mitigation

The Bayesian network starts with the next action, assessing the probability of its execution with the current battery level compared to the need for replanning. If the probability is higher or equal, the action is assumed to be completed, and the next action is evaluated. If the probability is lower, a mission failure flag is returned, signaling the need for a new plan by the Mission Planner and ROSPlan.

> The Risk Mitigation code is in: /src/decision_support/mission_fault_mitigation/scripts/mission_fault_mitigation_server.py


### Path Planning 

With the 'go_to' action, the system calls the Path Planning Server node, which receives information about the number of obstacles, current battery level, and straight-line distance between region centers. This information is processed by the KNN model, choosing the HGA as a planner for the route. The Mission Manager then sends the route and necessary commands for the aircraft to execute the action.

> The Path Planning Server code is in: /src/path_planning/scripts/path_planning_server.py


### Fault Detection System

Simultaneously, the Fault Detection System continually analyzes for any failure behavior. When a new value for one of the analyzed variables is received, it is transformed into two components by the algorithm and classified by the decision tree, returning Normal, Noise, Mild, or Anomalous.

These values are stored, and every 10 seconds, the system evaluates the percentage frequency of each category, assuming a classification for that window. Classifications range from Normal Pattern to Strong Anomalous Pattern, each associated with a numerical value (e.g., 0 for normal behavior).

These values are stored in a vector and analyzed over a 60-second interval, signaling the aircraft's behavior in the last minute. If there are many occurrences of erroneous behaviors in this 60-second window, the system decides whether to go to the nearest base or perform a vertical landing, considering the severity of the system's gravity.

> The Fault Detection code is in: src/fault_detection/anomaly_detection/scripts/anomaly_detector.py

## Instalation
<aside>
💡 Make sure that the system is updated →`sudo apt-get update`

</aside>

### System Versions

- Ubuntu: Ubuntu 16 LTS
- ROS → Melodic
- QGroundControl → v4.2.1

### Dependecies

- `sudo apt install curl libc6 libstdc++6 openjdk-11-jdk python3-prettytable python3-pip python3-lxml libxml2 libxslt1.1`
- `sudo apt-get install flex bison python3-opencv python3-matplotlib python3-catkin-tools python3-colcon-common-extensions libxml2 libxslt1-dev`
- `sudo -H pip3 install --upgrade pip`
- `pip install pyAgrum termcolor toml empy packaging jinja2 rospkg pandas pyproj shapely spicy scikit-learn psutil install future testresources kconfiglib jsonschema sympy==1.7.1 graphviz lxml  seaborn keras tensorflow pyspark plotly cloudpiclke jupyter jupyterlab pyros-genmsg`

### Ros Installation

- ROS Installation on Ubuntu → [official link](http://wiki.ros.org/melodic/Installation/Ubuntu)
- `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
- `sudo apt update`
- `sudo apt install ros-melodic-desktop-full`
- `echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`
- `source ~/.bashrc`
- `sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`
- sudo apt-get install python-jinja2
- sudo pip install numpy toml
- `sudo apt install python-rosdep`
- `sudo rosdep init`
- `rosdep update`

### MavROS Installation:

- [MAVROS documentation](http://wiki.ros.org/mavros)
- [MAVROS installation guide](https://docs.px4.io/main/en/ros/mavros_installation.html)
- `sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras`
- `wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts`
- `chmod +x install_geographiclib_datasets.sh`
- `sudo ./install_geographiclib_datasets.sh`
- `sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev`

### QGroundControl Installation:

- Download the app [here](https://github.com/mavlink/qgroundcontrol/releases/download/v4.1.6/QGroundControl.AppImage)
- `sudo usermod -a -G dialout $USER`
- `sudo apt-get remove modemmanager -y`
- `sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y`
- LogOut and Login again
- `wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage`
- `chmod +x ./QGroundControl.AppImage`
- Run `./QGroundControl.AppImage`

### PX4 Firmware:

- `git clone https://github.com/PX4/PX4-Autopilot`
- `cd PX4-Autopilot`
- `make`
- `bash ./Tools/setup/ubuntu.sh`


### Project setup

- `git clone [https://github.com/vvannini/harpia.git](https://github.com/vvannini/harpia.git).`

### Things you might need to do

- In the files `/opt/ros/noetic/share/mavros/launch/apm_config.yaml`, `/opt/ros/noetic/share/mavros/launch/px4_config.yaml` and `/opt/ros/noetic/share/mavros_extras/launch/px4flow_config.yaml` change the `timesync_rate` value to `0.0`.

### Build

- `cd harpia`
- `catkin build`

- go to `harpia/src/rosplan/rosplan_planning_system` and unzip common folder

## Running the system
### Starting the drone simulation
- `sudo su`
- `cd PX4-Autopilot`
- `export PX4_HOME_LAT=-22.001333; export PX4_HOME_LON=-47.934152; export PX4_HOME_ALT=847.142652; make px4_sitl gazebo`

### Starting harpia system
- In a new terminal:
- `source devel/setup.bash`
- `roslauch harpia.lauch`

### Starting a new mission
- In a new terminal:
- `source devel/setup.bash`
- `rosrun mission_planning teste_client.py <>`

## Simulation Video

- [Video link](https://ars.els-cdn.com/content/image/1-s2.0-S2772375523000217-mmc1.mp4)
  
## Articles 
- [Service-Oriented Architecture to Integrate Flight Safety and Mission Management Subsystems into UAVs, ICAS, BeloHorizonte, 2018](https://www.icas.org/ICAS_ARCHIVE/ICAS2018/data/papers/ICAS2018_0374_paper.pdf)
- [Harpia: A Hybrid System for Agricultural UAV Missions](https://doi.org/10.1016/j.atech.2023.100191)

## Implementation Schedule 

## References

### ROSPlan

ROSPlan (Robot Operating System Planning) is a planning framework for robotic systems that uses the Planning Domain Definition Language (PDDL) to describe the problem domain. It is an open-source project hosted on GitHub, and can be found at the following repository: https://github.com/KCL-Planning/ROSPlan

This repository is maintained by the Planning group at King's College London, and provides a set of tools and libraries for integrating planning capabilities into robotic systems using the ROS (Robot Operating System) framework, used in this project (Harpia). The ROSPlan project provides a PDDL compiler for generating plans, as well as a ROS-based implementation of the PDDL forward-planning algorithm. The repository also includes example problem domains and sample code for using the system with various robotic platforms.

ROSPlan is widely used in the field of automated planning for robotics, it has been used in multiple research projects and also in industry. It is actively developed and maintained by the research group at King's College London and has a growing community of contributors.

