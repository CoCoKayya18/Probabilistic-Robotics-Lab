# Probabilistic-Robotics-Lab

## Overview
The Probabilistic Robotics Lab is an implementation of a Extended Kalman Filter for robot localizatoin, utilizing odometry and lidar data. 

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)

## Installation
To set up the project, clone the repository and install the necessary dependencies:

```bash
git clone https://github.com/CoCoKayya18/Probabilistic-Robotics-Lab.git
cd Probabilistic-Robotics-Lab
catkin_make
```
Also make sure to have the PCL library installed

## Usage
To run the package, direct to the launch directory and run the launch file:

```bash
cd src/kayyalakkakam/src/launch
roslaunch run_turtlebot3.launch
```

## Project Structure
Each directory and file serves a specific purpose in the project:

- `include/kayyalakkakam/`: Contains the core implementation code for the EKF, goal sending node, and publisher/subscriber nodes.
- `src/code/`: Contains the main executable file for the project.
- `src/features/`: Includes a CSV file with extracted features from the map.
- `src/launch/`: Holds the launch file to initiate the program.
- `src/map/`: Contains the environment map and processed images from feature extraction.
- `src/rviz/`: Stores the RViz configuration file.
- `src/scripts/`: Contains various scripts for Harris corner detection, plot generation, and EKF evaluation. Also includes directories for plots generated after each run.
- `src/urdf/`: Holds the URDF file of the robot.
- `CMakeLists.txt`: Script for building the project using CMake.
- `.gitignore`: Specifies files and directories to be ignored by Git.

