# EKF SLAM for a Differential Drive Robot
* Nader Ahmed
* Winter 2024
# Package List
This repository consists of several ROS packages
- `nuturtle_description`: visualizes multiple turtlebots of different colors in rviz
- `turtlelib`: provides SE(2) transformation methods, SVG visualization, and EKF SLAM 
- `nusim`: initializes simulation environment with walls and obstacles
- `nuturtle_control`: controls the turtlebot and updates its odometry estimate (integration tests included)
- `nuslam`: applies the EKF SLAM to estimate the position of the robot and map the environment

# Movement Demo
To run the simulator and control with teleop, execute:
`ros2 launch nuturtle_control start_robot.launch.xml robot:=nusim cmd_src:=teleop use_rviz:=true`

To run with physical hardware, ssh into the turtlebot and run:
`ros2 launch nuturtle_control start_robot.launch.xml robot:=localhost cmd_src:=none use_rviz:=false`

On your own laptop, run:
`ros2 launch nuturtle_control start_robot.launch.xml robot:=none cmd_src:=none use_rviz:=true`

The odometry estimate after returning to the initial position was `x:.44507, y:0.0002199`. 

### Circle control
To move the robot in a circle, `cmd_src` can be changed to `circle` and the following service must be called with a velocity and radius:
`ros2 service call /control nuturtle_control/srv/Control {"velocity: 0.2, radius: 0.5"}`

<video src="https://github.com/ME495-Navigation/slam-project-nahder/assets/71537050/53fe6094-3c2d-410a-861c-9a2a37c1fa53"></video>

# SLAM Results (Known Landmark Association)
To run the SLAM launchfile and visualize all three robots in rviz:
`ros2 launch nuslam slam.launch.xml robot:=nusim cmd_src:=circle`

The following service call then moves the simulated turtlebot (red) move in a circle with a velocity of 0.2 m/s and radius of 1.0 m: 
`ros2 service call /control nuturtle_control/srv/Control {"velocity: 0.2, radius: 1.0"}`

The blue robot and its path represents its odometry estimate, and the green robot and its path represents the EKF SLAM estimate.
As seen, the odometry estimate is a perfect circle (as it is unaware of noise and wheel slippage in real world conditions). 
The estimated path from SLAM is seen by the green landmarks.
![slam](https://github.com/ME495-Navigation/slam-project-nahder/assets/71537050/5600f66c-c5b3-44ba-9b5d-5bbb8b55320a)


