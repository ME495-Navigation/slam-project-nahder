# ME495 Sensing, Navigation and Machine Learning For Robotics
* Nader Ahmed
* Winter 2024
# Package List
This repository consists of several ROS packages
- `nuturtle_description`: visualizes multiple turtlebots of different colors in rviz
- `turtlelib`: provides SE(2) transformation methods and SVG visualization
- `nusim`: initializes simulation environment with walls and obstacles


# Demo

Running simulator:
`ros2 launch nuturtle_control start_robot.launch.xml robot:=nusim cmd_src:=teleop use_rviz:=true`

Running on turtlebot:
`ros2 launch nuturtle_control start_robot.launch.xml robot:=localhost cmd_src:=none use_rviz:=false`
`ros2 launch nuturtle_control start_robot.launch.xml robot:=none cmd_src:=none use_rviz:=true`


<video src="https://github.com/ME495-Navigation/slam-project-nahder/assets/71537050/53fe6094-3c2d-410a-861c-9a2a37c1fa53"></video>

