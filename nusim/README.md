# NUSim Library
This library initializes a simulated environment for a Turtlebot3 robot. It uses rviz markers to create walls and cylindrical obstacles. The simulation runs at a fixed rate, broadcasting the the relative transformation between the robot and the world frame.

# Services
- `teleport`: Moves the robot to a given (x,y,θ) pose 
- `reset`: Restores the initial simulation state

# Launching
- To get started, run the command: `ros2 launch nusim nusim.launch.xml`. This:
  - opens `rviz2` with the appropriate configuration file `red_config.rviz`
  - initializes the world environment with properties defined in `basic_world.yaml`
  - sets the robot prpoerties defined in `diff_params.yaml`
  - runs the `nusim` node 

# Parameter Description

| Parameter         | Type         | Description                         | Units  |
|-------------------|--------------|-------------------------------------|--------|
| `rate`            | double       | Simulation frequency                | Hz     |
| `x0`              | double       | Initial x-coordinate                | m      |
| `y0`              | double       | Initial y-coordinate                | m      |
| `theta0`          | double       | Initial heading                     | rad    |
| `arena_x_length`  | double       | Length of the walls in x-direction  | m      |
| `arena_y_length`  | double       | Length of the walls in y-direction  | m      |
| `obstacles.r`     | double       | Cylindrical obstacle radius         | m      |
| `obstacles.x`     | std::vector\<double> | Obstacle x-coordinates              | m      |
| `obstacles.y`     | std::vector\<double> | Obstacle y-coordinates              | m      |

# Environment
![nusim1](https://github.com/ME495-Navigation/slam-project-nahder/assets/71537050/f6ab4d83-4758-4d8e-964a-48f95da48903)

# Citations

The following threads were referenced in generating the targets for the custom Teleport service:
- [Robotics Stack Exchange](https://robotics.stackexchange.com/questions/23171/creating-)
- [ROS Answers](https://answers.ros.org/question/318938/rclcppexceptionsrclerror/)




