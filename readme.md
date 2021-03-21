# ID6100: Robotics Laboratory

## Outline of Repository

```
.
├── src
    ├── rrbot_control
        ├── config      # declare and define all controllers
        ├── launch      # spawn all controllers
    ├── rrbot_description
        ├── meshes      
        ├── urdf        # definition files for the RR robot
    ├── rrbot_explore
        ├── launch      # launch gazebo world with robot
        ├── worlds      # gazebo world with selected object
        
```

## Modifications to the Robot
The robot definition is created by modifying the package given in [gazebo_ros_demos/rrbot_description](https://github.com/ros-simulation/gazebo_ros_demos/tree/kinetic-devel/rrbot_description). More specifically:
1. In 

## Procedure
> The following assumes that `ros-noetic-desktop-full` has been installed successfully

1. Create *catkin* workspace:
 ```bash
 mkdir -p ~/roslab/src
 cd roslab/
 catkin_make
 ```
