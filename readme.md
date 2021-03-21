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

## Procedure
> The following assumes that `ros-noetic-desktop-full` has been installed successfully

1. Create *catkin* workspace:
 ```bash
 mkdir -p ~/roslab/src
 cd roslab/
 catkin_make
 ```
2. Create `rrbot_control` package:
 ```bash
 catkin_create_pkg rrbot_control catkin controller_manager joint_state_controller robot_state_publishe
 mkdir rrbot_control/config
 mkdir rrbot_control/launch
 ```
3. 
