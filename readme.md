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

## Add Object to the World
To add the object to the world, the gazebo world in [gazebo_ros_demos/rrbot_gazebo/worlds](https://github.com/ros-simulation/gazebo_ros_demos/tree/kinetic-devel/rrbot_gazebo/worlds) is launched using 
```bash
roslaunch rrbot_gazebo rrbot_world.launch
```

## Modifications to the Robot
The robot definition is created by modifying the package given in [gazebo_ros_demos/rrbot_description/urdf](https://github.com/ros-simulation/gazebo_ros_demos/tree/kinetic-devel/rrbot_description/urdf). More specifically:
1. In `rrbot.xacro`, to add a revolute joint to the base the following changes are made
```xml
<!-- <joint name="fixed" type="fixed">
    <parent link="world"/>
    <child link="link1"/>
  </joint> -->
  
  <joint name="joint0" type="continuous">
    <parent link="world"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.7"/>
  </joint>
``` 
This removes the original fixed joint and adds a revolute joint with axis aligned along z-axis at the origin, which connects `link1` to the world.

2. To actuate the joint added previously, the following block of code is added in `rrbot.xacro`
```xml
<transmission name="tran0">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint0">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor0">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
```
The specification of the actuator for the joint is same as that of the other joints.

3. Similar changes have to be made in `robot.xml` to add the base joint
```xml
<!-- <joint name="fixed" type="fixed">
    <parent link="world"/>
    <child link="link1"/>
  </joint> -->
  <joint name="joint0" type="continous">
    <parent link="world"/>
    <child link="link1"/>
    <origin rpy="0 0 0" xyz="0 0.1 1.95"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.7"/>
  </joint>
```

and to add the actuator for the new joint
```xml
<transmission name="tran0">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint0"/>
    <actuator name="motor0">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
```

## Procedure
> The following assumes that `ros-noetic-desktop-full` has been installed successfully

1. Create *catkin* workspace:
 ```bash
 mkdir -p ~/roslab/src
 cd roslab/
 catkin_make
 ```
