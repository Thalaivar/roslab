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
To add the object to the world, the following lines are added to the gazebo world in [gazebo_ros_demos/rrbot_gazebo/worlds](https://github.com/ros-simulation/gazebo_ros_demos/tree/kinetic-devel/rrbot_gazebo/worlds)
```xml
  <!-- Spawn chosen object in world -->
  <include>
      <uri>model://checkerboard_plane</uri>
      <name>checkerboard_plane</name>
      <pose>-0.25 4.0 0.1 0 0 0</pose>
  </include>
```
This was adapted from the example in the [Gazebo tutorial on model population](http://gazebosim.org/tutorials?tut=model_population&cat=build_world). The checkerboard plane is placed slightly above the ground

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
## Adding Controller for New Joint
The configuration for the controllers is adapted from [gazebo_ros_demos/rrbot_control/config](https://github.com/ros-simulation/gazebo_ros_demos/tree/kinetic-devel/rrbot_control/config). The `rrbot_control.yaml` file specifies two "effort" controllers that take in a joint position (angle) as reference and output force/torque to the actuators. Because a new revolute joint and actuator is added to the robot, the following lines are added to this file to specify the new controller: 
```yaml
  joint0_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint0
    pid: {p: 100.0, i: 0.01, d: 10.0}
```
Note that the `joint` name should match the one specified in the modified robot. The `type` of controller used is the same one for the other joints, along with same PID gains.

## Procedure
> The following assumes that `ros-noetic-desktop-full` has been installed successfully
The following steps were followed to create the working repository
1. Create *catkin* workspace:
```bash
  mkdir -p ~/roslab/src
  cd roslab/
  catkin_make
```

2. Clone `gazebo_ros_demos/` in parent directory
```bash
  cd ~/ && git clone https://github.com/ros-simulation/gazebo_ros_demos.git
```

3. Create `rrbot_description/` package:
```bash
  cd ~/roslab/src
  catkin_create_pkg rrbot_description catkin joint_state_publisher robot_state_publisher

  # copy default data from gazebo_ros
  cp -r ~/gazebo_ros_demos/rrbot_description/meshes rrbot_description/
  cp -r ~/gazebo_ros_demos/rrbot_description/urdf rrbot_description/
```

4. Modify the `rrbot.xacro` and `rrbot.xml` files as mentioned previously.

5. Create `rrbot_control/` package:
```bash
  cd ~/roslab/src
  catkin_create_pkg
```
