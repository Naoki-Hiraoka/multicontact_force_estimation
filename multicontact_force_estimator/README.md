## install

```
mkdir -p catkin_ws/src
cd catkin_ws/src
wstool init .
wstool merge [このディレクトリの.rosintall]
cd ..
rosdep install -r --from-paths src --ignore-src -y
catkin build muticontact_force_estimator
```

## multicontact_force_estimator_node
### Subscribing Topics
* `joint_states` (`sensor_msgs/JointState`)
* `imu` (`sensor_msgs/Imu`)
* `[name of force sensors in robotmodel]` (`geometry_msgs/WrenchStamped`)
* `~contact_points` (`multicontact_force_estimator_msgs/ContactPointArray`)

#### Publishing Topics
* `~estimated_force` (`multicontact_force_estimator_msgs/WrenchStampedArray`)
* `~estimated_force_marker` (`visualization_msgs::MarkerArray`):
* `~[name of force sensors in robotmodel]_offset` (`geometry_msgs/WrenchStamped`)
* `~root_mass_offset` (`std_msgs/Float64`)

#### Parameters
* `~robotmodel` (String, REQUIRED)

  Path to VRML model. "package://" is supported.
* `~rate` (Double, default: `50`[Hz])
* `~offset_update_rate` (Double, default: `1.0`[Hz])
* `~force_offset_update_thre` (Double, default: `0.5`[N])
* `~moment_offset_update_thre` (Double, default: `0.1`[Nm])
* `~marker_scale` (Double, default: `0.01`[m/N])
