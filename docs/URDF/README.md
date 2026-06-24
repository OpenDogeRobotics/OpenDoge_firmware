# Opendoge

URDF Description package for Opendoge quadruped robot.

## Package Structure

```
Opendoge/
├── config/
│   └── joint_names_Opendoge.yaml    # Joint name configurations
├── launch/
│   ├── display.launch               # RViz visualization launch file
│   └── gazebo.launch               # Gazebo simulation launch file
├── meshes/
│   ├── base_link.STL               # Base link 3D model
│   ├── FL_*.STL, FR_*.STL         # Front legs 3D models
│   ├── RL_*.STL, RR_*.STL         # Rear legs 3D models
├── urdf/
│   ├── Opendoge.urdf               # Robot URDF description
│   └── Opendoge.csv                # Component CSV file
├── xml/
│   ├── Opendoge.xml                # OpenDoge XML configuration
│   └── scene.xml                   # Scene configuration
├── CMakeLists.txt
└── package.xml
```

## Robot Structure

Opendoge is a 4-legged robot with 3 actuated joints per leg:

| Leg | Hip | Thigh | Calf | Foot |
|-----|-----|-------|------|------|
| FL (Front Left) | FL_hip_joint | FL_thigh_joint | FL_calf_joint | FL_foot |
| FR (Front Right) | FR_hip_joint | FR_thigh_joint | FR_calf_joint | FR_foot |
| RL (Rear Left) | RL_hip_joint | RL_thigh_joint | RL_calf_joint | RL_foot |
| RR (Rear Right) | RR_hip_joint | RR_thigh_joint | RR_calf_joint | RR_foot |

## Dependencies

- ROS (Robot Operating System)
- robot_state_publisher
- joint_state_publisher
- rviz
- gazebo
- catkin

## Usage

### Visualize in RViz

```bash
roslaunch Opendoge display.launch
```

### Simulate in Gazebo

```bash
roslaunch Opendoge gazebo.launch
```

## License

BSD
