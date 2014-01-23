LCSR Barrett Configuration
==========================

## Debugging

You can command the manual trapezoidal trajectory generator from ROS like so:

Home position:
```
rostopic pub -1 /barrett/man_traj/joint_position_cmd trajectory_msgs/JointTrajectoryPoint "{ positions: [0., -1.57, 0.00, 3.0, 0.0, -0.8, 0.0] }"
```

```
rostopic pub -1 /barrett/man_traj/joint_position_cmd trajectory_msgs/JointTrajectoryPoint "{ positions: [-0.4, -1.5, 0.00, 1.5, -1.2, 1.0, 1.8] }
```

You can command the gripper over ros as well:
```
rostopic pub  /hand_cmd oro_barrett_msgs/BHandCmd "{ cmd: [2.0, 2.0, 2.0, 0.0], mode: [2,2,2,1] }" 
```
