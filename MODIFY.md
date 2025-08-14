### get object pose
1. `rl_sar/src/rl_sim.cpp`: ModelStatesCallback
    ```cpp
    // robot base's pose and vel
    geometry_msgs::msg::Pose base_pose;
    geometry_msgs::msg::Twist base_vel;
    // object's pose, considered as 4th model in gazebo environment
    geometry_msgs::msg::Pose object_pose;
    ```
    
### lidar's pointcloud
1. `robots/g1_description/xacro/robot.xacro`, pcl topic and frequence
    ```yaml
    topic="/front_points" hz="20" 
    topic="/back_points" hz="20" 
    ```

2. modify vertical and horizontal FOV in `velodyne_simulator/velodyne_description/urdf/HDL-32E.urdf.xacro`
    ```yaml
    <scan>
        <horizontal>
        <samples>${samples}</samples>
        <resolution>1</resolution>
        <min_angle>${min_angle}</min_angle>
        <max_angle>${max_angle}</max_angle>
        </horizontal>
        <vertical>
        <samples>${lasers}</samples>
        <resolution>1</resolution>
        <min_angle>-${5.0*M_PI/180.0}</min_angle>
        <max_angle> ${85.0*M_PI/180.0}</max_angle>
        </vertical>
    </scan>
    ```

### set 3D goal pose & show world
1. add `interactive_marker.py` in `rl_sar/scripts/` for 3D goal pose
2. add `object_visualizaer.py` in `rl_sar/scripts/` for world visualization
3. just launch `rviz.launch.py`.
<!-- 2. open `rviz` => set `Global Options`'s `fixed frame` = `world` -->
<!-- 3. add `InteractiveMarkers` and set `Interactive Markers Namespace` = `target_pose` -->
4. click 3D goal will send `/desired_goal_pose` topic (geometry_msgs::msg::PoseStamped)
