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