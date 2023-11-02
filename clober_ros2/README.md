<!-- run navigation2 -->
$ ros2 launch clober_navigation2 navigation2_launch.py

% patrol

$ ros2 run clober_patrol patrol_action (int)[argv] (int)[argv]

$ ros2 run clober_patrol patrol_keyboard

***********************************************
Summary

This is clober's navigation package. 

CASE
1. patrol_action 
    using argv, put goal_poses in list , and then navigation to goal_list in order.

    Using Action -> Can configure robot's navigation's state

2. patrol_keyboard
    using keyboard values, navigate to goal_pose.

    Using Topic -> It can only publish goal_pose. So, It can't know robot's state




***********************************************

Setting Reference



-rviz map color

    negate : invertor color (ex, white, black , gray set) - default 0
    %!!!!!!!!!!!
    free_thresh
    : Pixels with occupancy probability less than this threshold are considered completely free.

    0.196 => discrete white and gray
    0.25  => didn't appear white area


-automizing initial pose in amcl 
    ( if you hate point 2D Pose Estimate, Go ahead)
    ex)

    set_initial_pose: true
        initial_pose:
        x: 0.084
        y: -0.332
        z: -0.540
        yaw: -0.4914918853472364



