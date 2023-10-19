<!-- bring up rviz, navigation2, description -->
$ navigation

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