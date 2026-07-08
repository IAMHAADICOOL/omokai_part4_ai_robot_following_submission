# Maps

Save your turtlebot3_world map here as `turtlebot3_world.yaml` + `.pgm`:

    ros2 run nav2_map_server map_saver_cli -f \
        src/omokai_bringup/maps/turtlebot3_world

Then `colcon build --symlink-install` so the map installs and the default
`map:=` argument in core_pipeline.launch.py can find it.
