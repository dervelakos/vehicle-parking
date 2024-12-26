## Saving a map
To save a map
```
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Running ros nav
```
ros2 launch playground slam_launch.py 
ros2 run tf2_ros static_transform_publisher   0 0 0 0 0 0   base_link base_footprint
ros2 launch playground navigation_launch.py
```
