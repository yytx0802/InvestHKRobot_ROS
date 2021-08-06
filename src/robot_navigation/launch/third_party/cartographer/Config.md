# briefings
- `test_carto` is for common functions.

# to convert cartographer map to ros map
```sh
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '${HOME}/bag.pbstream', include_unfinished_submaps: "true"}"
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/lscm/ros_map -pbstream_filename=/home/lscm/bag.pbstream -resolution=0.05
```




