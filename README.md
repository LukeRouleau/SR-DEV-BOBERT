# SR-DEV-BOBERT

Create a ROS2-running robot powered by the Nvidia Jetson. Since the Jetson runs Ubuntu 18, use docker to accomplish this.

## ROS2 Commands
Tutorial link [here](https://www.youtube.com/watch?v=bFDfvKctvV8&list=PLRE44FoOoKf7NzWwxt3W2taZ7BiWyfhCp&index=1).

[View installed packages:](https://www.youtube.com/watch?v=X3Cmtg3Tq3Y&list=PLRE44FoOoKf7NzWwxt3W2taZ7BiWyfhCp&index=2)
```
ros2 pkg list
ros2 pkg executables <package_name>
```

[Package Creation, Sourcing, and Running:](https://www.youtube.com/watch?v=lN4_-l7FCWk&list=PLRE44FoOoKf7NzWwxt3W2taZ7BiWyfhCp&index=3)
```
colcon build
source install/setup.bash
ros2 pkg create --built-type ament_cmake <package_name>
ros2 pkg create --build-type ament_python <package_name>
ros2 run <package_name> <executable_name>
```

[Ros2 Executables and Nodes:](https://www.youtube.com/watch?v=aeOS9xqblrg&list=PLRE44FoOoKf7NzWwxt3W2taZ7BiWyfhCp&index=4)
```

```
