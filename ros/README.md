# C++ ROS Code
## Dependencies:
OpenCV
ROS sensor_msgs et cv_bridge

## CMakeLists.txt:
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

Make sure your CMakeLists.txt file is correctly configured for ROS and OpenCV, as described above.
## Compiling
Compile the project with 
```
catkin_make
```

## Run
```rosrun <your_package_name> ir_image_processor```
