# tp-ros2


## TP1

### Question 1

```
ros2 topic pub -r 2 /my_topic std_msgs/msg/String "data: Hello World"
```
Publishes the message "Hello World" on the topic /my_topic at a rate of 2Hz.

### Question 2

```
ros2 interface show std_msgs/msg/Float64
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

float64 data
```

```
ros2 interface show sensor_msgs/msg/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

geometry_msgs/Quaternion orientation
	float64 x 0
	float64 y 0
	float64 z 0
	float64 w 1
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
	float64 x
	float64 y
	float64 z
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
	float64 x
	float64 y
	float64 z
float64[9] linear_acceleration_covariance # Row major x, y z
```

```
ros2 interface show std_srvs/srv/Trigger
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

### Question 3

```
ros2 topic pub -r 1 /heading std_msgs/msg/Float64 "data: 90.0"
```

### Question 4

```
ros2 node list -a
WARNING: Be aware that are nodes in the graph that share an exact name, this can have unintended side effects.
/_ros2cli_19283
/_ros2cli_19294
/_ros2cli_61146
/_ros2cli_7508
/_ros2cli_8275
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0
/_ros2cli_daemon_0_04ae1acee27e49958201bd3d904c79fd
/_ros2cli_daemon_0_6893cb955cfb4e3ebed8592283aa922f
/_ros2cli_daemon_0_f8123de52e2e4618a0584d441648211b
/riptide_recorder
```

### Question 5

```
ros2 node info /_ros2cli_19283 --include-hidden
/_ros2cli_19283
  Subscribers:

  Publishers:
    /heading: std_msgs/msg/Float64
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:

  Service Clients:

  Action Servers:

  Action Clients:
```

### Question 6

```
~/Documents/ROS/tp-ros2$ ls
build  install  log  README.md  src
```

### Question 7

```
echo "source ~/workspaceRos/install/setup.bash" >> ~/.bashrc
```
The line was added to the .bashrc file

### Question 8

```
~/Documents/ROS/tp-ros2/src/tp1$ ls
CMakeLists.txt  include  package.xml  src
```

### Question 9

```
ros2 run tp1 nodeA
[INFO] [1677685268.522491313] [minimal_publisher]: Publishing: '90.000000'

ros2 topic echo /heading
data: 90.0
---
```

### Question 10

```
ros2 run tp1 nodeA
[INFO] [1677685700.570929238] [minimal_publisher]: Publishing: '42.835161'
[INFO] [1677685701.070991370] [minimal_publisher]: Publishing: '44.200915'
[INFO] [1677685701.570977269] [minimal_publisher]: Publishing: '34.742716'
[INFO] [1677685702.070828910] [minimal_publisher]: Publishing: '16.784450'
[INFO] [1677685702.570847082] [minimal_publisher]: Publishing: '-5.288454'
[INFO] [1677685703.070818060] [minimal_publisher]: Publishing: '-26.064636'
[INFO] [1677685703.570986611] [minimal_publisher]: Publishing: '-40.463916'
[INFO] [1677685704.070756283] [minimal_publisher]: Publishing: '-44.950488'
[INFO] [1677685704.570992422] [minimal_publisher]: Publishing: '-38.430657'
[INFO] [1677685705.070951719] [minimal_publisher]: Publishing: '-22.503668'
[INFO] [1677685705.570999643] [minimal_publisher]: Publishing: '-1.063990'
[INFO] [1677685706.070969665] [minimal_publisher]: Publishing: '20.633511'
```

