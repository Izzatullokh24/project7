# project7
# Here is source code for project 7

```
ubuntu2005@ubuntu:~$ git clone https://github.com/online-courses-materials/sms-project7.git
Cloning into 'sms-project7'...
remote: Enumerating objects: 24, done.
remote: Counting objects: 100% (24/24), done.
remote: Compressing objects: 100% (21/21), done.
remote: Total 24 (delta 4), reused 12 (delta 0), pack-reused 0
Unpacking objects: 100% (24/24), 10.93 KiB | 1.56 MiB/s, done.
ubuntu2005@ubuntu:~$ ls
assigment3_ws   assignment4_ws         midterm      project2_ws   SMS-WS
assignment_1    catkin_ws              midterm_ws   project4_ws   Templates
assignment2     Desktop                Music        project6_ws   Videos
assignment2_ws  Documents              Pictures     Public        week3
assignment3_ws  Downloads              project2     sms-project3  week3.sh
assignment4     install_ros_noetic.sh  project2+ws  sms-project7  week3.sh.save
ubuntu2005@ubuntu:~$ cd sms-project7/
ubuntu2005@ubuntu:~/sms-project7$ ls
README.md  src
ubuntu2005@ubuntu:~/sms-project7$ mkdir srx
ubuntu2005@ubuntu:~/sms-project7$ mkdir src
mkdir: cannot create directory ‘src’: File exists
ubuntu2005@ubuntu:~/sms-project7$ catkin_create_pkg project7 roscpp
Created file project7/package.xml
Created file project7/CMakeLists.txt
Created folder project7/include/project7
Created folder project7/src
Successfully created files in /home/ubuntu2005/sms-project7/project7. Please adjust the values in package.xml.
ubuntu2005@ubuntu:~/sms-project7$ cd ..
ubuntu2005@ubuntu:~$ ls
assigment3_ws  assignment2_ws  assignment4_ws  Documents              midterm     Pictures     project2_ws  Public        SMS-WS     week3
assignment_1   assignment3_ws  catkin_ws       Downloads              midterm_ws  project2     project4_ws  sms-project3  Templates  week3.sh
assignment2    assignment4     Desktop         install_ros_noetic.sh  Music       project2+ws  project6_ws  sms-project7  Videos     week3.sh.save
ubuntu2005@ubuntu:~$ cd sms-project7/
ubuntu2005@ubuntu:~/sms-project7$ catkin_make
Base path: /home/ubuntu2005/sms-project7
Source space: /home/ubuntu2005/sms-project7/src
Build space: /home/ubuntu2005/sms-project7/build
Devel space: /home/ubuntu2005/sms-project7/devel
Install space: /home/ubuntu2005/sms-project7/install
####
#### Running command: "cmake /home/ubuntu2005/sms-project7/src -DCATKIN_DEVEL_PREFIX=/home/ubuntu2005/sms-project7/devel -DCMAKE_INSTALL_PREFIX=/home/ubuntu2005/sms-project7/install -G Unix Makefiles" in "/home/ubuntu2005/sms-project7/build"
####
-- The C compiler identification is GNU 9.4.0
-- The CXX compiler identification is GNU 9.4.0
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Using CATKIN_DEVEL_PREFIX: /home/ubuntu2005/sms-project7/devel
-- Using CMAKE_PREFIX_PATH: /opt/ros/noetic
-- This workspace overlays: /opt/ros/noetic
-- Found PythonInterp: /usr/bin/python3 (found suitable version "3.8.10", minimum required is "3") 
-- Using PYTHON_EXECUTABLE: /usr/bin/python3
-- Using Debian Python package layout
-- Found PY_em: /usr/lib/python3/dist-packages/em.py  
-- Using empy: /usr/lib/python3/dist-packages/em.py
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/ubuntu2005/sms-project7/build/test_results
-- Forcing gtest/gmock from source, though one was otherwise available.
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python3 (found version "3.8.10") 
-- Found Threads: TRUE  
-- Using Python nosetests: /usr/bin/nosetests3
-- catkin 0.8.10
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - project7
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'project7'
-- ==> add_subdirectory(project7)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- Generating .msg files for action project7/Navigate2D /home/ubuntu2005/sms-project7/src/project7/action/Navigate2D.action
Generating for action Navigate2D
-- project7: 7 messages, 0 services
-- Configuring done
-- Generating done
-- Build files have been written to: /home/ubuntu2005/sms-project7/build
####
#### Running command: "make -j2 -l2" in "/home/ubuntu2005/sms-project7/build"
####
Scanning dependencies of target sensor_msgs_generate_messages_cpp
Scanning dependencies of target _project7_generate_messages_check_deps_Navigate2DActionGoal
[  0%] Built target sensor_msgs_generate_messages_cpp
Scanning dependencies of target _project7_generate_messages_check_deps_Navigate2DFeedback
[  0%] Built target _project7_generate_messages_check_deps_Navigate2DActionGoal
Scanning dependencies of target _project7_generate_messages_check_deps_Navigate2DAction
[  0%] Built target _project7_generate_messages_check_deps_Navigate2DFeedback
Scanning dependencies of target actionlib_msgs_generate_messages_cpp
[  0%] Built target _project7_generate_messages_check_deps_Navigate2DAction
[  0%] Built target actionlib_msgs_generate_messages_cpp
Scanning dependencies of target std_msgs_generate_messages_cpp
Scanning dependencies of target _project7_generate_messages_check_deps_Navigate2DResult
[  0%] Built target std_msgs_generate_messages_cpp
Scanning dependencies of target _project7_generate_messages_check_deps_Navigate2DGoal
[  0%] Built target _project7_generate_messages_check_deps_Navigate2DResult
[  0%] Built target _project7_generate_messages_check_deps_Navigate2DGoal
Scanning dependencies of target _project7_generate_messages_check_deps_Navigate2DActionResult
Scanning dependencies of target _project7_generate_messages_check_deps_Navigate2DActionFeedback
[  0%] Built target _project7_generate_messages_check_deps_Navigate2DActionResult
Scanning dependencies of target sensor_msgs_generate_messages_eus
[  0%] Built target _project7_generate_messages_check_deps_Navigate2DActionFeedback
[  0%] Built target sensor_msgs_generate_messages_eus
Scanning dependencies of target std_msgs_generate_messages_eus
Scanning dependencies of target actionlib_msgs_generate_messages_eus
[  0%] Built target std_msgs_generate_messages_eus
Scanning dependencies of target sensor_msgs_generate_messages_lisp
[  0%] Built target actionlib_msgs_generate_messages_eus
Scanning dependencies of target std_msgs_generate_messages_lisp
[  0%] Built target sensor_msgs_generate_messages_lisp
[  0%] Built target std_msgs_generate_messages_lisp
Scanning dependencies of target sensor_msgs_generate_messages_nodejs
Scanning dependencies of target actionlib_msgs_generate_messages_lisp
[  0%] Built target sensor_msgs_generate_messages_nodejs
[  0%] Built target actionlib_msgs_generate_messages_lisp
Scanning dependencies of target actionlib_msgs_generate_messages_nodejs
Scanning dependencies of target std_msgs_generate_messages_nodejs
[  0%] Built target std_msgs_generate_messages_nodejs
[  0%] Built target actionlib_msgs_generate_messages_nodejs
Scanning dependencies of target std_msgs_generate_messages_py
Scanning dependencies of target sensor_msgs_generate_messages_py
[  0%] Built target std_msgs_generate_messages_py
[  0%] Built target sensor_msgs_generate_messages_py
Scanning dependencies of target actionlib_msgs_generate_messages_py
Scanning dependencies of target project7_generate_messages_cpp
[  0%] Built target actionlib_msgs_generate_messages_py
Scanning dependencies of target project7_generate_messages_eus
[  2%] Generating C++ code from project7/Navigate2DAction.msg
[  4%] Generating EusLisp code from project7/Navigate2DAction.msg
[  7%] Generating EusLisp code from project7/Navigate2DActionGoal.msg
[  9%] Generating C++ code from project7/Navigate2DActionGoal.msg
[ 12%] Generating EusLisp code from project7/Navigate2DActionResult.msg
[ 14%] Generating C++ code from project7/Navigate2DActionResult.msg
[ 17%] Generating EusLisp code from project7/Navigate2DActionFeedback.msg
[ 19%] Generating C++ code from project7/Navigate2DActionFeedback.msg
[ 21%] Generating EusLisp code from project7/Navigate2DGoal.msg
[ 24%] Generating C++ code from project7/Navigate2DGoal.msg
[ 26%] Generating EusLisp code from project7/Navigate2DResult.msg
[ 29%] Generating C++ code from project7/Navigate2DResult.msg
[ 31%] Generating EusLisp code from project7/Navigate2DFeedback.msg
[ 34%] Generating C++ code from project7/Navigate2DFeedback.msg
[ 36%] Generating EusLisp manifest code for project7
[ 36%] Built target project7_generate_messages_cpp
Scanning dependencies of target project7_generate_messages_lisp
[ 39%] Generating Lisp code from project7/Navigate2DAction.msg
[ 41%] Generating Lisp code from project7/Navigate2DActionGoal.msg
[ 43%] Generating Lisp code from project7/Navigate2DActionResult.msg
[ 46%] Generating Lisp code from project7/Navigate2DActionFeedback.msg
[ 48%] Generating Lisp code from project7/Navigate2DGoal.msg
[ 51%] Generating Lisp code from project7/Navigate2DResult.msg
[ 53%] Generating Lisp code from project7/Navigate2DFeedback.msg
[ 53%] Built target project7_generate_messages_lisp
Scanning dependencies of target project7_generate_messages_nodejs
[ 53%] Built target project7_generate_messages_eus
[ 56%] Generating Javascript code from project7/Navigate2DAction.msg
Scanning dependencies of target project7_generate_messages_py
[ 58%] Generating Python from MSG project7/Navigate2DAction
[ 60%] Generating Javascript code from project7/Navigate2DActionGoal.msg
[ 63%] Generating Javascript code from project7/Navigate2DActionResult.msg
[ 65%] Generating Javascript code from project7/Navigate2DActionFeedback.msg
[ 68%] Generating Javascript code from project7/Navigate2DGoal.msg
[ 70%] Generating Python from MSG project7/Navigate2DActionGoal
[ 73%] Generating Javascript code from project7/Navigate2DResult.msg
[ 75%] Generating Javascript code from project7/Navigate2DFeedback.msg
[ 75%] Built target project7_generate_messages_nodejs
Scanning dependencies of target action_client
[ 78%] Building CXX object project7/CMakeFiles/action_client.dir/src/action_client.cpp.o
[ 80%] Generating Python from MSG project7/Navigate2DActionResult
[ 82%] Generating Python from MSG project7/Navigate2DActionFeedback
[ 85%] Generating Python from MSG project7/Navigate2DGoal
[ 87%] Generating Python from MSG project7/Navigate2DResult
[ 90%] Generating Python from MSG project7/Navigate2DFeedback
[ 92%] Generating Python msg __init__.py for project7
[ 92%] Built target project7_generate_messages_py
Scanning dependencies of target action_server
[ 95%] Building CXX object project7/CMakeFiles/action_server.dir/src/action_server.cpp.o
[ 97%] Linking CXX executable /home/ubuntu2005/sms-project7/devel/lib/project7/action_client
[100%] Linking CXX executable /home/ubuntu2005/sms-project7/devel/lib/project7/action_server
[100%] Built target action_client
Scanning dependencies of target project7_generate_messages
[100%] Built target project7_generate_messages
[100%] Built target action_server
ubuntu2005@ubuntu:~/sms-project7$ source devel/setup.bash
ubuntu2005@ubuntu:~/sms-project7$ rosmsg list
actionlib/TestAction
actionlib/TestActionFeedback
actionlib/TestActionGoal
actionlib/TestActionResult
actionlib/TestFeedback
actionlib/TestGoal
actionlib/TestRequestAction
actionlib/TestRequestActionFeedback
actionlib/TestRequestActionGoal
actionlib/TestRequestActionResult
actionlib/TestRequestFeedback
actionlib/TestRequestGoal
actionlib/TestRequestResult
actionlib/TestResult
actionlib/TwoIntsAction
actionlib/TwoIntsActionFeedback
actionlib/TwoIntsActionGoal
actionlib/TwoIntsActionResult
actionlib/TwoIntsFeedback
actionlib/TwoIntsGoal
actionlib/TwoIntsResult
actionlib_msgs/GoalID
actionlib_msgs/GoalStatus
actionlib_msgs/GoalStatusArray
actionlib_tutorials/AveragingAction
actionlib_tutorials/AveragingActionFeedback
actionlib_tutorials/AveragingActionGoal
actionlib_tutorials/AveragingActionResult
actionlib_tutorials/AveragingFeedback
actionlib_tutorials/AveragingGoal
actionlib_tutorials/AveragingResult
actionlib_tutorials/FibonacciAction
actionlib_tutorials/FibonacciActionFeedback
actionlib_tutorials/FibonacciActionGoal
actionlib_tutorials/FibonacciActionResult
actionlib_tutorials/FibonacciFeedback
actionlib_tutorials/FibonacciGoal
actionlib_tutorials/FibonacciResult
bond/Constants
bond/Status
control_msgs/FollowJointTrajectoryAction
control_msgs/FollowJointTrajectoryActionFeedback
control_msgs/FollowJointTrajectoryActionGoal
control_msgs/FollowJointTrajectoryActionResult
control_msgs/FollowJointTrajectoryFeedback
control_msgs/FollowJointTrajectoryGoal
control_msgs/FollowJointTrajectoryResult
control_msgs/GripperCommand
control_msgs/GripperCommandAction
control_msgs/GripperCommandActionFeedback
control_msgs/GripperCommandActionGoal
control_msgs/GripperCommandActionResult
control_msgs/GripperCommandFeedback
control_msgs/GripperCommandGoal
control_msgs/GripperCommandResult
control_msgs/JointControllerState
control_msgs/JointJog
control_msgs/JointTolerance
control_msgs/JointTrajectoryAction
control_msgs/JointTrajectoryActionFeedback
control_msgs/JointTrajectoryActionGoal
control_msgs/JointTrajectoryActionResult
control_msgs/JointTrajectoryControllerState
control_msgs/JointTrajectoryFeedback
control_msgs/JointTrajectoryGoal
control_msgs/JointTrajectoryResult
control_msgs/PidState
control_msgs/PointHeadAction
control_msgs/PointHeadActionFeedback
control_msgs/PointHeadActionGoal
control_msgs/PointHeadActionResult
control_msgs/PointHeadFeedback
control_msgs/PointHeadGoal
control_msgs/PointHeadResult
control_msgs/SingleJointPositionAction
control_msgs/SingleJointPositionActionFeedback
control_msgs/SingleJointPositionActionGoal
control_msgs/SingleJointPositionActionResult
control_msgs/SingleJointPositionFeedback
control_msgs/SingleJointPositionGoal
control_msgs/SingleJointPositionResult
controller_manager_msgs/ControllerState
controller_manager_msgs/ControllerStatistics
controller_manager_msgs/ControllersStatistics
controller_manager_msgs/HardwareInterfaceResources
diagnostic_msgs/DiagnosticArray
diagnostic_msgs/DiagnosticStatus
diagnostic_msgs/KeyValue
dynamic_reconfigure/BoolParameter
dynamic_reconfigure/Config
dynamic_reconfigure/ConfigDescription
dynamic_reconfigure/DoubleParameter
dynamic_reconfigure/Group
dynamic_reconfigure/GroupState
dynamic_reconfigure/IntParameter
dynamic_reconfigure/ParamDescription
dynamic_reconfigure/SensorLevels
dynamic_reconfigure/StrParameter
gazebo_msgs/ContactState
gazebo_msgs/ContactsState
gazebo_msgs/LinkState
gazebo_msgs/LinkStates
gazebo_msgs/ModelState
gazebo_msgs/ModelStates
gazebo_msgs/ODEJointProperties
gazebo_msgs/ODEPhysics
gazebo_msgs/PerformanceMetrics
gazebo_msgs/SensorPerformanceMetric
gazebo_msgs/WorldState
geometry_msgs/Accel
geometry_msgs/AccelStamped
geometry_msgs/AccelWithCovariance
geometry_msgs/AccelWithCovarianceStamped
geometry_msgs/Inertia
geometry_msgs/InertiaStamped
geometry_msgs/Point
geometry_msgs/Point32
geometry_msgs/PointStamped
geometry_msgs/Polygon
geometry_msgs/PolygonStamped
geometry_msgs/Pose
geometry_msgs/Pose2D
geometry_msgs/PoseArray
geometry_msgs/PoseStamped
geometry_msgs/PoseWithCovariance
geometry_msgs/PoseWithCovarianceStamped
geometry_msgs/Quaternion
geometry_msgs/QuaternionStamped
geometry_msgs/Transform
geometry_msgs/TransformStamped
geometry_msgs/Twist
geometry_msgs/TwistStamped
geometry_msgs/TwistWithCovariance
geometry_msgs/TwistWithCovarianceStamped
geometry_msgs/Vector3
geometry_msgs/Vector3Stamped
geometry_msgs/Wrench
geometry_msgs/WrenchStamped
map_msgs/OccupancyGridUpdate
map_msgs/PointCloud2Update
map_msgs/ProjectedMap
map_msgs/ProjectedMapInfo
nav_msgs/GetMapAction
nav_msgs/GetMapActionFeedback
nav_msgs/GetMapActionGoal
nav_msgs/GetMapActionResult
nav_msgs/GetMapFeedback
nav_msgs/GetMapGoal
nav_msgs/GetMapResult
nav_msgs/GridCells
nav_msgs/MapMetaData
nav_msgs/OccupancyGrid
nav_msgs/Odometry
nav_msgs/Path
pcl_msgs/ModelCoefficients
pcl_msgs/PointIndices
pcl_msgs/PolygonMesh
pcl_msgs/Vertices
project7/Navigate2DAction
project7/Navigate2DActionFeedback
project7/Navigate2DActionGoal
project7/Navigate2DActionResult
project7/Navigate2DFeedback
project7/Navigate2DGoal
project7/Navigate2DResult
roscpp/Logger
rosgraph_msgs/Clock
rosgraph_msgs/Log
rosgraph_msgs/TopicStatistics
rospy_tutorials/Floats
rospy_tutorials/HeaderString
sensor_msgs/BatteryState
sensor_msgs/CameraInfo
sensor_msgs/ChannelFloat32
sensor_msgs/CompressedImage
sensor_msgs/FluidPressure
sensor_msgs/Illuminance
sensor_msgs/Image
sensor_msgs/Imu
sensor_msgs/JointState
sensor_msgs/Joy
sensor_msgs/JoyFeedback
sensor_msgs/JoyFeedbackArray
sensor_msgs/LaserEcho
sensor_msgs/LaserScan
sensor_msgs/MagneticField
sensor_msgs/MultiDOFJointState
sensor_msgs/MultiEchoLaserScan
sensor_msgs/NavSatFix
sensor_msgs/NavSatStatus
sensor_msgs/PointCloud
sensor_msgs/PointCloud2
sensor_msgs/PointField
sensor_msgs/Range
sensor_msgs/RegionOfInterest
sensor_msgs/RelativeHumidity
sensor_msgs/Temperature
sensor_msgs/TimeReference
shape_msgs/Mesh
shape_msgs/MeshTriangle
shape_msgs/Plane
shape_msgs/SolidPrimitive
smach_msgs/SmachContainerInitialStatusCmd
smach_msgs/SmachContainerStatus
smach_msgs/SmachContainerStructure
std_msgs/Bool
std_msgs/Byte
std_msgs/ByteMultiArray
std_msgs/Char
std_msgs/ColorRGBA
std_msgs/Duration
std_msgs/Empty
std_msgs/Float32
std_msgs/Float32MultiArray
std_msgs/Float64
std_msgs/Float64MultiArray
std_msgs/Header
std_msgs/Int16
std_msgs/Int16MultiArray
std_msgs/Int32
std_msgs/Int32MultiArray
std_msgs/Int64
std_msgs/Int64MultiArray
std_msgs/Int8
std_msgs/Int8MultiArray
std_msgs/MultiArrayDimension
std_msgs/MultiArrayLayout
std_msgs/String
std_msgs/Time
std_msgs/UInt16
std_msgs/UInt16MultiArray
std_msgs/UInt32
std_msgs/UInt32MultiArray
std_msgs/UInt64
std_msgs/UInt64MultiArray
std_msgs/UInt8
std_msgs/UInt8MultiArray
stereo_msgs/DisparityImage
tf/tfMessage
tf2_msgs/LookupTransformAction
tf2_msgs/LookupTransformActionFeedback
tf2_msgs/LookupTransformActionGoal
tf2_msgs/LookupTransformActionResult
tf2_msgs/LookupTransformFeedback
tf2_msgs/LookupTransformGoal
tf2_msgs/LookupTransformResult
tf2_msgs/TF2Error
tf2_msgs/TFMessage
theora_image_transport/Packet
trajectory_msgs/JointTrajectory
trajectory_msgs/JointTrajectoryPoint
trajectory_msgs/MultiDOFJointTrajectory
trajectory_msgs/MultiDOFJointTrajectoryPoint
turtle_actionlib/ShapeAction
turtle_actionlib/ShapeActionFeedback
turtle_actionlib/ShapeActionGoal
turtle_actionlib/ShapeActionResult
turtle_actionlib/ShapeFeedback
turtle_actionlib/ShapeGoal
turtle_actionlib/ShapeResult
turtle_actionlib/Velocity
turtlesim/Color
turtlesim/Pose
visualization_msgs/ImageMarker
visualization_msgs/InteractiveMarker
visualization_msgs/InteractiveMarkerControl
visualization_msgs/InteractiveMarkerFeedback
visualization_msgs/InteractiveMarkerInit
visualization_msgs/InteractiveMarkerPose
visualization_msgs/InteractiveMarkerUpdate
visualization_msgs/Marker
visualization_msgs/MarkerArray
visualization_msgs/MenuEntry
ubuntu2005@ubuntu:~/sms-project7$ rosmsg show project7/Navigate2DGoal
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z

ubuntu2005@ubuntu:~/sms-project7$ rosmsg show project7/Navigate2DAction
project7/Navigate2DActionGoal action_goal
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  actionlib_msgs/GoalID goal_id
    time stamp
    string id
  project7/Navigate2DGoal goal
    geometry_msgs/Point point
      float64 x
      float64 y
      float64 z
project7/Navigate2DActionResult action_result
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  actionlib_msgs/GoalStatus status
    uint8 PENDING=0
    uint8 ACTIVE=1
    uint8 PREEMPTED=2
    uint8 SUCCEEDED=3
    uint8 ABORTED=4
    uint8 REJECTED=5
    uint8 PREEMPTING=6
    uint8 RECALLING=7
    uint8 RECALLED=8
    uint8 LOST=9
    actionlib_msgs/GoalID goal_id
      time stamp
      string id
    uint8 status
    string text
  project7/Navigate2DResult result
    float32 elapsed_time
project7/Navigate2DActionFeedback action_feedback
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  actionlib_msgs/GoalStatus status
    uint8 PENDING=0
    uint8 ACTIVE=1
    uint8 PREEMPTED=2
    uint8 SUCCEEDED=3
    uint8 ABORTED=4
    uint8 REJECTED=5
    uint8 PREEMPTING=6
    uint8 RECALLING=7
    uint8 RECALLED=8
    uint8 LOST=9
    actionlib_msgs/GoalID goal_id
      time stamp
      string id
    uint8 status
    string text
  project7/Navigate2DFeedback feedback
    float32 distance_to_point

ubuntu2005@ubuntu:~/sms-project7$

```
