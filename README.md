# Intelligent Robots Course Project
Group 6

20151146 Hyeonggyu Kim
20181098 Eunseong Park
20205072 Suyeong Kim (Gradudate)

----
## Required packages
- darknet_ros
- turtlebot3_navigation
- turtlebot3_bringup
- turtlebot3_manipulation_bringup
- turtlebot3_manipulation_moveit_config : move_group
- task1, task2, task3_init, task4_init
- [optional] convertor.py, ros_deep_learning

Additionally, you need map files for task 2,3 and 4.
Before you use it, you need to change path of map.pgm to yours in yaml file.

## Simulation
You can test this package in simulated environment. It can be done by modifying turtlebot3_manipulation_gazebo package like below.
![gazebo_simulation](https://user-images.githubusercontent.com/52950649/138919717-b7ffa2eb-da0c-4bf2-b042-652901f88496.png)

## Task 1
Task 1 can be divided into several subtasks as follows:

1. Make the turtlebot go straight
2. Stop the turtlebot when it reaches to a given threshold
3. Control turtlebot's arm to grasp the bottle

For subtask 1, we use `cmd_vel` topic and `geometry_msgs::Twist` message. By using this, we can specify the turtlebot's linear/angular velocity and publish the command. 

When the turtlebot noticed it is approached to the target(bottle), then slows down its speed and stops finally (at the threshold). This subtask requries a way to estimate the distance between turtlebot and bottle. For this, we use YOLO and `darknet_ros_msgs::BoundingBoxes` message. Based on the fact that the size of bounding box for bottle becomes bigger as the turtlebot approaches, we tunned our stop point and corresponding bounding box size, so that the turtlebot can grasp the bottle with its arm.

Lastly, we use OpenMANIPULATOR and MoveIt, to control the turtlebot's arm. We can manipulate the arm's state(joint arm / gripper angle) with `moveit::planning_interface::MoveGroupInterface`, specifying the target state. 


### Execution command
```sh
roscore
roslaunch turtlebot3_bringup turtlebot3_robot.launch (on turtlebot3 SBC)

// Execute YOLOv3 and its camera setting
roslaunch ros_deep_learning video_source.ros1.launch input:=csi://0 output:=display://0
python convertor.py
roslaunch darknet_ros darknet_ros.launch

// Setting controller for manipulator
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
roslaunch turtlebot3_manipulation_moveit_config move_group.launch

// Run node for task 1
rosrun task1 task1

```

## Task 2
Task 2 requires two more phases: returning the turtlebot to initial position(1) and opening the gripper to release the bottle(2).

For returning, we use navigation and `move_base_simple/goal` topic. After the turtlebot successfully grasp the bottle, our program publishes `geometry_msgs::PoseStamped` message with `/move_base_simple/goal` topic, which specifies the goal position and orientation. Then the turtlebot returns to the base.

The trick for the latter is same with the subtask 3 in task 1. But it is important to know whether the turtlebot had returned to the base - otherwise, the turtlebot may release the bottle before return. So we subscribe `move_base/status` topic message which shows the status of goal and navigation. After checking the navigation was successful (i.e., the turtlebot reached to the base), we can open the gripper.

### Execution command
```sh
roscore
roslaunch turtlebot3_bringup turtlebot3_robot.launch (on turtlebot3 SBC)

// Execute YOLOv3 and its camera setting
roslaunch ros_deep_learning video_source.ros1.launch input:=csi://0 output:=display://0
python convertor.py
roslaunch darknet_ros darknet_ros.launch

// Execute turtlebot3 navigation node
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

// Setting controller for manipulator
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
roslaunch turtlebot3_manipulation_moveit_config move_group.launch

// Run node for task 2
rosrun task2 task2

```

## Task 3
In task 3, there is no assumption about the position of bottle. Thus, our turtlebot's sight has to span entire space of arena. For this, our turtlebot moves in various direction so that it can find the bottle no matter where the bottle is.

Rotation can be performed by publishing `geometry_msgs::Twist` messages, specifying the turtlebot's angular velocity($z$) and its duration.

Other subtasks are same with previous task, just repeating once more for picking up two bottles.

### Execution command
```sh
roscore
roslaunch turtlebot3_bringup turtlebot3_robot.launch (on turtlebot3 SBC)

// Execute YOLOv3 and its camera setting
roslaunch ros_deep_learning video_source.ros1.launch input:=csi://0 output:=display://0
python convertor.py
roslaunch darknet_ros darknet_ros.launch

// Execute turtlebot3 navigation node
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

// Setting controller for manipulator
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
roslaunch turtlebot3_manipulation_moveit_config move_group.launch

// Run node for task 3
rosrun task3_init task3_init

```

## Task 4
As our turtlebot returns to the base by using navigation, we can use navigation to avoid the wall and span all the space for the given map. At the beginning, we publish `move_base_simple/goal` to move the turtlebot. During navigation, if the turtlebot found the bottle, stops the navigation with `move_base/cancel` topic in order to make the turtlebot pick up the bottle. Then publishes `move_base_simple/goal` again, to return to the base. 


### Execution command
```sh
roscore
roslaunch turtlebot3_bringup turtlebot3_robot.launch (on turtlebot3 SBC)

// Execute YOLOv3 and its camera setting
roslaunch ros_deep_learning video_source.ros1.launch input:=csi://0 output:=display://0
python convertor.py
roslaunch darknet_ros darknet_ros.launch

// Execute turtlebot3 navigation node
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map4.yaml

// Setting controller for manipulator
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
roslaunch turtlebot3_manipulation_moveit_config move_group.launch

// Run node for task 4
rosrun task4_init task4_init

```
