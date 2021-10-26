#include "ros/ros.h"
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <unistd.h>
#include <string>

/* ROS and messages */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h> // abcabc
#include <geometry_msgs/Pose.h> // abcabc
// #include <cmath> // abcabc


/* MoveIt! related */
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>

using namespace std;

// width 1280, height 720
#define CENTER_LEFT 610
#define CENTER_RIGHT 670

// Upper bound of bbox for fast approaching
#define BB_THRESH_FAST 150
// when the size of bb reaches to BB_THRESH, use arm to grip
#define BB_THRESH 270
/*

turtlebot/turtlebot3_navigation/launch/amcl.launch
initial_pose
  <arg name="scan_topic"     default="scan"/>
  <arg name="initial_pose_x" default="-1.0"/>
  <arg name="initial_pose_y" default="1.0"/>
  <arg name="initial_pose_a" default="5.497787143782138"/>
*/

// twist linear velocity
#define TWIST_LV_FAST 0.15
#define TWIST_LV 0.05

ros::Publisher teleop_pub;

bool approach=false;
bool centered=false;
bool stop = false;
bool rotate_ = false;
bool found = false;
bool goal_reached = false;
bool start_approach = false;

/* Vectors for joint states */
static vector<double> state_arm(4);
static vector<double> state_gripper(2);
static vector<double> temp_arm(4);
static vector<double> temp_gripper(2);

/* Read cmd_vel to know whether the robot is stopped*/
void msgCallback_goal(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
	if(msg->status_list[0].status == 3) {
        ROS_INFO("GOAL REACHED!");
	    goal_reached = true;
    }
}


/* Approach to object, assuming the object i */
void msgCallback_approach(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    if(rotate_)
		return;
    // if(approach && centered)
    //     return;

//    ROS_INFO("APPROACH Callback");
    int size = msg->bounding_boxes.size();
    int max_width = -1;
    int target_index = -1;
    float center = -1.0;

    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;


    // TODO: Find biggest one
    for (int i = 0; i<size; i++) {
        string object = msg->bounding_boxes[i].Class;
	    cout << "object:" << object << endl;
        if (object == "bottle"|| object == "vase" || object == "person"){
	        ROS_INFO("BOTTLE!");
            found = true;
            float xmin = msg->bounding_boxes[i].xmin;
            float xmax = msg->bounding_boxes[i].xmax;
            float width = xmax - xmin;
            if (width < 0)
                width = - width;
            if (max_width < width) {
                max_width = width;
                target_index = i;
                center = (xmax + xmin)/2.0;
            }
        }
    }
    cout << "width: " << max_width << endl;
    cout << "center: " << center << endl;

    if (start_approach && max_width > 0){

        if ( center > CENTER_RIGHT + 100) {
            ROS_INFO("VERY RIGHT");
            centered = false;
            twist.angular.z = -0.2;
        }
        else if(center > CENTER_RIGHT) {
            ROS_INFO("RIGHT");
            centered = false;
            twist.angular.z = -0.1;
        }
        else if(center < CENTER_LEFT-100) {
            ROS_INFO("VERY LEFT");
            centered = false;
            twist.angular.z = 0.2;
        }
        else if(center < CENTER_LEFT) {
            ROS_INFO("LEFT");
            centered = false;
            twist.angular.z = 0.1;
        }
        else {
            ROS_INFO("CENTER");
            twist.angular.z = 0.0;
            centered = true;
        }

        if (max_width <= BB_THRESH_FAST) {
                ROS_INFO("RUN!");
                twist.linear.x = TWIST_LV_FAST;
                approach = false;
            } else if (BB_THRESH_FAST < max_width && max_width <= BB_THRESH){
                ROS_INFO("WALK!");
                twist.linear.x = TWIST_LV;
                approach = false;
            } else{
                ROS_INFO("STOP");
                twist.linear.x = 0;
                approach = true;   
            }

        teleop_pub.publish(twist);
    }
	
    return;
    
}

void move_arm(float a1, float a2, float a3, float a4, moveit::planning_interface::MoveGroupInterface &move_group_arm){

	temp_arm[0] = a1;
	temp_arm[1] = a2;
	temp_arm[2] = a3;
	temp_arm[3] = a4;

	move_group_arm.setJointValueTarget(temp_arm);
	move_group_arm.move();
}

void move_grip(bool close, moveit::planning_interface::MoveGroupInterface &move_group_gripper){

    if (close) {
        temp_gripper[0] = -0.01;
	    temp_gripper[1] = -0.01;
    } else {
        temp_gripper[0] = 0.01;
	    temp_gripper[1] = 0.01;
    }

	move_group_gripper.setJointValueTarget(temp_gripper);
	move_group_gripper.move();	
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "yolo_bottle_tracker");;
    ros::NodeHandle nh;
    teleop_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    
    // ros::Subscriber count_stop;
    ros::Publisher navGoal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
    geometry_msgs::Twist twist;
    
    ros::Publisher cancel_goal = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);
    geometry_msgs::PoseStamped goal;

    for ( int num = 0; num < 2; num++){
        approach=false;
        centered=false;
        stop = false;
        found = false;
        rotate_ = true;
        goal_reached =false;
        start_approach = false;
        
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
        
        ros::Subscriber yolo_bounding_box = nh.subscribe("/darknet_ros/bounding_boxes", 100, msgCallback_approach);
        
        
        /* ROS spinning for allowing movegroup to get state (Reference: Moveit tutorial) */
        ros::AsyncSpinner spinner(1);
        spinner.start();

        /* Planning group for arm */
        string planning_group_arm = "arm";
        moveit::planning_interface::MoveGroupInterface move_group_arm(planning_group_arm);

        /* Planning group for gripper */
        string planning_group_gripper = "gripper";
        moveit::planning_interface::MoveGroupInterface move_group_gripper(planning_group_gripper);

        if (num==0) {
            ROS_INFO("FIRST START - ALREADY START WITH INIT POSE!");
        } else {
            ROS_INFO("SECOND START - SET ARM!");
            move_arm(0,0,0,0,move_group_arm);

            twist.angular.z = -1;
            float duration = 3.92699;
            float time = 0.0;
            while ( teleop_pub.getNumSubscribers() < 1)
		    usleep(10000);
            ROS_INFO("TELEOP CONNECTED!");
            while (ros::ok() && time < duration){
                teleop_pub.publish(twist);
                usleep(10000);
                time += 0.01;
            }
            twist.angular.z = 0.0;
            for (int i = 0; ros::ok() && i < 5 ; i++) {
                usleep(10000);
                teleop_pub.publish(twist);
            }
        }
        rotate_ = false;

        /* explore */
        goal.header.frame_id = "map";
        goal.pose.position.x = 0.7;
        goal.pose.position.y = 0.61;
        goal.pose.position.z = 0;
        goal.pose.orientation.w = 0.707106;
        goal.pose.orientation.x = 0;
        goal.pose.orientation.y = 0;
        goal.pose.orientation.z = 0.707106;
        
        int count = 0;
        while(ros::ok() && count < 10) {
            if (navGoal.getNumSubscribers() > 1) {
                ROS_INFO("NAVGOAL CONNECTED!");
                navGoal.publish(goal);
                count++;
		        usleep(1000);
            }
        }

        while (!found) {
            usleep(1000);
        }
        ROS_INFO("FOUND!!!");

        actionlib_msgs::GoalID empty;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
        count = 0;
        while(ros::ok() && count < 10) {
            // if (cancel_goal.getNumSubscribers() >= 1) {
                ROS_INFO("CANCEL_GOAL CONNECTED!");
                cancel_goal.publish(empty);
                teleop_pub.publish(twist);
                usleep(10000);
                count++;
            // }
        }
        start_approach = true;

        while ( ros::ok() && !(approach && centered) ) {
            usleep(10000);
            // ros::spinOnce();
        }

        yolo_bounding_box.shutdown();


        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
        for ( int i =0; i<50; i++)
            teleop_pub.publish(twist);

        move_arm(0,1.271,-0.797,-0.268,move_group_arm);
        sleep(5);
        move_grip(true,move_group_gripper);

        
        goal.header.frame_id = "map";
        goal.pose.position.x = -0.875;
        goal.pose.position.y = 0.875;
        goal.pose.position.z = 0;
        goal.pose.orientation.w = 0.38268343236509;
        goal.pose.orientation.x = 0;
        goal.pose.orientation.y = 0;
        goal.pose.orientation.z = 0.923879532511287;
        
        count = 0;
        while(ros::ok() && count < 10) {
            if (navGoal.getNumSubscribers() > 1) {
                ROS_INFO("RETURN NAVGOAL CONNECTED!");
                navGoal.publish(goal);
                count++;
            }
        }
        sleep(0.1);

        ros::Subscriber goal_status = nh.subscribe("/move_base/status", 100, msgCallback_goal);
        while ( ros::ok() && !goal_reached ) {
            usleep(100000);
            ros::spinOnce();
        }
        ROS_INFO("GOAL REACHED!");
        goal_status.shutdown();

        move_grip(false,move_group_gripper);

    }

    return 0;

}
