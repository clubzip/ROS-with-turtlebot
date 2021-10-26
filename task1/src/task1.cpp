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
#define BB_THRESH_FAST 200
// when the size of bb reaches to BB_THRESH, use arm to grip
#define BB_THRESH 340


// twist linear velocity
#define TWIST_LV_FAST 0.15
#define TWIST_LV 0.05

ros::Publisher teleop_pub;

bool approach=false;
bool centered=false;

/* Vectors for joint states */
static vector<double> state_arm(4);
static vector<double> state_gripper(2);
static vector<double> temp_arm(4);
static vector<double> temp_gripper(2);

/* Approach to object, assuming the object i */
void msgCallback_approach(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

    ROS_INFO("APPROACH Callback");
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
    if (max_width > 0){

        if(center > CENTER_RIGHT) {
            ROS_INFO("RIGHT");
            centered = false;
            twist.angular.z = -0.1;
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
	state_arm.clear();
	state_arm = move_group_arm.getCurrentJointValues();

	if (state_arm.empty() || state_arm.empty()){
		ROS_ERROR("Getting robot state failed");
	}
	
    copy(state_arm.begin(), state_arm.begin() + 4, temp_arm.begin());

	temp_arm[0] = a1;
	temp_arm[1] = a2;
	temp_arm[2] = a3;
	temp_arm[3] = a4;

	move_group_arm.setJointValueTarget(temp_arm);
	move_group_arm.move();
}

void move_grip(bool close, moveit::planning_interface::MoveGroupInterface &move_group_gripper){
	state_gripper.clear();
	state_gripper = move_group_gripper.getCurrentJointValues();
	
	copy(state_gripper.begin(), state_gripper.begin() + 2, temp_gripper.begin()); 
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
    ros::Subscriber yolo_bounding_box = nh.subscribe("/darknet_ros/bounding_boxes", 100, msgCallback_approach);
    geometry_msgs::Twist twist;


    /* ROS spinning for allowing movegroup to get state (Reference: Moveit tutorial) */
	ros::AsyncSpinner spinner(1);
	spinner.start();

    /* Planning group for arm */
    string planning_group_arm = "arm";
    moveit::planning_interface::MoveGroupInterface move_group_arm(planning_group_arm);

    /* Planning group for gripper */
    string planning_group_gripper = "gripper";
    moveit::planning_interface::MoveGroupInterface move_group_gripper(planning_group_gripper);
    move_grip(false,move_group_gripper);
    move_arm(0,0,0,0,move_group_arm);

    /* Go forward until detected*/
    twist.linear.x = 0.1;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    for ( int i =0; i<50; i++)
	    teleop_pub.publish(twist);

    while ( ros::ok() && !(approach && centered) ) {
	sleep(0.1);
        ros::spinOnce();
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
    sleep(0.5);
    move_arm(0,0,0,0,move_group_arm);

    return 0;

}
