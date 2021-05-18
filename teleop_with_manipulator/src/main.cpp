/* Basic */
#include <thread>
#include <mutex>
#include <cstdio>
#include <iostream>
#include <string>
#include <algorithm>

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

/* Include */
#include "../include/teleop_with_manipulator/keyhandler.hpp"

using namespace std;

#define BURGER_MAX_LIN_VEL 0.22
#define BURGER_MAX_ANG_VEL 2.84

#define WAFFLE_MAX_LIN_VEL 0.26
#define WAFFLE_MAX_ANG_VEL 1.82

#define LIN_VEL_STEP_SIZE 0.01
#define ANG_VEL_STEP_SIZE 0.1

/* Vectors for joint states */
static vector<double> state_arm(4);
static vector<double> state_gripper(2);
static vector<double> temp_arm(4);
static vector<double> temp_gripper(2);

void vels(float target_linear_vel, float target_angular_vel){
	cout << "currently:\t linear vel " << target_linear_vel << "\t angular vel " << target_angular_vel << endl;
}

float makeSimpleProfile(float output, float input, float slop){
	if (input > output)
        output = min( input, output + slop );
    else if (input < output)
        output = max( input, output - slop );
    else
        output = input;

    return output;
}
    

float constrain(float input, float low, float high){
	if (input < low)
      input = low;
    else if (input > high)
      input = high;
    else
      input = input;

    return input;
}

float checkLinearLimitVelocity(float vel) {
	vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL);
	return vel;
}

float checkAngularLimitVelocity(float vel) {
	vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL);
	return vel;
}


int main(int argc, char **argv){

	/* ROS Initialization */
	ros::init(argc, argv, "main");
	ros::NodeHandle nh;

	/* Init publisher for teleoperation */
	ros::Publisher teleop_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
	
	/* ROS spinning for allowing movegroup to get state (Reference: Moveit tutorial) */
	ros::AsyncSpinner spinner(1);
	spinner.start();
	 
	/* Terminal Initialization */
	int code;
	if (terminal_init()) {
		ROS_ERROR("Terminal initialization failed.");
		return -1;
	}
	
	/* Planning group for arm */
	string planning_group_arm = "arm";
	moveit::planning_interface::MoveGroupInterface move_group_arm(planning_group_arm);
	
	/* Planning group for gripper */
	string planning_group_gripper = "gripper";
	moveit::planning_interface::MoveGroupInterface move_group_gripper(planning_group_gripper);

	/* Initialize velocities */
	float target_linear_vel = 0.0;
	float target_angular_vel = 0.0;
	float control_linear_vel = 0.0;
	float control_angular_vel = 0.0;

	
	/* Guide */
	cout << "Ctrl+C : Quit" << endl
		 << " 1 Key : Close gripper " << endl
		 << " 2 Key : Open gripper" << endl
		 << " 3 Key : Init pose" << endl
		 << " 4 Key : Home pose" << endl
		 << " 5 Key : Rotate counterclockwise" << endl
		 << " 6 Key : Rotate clockwise" << endl
		 << " 0 Key : Help" << endl
		 << " w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)" << endl
		 << " a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)" << endl
		 << " s : force stop" << endl;
	
	/* Main loop */
	while ((code = getc(stdin)) != EOF && ros::ok()) {
		/* Only numkey and spacebar are valid */
		code -= 48;
		if (!((code >= 0 && code <= 6) || code == -16 || code == 67 || code == 71 || code == 72 || code == 49 || code == 52))
			continue;
		// s:67 w:71 x:72 a:49 d:52

		/* Update joint state and print */
		state_arm.clear();
		state_gripper.clear();
		state_arm = move_group_arm.getCurrentJointValues();
		state_gripper = move_group_gripper.getCurrentJointValues();
		
		/* Getting state was successful? */
		if (state_arm.empty() || state_arm.empty()){
			ROS_ERROR("Getting robot state failed");
			continue;
		}
		
		/* Print current state */
		ROS_INFO("Current State:");
		cout << "         Gripper : " << state_gripper.at(0) << endl
		     << "     Gripper_sub : " << state_gripper.at(1) << endl
		     << "          Joint1 : " << state_arm.at(0) << endl
		     << "          Joint2 : " << state_arm.at(1) << endl
		     << "          Joint3 : " << state_arm.at(2) << endl
		     << "          Joint4 : " << state_arm.at(3) << endl
		     << " Linear velocity : " << target_linear_vel << endl
		     << "Angular velocity : " << target_angular_vel << endl;
		     
		/* Planning */		
		copy(state_arm.begin(), state_arm.begin() + 4, temp_arm.begin());
		copy(state_gripper.begin(), state_gripper.begin() + 2, temp_gripper.begin()); 
		
		switch (code) {
			/* Close gripper (-0.01) */
			case 1: 	
				cout << "[COMMAND] Close gripper" << endl;
				temp_gripper[0] = -0.01;
				temp_gripper[1] = -0.01;
				break; 
			
			/* Open gripper (0.01) */
			case 2: 	
				cout << "[COMMAND] Open gripper" << endl;
				temp_gripper[0] = 0.01;
				temp_gripper[1] = 0.01;
				break;
			
			/* Init pose (0  0  0  0) */
			case 3:
				cout << "[COMMAND] Init pose" << endl;
				temp_arm[0] = 0;
				temp_arm[1] = 0;
				temp_arm[2] = 0;
				temp_arm[3] = 0;
				break;
			
			/* Home pose (0 -1 .3 .7) */
			case 4:
				cout << "[COMMAND] Home pose" << endl;
				temp_arm[0] = 0;
				temp_arm[1] = -1;
				temp_arm[2] = 0.3;
				temp_arm[3] = 0.7;
				break;
			
			/* Rotate counterclockwise (J1 + 0.1 rad) */
			case 5: 
				cout << "[COMMAND] Rotate counterclockwise" << endl;
				temp_arm[0] = (temp_arm[0] + 0.1 > 3.14) ? 3.14 : temp_arm[0] + 0.1;
				break; 
			
			/* Rotate clockwise (J1 - 0.1 rad) */
			case 6:	
				cout << "[COMMAND] Rotate clockwise" << endl;
				temp_arm[0] = (temp_arm[0] + 0.1 < -3.14) ? -3.14 : temp_arm[0] - 0.1;
				break;
			
			/* Guide */
			case 0:
				cout << "\n\nCtrl+C : Quit" << endl
					 << " 1 Key : Close gripper " << endl
					 << " 2 Key : Open gripper" << endl
					 << " 3 Key : Init pose" << endl
					 << " 4 Key : Home pose" << endl
					 << " 5 Key : Rotate counterclockwise" << endl
					 << " 6 Key : Rotate clockwise" << endl
					 << " 0 Key : Help\n\n" << endl
					 << " w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)" << endl
					 << " a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)" << endl
					 << " s : force stop" << endl;
	
				continue;
			
			/* Update */
			case -16:
				cout << "[COMMAND] State update" << endl;
				continue;
			
			// s:67 w:71 x:72 a:49 d:52
			case 67:
				target_linear_vel   = 0.0;
                control_linear_vel  = 0.0;
                target_angular_vel  = 0.0;
                control_angular_vel = 0.0;
				vels(target_linear_vel, target_angular_vel);
				break;

			/* w - inclease linear velocity */
			case 71:
				target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE);
                vels(target_linear_vel,target_angular_vel);
				break;

			/* x - decrease linear velocity */
			case 72:
				target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE);
                vels(target_linear_vel,target_angular_vel);
				break;

			/* a - increase angular velocity */
			case 49:
				target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE);
                vels(target_linear_vel,target_angular_vel);
				break;

			/* d - decrease angular velocity */
			case 52:
				target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE);
                vels(target_linear_vel,target_angular_vel);
				break;
			
			/* Not reached */
			default:
				ROS_ERROR("Invalid key input");
		}
            
		
		if((code >= 0 && code <= 6) || code == -16) {
			move_group_arm.setJointValueTarget(temp_arm);
			move_group_gripper.setJointValueTarget(temp_gripper);
		
			move_group_arm.move();
			move_group_gripper.move();		
		} else if(code == 67 || code == 71 || code == 72 || code == 49 || code == 52) {
			geometry_msgs::Twist twist;
			
			control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0));
			twist.linear.x = control_linear_vel;
			twist.linear.y = 0.0;
			twist.linear.z = 0.0;

			control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0));
			twist.angular.x = 0.0;
			twist.angular.y = 0.0;
			twist.angular.z = control_angular_vel;

			teleop_pub.publish(twist);
		}
		
	}
	
	ROS_ERROR("Cannot reach!");
	return -1;
done:
	ROS_INFO("Bye");
	return 0;

}
