#include "ros/ros.h"
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>

// width 1280, height 720
#define CENTER_LEFT 540
#define CENTER_RIGHT 740

ros::Publisher teleop_pub;
geometry_msgs::Twist twist;
int count;

void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
//	std::cout<<"Bouding Boxes (header):" << msg->header <<std::endl;
//    darknet_ros_msgs::BoundingBoxes boxes = msg->bounding_boxes;    

    //for(auto iter = begin(msg->bounding_boxes); iter != end(msg->bounding_boxes); iter++)

   ROS_INFO("call back"); 
        if(msg->bounding_boxes[0].Class == "bottle")
        {
		count = 0;
		ROS_INFO("found bottle");
            float center = (msg->bounding_boxes[0].xmin + msg->bounding_boxes[0].xmax)/2.0;
            if(center > CENTER_RIGHT)
            {
		    ROS_INFO("right");
                twist.angular.z = -0.1;
                //teleop_pub.publish(twist);
                return;
            }
            else if(center < CENTER_LEFT)
            {
		    ROS_INFO("LEFT");
                twist.angular.z = 0.1;
                //teleop_pub.publish(twist);
                return;
            }
            else
            {
		    ROS_INFO("CENTER");
                twist.angular.z = 0.0;
                //teleop_pub.publish(twist);
                return;
            }
        }
}

int main(int argc, char** argv)
{
	count = 3;
	ROS_INFO("asdfasdf");
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    ros::init(argc, argv, "yolo_bottle_tracker");;
    ros::NodeHandle nh;
    teleop_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);


    ros::Subscriber yolo_bounding_box = nh.subscribe("/darknet_ros/bounding_boxes", 100, msgCallback);


//    for(int i = 0; i<
   while(ros::ok())
    {
	count++;
	    if(count>  2)
	    {
		ROS_INFO("search");
    		twist.angular.z = 0.1;
	    }
    ros::spinOnce();
    teleop_pub.publish(twist);
    sleep(1);
    }
    //while(ros::ok());
	    

    

    return 0;

}
