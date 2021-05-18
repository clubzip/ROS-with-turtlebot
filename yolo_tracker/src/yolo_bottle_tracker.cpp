#include "ros/ros.h"
#include <darknet_ros_msgs/BoundingBox.h>
#include <geometry_msgs/Twist.h>

// width 1280, height 720
#define CENTER_LEFT 640
#define CENTER_RIGHT 360

ros::Publisher teleop_pub;
geometry_msgs::Twist twist;
twist.linear.x = 0.0;
twist.linear.y = 0.0;
twist.linear.z = 0.0;
twist.angular.x = 0.0;
twist.angular.y = 0.0;
twist.angular.z = 0.0;

void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{

    for(auto iter = begin(msg); iter != end(msg); iter++)
    {
        if(*iter->Class == "bottle")
        {
            float center = (msg->xmin + msg->xmax)/2.0;
            if(center > CENTER_RIGHT)
            {
                twist.angular.z = 0.2;
                teleop_pub.publish(twist);
                return;
            }
            else if(center < CENTER_LEFT)
            {
                twist.angular.z = -0.2;
                teleop_pub.publish(twist);
                return;
            }
            else
            {
                twist.angular.z = 0.0;
                teleop_pub.publish(twist);
                return;
            }
        }
    }
    twist.angular.z = 0.1;
    teleop_pub.publish(twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolo_bottle_tracker");;
    ros::NodeHandle nh;
    teleop_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::Subscriber yolo_bounding_box = nh.advertise("bounding_boxes", 100, msgCallback);
    
    ros::spin();

    return 0;

}