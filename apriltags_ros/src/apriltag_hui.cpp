//visual-flow-landing
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "stdio.h"

using namespace std;

geometry_msgs::TwistStamped vs_body_axis;
geometry_msgs::PoseStamped uavPose;

double uavRollENU, uavPitchENU, uavYawENU;
float uav_altitude = 0.0;     
float uav_x_distance = 0.0;
float uav_y_distance = 0.0;

FILE *fd = NULL;


//obtain the apriltags pose
void TagDetections(const geometry_msgs::PoseStamped::ConstPtr& tag_msg)
{   

     // 获取无人机相对apriltag的xy距离  
    if (abs(tag_msg->pose.position.x) > 0 && abs(tag_msg->pose.position.y) > 0) 
    {
        uavPose.pose.position.x = tag_msg->pose.position.x;
        uavPose.pose.position.y = tag_msg->pose.position.y;
        uavPose.pose.position.z = tag_msg->pose.position.z;
        uavPose.pose.orientation.x = tag_msg->pose.orientation.x; // 四元数
        uavPose.pose.orientation.y = tag_msg->pose.orientation.y;
        uavPose.pose.orientation.z = tag_msg->pose.orientation.z;
        uavPose.pose.orientation.w = tag_msg->pose.orientation.w;
        
        // Using ROS tf to get RPY angle from Quaternion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(uavPose.pose.orientation, quat);  // 将四元数消息msg转换为四元数
        tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU); // 由四元数得到欧拉角
/*        ROS_INFO("Current relative position: \n X = %0.3f \n Y = %0.3f \n Z = %0.3f \n \
                Current UAV angles: \n roll=%0.3f \n pitch=%0.3f \n  yaw=%0.3f", uavPose.pose.position.x,
                uavPose.pose.position.y,uavPose.pose.position.z,uavRollENU*180/3.1415926, uavPitchENU*180/3.1415926, uavYawENU*180/3.1415926);  
*/
        fprintf(fd,"X = %0.3f \n Y = %0.3f \n Z = %0.3f \n ", uavPose.pose.position.x,
                uavPose.pose.position.y,uavPose.pose.position.z);  

    } 
    else    
    {
       
        ROS_INFO_STREAM("Fail to found mark, enter Position Hold");

    }

    cout<<"TagDetectionsReceived! "<<endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_hui");
    ros::NodeHandle nh;

    fd = fopen("/home/fantasy/bagfiles/fantasy.txt","a+");
    
    // sub tag
    //ros::Subscriber TagDetectionsSubscriber = nh.subscribe("/tag_detections",1,TagDetections);  
    ros::Subscriber TagDetectionsSubscriber = nh.subscribe("/rel_pose",30,TagDetections); 

    ros::Rate loopRate(50.0);
    while(ros::ok()){
      
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}

