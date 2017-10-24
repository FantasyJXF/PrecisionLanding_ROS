//visual-flow-landing
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace std;

apriltags_ros::AprilTagDetectionArray apriltag_pos_msg;
float uav_altitude = 0.0;     
float uav_x_distance = 0.0;
float uav_y_distance = 0.0;

//obtain the apriltags pose
void TagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{   

    static int flag_not_found_mark = 0;


     // 获取无人机相对apriltag的xy距离  
    apriltags_ros::AprilTagDetection tag_msg = msg->detections[0];

    uav_x_distance = tag_msg.pose.pose.position.x;
    uav_y_distance = tag_msg.pose.pose.position.y;
    uav_altitude = tag_msg.pose.pose.position.z;

    if (abs(uav_x_distance) > 0.01f && abs(uav_y_distance) > 0.01f) 
    {

        cout<<" err_x "<<uav_x_distance<<endl;

        cout<<" err_y "<<uav_y_distance<<endl;

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

    // sub tag
    ros::Subscriber TagDetectionsSubscriber = nh.subscribe("/tag_detections",1,TagDetections);  
 

    ros::Rate loopRate(50.0);
    while(ros::ok()){
      
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}

