#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/State.h>


using namespace std;

// 主点坐标
# define U0 319.0
# define V0 220.0
// 归一化焦距
# define FOCAL_X 487.0
# define FOCAL_Y 488.0

geometry_msgs::TwistStamped vs_body_axis;
geometry_msgs::PoseStamped uavPose;


double uavRollENU, uavPitchENU, uavYawENU;

double err_x, err_y, err_z;
double err_roll, err_pitch, err_yaw;
double last_err_x, last_err_y, last_err_z;
double last_err_roll, last_err_pitch, last_err_raw;
double last_timestamp;

double xyP, xyI, xyD, zP, zI, zD, yawP, yawD;
double dt;

float uav_init_altitude = 0.0;
float uav_altitude = 0.0;     
float uav_x_distance = 0.0;
float uav_y_distance = 0.0;

geometry_msgs::Point huihui;


// 无人机位置和姿态，From 内部传感器
void uavPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{    
    uavPose.pose.position.x = msg->pose.position.x;
    uavPose.pose.position.y = msg->pose.position.y;
    uavPose.pose.position.z = msg->pose.position.z;
    uavPose.pose.orientation.x = msg->pose.orientation.x; // 四元数
    uavPose.pose.orientation.y = msg->pose.orientation.y;
    uavPose.pose.orientation.z = msg->pose.orientation.z;
    uavPose.pose.orientation.w = msg->pose.orientation.w;
    
    // Using ROS tf to get RPY angle from Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(uavPose.pose.orientation, quat);  // 将四元数消息msg转换为四元数
    tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU); // 由四元数得到欧拉角
    ROS_INFO("Current UAV angles: roll=%0.3f, pitch=%0.3f, yaw=%0.3f", uavRollENU*180/3.1415926, uavPitchENU*180/3.1415926, uavYawENU*180/3.1415926);  
}


// 获取 aruco 坐标中心，并计算无人机相对 x y 距离 
void markerCenterReceived(const geometry_msgs::Point& msg)
{	
    static geometry_msgs::Point markcenter;

	// 获取 aruco 坐标中心
	markcenter.x = msg.x;
	markcenter.y = msg.y;
    cout<<"ceter x is "<<markcenter.x<<"  center y is "<<markcenter.y<<endl;
    uav_altitude = 1;

    // 当标志中心坐标有效时，计算无人机相对标志 x y 距离
    if (markcenter.x > 0.01f && markcenter.y > 0.01f) 
    {
        ROS_INFO_STREAM("haode ");
/*    	uav_x_distance = uav_altitude*(markcenter.x-U0)/FOCAL_X; // 小孔成像
    	uav_y_distance = uav_altitude*(markcenter.y-V0)/FOCAL_Y;

        cout<<"x "<<uav_x_distance<<"\t"<<"y "<<uav_y_distance<<endl;
*/
        huihui.x = uav_altitude*(markcenter.x-U0)/FOCAL_X; // 小孔成像
        huihui.y = uav_altitude*(markcenter.y-V0)/FOCAL_Y;
        huihui.y = 0.666;

        cout<<"x "<<huihui.x<<"\t"<<"y "<<huihui.y<<endl;
        // 将无人机与mark在 x y z 方向的距离偏差，分别表示为err_ ,便于控制部分的理解
        // 图像坐标系与Vicon坐标系，x轴反向，y轴重合
    	err_x = -uav_x_distance;
    	err_y = uav_y_distance;

    } 
    else
    {
        
        ROS_INFO_STREAM("Fail to found mark, enter Position Hold");

    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_hui_node");
    ros::NodeHandle nh;

    int flag_time = 0;


    printf("------------that's funny -------------\n");

    // sub image center pixel
    ros::Subscriber markerCenterSubscriber = nh.subscribe("/aruco_single/pixel",1000,markerCenterReceived);  

    // sub uavpose
    ros::Subscriber uavPoseSubscriber = nh.subscribe("/mavros/local_position/pose", 1000, uavPoseReceived);

    ros::Publisher mark_err = nh.advertise<geometry_msgs::Point>("/huihui", 10);

    ros::Rate loopRate(30.0);
    while(ros::ok())
    {	 
      
        mark_err.publish(huihui);
	    ros::spinOnce();
  	    loopRate.sleep();
    }
    
    return 0;
}
