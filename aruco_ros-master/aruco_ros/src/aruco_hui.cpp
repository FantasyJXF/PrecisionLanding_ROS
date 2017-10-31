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
# define U0 349.701395
# define V0 242.461950

// 归一化焦距
# define FOCAL_X 941.776547
# define FOCAL_Y 942.487566

geometry_msgs::TwistStamped vs_body_axis;
geometry_msgs::PoseStamped uavPose;


double uavRollENU, uavPitchENU, uavYawENU;

double err_x, err_y, err_z;
double err_roll, err_pitch, err_yaw;
double last_err_x, last_err_y, last_err_z;
double last_err_roll, last_err_pitch, last_err_raw;
double last_timestamp;


float uav_init_altitude = 0.0;
float uav_altitude = 0.0;     
float uav_x_distance = 0.0;
float uav_y_distance = 0.0;

geometry_msgs::Point huihui;

static const unsigned MAX_NO_LOGFILE = 999;     /**< Maximum number of log files */
static const char *log_dir = "/home/fantasy/logs";

FILE *fd = NULL;

bool file_exist(char *file)  
{  
    return (0 == access(file,F_OK));  // 0 means the file exists; -1 means not
}  

FILE* open_log_file( )
{

    /* string to hold the path to the log */
    char log_file_name[64] = "";
    char log_file_path[sizeof(log_file_name) + 64] = "";

    unsigned file_number = 1; // start with file log001

    /* look for the next file that does not exist */
    while (file_number <= MAX_NO_LOGFILE) {

        /* format log file path: e.g. /home/fantasy/logs/log001.txt */
        snprintf(log_file_name, sizeof(log_file_name), "aruco_%03u.txt", file_number);
        snprintf(log_file_path, sizeof(log_file_path), "%s/%s", log_dir,log_file_name);

        if (!file_exist(log_file_path)) {
            break;
        }

        file_number++;
    }

    FILE *_fd = fopen(log_file_path,"a+");

    return _fd;
}

// 无人机位置和姿态，From 内部传感器
void uavPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{    
/*    uavPose.pose.position.x = msg->pose.position.x;
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
*/
    static bool init_flag = false; 
    if (msg->pose.position.z > 0.28f)
    {
        if(!init_flag)
        {
            uav_init_altitude = msg->pose.position.z;
            cout<<"the init alt is "<<uav_init_altitude<<endl;
            init_flag = true; 
        }
        else
        {           
            uav_altitude = msg->pose.position.z;
            cout<<"actural alt is "<<uav_init_altitude<<endl;
        }

        //cout << "uav_altitude = " << uav_altitude << endl;
        // 首先控制高度恒定，z轴偏差为目前高度与初始高度之差
        err_z = uav_init_altitude - uav_altitude;
    }else{
        ROS_INFO_STREAM("The height is under 0.28m");
    }  
}

// 从超声传感器获取飞机高度,并计算高度偏差 
/*void uavAltitudeReceived(const sensor_msgs::Range& msg)
{
     ROS_INFO("hey jude  dont make me cry");
    static bool init_flag = false; 
    if (msg.range > 0.28f)
    {
        if(!init_flag)
        {
            uav_init_altitude = msg.range;
            cout<<"the init alt is "<<uav_init_altitude<<endl;
            init_flag = true; 
        }
        else
        {           
            uav_altitude = msg.range;
            cout<<"actural alt is "<<uav_init_altitude<<endl;
        }

        //cout << "uav_altitude = " << uav_altitude << endl;
        // 首先控制高度恒定，z轴偏差为目前高度与初始高度之差
        err_z = uav_init_altitude - uav_altitude;
    }else{
        ROS_INFO_STREAM("The height is under 0.28m");
    }
}*/

// 获取 aruco 坐标中心，并计算无人机相对 x y 距离 
void markerCenterReceived(const geometry_msgs::Point& msg)
{	
    static geometry_msgs::Point markcenter;

	// 获取 aruco 坐标中心
	markcenter.x = msg.x;
	markcenter.y = msg.y;
    //cout<<"ceter x is "<<markcenter.x<<"  center y is "<<markcenter.y<<endl;

    // 当标志中心坐标有效时，计算无人机相对标志 x y 距离
    if (markcenter.x > 0.01f && markcenter.y > 0.01f) 
    {

/*    	uav_x_distance = uav_altitude*(markcenter.x-U0)/FOCAL_X; // 小孔成像
    	uav_y_distance = uav_altitude*(markcenter.y-V0)/FOCAL_Y;

        cout<<"x "<<uav_x_distance<<"\t"<<"y "<<uav_y_distance<<endl;
*/
        huihui.x = uav_altitude*(markcenter.x-U0)/FOCAL_X; // 小孔成像
        huihui.y = uav_altitude*(markcenter.y-V0)/FOCAL_Y;
        huihui.z = uav_altitude;

        cout<<"x "<<huihui.x<<"\t"<<"y "<<huihui.y<<endl;

        fprintf(fd,"X = %0.3f \n Y = %0.3f \n Z = %0.3f \n ", huihui.x,huihui.y,huihui.z);  


        // 将无人机与mark在 x y z 方向的距离偏差，分别表示为err_ ,便于控制部分的理解
        // 图像坐标系与Vicon坐标系，x轴反向，y轴重合
    	err_x = -huihui.x;
    	err_y = huihui.y;

    } 
    else
    {
        
        ROS_INFO_STREAM("Fail to found mark");

    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_hui_node");
    ros::NodeHandle nh;

    int flag_time = 0;
    fd = open_log_file();

    printf("------------that's funny -------------\n");

    // sub image center pixel
    ros::Subscriber markerCenterSubscriber = nh.subscribe("/aruco_single/pixel",1000,markerCenterReceived);  

    // sub uavpose
    ros::Subscriber uavPoseSubscriber = nh.subscribe("/mavros/local_position/pose", 1000, uavPoseReceived);

    // sub radar distance
    //ros::Subscriber uavAltitudeSubscriber = nh.subscribe("/mavros/px4flow/ground_distance", 1000, uavAltitudeReceived);

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
