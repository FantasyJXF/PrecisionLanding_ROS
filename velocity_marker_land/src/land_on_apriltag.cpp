#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Range.h>

using namespace std;

geometry_msgs::TwistStamped vs_body_axis;
geometry_msgs::PoseStamped uavPose;

double uavRollENU, uavPitchENU, uavYawENU;

double err_x, err_y, err_z;
double last_err_x, last_err_y, last_err_z;
double last_timestamp;

double xyP, xyI, xyD, zP, zI, zD, yawP, yawD;
double dt;

float uav_init_altitude = 0.0;
float uav_altitude = 0.0;     
float uav_x_distance = 0.0;
float uav_y_distance = 0.0;
float uav_z_distance = 0.0;


bool flag_offboard_mode = false;
bool flag_enter_position_hold = false;
bool vision_flag = false;

static const unsigned MAX_NO_LOGFILE = 999;     /**< Maximum number of log files */
static const char *log_dir = "/home/odroid/logs";

FILE *fd = NULL;

// 判断飞行模式
mavros_msgs::State current_state;
void stateReceived(const mavros_msgs::State::ConstPtr& msg) 
{
    static bool init_flag = false;
    current_state = *msg;

    if (current_state.mode == "OFFBOARD")
    {
        if(!init_flag)
        {
            ROS_INFO_STREAM("Offboard Mode"); 
            init_flag = true;         
        }

        flag_offboard_mode = true;
    }
    else
    {
        flag_offboard_mode = false;
        init_flag = false;  
    }
}

bool file_exist(char *file)  
{  
    return (0 == access(file,F_OK));  // 0 means the file exists; -1 means not
}  

FILE* open_log_file()
{

    /* string to hold the path to the log */
    char log_file_name[64] = "";
    char log_file_path[sizeof(log_file_name) + 64] = "";

    unsigned file_number = 1; // start with file log001

    /* look for the next file that does not exist */
    while (file_number <= MAX_NO_LOGFILE) {

        /* format log file path: e.g. /home/fantasy/logs/log001.txt */
        snprintf(log_file_name, sizeof(log_file_name), "position_%03u.txt", file_number);
		// create the directory if it not exist, or will Segment Fault
        snprintf(log_file_path, sizeof(log_file_path), "%s/%s", log_dir, log_file_name);

        if (!file_exist(log_file_path)) {
            break;
        }

        file_number++;
    }

    FILE *_fd = fopen(log_file_path, "a+");

    return _fd;
}

void uavAltReceived(const sensor_msgs::Range::ConstPtr& msg)
{    

    uav_altitude = msg->range;
    cout<<"The altitude is "<<uav_altitude << "m"<<endl;
}

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
    //ROS_INFO("Current UAV angles: roll=%0.3f, pitch=%0.3f, yaw=%0.3f", uavRollENU*180/3.1415926, uavPitchENU*180/3.1415926, uavYawENU*180/3.1415926);  

    // uav_altitude = msg->pose.position.z;
    // //cout<<"The altitude is "<<uav_altitude<<endl;

    // err_z = -uav_altitude;
}

//obtain the apriltags pose
void TagDetectionsReceived(const geometry_msgs::Pose::ConstPtr& tag_msg)
{   
    last_timestamp = ros::Time::now().toSec();
    vision_flag = true;

    // 获取无人机相对apriltag的xy距离  
    // For geometry_msgs::PoseStamped::ConstPtr& tag_msg
    // uav_x_distance = tag_msg->pose.position.x;
    // uav_y_distance = tag_msg->pose.position.y;
    // uav_z_distance = tag_msg->pose.position.z;
    
    // For geometry_msgs::Pose::ConstPtr& tag_msg   
    uav_x_distance = tag_msg->position.x;
    uav_y_distance = tag_msg->position.y;
    //uav_z_distance = uav_altitude;


    // static int flag_not_found_mark = 0;

    // if (abs(uav_x_distance) > 0 && abs(uav_y_distance) > 0) 
    // {

    //    // 将无人机与mark在 x y z 方向的距离偏差，分别表示为err_ ,便于控制部分的理解
    //     // x轴反向，y,z轴重合
    //     err_x = uav_x_distance;
    //     err_y = -uav_y_distance;

    //     //一旦发现标志，将未发现mark的计数标志复位0
    //     flag_not_found_mark = 0;
    // } 
    
    // // 说明： 1.8m 只是尝试值，明显有点大
    // // offboard 模式下，高度大于1.8m？，连续 10次 未发现标志，认为目标丢失，自动进入定点悬停模式
    // // 高度小于1.8m？，由于 Tag 占图像大部分，飞机晃动，Tag很容易出视野，识别失败，此时保持继续降落
    // else    
    // {
    //     flag_not_found_mark++;
    //     cout<<"It has been "<<flag_not_found_mark<<" times not seen the tag"<<endl;
    //     if (uav_altitude > 1.8 && (flag_not_found_mark >= 4))
    //     {           
    //        ROS_INFO_STREAM("Fail to found mark, enter Position Hold");
    //        flag_enter_position_hold = true;
    //     }
    //     else if(uav_altitude < 1.8 && (flag_not_found_mark >= 4))
    //     {
    //         err_x = 0.0;
    //         err_y = 0.0;
    //         last_err_x = 0.0;
    //         last_err_y = 0.0;
    //     }else{
    //         err_x = uav_x_distance;
    //         err_y = -uav_y_distance;
    //     }
    // }

    // //fprintf(fd,"X = %0.3f \n Y = %0.3f \n Z = %0.3f \n ",err_x,err_y,uav_altitude);  

}

// 飞机降落速度控制
void landingVelocityControl()
{
    vs_body_axis.header.seq++;
    vs_body_axis.header.stamp = ros::Time::now();
    //cout<<"landing velocity control err_x/y: "<<err_x<<" "<<err_y<<endl;
    dt = ros::Time::now().toSec() - last_timestamp;

    if(dt > 2e-1)
    {
        vision_flag = false;
    }

    static int flag_not_found_mark = 0;

    if (vision_flag) 
    {
        // tag 在图像中心时没有误差
        // tag在图片左边x方向距离为负，tag在右边x方向为正，对应的是飞机的Y轴横滚方向
        // tag在图片下侧y方向距离为负，tag在上侧y方向为正，对应的是飞机的X轴俯仰方向 
        // 将无人机与mark在 x y z 方向的距离偏差，分别表示为err_ ,便于控制部分的理解
        // x轴反向，y,z轴重合
        err_x = uav_y_distance;
        err_y = uav_x_distance;

        //一旦发现标志，将未发现mark的计数标志复位0
        flag_not_found_mark = 0;
    } 
    // 说明： 1.8m 只是尝试值，明显有点大
    // offboard 模式下，高度大于1.8m？，连续 10次 未发现标志，认为目标丢失，自动进入定点悬停模式
    // 高度小于1.8m？，由于 Tag 占图像大部分，飞机晃动，Tag很容易出视野，识别失败，此时保持继续降落
    else    
    {
        flag_not_found_mark++;
        cout<<"It has been "<<flag_not_found_mark<<" times not seen the tag"<<endl;
        if (uav_altitude >= 1.8 && (flag_not_found_mark >= 4))
        {           
           ROS_INFO_STREAM("Fail to found mark, enter Position Hold");
           flag_enter_position_hold = true;
        }
        else
        {
            err_x = 0.0;
            err_y = 0.0;
            last_err_x = 0.0;
            last_err_y = 0.0;
        }
    }

    // 当 x y方向位置偏差小于阈值时(阈值与高度相关)，逐渐降落，同时x y方向在继续调整偏差(0.5待调整)
    if (!(fabs(err_x) < 0.5) && !(fabs(err_y) < 0.5))
    {
        vs_body_axis.twist.linear.x = 0.8 * (err_x * xyP + (err_x - last_err_x) / dt * xyD);
        vs_body_axis.twist.linear.y = 0.8 * (err_y * xyP + (err_y - last_err_y) / dt * xyD);
        vs_body_axis.twist.linear.z = 0;
    }
    else 
    {
        vs_body_axis.twist.linear.x = err_x * xyP + (err_x - last_err_x) / dt * xyD;
        vs_body_axis.twist.linear.y = err_y * xyP + (err_y - last_err_y) / dt * xyD;
        vs_body_axis.twist.linear.z = -0.25;
    }

    // 由于 x y方向的 err值与高度相关，当高度很小时，err很小，造成控制量很小，因此加大控制的 P 值
    if(uav_altitude < 1.0) 
    {
        // PD控制(x y方向)
        vs_body_axis.twist.linear.x = err_x * xyP;              
        vs_body_axis.twist.linear.y = err_y * xyP;
    }
   
    // 速度限幅
    vs_body_axis.twist.linear.x = (vs_body_axis.twist.linear.x>0.8)?0.8\
                                  :((vs_body_axis.twist.linear.x<-0.8)?-0.8:vs_body_axis.twist.linear.x);
    vs_body_axis.twist.linear.y = (vs_body_axis.twist.linear.y>0.8)?0.8\
                                  :((vs_body_axis.twist.linear.y<-0.8)?-0.8:vs_body_axis.twist.linear.y);
    vs_body_axis.twist.linear.z = (vs_body_axis.twist.linear.z>0.5)?0.5\
                                  :((vs_body_axis.twist.linear.z<-0.5)?-0.5:vs_body_axis.twist.linear.z);  

    // 更新偏差值 
    last_err_x = err_x;
    last_err_y = err_y;
    last_timestamp = ros::Time::now().toSec();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "land_on_apriltag");
    ros::NodeHandle nh;
	
	// arm 与 disarm service 使用
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false; // 未解锁

    fd = open_log_file();

    int flag_low_altitude = 0;
    int flag_time = 0;
    int flag_time1 = 0;

    last_timestamp = ros::Time::now().toSec();
    ros::Time time_disarm(0);
    ros::Time time_land(0);

    ROS_INFO("------------landing control node running successfully-------------\n");

    // pub vel_sp
    ros::Publisher bodyAxisVelocityPublisher = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    // sub flight mode
    ros::Subscriber stateSubscriber = nh.subscribe("mavros/state", 10, stateReceived);

    // sub uav altitude
    ros::Subscriber uavAltSubscriber = nh.subscribe("/mavros/px4flow/ground_distance", 100, uavAltReceived);

    // sub uavpose
    ros::Subscriber uavPoseSubscriber = nh.subscribe("/mavros/local_position/pose", 1000, uavPoseReceived);

    // sub tag
    //ros::Subscriber TagDetectionsSubscriber = nh.subscribe("/apriltags/detections", 5, TagDetectionsReceived);  
    ros::Subscriber TagDetectionsSubscriber = nh.subscribe("/apriltags/relative_position", 20, TagDetectionsReceived);  

    // client arm/disarmed
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    // client set_mode
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    last_err_x = 0;
    last_err_y = 0;
    last_err_z = 0;
    //last_err_yaw = 0;
    // cout<<"main in errxy: "<<err_x<<endl;
    // 获取 PID 参数值
    ros::param::get("~xyP",xyP);
    ros::param::get("~xyD",xyD);

    // ros::param::param("~xyP", xyP, 0.3);
    // ros::param::param("~xyD", xyD, 0.2);
    cout << "got xyP = " << xyP << endl;
    cout << "got xyD = " << xyD << endl;

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    ros::Rate loopRate(50.0);
    //set mode offboard
    while(ros::ok()){
 
        if(flag_offboard_mode)
        {
            // 高度大于0.35m，正常控制飞行
            if(uav_altitude >= 0.35)
            {             
                // 切换到 offboard 模式，且未进入位置保持状态
                if(!flag_enter_position_hold)
                {
                    landingVelocityControl();   
                    //ROS_INFO_STREAM("Offboard auto control");     
                }
                // 在 offboard 模式下，进入位置保持状态, 未进入手动调节模式
                else
                {
                    ROS_INFO_STREAM("Position hold control");
                    vs_body_axis.header.seq++;
                    vs_body_axis.header.stamp = ros::Time::now();
                    vs_body_axis.twist.linear.x = 0.0;
                    vs_body_axis.twist.linear.y = 0.0;
                    vs_body_axis.twist.linear.z = 0.0;
                    vs_body_axis.twist.angular.z = 0.0;
                       
                    if (flag_time1 == 0)
                    {        
                        time_land = ros::Time::now();
                        flag_time1 = 1;
                    }         

                    if (ros::Time::now() - time_land > ros::Duration(5.0))
                    {
                        if( set_mode_client.call(land_set_mode) &&
                            land_set_mode.response.mode_sent){
                            ROS_INFO("AUTO LAND enabled");
                        }else{
                            ROS_INFO_STREAM("Fail to change mode");
                        }
                    }                    
                }

                flag_low_altitude = 0;
            }

            // 高度小于 0.35m，继续降落，1s 后飞机锁定(防止超声波数据出错，要求低高度预警连续触发3次)
            else
            {   
                ROS_INFO_STREAM("Almost landed");
                flag_low_altitude++;
                if(flag_low_altitude > 2)  
                {   
                    flag_low_altitude = 3;
                    vs_body_axis.header.seq++;
                    vs_body_axis.header.stamp = ros::Time::now();
                    vs_body_axis.twist.linear.x = 0;
                    vs_body_axis.twist.linear.y = 0;
                    vs_body_axis.twist.linear.z = -0.1;

                    if (flag_time == 0)
                    {        
                        time_disarm = ros::Time::now();
                        flag_time = 1;
                    }

                    if ((ros::Time::now() - time_disarm) > ros::Duration(1.0))
                    {
                        ROS_INFO_STREAM("getting here");
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_INFO("Vehicle disarmed");
                            return 0;
                        }else{
                            ROS_INFO_STREAM("that's should not happen");
                        }
                    }
                }
            }
        }
        else
        {
            //ROS_INFO_STREAM("Offboard not running");
        }

        // 此句为测试代码，不用 arm 飞机，直接看速度控制输出量 
        //landingVelocityControl();

        //发布速度控制量
        bodyAxisVelocityPublisher.publish(vs_body_axis);
      
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}

