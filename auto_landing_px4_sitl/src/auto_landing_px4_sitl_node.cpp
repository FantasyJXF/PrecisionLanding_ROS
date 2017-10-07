//visual-flow-landing
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

using namespace std;

geometry_msgs::TwistStamped vs_body_axis;
//geometry_msgs::PoseStamped uavPose;
apriltags_ros::AprilTagDetectionArray apriltag_pos_msg;
mavros_msgs::State current_state;

//double uavRollENU, uavPitchENU, uavYawENU;

double err_x, err_y, err_z;
double last_err_x, last_err_y, last_err_z;
double last_timestamp;

double xyP, xyI, xyD, zP, zI, zD, yawP, yawD;
double dt;

float uav_init_altitude = 0.0;
float uav_altitude = 0.0;     
float uav_x_distance = 0.0;
float uav_y_distance = 0.0;

bool flag_enter_position_hold = false;

void stateReceived(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//obtain the apriltags pose
void TagDetectionsReceived(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{   static int flag_not_found_mark = 0;
     // 获取无人机相对apriltag的xy距离
    apriltags_ros::AprilTagDetection tag_msg = msg->detections[0];
    uav_x_distance = tag_msg.pose.pose.position.x;
    uav_y_distance = tag_msg.pose.pose.position.y;
    uav_altitude = tag_msg.pose.pose.position.z;
    cout<<"x: "<<uav_x_distance<<"y: "<<uav_y_distance<<endl;
    if (abs(uav_x_distance) > 0.01f && abs(uav_y_distance) > 0.01f) 
    {   // 将无人机与mark在 x y z 方向的距离偏差，分别表示为err_ ,便于控制部分的理解
        // x轴反向，y,z轴重合
        err_x = uav_x_distance;
        err_y = -uav_y_distance;
        err_z = -uav_altitude;
        //一旦发现标志，将未发现mark的计数标志复位0
        flag_not_found_mark = 0;
        cout<<"if value errxy"<<endl;
    } 
    
    // 说明： 1.8m 只是尝试值，明显有点大
    // offboard 模式下，高度大于1.8m？，连续 10次 未发现标志，认为目标丢失，自动进入定点悬停模式
    // 高度小于1.8m？，由于 Tag 占图像大部分，飞机晃动，Tag很容易出视野，识别失败，此时保持继续降落
    else    
    {
        flag_not_found_mark++;
        if (uav_altitude > 1.8 && (flag_not_found_mark > 10))
        {           
           ROS_INFO_STREAM("Fail to found mark, enter Position Hold");
           flag_enter_position_hold = true;
        }
        if(uav_altitude < 1.8 && (flag_not_found_mark > 10))
        {
            err_x = 0.0;
            err_y = 0.0;
            last_err_x = 0.0;
            last_err_y = 0.0;
        }
    }
    cout<<"TagDetectionsReceived terrxy: "<<err_x<<endl;
}



// 飞机降落速度控制
void landingVelocityControl()
{
    vs_body_axis.header.seq++;
    vs_body_axis.header.stamp = ros::Time::now();
cout<<"landingvelocitycontrol errxy: "<<err_x<<" "<<err_y<<endl;
    dt = ros::Time::now().toSec() - last_timestamp;
    //velocity_z set as a constant // PD控制(x y方向)
    vs_body_axis.twist.linear.x = err_x * xyP + (err_x - last_err_x) / dt * xyD;
    vs_body_axis.twist.linear.y = err_y * xyP + (err_y - last_err_y) / dt * xyD;
    cout<<vs_body_axis.twist.linear.x<<"  "<< vs_body_axis.twist.linear.y<<endl;
/*    vs_body_axis.twist.linear.x = err_x * xyP;
    vs_body_axis.twist.linear.y = err_y * xyP;*/
    vs_body_axis.twist.linear.z = -0.35;//should it be different?
   
    // 速度限幅
    if(vs_body_axis.twist.linear.x > 0.8)
        vs_body_axis.twist.linear.x = 0.8;
    if(vs_body_axis.twist.linear.x < -0.8)
        vs_body_axis.twist.linear.x = -0.8;   
    if(vs_body_axis.twist.linear.y > 0.8)
        vs_body_axis.twist.linear.y = 0.8;
    if(vs_body_axis.twist.linear.y < -0.8)
        vs_body_axis.twist.linear.y = -0.8;  
    if(vs_body_axis.twist.angular.z > 0.4)
        vs_body_axis.twist.angular.z = 0.4;
    if(vs_body_axis.twist.angular.z < -0.4)
        vs_body_axis.twist.angular.z = -0.4;   

    // 更新偏差值 
    last_err_x = err_x;
    last_err_y = err_y;
    last_timestamp = ros::Time::now().toSec();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_marker_land_node");
    ros::NodeHandle nh;

    printf("------------landing control node running successfully-------------\n");

    ros::Publisher bodyAxisVelocityPublisher = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber stateSubscriber = nh.subscribe("mavros/state", 10, stateReceived);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Subscriber TagDetectionsSubscriber = nh.subscribe("/tag_detections",1,TagDetectionsReceived);  
    //ros::Subscriber uavPoseSubscriber = nh.subscribe("/mavros/local_position/pose", 1000, uavPoseReceived);
     //ros::Publisher initial_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    last_err_x = 0;
    last_err_y = 0;
    last_err_z = 0;
    //last_err_raw = 0;
    cout<<"main in errxy: "<<err_x<<endl;
    // 获取 PID 参数值
    ros::param::param("~xyP", xyP, 0.2);
    ros::param::param("~xyD", xyD, 0.3);
    cout << "got xyP = " << xyP << endl;
    cout << "got xyD = " << xyD << endl;

    ros::Rate loopRate(20.0);
//setpoint_velocity
    //wait for FCU connection// This loop should exit as soon as a heartbeat message is received.
    // while(ros::ok() && current_state.connected){
    //     ros::spinOnce();
    //     loopRate.sleep();
    // }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
      
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    //set mode offboard
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                //offb_set_mode.response.mode_sent){
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else{
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
          // 高度大于1.1m，vel_x & vel_y are PD control
        if(!flag_enter_position_hold && uav_altitude >= 1.1){
            cout<<"......"<<endl;
            landingVelocityControl();  
            ROS_INFO_STREAM("Offboard auto landing");     
        }
        else{
               vs_body_axis.header.seq++;
               vs_body_axis.header.stamp = ros::Time::now();
/*             vs_body_axis.twist.linear.x = 0;
               vs_body_axis.twist.linear.y = 0;*/
               vs_body_axis.twist.linear.z = 0;
               
        }               
      //发布速度控制量
        bodyAxisVelocityPublisher.publish(vs_body_axis);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}


