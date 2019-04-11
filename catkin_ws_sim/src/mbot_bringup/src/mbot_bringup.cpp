#include "mbot_bringup/mbot.h"

double RobotV_  = 0;  //线速度回调
double YawRate_ = 0;  //角速度回调

// 速度控制消息的回调函数
void cmdCallback(const geometry_msgs::Twist& msg)
{
    RobotV_  = msg.linear.x * 1000;
    YawRate_ = msg.angular.z;
}
    

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mbot_bringup");									
    ros::NodeHandle nh;
    
    //初始化mbot类
    mbot::mbot robot;

    if(!robot.init())
    {
        ROS_ERROR("mbot initialized failed.");
    }
    
    ROS_INFO("mbot initialized successful.");
    
    ros::Subscriber sub = nh.subscribe("cmd_vel", 50, cmdCallback);

    //运行频率50Hz
    ros::Rate r(50);

    while (ros::ok()) 
    {
       ros::spinOnce();
       //机器人运动控制
       robot.spinOnce(RobotV_, YawRate_);

       r.sleep();
    }

     return 0;
}







