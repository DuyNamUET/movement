#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8>

// define some specification of robot


geometry_msgs::Twist vel;
void robotSetVelSub(const geometry_msgs::Twist::ConstPtr& msg);
void convertVel2Speed(const geometry_msgs::Twist& robot_vel, int& spl, int& spr);

int main(int argc, char**argv)
{
    // initialize ros node
    ros::init(argc, argv, "convert2speed");
    ros::NodeHandle nh;
    
    // subscribe velocity of robot
    ros::Subscriber sub_set_vel = nh.subscribe("/robot/cmd_vel", 1000, &robotSetVelSub);

    // publish speed of each motor
    ros::Publisher pub_speed_left = nh.advertise<std_msgs::Int8>("/speed_left", 100);
    ros::Publisher pub_speed_right = nh.advertise<std_msgs::Int8>("/spped_right", 100);
    
    ros::Rate r(1000);
    while (nh.ok())
    {
        int spl, spr;
        // convert robot's vel to speed of each motor
        convertVel2Speed(vel, spl, spr);
        pub_speed_left.publish(spl);
        pub_speed_right.publish(spr);
        r.sleep();
        ros::spinOnce();
    }
    
    return 0;
}

void robotSetVelSub(const geometry_msgs::Twist::ConstPtr& msg)
{
    vel = *msg;
    return;
}

void convertVel2Speed(const geometry_msgs::Twist& robot_vel, int& spl, int& spr)
{
    return;
}