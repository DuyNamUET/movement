#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "omni_wheel_ros/SpeedMotor.h"

// define some specification of robot
#define VELMAX 12   // speed max of motor
#define L 10        // distance from center robot to center wheel
#define R 4.8/2     // radius of omni wheel

geometry_msgs::Twist vel;
void robotSetVelSub(const geometry_msgs::Twist::ConstPtr& msg);
omni_wheel_ros::SpeedMotor convertVel2Speed(const geometry_msgs::Twist& robot_vel);

int main(int argc, char**argv)
{
    // initialize ros node
    ros::init(argc, argv, "convert2speed");
    ros::NodeHandle nh;
    
    // subscribe velocity of robot
    ros::Subscriber sub_set_vel = nh.subscribe("/robot/cmd_vel", 1000, &robotSetVelSub);

    // publish speed of each motor
    ros::Publisher pub_speed = nh.advertise<omni_wheel_ros::SpeedMotor>("/speed_motor", 1000);  
    
    ros::Rate r(1000);
    while (nh.ok())
    {
        // convert robot's vel to speed of each motor
        omni_wheel_ros::SpeedMotor sp = convertVel2Speed(vel);
        pub_speed.publish(sp);
        r.sleep();
        ros::spinOnce();
    }
    
    return 0;
}

void robotSetVelSub(const geometry_msgs::Twist::ConstPtr& msg)
{
    vel = *msg;
}

omni_wheel_ros::SpeedMotor convertVel2Speed(const geometry_msgs::Twist& robot_vel)
{
    // get value from vel
    float l_x = robot_vel.linear.x;
    float l_y = robot_vel.linear.y;
    float a_z = robot_vel.angular.z;

    // convert 2 velocity of each motor
    float v1 = 2*l_y/3 + L*a_z/3;
    float v2 = -l_x/sqrt(3) - l_y/3 + L*a_z/3;
    float v3 = l_x/sqrt(3) - l_y/3 + L*a_z/3;

    // convert 2 speed of each motor (0-255)
    omni_wheel_ros::SpeedMotor sp;
    sp.speed1 = 255*v1/VELMAX/R;
    sp.speed2 = 255*v2/VELMAX/R;
    sp.speed3 = 255*v3/VELMAX/R;
    return sp;
}