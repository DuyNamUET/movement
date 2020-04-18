#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

// define some specification of robot
const float PI = 3.14159265;
const int MAX_SPEED = 1000;             // step per second
const int MAX_W = 20;                   // rpm
const float R = 0.06;                   // radius of wheel
const float MAX_VEL = R*MAX_W*2*PI/60;  // m/s (max velocity of robot)

geometry_msgs::Twist vel;
void robotSetVelSub(const geometry_msgs::Twist::ConstPtr& msg);
void convertVel2Speed(const geometry_msgs::Twist& robot_vel, std_msgs::Int16& spl, std_msgs::Int16& spr);

ros::Time vel_time;

int main(int argc, char**argv)
{
    // initialize ros node
    ros::init(argc, argv, "convert2speed");
    ros::NodeHandle nh;
    
    // subscribe velocity of robot
    ros::Subscriber sub_set_vel = nh.subscribe("/robot/cmd_vel", 100, &robotSetVelSub);

    // publish speed of each motor
    ros::Publisher pub_speed_left = nh.advertise<std_msgs::Int16>("/speed_left", 100);
    ros::Publisher pub_speed_right = nh.advertise<std_msgs::Int16>("/speed_right", 100);
    
    ros::Rate r(1000);
    while (nh.ok())
    {
        std_msgs::Int16 spl, spr;
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
    vel_time = ros::Time::now();
    vel = *msg;
    return;
}

void convertVel2Speed(const geometry_msgs::Twist& robot_vel, std_msgs::Int16& spl, std_msgs::Int16& spr)
{
    ros::Time now = ros::Time::now();
    float l_x = robot_vel.linear.x;
    float a_z = robot_vel.angular.z;
    if((now-vel_time).toSec() > 0.5)
    {
        l_x = a_z = 0.0;
    }
    // caculate velocity of each wheel
    float vl = l_x - a_z;
    if(vl > MAX_VEL) vl = MAX_VEL;
    else if(vl < -MAX_VEL) vl = -MAX_VEL;
    
    float vr = l_x + a_z;
    if(vr > MAX_VEL) vr = MAX_VEL;
    else if(vr < -MAX_VEL) vr = -MAX_VEL;
    
    // convert to speed motor (step per second)
    spl.data = int(vl/R*60/(2*PI)/MAX_W*MAX_SPEED);
    spr.data = int(vr/R*60/(2*PI)/MAX_W*MAX_SPEED);
    
    return;
}