#include <ros.h>
#include <omni_wheel_ros/SpeedMotor.h>

ros::NodeHandle nh;
void speedMotorSub(const omni_wheel_ros::SpeedMotor& msg)
{
    // do something here
}

ros::Subscriber<omni_wheel_ros::SpeedMotor> speed_sub("/speed_motor",&speedMotorSub);

void setup()
{
    nh.initNode();
    nh.subscribe(speed_sub);
}

void loop()
{
    nh.spinOnce();
    delay(10);
}