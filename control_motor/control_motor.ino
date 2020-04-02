// ros library
#include <ros.h>
#include <std_msgs/Int16>
// include the AccelStepper library:
#include <AccelStepper.h>

// left stepper pin definitions
#define stepLPin1 8     //IN1
#define stepLPin2 9     //IN2
#define stepLPin3 10    //IN3
#define stepLPin4 11    //IN4
// right stepper pin definitions
#define stepRPin1 8     //IN1
#define stepRPin2 9     //IN2
#define stepRPin3 10    //IN3
#define stepRPin4 11    //IN4

// define max step per second of stepper motor
#define MAX_SPEED 1000

// define AccelStepper interface
#define MotorInterfaceType 8

// initialize 
AccelStepper stepL = AccelStepper(MotorInterfaceType, stepLPin1, 
                                    stepLPin2, stepLPin3, stepLPin4);
AccelStepper stepR = AccelStepper(MotorInterfaceType, stepRPin1, 
                                    stepRPin2, stepRPin3, stepRPin4);

// initialize ros
ros::NodeHandle nh;
int spl,spr;
void speedLeftSub(const std_msgs::Int16& msg)
{
    spl = msg->data;
}

void speedRightSub(const std_msgs::Int16& msg)
{
    spr = msg->data;
}

ros::Subscriber<std_msgs::Int16> spl_sub("/speed_left",&speedLeftSub);
ros::Subscriber<std_msgs::Int16> spr_sub("/speed_right",&speedRightSub);

void setup()
{
    // init node and subscribe topic
    nh.initNode();
    nh.subscribe(spl_sub);
    nh.subscribe(spr_sub);

    // set max steps per second
    stepper.setMaxSpeed(MAX_SPEED);
}

void loop()
{
    // set speed and run
    stepL.setSpeed(spl);
    stepR.setSpeed(spr);
    stepL.runSpeed();
    stepR.runSpeed();
    
    nh.spinOnce();
    delay(10);
}