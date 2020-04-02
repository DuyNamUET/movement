## Movement
Type of motion: Two wheeled differential drive <br>
Motor type: Stepper motor <br>
Stepper motor name: 28BYJ-48 <br>
Driver: ULN2003 <br>
Power input: 11.1V <br>
Max speed of stepper in this input about 20-25rpm (need to check) <br>

## Step to step
#### Control Stepper Motor with Arduino <br> 
[28BYJ-48 Stepper Motor with ULN2003 Driver and Arduino Tutorial](https://www.makerguides.com/28byj-48-stepper-motor-arduino-tutorial/)
#### ROS Arduino <br>
[Installation](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) <br>
[Tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials) <br>
#### Control Stepper Motor with ROS Arduino <br>
Download and install in your workspace<br>
```
$ cd [your_ws]/src
$ git clone https://github.com/DuyNamUET/movement
$ cd ..
$ catkin_make
```
Run
```
$ roslaunch movement run_motor.launch
```