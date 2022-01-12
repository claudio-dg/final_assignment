Research Track Final Assignment
================================

This repository contains the result of my personal work for the final Assignment of the course.
The goal of this assignment is to Develop a software architecture for the control of the robot in the environment. The software will rely on the move_base
and gmapping packages for localizing the robot and plan the motion.
The architecture should be able to get the user request, and let the robot execute one of the following behaviors
(depending on the user’s input):

1. autonomously reach a x,y coordinate inserted by the user.
2. let the user drive the robot with the keyboard
3. let the user drive the robot assisting them to avoid collisions
4. autonomously cancel a goal if it is not reachable using a timeout
5. in addition we could add the functionality of manually canceling a given goal

To do this we had to use ROS for controlling the robot and Gazebo and RViz environments.
I decided to use C++ as programming language.



Table of contents
----------------------

* [Setup](#setup)
* [Gazebo and Rviz Maps](#gazebo-and-rviz-maps)
* [Project structure and behaviour description](#project-structure-and-behaviour-description)
* [PseudoCode](#pseudocode)


## Setup

This repository Contains all the useful files to run the script that i produced for this assignment.
To try it, it is sufficient to clone this repository in your ROS workspace: 

```bash
$ git clone https://github.com/claudio-dg/final_assignment.git
```

and then type the following command in the terminal to simultaneously launch all the necessary nodes through the **"launchFile"**:

```bash
$ roslaunch final_assignment final.launch

```
This Launch File has been made to make it easier to run the project , but if you like you can manually run every part of the project by launching the following launch files:

```bash
$ roslaunch final_assignment move_base.launch
$ roslaunch final_assignment simulation_gmapping.launch

```
To run the simulation environment and the move_base functions.

```bash
$ roslaunch final_assignment teleop.launch
$ roslaunch final_assignment my_scripts.launch
```
To run the teleop_Twist_keyboard node and my scripts produced for this assignment.

## Gazebo and Rviz Maps

The environment used for this assignment consists in the map illustrated (froma Gazebo view) in the following image:

<p>
<img src="https://github.com/claudio-dg/final_assignment/blob/main/images/Gazebo.png?raw=true" width="450" />
<p>

Rviz instead gives another point of view of the same environment, that is from robot sensors' point of view: the robot, in fact, does not know from the beginning the full map he's in, but thanks to the laser sensors and the ```gmapping``` package he is capable of creating it.

<p>
<img src="https://github.com/claudio-dg/final_assignment/blob/main/images/Rviz.png?raw=true" width="400"/>
<p>
	
## Project structure and behaviour description

The project is based on the ROS scheme that is shown in the following graph:

<p align="center">
<img src="https://github.com/claudio-dg/final_assignment/blob/main/images/final_assign_rosgraph.png?raw=true" width="900" height="200" />
<p>
 
The ROS package of the project is called ```"final_assignment"```, it exploits two already given packages: ```slam_gmapping```, which opens the environment and allows the robot to create a map of what sorrounds him, and ```move_base```, which requires a goal to be sent to the topic ```move_base/goal``` in order to make the robot move towards it.
In addition to this i created two nodes contained in ```src``` folder named ```InputConsole``` and ```controller```; as the name suggests the first one is encharged of taking user's inputs to select the desired behaviour of the robot, while the second one manages the consequences of user's request by communicating with other nodes, for instance by sending the goal coordinates to ```move_base/goal``` with a msg of type :```move_base_msgs/MoveBaseActionGoal```.
The communication between this two nodes is implemented through a publish/subscribe structure using two different topics ```MY_topic_teleop```  & ```MY_topic_send_goal```: in this way I made a structure in which the input given by the user determines which callback is going to be called in the controller node, so that the "async structure" required by this assignment was possible.
	
- Regarding point 1) I used the ```\move_base\feedback``` topic to retreive information about robot's status such as the current position or the time counter: thanks to these two pieces of information I implemented an algorithm to state whether the goal was reached or not (considering an approximation error due to the fact that the robot seemed to get really close to the goal but never reaching its exact coordinates), and a TIMEOUT, so that if the robot doesen't reach the goal in Time it is considered unreachable and will be canceled by sending a msg to ```\move_base\cancel``` topic
		
- Regarding points 2) and 3) of the assignment I remapped an already existing topic (```teleop_twist_keyboard```) so that instead of publishing directly on ```cmd_vel``` it publishes on my personal topic ```myRemapped_cmd_vel```: by doing this I manage to consider the velocities published by this topic only when required, that is when the user selected mode 2) or 3), furthermore it allowed me to add the collision avoidance functionality needed for the third part of the assignment. 


 ### Behaviour description  : ### 

After having launched all the required launch files Gazebo and Rviz environments will open, along with 3 different terminals:
* ```Input Console``` : in which you can select what to do and that will show the following user interface:
```bash
***********THIS IS THE INPUT CONSOLE***********

Which Action do you want to use to move the robot?
ENTER 'c' to cancel last given goal
ENTER '1' to send a new goal to the robot
ENTER '2  to manually drive the robot
ENTER '3' to manually drive the robot WITH assisted collision avoidance
ENTER 'q' to terminate this node
```

* ```Controller Console``` : that will show some useful real-time info depending on the modality selected in the input console, such as the elapsed time since the goal was given or some notifications to inform the user tha a certain direction will probably cause a collision.
* ```TeleopTwist Keyboard Console``` : in which the user can insert commands to manually drive the robot that will only be read if modality 2) or 3) were previously selected through the input console. 
 

	
	
 ## Pseudocode
 
 To reproduce the behaviour previously described i wrote 3 C++ programms contained in the ```src``` folder:
 - controller.cpp 
 - server.cpp
 - input_console.cpp
	
### Controller.cpp  : ###	

 The ```main``` of this script is simply the following:
```bash
int main (int argc, char **argv)
{ 
ros::init(argc, argv, "controller"); 
ros::NodeHandle nh;
ros::Subscriber sub = nh.subscribe("/base_scan", 1, LasersCallback); 
pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
ros::ServiceServer service= nh.advertiseService("/updatevel", Servicecallback);
ros::spin();
return 0;
}
```
Here we the have initialization of the node and the susbcription to the  topic ```/base_scan```, along with the definition of the publisher on ```/cmd_vel``` topic and of the server for the ```/UpdateVel``` service . 
I had to implement two different callback functions: ```LasersCallback``` &  ```Servicecallback```.

The first one is based on feedbacks received from robot'lasers:
```bash
void LasersCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
 for(int i=0; i<=720;i++)
 {
  ranges_array[i] = msg->ranges[i]; 
 }
 float right_dist = GetMinDistance(20,120, ranges_array);
 float left_dist = GetMinDistance(600,700, ranges_array);
 float frontal_dist = GetMinDistance(300,420,ranges_array);
//if close to a FRONTAL wall
 if(frontal_dist<1.5) 
 {
 	if(right_dist<left_dist) 
 	 Turn_left();
 	else
 	 Turn_right();
 }
	else 
 	Move_forward();
 pub.publish(my_vel); 
 if(previous_vel != my_vel.linear.x){
 system("clear");
 printf("\n velocita attuale e'  %f  [variazione totale = %f]\n", my_vel.linear.x, variation );
 printf("\n velocita angolare attuale e'  %f\n", my_vel.angular.z );
 previous_vel = my_vel.linear.x;
 }
}
```
It is a pretty simple function that takes information from laser scanners by reading their ```ranges[]``` parameter, and checks for the closest wall in 3 different directions with an algorithm similar to the one of the first assignnment. I divided the range of detection in 3 parts: one for the front side of the robot and the others for its left and right; this function calculates the minimum distance to a wall for each part thanks to a function that i called ```GetMinDistance``` which is defined as follows:
	
```bash
 float GetMinDistance(int min_index,int max_index, float ranges_array[])
{
	float min_distance = 999;
	for(int i = min_index; i <= max_index; i++)
	{
		if(ranges_array[i] < min_distance)
			min_distance = ranges_array[i];
	}
	return min_distance;
}
```
 After doing this the robot moves forward if there are no walls in front of him. otherwise it curves a bit towards the opposite direction of the closest wall: in order to do this movement i had to publish a message on ```cmd_vel``` topic after having modified the values in these ways:
```bash
	void Move_forward()
{
	my_vel.linear.x = STARTING_VEL + variation; 
	my_vel.angular.z = 0;
}
```
	
```bash
	void Turn_left()
{	
	my_vel.linear.x = 0.8;
 	my_vel.angular.z = 2;
}
```
 Notice that when moving forward the robot has an additional component called "variation": this is the value that is going to be modified through the call to the ```/UpdateVel``` service, and that will modify the current velocity according to user's inputs.
	
So the Callback to this specific service is the following and will simply modify this "variation" value: in addiction to the update of the "variation" variable it only has one "if statement" that makes the robot stop in case the user wrote 's' (that corresponds to the -1 flag value) or in case the total variation would cause the robot to move backwards.
 ```bash
bool Servicecallback (second_assignment::UpdateVel::Request &req, second_assignment::UpdateVel::Response &res)
{
 variation = variation + req.value;
	
 if(variation < -STARTING_VEL or req.value == -1 )
 {
  variation = -STARTING_VEL; //the robot stands still
 }
return true;
}
```

### Server.cpp  : ###
Within the ```main``` of this script we have again the initialization of the node and of the server for the ```/changevel``` service this time:
			     			     
```bash
int main(int argc, char **argv)
{
//initalizing the node and the Service
ros::init(argc, argv, "my_server");
ros::NodeHandle n;
ros::ServiceServer service= n.advertiseService("/changevel", Mycallback);

ros::spin();
return 0;
}
```
The callback of this service (named ```Mycallback```) simply contains a "Switch" statement that, based on the "char" value received as request from the service, puts the correct "float" value in the response, and in case of 'r', calls the already given ```/reset_position``` service through the ```ros::service::call``` 		     
			     
```bash
bool Mycallback (second_assignment::ChangeVel::Request &req, second_assignment::ChangeVel::Response &res)
{
char given_input = req.input;
switch(given_input)
{ 
 case 'r' : //reset -> call /reset_positions service
 	ros::service::call("/reset_positions", my_reset);
 	ROS_INFO("RESET RECEIVED");
 	res.change_value = 0; 
 	break;
 	
 case 's' : //stop -> stop the robot
 	ROS_INFO("STOP RECEIVED");
 	res.change_value = -1; 
 	break;	
 	
 case 'i' :  //increase -> set the response of the service as +0.5
 	ROS_INFO("INCREASE RECEIVED");
 	res.change_value = +0.5;  	
 	break;
 	
 case 'd' : //decrease -> set the response of the service as -0.5
 	ROS_INFO("DECREASE RECEIVED");
 	res.change_value = -0.5;
 	break;
 	
 default :
 	ROS_INFO("WRONG COMMAND");
 	res.change_value = 0;
 	break;
}
return true;
}
```
 ### Input_console.cpp  : ###

The last script is the one that takes input from the user, its main contains the node initialization, the subsciber to ```/base_scan``` topic (used for having a loop as previously said), and the definition of two clients for the two custom services: this way it is capable of calling both of them when required, that is when the user inserts a command in the terminal.
	
```bash	
//starting global definitions 
ros::ServiceClient client1;
ros::ServiceClient client2;
second_assignment::ChangeVel change_vel;
second_assignment::UpdateVel up_vel;
	
	
int main (int argc, char **argv)
{
ros::init(argc, argv, "console");
ros::NodeHandle nh;

ros::Subscriber sub = nh.subscribe("/base_scan", 1, myCallback);
client1 = nh.serviceClient<second_assignment::ChangeVel>("/changevel"); 
client2 = nh.serviceClient<second_assignment::UpdateVel>("/updatevel"); 

ros::spin();
return 0;
}
```	
So "myCallback" as first shows to the user which commands are accepted, then starts waiting for an input with a ```scanf()``` : only when the user presses something on the keyboard this function calls the ```/ChangeVel``` service to receive the corresponding float value to that command, then it calls the ```/UpdateVel``` for notifying the ```Controller``` of the user's request and to actually modify the current speed.

```bash
void myCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
 printf("--- \n PRESS 'r' to reset the robot to the starting position \n PRESS 's' to stop the robot   \n PRESS 'i' to increase velocity  \n PRESS 'd' to decrease velocity \n--- \n");
 scanf(" %c", &input);
 system("clear");

 change_vel.request.input = input;
 client1.waitForExistence(); 
 client1.call(change_vel);
 
 float resp = change_vel.response.change_value;
 up_vel.request.value = resp;
 client2.waitForExistence(); 
 client2.call(up_vel);
}	
```
 * REMARK: within the .cpp files contained in the ```src``` you'll find the whole code introduced in this ```README``` wiht all the "#include" used along with futher explanations through comments in which, for example, I explain more in details the inputs and otputs of every function.
I decided to remove the major part of the comments from the bodies of the functions reported in this README in order to avoid weighting too much its reading.
 
 
	

	

 
 
 
 
 

