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
4. in addition we could add the functionality of manually canceling a given goal

To do this we had to use ROS for controlling the robot and Gazebo and RViz environments.
I decided to use C++ as programming language.



Table of contents
----------------------

* [Setup](#setup)
* [Gazebo and Rviz Maps](#gazebo-and-rviz-maps)
* [Project structure and behaviour description](#project-structure-and-behaviour-description)
* [PseudoCode](#pseudocode)

* [](#)


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

<p align="center">
<img src="https://github.com/claudio-dg/final_assignment/blob/main/images/Gazebo.png?raw=true" width="450" />
<p>

Rviz instead gives another point of view of the same environment, that is from robot sensors' point of view: the robot, in fact, does not know from the beginning the full map he's in, but thanks to the laser sensors and the ```gmapping``` package he is capable of creating it.

<p align="center">
<img src="https://github.com/claudio-dg/final_assignment/blob/main/images/Rviz.png?raw=true" width="400"/>
<p>
	
## Project structure and behaviour description

The project is based on the ROS scheme that is shown in the following graph:

<p align="center">
<img src="https://github.com/claudio-dg/final_assignment/blob/main/images/final_assign_rosgraph.png?raw=true" width="900" height="200" />
<p>
 
The ROS package of the project is called ```"final_assignment"```, it exploits two already given packages: ```slam_gmapping```, which open the environment and allows the to create a map of what sorrounds him, and ```move_base```, which requires a goal to be sent to the 
topic ```move_base/goal``` in order to make the robot move towards it.
In addition to this i created two nodes contained in ```src``` folder named ```InputConsole``` and ```controller```; as the name suggests the first one is encharged of taking user's inputs to select the desired behaviour of the robot, while the second one manages the consequences of user's request by communicating with other nodes, for instance by sending the goal coordinates to ```move_base/goal``` with a msg of type :```move_base_msgs/MoveBaseActionGoal```.
The communication between this two node is implemented through a publish/subscribe structure using two different topics ```MY_topic_teleop```  & ```MY_topic_send_goal```: in this way I made a structure in which the input given by the user determines which callback is going to be called in the controller node, so that the "async structure" required by this assignment was possible.
 1. **/world** : 
 - which was already given and sets the simulation environment. As we can see from the image it publishes on the topic ```/base_scan``` with information regarding robot's lasers scan, and is subscribed to ```/cmd_vel``` topic so that it can receive msgs to set the robot' speed.
2. **/controller_node**	:
- It subscribes to the ```/base_scan``` topic for havig instant information about the environment sorrounding the robot. Then it also implements the server of the custom service **```"/UpdateVel```"** (that simply takes a float value as input and has no response values) for being able of receiving the velocity changes required from the user via input. In the end it publishes robot's speed on the ```/cmd_vel``` topic.
3. **/server_node** :	
- It implements the server of the **custom service ```"/ChangeVel"```** that receives a "char" as input and returns a float value, that is the variation of speed required from that specific input (e.g. 'i' corresponds to +0.5).
4. **/console_node** :	
- This is the input console which subscribes to the ```/base_scan``` topic (*); it calls the custom service ```/ChangeVel``` giving as input the command received from the user, then it communicates the response to the controller node by calling the other custom service ```/UpdateVel``` as previously said, accordingly to the client-server model.

(*) ```REMARK``` : this subscription was simply made for having a continuous callback: it is not actually interested in the messages published in there, but it just uses it as an infinite while loop. 


 ### Behaviour description  : ### 
 
 After running the ROS launchFile the whole project begins to work.
 - First it will open the simulation environment that represents Monza's Circuit:
 
 <p align="center">
<img src="https://github.com/claudio-dg/second_assignment/blob/main/images/Monza_circuit.png?raw=true" width="800"  />
<p>
	
	
 - Then it will run the  ```controller```  : it will make the robot start moving along the circuit with a constant linear velocity, only modyfing it in case of curves for avoiding crashes: when an "obstacle" is met the robot slows down a bit, and it steers in the opposite way of where the nearest wall is with a certain angular velocity, that allows to simulate the real behaviour of a car running in the circuit. When a variation of speed is received (that is when the custom service ```/UpdateVel``` is called), the controller modifies the linear velocity of the robot, also setting it to zero if the user wanted to stop the robot. Note that Robot's velocity won't go under 0: this to avoid the robot going bacwards and crashing.

- On a second terminal the ```server``` starts running: it does nothing until someone (that is the ```input_console```) calls the service ```/ChangeVel```, in that case it will answer with the increment of speed required :+0.5 or -0.5 in case of increment and decrement, -1 in case of 'stop' (this is just a flag to notify that the speed must go to zero). In case of Reset it is encharged of calling an already given service ```/reset_position```, which sets the  robot to the starting position. In this project the reset only changes the position and not the speed, so if you want the robot to stand still in the starting position you'll first have to stop it and then reset. This terminal also prints the command received, so it can be used to have an history of the commands received from the user.
	
- Another terminal will be opened for the  ```input_console```: it will show to the user the commands that he can write, that is:
	* ```r``` to RESET the robot's position
	* ```s``` to STOP the robot
	* ```i``` to INCREASE velocity
	* ```d``` to DECREASE velocity
	
This node does nothing until the user inserts an input: in that case it calls the ```/ChangeVel``` service (**) putting the input in the request, to receive the corresponding value of speed variation as response; in the end it will communicate this response to the controller by calling the ```/UpdateVel``` service to actually modify the current speed.
 
(**) ```REMARK``` : This service ```/ChangeVel``` could be avoided and I could basically have the same behaviour just by making the mapping of the commands directly within the input_console , nevertheless I decided to implement it this way in order to better understand the mechanisms of services and to give more modularity to the project.
	
	
 ## Code explanation
 
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
 
 
	

	

 
 
 
 
 

