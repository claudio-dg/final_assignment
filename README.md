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

This repository Contains all the useful files to run the scripts that i produced for this assignment.
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
 
 To reproduce the behaviour previously described i wrote 2 C++ programms contained in the ```src``` folder:
 - input_console.cpp
 - controller.cpp 

### Input_console.cpp  : ###


```bash
initialize node
initialize necessary publishers and subscribers
Use continuous callback from clock topic
	
	print User Interface menu
	wait for a keyboard input and put it into a variable "input"
	switch(input)
	    case 'c':
	    	publish on move_base/cancel topic to cancel last given goal
		publish a flag msg on MY_topic_send_goal to reset variables in controller
		print that goal has been canceled
		break;
	    case '1':
		print autonomous driving introduction
                ask for goal coordinates
	    	read goal coordinates from the user
                publish them on MY_topic_send_goal topic to notify the controller
		print sent coordinates
	    	break;
	    case '2':
	    	print manual driving introduction
		print a message to tell the user to look at teleopkeyboard console
		publish a boolean msg equal to 0 on MY_topic_teleop to inform the controller that manual driving was selected
		break;           
	    case '3':
	    	print ASSISTED manual driving introduction
		print a message to tell the user to look at teleopkeyboard console
		publish a boolean msg equal to 1 on MY_topic_teleop to inform the controller that ASSISTED manual driving was selected
		break;
	    case 'q':
		print an exiting message
		exit the program
	    default:
	    	print an error for a wrong command inserted
	    	break;
	
```
	
### Controller.cpp  : ###	

```bash
initialize node
initialize necessary publishers and subscribers
print controller console introduction	
	//the MAIN loops with ros::SpinOnce whilethe program isn't killed
	//check for global flag values
	
	if user asked for manual drive
		take velocity from teleopKeyboard contained into myRemapped_cmd_vel topic
		publish it on cmd_vel topic to make the robot move
	if user asked for ASSISTED manual drive
		call assistedMovement function**
	if user didn't ask for these 2 modalities, wait for other callbacks to be called
	
	//**assitedMovement function:
		check distances received from laser sensors
		if there is an obstacle in front of the robot
			check for nearest obstacles at his sides
			if nearest obstacle is at his right
				turn left a bit
				print feedback to the user
			if nearest obstacle is at his left
				turn right a bit
				print feedback to the user
			set flag changedVel to 1 to state that the direction has been modified
			sleep 0.3 seconds
		if the direction has been modified
			stop the robot
		else
			clear the console
			take velocity from teleopKeyboard contained into myRemapped_cmd_vel topic
			publish it on cmd_vel topic to make the robot move
	
//basing on which msg is received from input cosole, different callbacks are executed
	if 'c' or '1' were inserted in input cosole  "myCallback" is executed
		
		reset manual drive global flag value
		put goal coordinates in a global variable
		if msg contains flag value for "goal canceled"
			clear the terminal
			print that goal has been canceled
			set "canceled" global flag value to 1
		else
			reset global flags variables
			publish goal on  move_base/goal topic
			print that goal has been published
	return
	
	if '2' or '3' were inserted in input cosole "TeleopCallback" is executed
		
		put msg's boolean value in the global flag variable "manualdrive"
		cancel possibly existing goals
		reset velocities to 0
		if msg asked for manual drive
			print manual driving introduction
			print a message to tell the user to look at teleopkeyboard console
		else if msg asked for ASSISTED manual drive
			print ASSISTED manual driving introduction
			print a message to tell the user to look at teleopkeyboard console
	
//last 2 callbacks (myCmdCallback & CurrentPositionCallback) are called respectively when user inserts command on teleopKeyboard console and when the robot status changes
	
	//myCmdCallback
	put the received vel in a global variable
	reset changedVel flag each time a new command is inserted
	
	
	//CurrentPositionCallback
	execute the callback only if goal wasn't reached yet (goal_reached==0)
	take current position coordinates from /move_base/feedback topic
	if the status has changed just now (firstTime==1)
		take current time as Starting time
		reset firdttime flag value
	keep updating current time
	compute elapsed time since the goal was given
	print info about goal coordinates and time elapsed
	compute Error between current position and Goal position
	if error is small enough
		set goal_reached flag to TRUE
	if the timeout is over and the goal hasn't been reached
		cancel the goal 
		print that goal has been canceled
	else if  goal has been reached before timeout was over
		clear console
		print that goal was reached
```



 


 
	

	

 
 
 
 
 

