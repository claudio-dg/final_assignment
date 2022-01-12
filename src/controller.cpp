#include <ros/ros.h> 

//for taking the elapsed time
#include "chrono"

//for cin >> 
#include "string"

//for /cmd_vel topic
#include "geometry_msgs/Twist.h" 

//for move_base/goal topic
#include "move_base_msgs/MoveBaseActionGoal.h"

//for /move_base/status topic PROVO anceh se action? NIENTE
#include "actionlib_msgs/GoalStatusArray.h"

//for move_base/cancel
#include "actionlib_msgs/GoalID.h"  

//for move_base/feedback topic
#include "move_base_msgs/MoveBaseActionFeedback.h"

//for MY_topic_send_goal
#include <geometry_msgs/Point.h>

//for MY_topic_teleop 
#include <std_msgs/Bool.h> 

//for /scan topic
#include "sensor_msgs/LaserScan.h" 

//TIMER in seconds 
#define TIMEOUT 45 

//the position given by "feedback topic" is almost never exactly the same as the one given in input, so through this closeness error it is possible to estimate if the robot reached the goal or not
#define ERROR 0.3 

//minimum distance for avoiding collisions
#define MINDISTANCE 1


//global definitions
ros::Publisher pub1;
move_base_msgs::MoveBaseActionGoal my_pos; 

ros::Publisher pubCancel;
actionlib_msgs::GoalID my_cancel;

ros::Publisher pubVel;
geometry_msgs::Twist desiredVel;

//flag values
bool goal_reached = 0;
bool canceled_by_user = 0;
bool firstTime = 0;
int manualDrive = 100;
int changedVel = 0;


float x_goal ;
float y_goal ;

ros::Time  currentTime;
ros::Time StartTime;
float errorX;
float errorY;
 

 //to work with lasers
float ranges_array[721]; 
float right_dist =200;
float left_dist = 200;
float frontal_dist = 200;



//global variable for saving teleop input received
geometry_msgs::Twist velFromTeleop;

 
 //CALLBACK for the myRemapped_cmd_vel topic
/* ***************************************************************************/
  void myCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
//put the received vel in a global variable
 velFromTeleop = *msg; 
 //reset flag each time a new command is inserted
 changedVel = 0;
}


float GetMinDistance(int min_index,int max_index, float ranges_array[])
{
	/*
    	 Function for detecting nearest wall in a certain direction.
    	 
    	 INPUT: ranges_array[] -> the array of distances received from LaserScan
    	       min/max_index -> values that allow to consider only portion of the array,
    	                        so that it only detects walls in a certain direction
    	                        
    	 OUTPUT: min_distance -> the min distance from a wall in the specified direction                  
    	*/
	float min_distance = 999;
	for(int i = min_index; i <= max_index; i++)
	{
		if(ranges_array[i] < min_distance)
			min_distance = ranges_array[i];
	}
	return min_distance;
}

//CALLBACK for the /_scan Topic
/* ***************************************************************************/
void LasersCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

//put lasers_scan values into an array
 for(int i=0; i<=720;i++)
 {
  ranges_array[i] = msg->ranges[i]; 
 }
 
 
 //get the min distance from a wall at the RIGHT of the robot
 right_dist = GetMinDistance(20,120, ranges_array);
 
 //get the min distance from a wall at the LEFT of the robot
 left_dist = GetMinDistance(600,700, ranges_array);
 
 //get the min distance forma a wall in FRONT of the robot
 frontal_dist = GetMinDistance(260,460,ranges_array);
}


 void TeleopCallback(const std_msgs::Bool::ConstPtr& msg)
{

int selected_tele = msg->data;

//modify global falg to enter if/else statement in main scope
manualDrive = selected_tele;

//cancel the goal because if the robot is going towards a goal and user selects 2 it stops and waits for a keyboard input
  my_cancel.stamp.sec = 0;
  my_cancel.stamp.nsec = 0;
  my_cancel.id = "";
  pubCancel.publish(my_cancel); 
 
//reset the vel each time teleop is selected to avoid keeping old velocities
velFromTeleop.linear.x = 0; 
velFromTeleop.linear.y = 0;
velFromTeleop.angular.z = 0;

if(selected_tele == 0) //received fully manual
{
 printf("\nRECEIVED REQUEST FOR  FULLY MANUAL DRIVE\n");
 printf("\nPLEASE look at the teleopKeyboard Terminal to move the robot manually");
}
else //received assisted
{
printf("\nRECEIVED REQUEST FOR  ASSISTED DRIVE \n");
printf("\n\nPLEASE look at the teleopKeyboard Terminal to move the robot manually");
printf("\nThe assistance will try to avoid you from crushing");
}
}

  
  
void assistedMovement()
{ 
	/*
    	 Function for assisting manual drive.
    	 
    	 in case it detects a possible collision it will notify the user and deny the movement,
    	 it will repositon the robot in a safier direction and then stop it, waiting for another input of the user.
    	 Otherwise it will simply move the robot as the user asked.              
    	*/
 
  if(frontal_dist<MINDISTANCE)//possible collision detected
 {
 
 //check if the next closest wall is at the RIGHT of the robot turn left and viceversa
 	if(right_dist<left_dist)
 	{
 	 desiredVel.linear.x = 0.3;
 	 desiredVel.angular.z = 2;
 	 printf("\nIf I go this way I will crash :( \n");
 	 printf("\nRepositioning in a better direction... \n");
 	 
 	  
 	 }
 	else
 	{	
 	  desiredVel.linear.x = 0.3;
 	  desiredVel.angular.z = -2;
 	  printf("\nIf I go this way I will crash :( \n");
 	  printf("\nRepositioning in a better direction... \n");
 	   
 	}
   changedVel = 1;
   pubVel.publish(desiredVel);
   
   ros::Duration(0.3).sleep();
   
   
    
 }
//after having modified the direction, stop the robot and wait for another input
 if(changedVel == 1)
   {
   
    desiredVel.linear.x = 0; 
   desiredVel.linear.y = 0;
   desiredVel.angular.z = 0; 
   }
//if position has not been "assisted", keep going with user's inputs
   else 
   {
   system("clear");
   printf("\nFollowing your instructions.. \n");
   desiredVel = velFromTeleop;
   }
   
 
 pubVel.publish(desiredVel);
}





//CALLBACK for the  Topic MY_topic_send_goal
/* ***************************************************************************/
void myCallback(const geometry_msgs::Point::ConstPtr& msg)
{

//reset to avoid robot keeping old vel
manualDrive = 100; 

x_goal = msg->x;
y_goal = msg->y;
//if goal was canceled
if(x_goal == 666)
{
  system("clear");
  printf("\nGOAL CANCELED BY USER\n");
  canceled_by_user = 1;

}
else{


//reset cancel and reached  flag
goal_reached = 0;
canceled_by_user = 0;
//reset this var for being able of recognising the first Timestamp as beginning for my implementation of the timeout
firstTime =1; 


//publish pos to move the robot
my_pos.goal.target_pose.header.frame_id = "map"; 
my_pos.goal.target_pose.pose.orientation.w = 1;
my_pos.goal.target_pose.pose.position.x =x_goal;
my_pos.goal.target_pose.pose.position.y = y_goal;
pub1.publish(my_pos);
printf("\n GOAL PUBLISHED");

return;
}
}


 
//CALLBACK for the /move_base/feedback
/* ***************************************************************************/
void CurrentPositionCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
//execute the callback only if goal wasn't reached yes.
//This because the topic keeps sending feedback when the robot has reached the goal but it's "a don't care" info
if(goal_reached == 0){ 
 float current_x = msg->feedback.base_position.pose.position.x;
 float current_y = msg->feedback.base_position.pose.position.y;
//take first TimeStamp as the starting time for my implementation of the timeout
 if(firstTime == 1) 
 {
 StartTime =ros::Time::now(); 
 firstTime = 0;
 }
//save time in a var to compute elapsed time
currentTime = msg->feedback.base_position.header.stamp; 
double elapsed_time = currentTime.toSec() - StartTime.toSec();

system("clear");
//notify the user and show elapsed time since the goal was given
printf("\nPosition RECEIVED : X=%f  Y=%f",x_goal,y_goal);
printf("\nelapsed time %d", (int)elapsed_time);

//compute errors to determine if goal was reached or not
errorX = current_x - x_goal;
errorY = current_y - y_goal;

//take the module to have a positive error and simplyfing operations
if(errorX < 0) errorX = -errorX;
if(errorY < 0) errorY = -errorY;

//if close enough to the point consider the goal as reached
if(errorX <= ERROR && errorY <= ERROR) 
{
  goal_reached = 1;
}

//if the timeout is over and the goal hasn't been reached, cancel the goal and notify the user
if(elapsed_time > TIMEOUT && goal_reached == 0) 
 {
  printf("\n\nTIME IS OVER AND GOAL WAS NOT REACHED \n(are you sure you inserted a reachable position?) \n");
  my_cancel.stamp.sec = 0;
  my_cancel.stamp.nsec = 0;
  my_cancel.id = "";
  pubCancel.publish(my_cancel);
  printf("\nGOAL CANCELED\n"); 

 }
 //if goal has been reached before timeout was over
 else if(elapsed_time < TIMEOUT && goal_reached == 1)
 {
   system("clear");
   printf("\nPOSITION REACHED, please select new destination.. \n");
 } 
}
}



int main (int argc, char **argv)
{
// Initialize the node, setup the NodeHandle for handling the communication with the ROS system 
  ros::init(argc, argv, "my_controller");
  ros::NodeHandle nh;

   //define the publisher for  robot's GOAL
   pub1 = nh.advertise<move_base_msgs::MoveBaseActionGoal> ("/move_base/goal", 1);
   
   //define the publisher for move_base/cancel to cancel the goal
   pubCancel = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
   
   //define the publisher for cmd_vel to move the robot by keys
   pubVel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   
   
   
   //Define the subscriber to /move_base/feedback topic for having info about robot status
   ros::Subscriber subFeedback = nh.subscribe("/move_base/feedback", 1, CurrentPositionCallback);
   
   //Define the subscriber to my topic for receiving goal from inputConsole
   ros::Subscriber subInputPoint = nh.subscribe("MY_topic_send_goal", 1, myCallback);
    
   //Define the subscriber to teleop topic to receive info from inputConsole
   ros::Subscriber subTeleop = nh.subscribe("MY_topic_teleop", 1, TeleopCallback);
   
   //Define the subscriber to my remappedVel topic to receive velocity requested by user via keyboard
   ros::Subscriber subMyRemappedVel = nh.subscribe("myRemapped_cmd_vel", 1, myCmdCallback);  	
   
   //Define the subscriber to robot's lasers
   ros::Subscriber sub = nh.subscribe("/scan", 1, LasersCallback);

   printf("\n\n***********THIS IS THE CONTROLLER CONSOLE***********\n");
   
    while (ros::ok()) {
    
    if (manualDrive == 0) // If manual drive is chosen 
    {   
     //simply publish the vel that was remapped on myRemappedCmd on cmd/vel
     //in order to make the robot move with teleopKey only if the user selected this option
        desiredVel = velFromTeleop;
    	pubVel.publish(desiredVel);
    } 
    else if (manualDrive == 1)//If manual ASSISTED drive is chosen 
    {  
     assistedMovement();
    }

    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }   
return 0;
}




