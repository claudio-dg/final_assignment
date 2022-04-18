/**
* \file inputConsole.cpp
* \brief input Console to select the desired action of the robot
* \author Claudio Del Gaizo
* \version 0.1
* \date 12/01/2022
*
*
* \details
*
* Subscribes to: <BR>
* /clock : for a continuous callback
*
* Publishes to: <BR>
* /move_base/cancel : to cancel current given goal
*
* /MY_topic_send_goal : to send a new position goal to the controller
*
* /MY_topic_teleop : uses a boolean msg to specify to the controller which "manual drive mode" has been selected 
*
* Description:
*
* This Console receives keyboard's inputs from the user, allowing him to select the desired behaviour of the robot among the possible ones, shown from the console itself.
**/




#include <ros/ros.h> 

//for cin >> 
#include "string"

//for MY_topic_send_goal
#include <geometry_msgs/Point.h>

//for move_base/cancel
#include "actionlib_msgs/GoalID.h"  

//for MY_topic_teleop
#include <std_msgs/Bool.h> 

//for clock topic
#include  "rosgraph_msgs/Clock.h"

ros::Publisher pubGoalPoint; ///< global publisher for MY_topic_send_goal
geometry_msgs::Point inputPoint;///< message containing goal's coordinates (x,y)

ros::Publisher pubCancel; ///< global publisher for move_base/cancel
actionlib_msgs::GoalID my_cancel; ///< message for cancelling goal

ros::Publisher pubTeleop; ///< global publisher for MY_topic_teleop
std_msgs::Bool whichTeleop; ///< boolean message containing manual drive's modality: 0 = Fully manual ; 1 = Assisted manual


float x_goal ; ///< global float variable for x coordinate of the goal
float y_goal ; ///< global float variable for y coordinate of the goal


std::string goalX;  ///< global std::string variable for x coordinate of the goal
std::string goalY; ///< global std::string variable for y coordinate of the goal
char input; ///< global char variable for user's keyboard input


/**
* \brief Callback for /clock topic for a continous and infinte internal loop
* \param msg contains the clock
*
* \return void
*
* This function shows an interface to the user asking for an input, based on which it can:
*
* ° cancel a goal by publishing an actionlib_msgs::GoalID msg on move_base/cancel topic
*
* ° set a new goal by publishing a geometry_msgs::Point msg on MY_topic_send_goal topic
*
* ° set the chosen modality of manual drive by publishing a std_msgs::Bool on MY_topic_teleop topic
*
* ° terminate the node
**/
void ClockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{

printf("\n\n***********THIS IS THE INPUT CONSOLE***********\n");

printf("\nWhich Action do you want to use to move the robot?"); 
printf("\nENTER 'c' to cancel last given goal");
printf("\nENTER '1' to send a new goal to the robot");
printf("\nENTER '2  to manually drive the robot");
printf("\nENTER '3' to manually drive the robot WITH assisted collision avoidance");
printf("\nENTER 'q' to terminate this node\n\n");

//receive input from user via keyboard
std::cin >> input;
system("clear");
switch(input)
{ 
 case 'c' : //cancel goal
 	my_cancel.stamp.sec = 0;
        my_cancel.stamp.nsec = 0;
        my_cancel.id = "";
        pubCancel.publish(my_cancel);
        printf("\nGOAL CANCELED\n");
        
        //flag value useful to reset variables in controller
        inputPoint.x = 666;
        pubGoalPoint.publish(inputPoint); 
 	break;
 	
 case '1' : //autonomous drive
 	printf("\n+++ you selected AUTONOUMOUS drive +++ \n");
 	printf("\nENTER DESIRED X-Position : ");
 	std::cin >> goalX;
    
        printf("\nENTER DESIRED Y Position : ");
        std::cin >> goalY;        
        system("clear");
        //cast them to float
	x_goal = atof(goalX.c_str());
	y_goal = atof(goalY.c_str());
	
	//send point to the controller
	inputPoint.x = x_goal;
	inputPoint.y = y_goal;
	pubGoalPoint.publish(inputPoint);
	printf("\nSENT GOAL : X=%f -- Y=%f \n\n",x_goal,y_goal);
 	break;	
 	
 case'2' :  //manual drive
 	printf("\n+++ you selected FULLY MANUAL drive +++\n");
 	printf("\nPLEASE look at the teleopKeyboard Terminal to move the robot manually");
 	 whichTeleop.data = 0;
 	pubTeleop.publish(whichTeleop);
 	break;
 	
 case'3' : //assisted manual drive
 	printf("\n+++ you selected ASSISTED MANUAL drive +++\n");
 	printf("\nPLEASE look at the teleopKeyboard Terminal to move the robot manually");
 	whichTeleop.data = 1;
 	pubTeleop.publish(whichTeleop);
 	
 	break;
 case'q' : //exit the node
 	ROS_INFO("\n+++ exiting the INPUT CONSOLE... +++");
 	
 	exit(1);
 	
 	break;
 	
 default :
 	ROS_INFO("WRONG COMMAND");
 	
 	break;

}


}


int main (int argc, char **argv)
{
// Initialize the node, setup the NodeHandle for handling the communication with the ROS    system 
  ros::init(argc, argv, "input_console");
  ros::NodeHandle nh;

   //publisher for move_base/cancel
   pubCancel = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
   
   //publisher for my_topic to send goal to the controller 
   pubGoalPoint = nh.advertise<geometry_msgs::Point>("MY_topic_send_goal", 1); 
   
   //publisher for my_topic_teleop to tell the controller to look at teleop_keyboard input s
   pubTeleop = nh.advertise<std_msgs::Bool>("MY_topic_teleop", 1); 
   
   //subscribe to /clock topic for a continuous callback
   ros::Subscriber subClock = nh.subscribe("/clock", 100000, ClockCallback);
   
   ros::spin();
return 0;
}
