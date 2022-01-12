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

ros::Publisher pubGoalPoint;
geometry_msgs::Point inputPoint;

ros::Publisher pubCancel;
actionlib_msgs::GoalID my_cancel;

ros::Publisher pubTeleop;
std_msgs::Bool whichTeleop;


float x_goal ;
float y_goal ;


std::string goalX;
std::string goalY;
char input;

void ClockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{

printf("\n\n***********THIS IS THE INPUT CONSOLE***********\n");

printf("\nWhich Action do you want to use to move the robot?"); //scriverer meglio
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
