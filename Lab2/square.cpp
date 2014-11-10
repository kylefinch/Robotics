/*Kyle Vandelac
11193077
Lab 2 */



#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
	const double FORWARD_SPD = 0.5;
	const double TURN_SPD = -1.0;
	
	//create node
	ros::init(argc, argv, "move_square");
	ros::NodeHandle node;
	//create publisher
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	//create Twist msg
	geometry_msgs::Twist cmd;
	
	int i = 0, edgeCount = 0;
	bool corner = false;
	
	ros::Rate rate(10);
	ROS_INFO("Square begin");
	while(ros::ok())
	{
	for(i = 0; i< 17; i++)
	{	
		if(corner == false)
		{
			ROS_INFO("Going Forward");
			cmd.linear.x = FORWARD_SPD;
			cmd.angular.z = 0.0;
			if(i == 8)
			{
				
				edgeCount = edgeCount +1;
				corner = true;
				i = 0;
			}
		}
		else	//turning
		{
			ROS_INFO("Turning Left");
			cmd.angular.z = TURN_SPD;
			cmd.linear.x = 0.0;
			if(i == 16)
			{
				corner = false;
			}
		}
		pub.publish(cmd);
		
		rate.sleep();
		
		
	}
	if(edgeCount == 4)	//if true, then completed 4 edges
		{
			ROS_INFO("Square Complete");
			break;
		}	
	
	}	
	return 0;
}
