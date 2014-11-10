/*Kyle Vandelac
11193077
Lab 2*/




#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
	double FORWARD_SPD = 0.5;
	double TURN_SPD = 1.0;
	
	//create node
	ros::init(argc, argv, "move_spiral");
	ros::NodeHandle node;
	//create publisher
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	//create Twist msg
	geometry_msgs::Twist cmd;
	
	int i = 0, j = 0;
	bool ring = false;
	
	ros::Rate rate(10);
	ROS_INFO("Spiral begin");
	while(ros::ok())
	{
		if(i == 15)
		{
			i--;
			j++;
			cmd.linear.x = FORWARD_SPD;
			cmd.angular.z = 0.1;
			if(j == 5)
			{
				j = 0;
				i = -1;	
				TURN_SPD-= 0.1;
				FORWARD_SPD+= 0.2;
			}
		}
		else 
		{
			cmd.linear.x = FORWARD_SPD;
			cmd.angular.z = TURN_SPD;
		}
		
			
		pub.publish(cmd);
		rate.sleep();	
		i++;
	}	
	return 0;
}
