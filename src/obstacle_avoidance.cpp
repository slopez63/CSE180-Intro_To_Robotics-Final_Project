#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"

float closest, distance;
bool angle_adjustment;
int count;
bool finished;

///// functions to receive laser data //////

void Received(const sensor_msgs::LaserScan&msg) {
	ROS_INFO_STREAM("Received Scan Finding closest Obstacle");
	closest = msg.ranges[0];
	int closestIndex = 0;
	for(int i = 1; i < msg.ranges.size(); i++){
		if(msg.ranges[i] < closest){
			closest = msg.ranges[i];
			closestIndex = i;
		}
	}
	if( closest < 0.7){
		angle_adjustment = true;

	}else{
		angle_adjustment = false;
	}

	ROS_INFO_STREAM("closest obstacle at distance " << closest);

}


void checkFinished(const std_msgs::Bool&msg){
    finished = msg.data;
}



int main(int argc,char ** argv){

	ros::init(argc,argv,"Obstacle_Avoidance");
	ros::NodeHandle nh;
	
	//Publishers
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",1000);
	ros::Publisher stop = nh.advertise<std_msgs::Bool>("stop",1000);

	//Subscribers
	ros::Subscriber laser_feed = nh.subscribe("/scan",1000,&Received);
	ros::Subscriber plannerFinished = nh.subscribe("plannerFinished", 1000, &checkFinished);


	std_msgs::Bool hault;
	geometry_msgs::Twist movement;

	angle_adjustment = finished = false;

	count = 0;

	ros::Rate rate(20);
	ros::Rate rate2(5);


  while (!finished){

				//Rotate right of z axis
				if(angle_adjustment == true){					
					hault.data = true;
					stop.publish(hault);

					movement.angular.z = 0.5;
					movement.linear.x = 0;
					movement.linear.y = 0;
				
					pub.publish(movement);
					rate2.sleep();
					
				//Move Forward
				}else{					
					hault.data = false;
					stop.publish(hault);
				}

		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO_STREAM("end of obstacle avoidance");
}
