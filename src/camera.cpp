#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Bool.h"
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include "logical_camera_plugin/logicalImage.h"

#include <vector>
#include <string>

using namespace std;


//Structure to store treasure information
struct Treasure{
	float x;
	float y;
	string name;
};


//Variables and function needed to receive camera data
float arr[6];
string name;
bool finished;

void Received(const logical_camera_plugin::logicalImage&msg) {
	 name			= msg.modelName;
	 arr[0] 	= msg.pose_pos_x;
	 arr[1]  	= msg.pose_pos_y;
	 arr[2]  	= msg.pose_pos_z;
	 arr[3] 	= msg.pose_rot_x;
	 arr[4] 	= msg.pose_rot_y;
	 arr[5] 	= msg.pose_rot_z;
}


void checkFinished(const std_msgs::Bool&msg){
    finished = msg.data;
}


int main(int argc,char ** argv){

	ros::init(argc,argv,"camera");
	ros::NodeHandle nh;

	//Subscribers
	ros::Subscriber sub = nh.subscribe("/objectsDetected",1000, &Received);
	ros::Subscriber plannerFinished = nh.subscribe("plannerFinished", 1000, &checkFinished);

	//Data needed for transforming coordinates
	tf2_ros::Buffer buffer;
 	tf2_ros::TransformListener listener(buffer);
	geometry_msgs::TransformStamped transformStamped;
	std::vector<Treasure> foundTreasures;


	ros::Rate rate(20);

	finished = false;

  while (!finished){

	  	int found = 0;


		//Gather information from the camera sensor

		ROS_INFO_STREAM("Scan: ");

		ros::spinOnce();

		cout << name << endl;

		for(int x = 0; x < 6; x++){
			cout << arr[x] << " " << endl;
		}

		
		//Check if the treasure has been found before
		for(vector<Treasure>::const_iterator i = foundTreasures.begin(); i != foundTreasures.end(); ++i) {
			Treasure temp;
			temp = *(i);
			if(name == temp.name){
				found ++;
			}
		}

		//If it hasn't been found store it in the found treasures vector
		if(found == 0 && name != ""){

			Treasure temp;

			try{
			transformStamped = buffer.lookupTransform("odom","base_link",ros::Time(0));
			}catch (tf2::TransformException &ex) {
				ROS_WARN("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			//Calculate a transformation of where it has been found in the global map

			tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, 	transformStamped.transform.rotation.z,transformStamped.transform.rotation.w);

			tf::Matrix3x3 m(q);

			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			temp.x = transformStamped.transform.translation.x - arr[0];
			temp.y = transformStamped.transform.translation.y - arr[1];
			temp.name = name;

			foundTreasures.push_back(temp);
		}

		///// Print found treasures in every scan //////
		for(vector<Treasure>::const_iterator i = foundTreasures.begin(); i != foundTreasures.end(); ++i) {
			Treasure temp = *(i);

			ROS_INFO_STREAM( "found: " << temp.name << " at x: " << temp.x << " y: " << temp.y);
		}
		
		rate.sleep();
	}

	ROS_INFO_STREAM("End of scan");
	ROS_INFO_STREAM(" ");
	ROS_INFO_STREAM("These are the treasures that the robot found: ");

	for(vector<Treasure>::const_iterator i = foundTreasures.begin(); i != foundTreasures.end(); ++i) {
			Treasure temp = *(i);

			ROS_INFO_STREAM( "found: " << temp.name << " at x: " << temp.x << " y: " << temp.y);
		}
}
