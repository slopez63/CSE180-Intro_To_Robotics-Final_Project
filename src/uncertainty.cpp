#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigenvalues>

float covariance[3][3];
Eigen::MatrixXf m(3, 3);

void odomMessage(const nav_msgs::Odometry&msg){
	covariance[0][0] = msg.pose.covariance[0];
	covariance[0][1] = msg.pose.covariance[1];
	covariance[0][2] = msg.pose.covariance[5];
	covariance[1][0] = msg.pose.covariance[6];
	covariance[1][1] = msg.pose.covariance[7];
	covariance[1][2] = msg.pose.covariance[11];
	covariance[2][0] = msg.pose.covariance[30];
	covariance[2][1] = msg.pose.covariance[31];
	covariance[2][2] = msg.pose.covariance[35];

	
	m << covariance[0][0], covariance[0][1], covariance[0][2],
    	 covariance[1][0], covariance[1][1], covariance[1][2],
     	 covariance[2][0], covariance[2][1], covariance[2][2];
}

int main(int argc,char **argv) {

	ros::init(argc,argv, "uncertainty");
	ros::NodeHandle nh;
	
	ros::Subscriber ekf = nh.subscribe("/odometry/filtered", 1000, &odomMessage);

	float volume, a, b, c;
	
	Eigen::EigenSolver<Eigen::MatrixXf> es(m);

	ros::Rate rate(10);

	
	while(ros::ok()){
		Eigen::EigenSolver<Eigen::MatrixXf> es(m);

		//ROS_INFO_STREAM(es.eigenvalues().col(0)[0].col(0));
		

		a = sqrt(0.05*es.eigenvalues().coeffRef(0,0).real());
		b = sqrt(0.05*es.eigenvalues().coeffRef(1,0).real());
		c = sqrt(0.05*es.eigenvalues().coeffRef(2,0).real());
		volume = 4/3*a*b*c*M_PI;

		ROS_INFO_STREAM("Volume of Sphere: ");
		ROS_INFO_STREAM(volume);

		rate.sleep();

		ros::spinOnce();
	}
	
}
