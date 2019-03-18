#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>

float x_target;
float y_target;
float theta_target;
bool feedback;
bool stop1;
int count;
std_msgs::Bool status;
bool finished;



/////Functions to receive data/////
void poseMessageReceived(const geometry_msgs::Pose2D&msg){
	x_target = msg.x;
	y_target = msg.y;
	theta_target = msg.theta;
	feedback = true;
    count++;
}


void checkFinished(const std_msgs::Bool&msg){
    finished = msg.data;
}

void stopServer(const std_msgs::Bool&msg){
    stop1 = msg.data;
}

////// Functions for server //////
void serviceActivated() {
    ROS_INFO_STREAM("Service received goal");
}

void serviceDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO_STREAM("Service completed");
    ROS_INFO_STREAM("Final state " << state.toString().c_str());
}

void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
    ROS_INFO_STREAM("Service still running");
    ROS_INFO_STREAM("Current pose (x,y) " <<
		    fb->base_position.pose.position.x << "," <<
		    fb->base_position.pose.position.y);
}




int main(int argc,char **argv) {

    ros::init(argc,argv,"explore");
    ros::NodeHandle nh;

    //Publishers
    ros::Publisher exploreStatus = nh.advertise<std_msgs::Bool>("exploreStatus", 1000);

    //Subscribers
    ros::Subscriber target = nh.subscribe("sendCoordinates", 1000, &poseMessageReceived);
    ros::Subscriber plannerFinished = nh.subscribe("plannerFinished", 1000, &checkFinished);
    ros::Subscriber stop = nh.subscribe("stop", 1000, &stopServer);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>ac("move_base",true);


    x_target = 0;
    y_target = 0;
    theta_target = 0;
    feedback = false;
    finished = false;
    stop1 = false;
    count = 0;

   
    //Waits until move base server is available
    ROS_INFO_STREAM("Waiting for server to be available...");

    while (!ac.waitForServer()) {
    }

    ROS_INFO_STREAM("The server is ready!");



    ros::Rate rate(1);
    
    while(!finished){

        count++;
       
       //Gather new coordinates from planner
        while (!feedback && !finished){
            ROS_INFO_STREAM("Waiting for new coordinates...");
            rate.sleep();
            ros::spinOnce();
        }

        feedback = false;

        //Store new coordinates in a goal and send goal to move base server
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x_target;
        goal.target_pose.pose.position.y = y_target;
        goal.target_pose.pose.orientation.w = theta_target;

        ac.sendGoal(goal,&serviceDone,&serviceActivated,&serviceFeedback);
        rate.sleep();

        ac.waitForResult(ros::Duration(30));

        //Prints if the robot has reached the goal or not. Also asks planner for another goal.
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO_STREAM("The robot reached the goal");
            status.data = true;
            exploreStatus.publish(status);
        }else{
            ROS_INFO_STREAM("The robot did not reach the goal");
            status.data = true;
            exploreStatus.publish(status);
        }

    }

    ROS_INFO_STREAM("end of explore");

    return 0;    
}
