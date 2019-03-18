#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

std_msgs::Bool finished;

/////  Array Index //////
int column = 0;
int row = 0;

/////  Variables for Coordinates Calculations //////
int number, boxSize;
bool calculated, exploreDone;

/////  Structure for cordinates to store in one array //////
struct coordinates{
	public:
		double x;
		double y;
};

//Function that receives boolean if new data needs to be sent to explore
void recievedExploreStatus(const std_msgs::Bool&msg){
    exploreDone = msg.data;
}


int main(int argc,char **argv) {

    ros::init(argc,argv,"planner");
    ros::NodeHandle nh;

    //Publishers
    ros::Publisher sendToExplore = nh.advertise<geometry_msgs::Pose2D>("sendCoordinates", 1000); //sends coordinates to explore
    ros::Publisher plannerFinished = nh.advertise<std_msgs::Bool>("plannerFinished", 1000);

    //Subscribers
    ros::Subscriber exploreStatus = nh.subscribe("exploreStatus", 1000, &recievedExploreStatus); //checks when this node needs to send new info to explore
    

    geometry_msgs::Pose2D pose;




    ///// Generate grid coordinates to send to explore //////

    ROS_INFO_STREAM("What is the length of the squares you want to grid? ");
   
    std::cin >> number;

    boxSize = number*number;

    coordinates list[number][number];

    ros::Rate rate(4);

    //Coordinates get stored in an array
        for(int i = 0; i < sqrt(boxSize); i++){
                for(int j = 0; j < sqrt(boxSize); j++){
                    list[i][j].x = 0; 
                    list[i][j].y = 0; 
                }
            }

            for(int k = 0; k < sqrt(boxSize); k++){
                list[0][k].x = -8;
                list[k][0].y = -8;
            }

            list[0][0].y = -8;
            
            for(int i = 0; i < sqrt(boxSize); i++){
                for(int j = 1; j < sqrt(boxSize); j++){
                    list[i][j].y = list[i][j-1].y + 16/(sqrt(boxSize)-1);
                    list[j][i].x = list[j-1][i].x + 16/(sqrt(boxSize)-1);
                }
            }


            for(int i = 0; i < sqrt(boxSize); i++){
                for(int j = 0; j < sqrt(boxSize); j++){
                    ROS_INFO_STREAM("(" << list[i][j].x << " ," << list[i][j].y << ") ");
                }
                ROS_INFO_STREAM("");
            }




    while(row < number - 1){

        exploreDone = false;

        finished.data = false;
        plannerFinished.publish(finished);

        //Creates index of next coordinate to send
        if(column > number){
				row++;
				column = 0;
		}

        //Stores values to message and publish them
        pose.x = list[row][column].x;
        pose.y = list[row][column].y;
        pose.theta = M_PI;

        sendToExplore.publish(pose);
        ROS_INFO_STREAM("Sent coordinates (" << list[row][column].x << ", " << list[row][column].y <<  ") to explore");
        column++;

        //Loop that makes this node stop until explore requests new data
        while(!exploreDone){
            ros::spinOnce();
            rate.sleep();
        }

    }

    finished.data = true;
    plannerFinished.publish(finished);

    ROS_INFO_STREAM("All of the coordinates have been sent");

    return 0;

}