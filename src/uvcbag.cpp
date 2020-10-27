#include <rosbag/bag.h>
#include<ros/ros.h>
#include "../include/topics.h"
/*
#include <iostream>
#include <fstream>
*/
#include<stdio.h>
#define STRLEN 256
#include <unistd.h>
#include <limits.h>
#include<string>
#include<mutex>
#include<uvcbag/SetState.h>

//#define PATH_MAX 1024

std::mutex mtx;

typedef enum BagState{BAG_NORMAL,BAG_RECOVERY,BAG_OFF} BagState;

	BagState stateCurr=BAG_NORMAL;
	BagState statePrev=BAG_NORMAL;
bool setState(uvcbag::SetState::Request &req,uvcbag::SetState::Response & resp)
{
	std::lock_guard<std::mutex> guard(mtx);
	std::cout<<"set state service recived"<<req.state;
	stateCurr=static_cast<BagState>(req.state);
}
int main(int argc, char** argv)
{


	ros::init(argc,argv,"uvcbag");
	ros::NodeHandle n;
	
	ros::ServiceServer ss = n.advertiseService("set_bag_state", setState);



	rosbag::Bag bag;
	bag.open("/home/melodic3/catkin_ws/src/uvcbag/uvcbag.bag",rosbag::bagmode::Write);

	while(ros::ok())
	{
					ros::spinOnce();
          dumpAnyTopic(bag,n);
		
	}

}
