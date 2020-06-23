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
std::string getApplicationDirectory() 
{
    char result[ PATH_MAX ];
    ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
    std::string appPath = std::string( result, (count > 0) ? count : 0 );

    std::size_t found = appPath.find_last_of("/\\");
    return appPath.substr(0,found);
}
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

	typedef std::array<std::string,2> _2str;
	std::vector<_2str> topics;
	std::vector<_2str> topics_recovery;

	char type[STRLEN],topicname[STRLEN];
	int fr=0;
	FILE* f=fopen("/home/kmros/catkin_ws/src/uvcbag/topics.txt","r");
	std::cout<<"Opened topics file"<<std::endl;
	while(fscanf(f,"%s %s\n",  topicname,type)!=EOF)
	{
		
		topics.push_back({std::string(topicname),std::string(type)});

		std::cout<<"typei:"<<std::string(type)<<std::endl;
		std::cout<<"topicname:"<<std::string(topicname)<<std::endl;
	}
	fclose(f);



	f=fopen("/home/kmros/catkin_ws/src/uvcbag/topics_recovery.txt","r");
	std::cout<<"Opened topics file"<<std::endl;
	while(fscanf(f,"%s %s\n",  topicname,type)!=EOF)
	{
		
		topics_recovery.push_back({std::string(topicname),std::string(type)});

		std::cout<<"typei:"<<std::string(type)<<std::endl;
		std::cout<<"topicname:"<<std::string(topicname)<<std::endl;
	}
	fclose(f);


	ros::init(argc,argv,"uvcbag");
	ros::NodeHandle n;
	
	ros::ServiceServer ss = n.advertiseService("set_bag_state", setState);



	rosbag::Bag bag;
	bag.open("/home/kmros/catkin_ws/src/uvcbag/uvcbag.bag",rosbag::bagmode::Write);

	while(ros::ok())
	{
		switch(stateCurr)
		{
			case BAG_NORMAL:
			{

				for(auto line:topics)
				{
					if(stateCurr!=BAG_NORMAL)
					{
						break;
					}
					std::cout<<"typei:"<<line[1]<<std::endl;
					dumpAnyTopic(bag,line[0],line[1]);
					std::cout<<"topicname:"<<line[0]<<std::endl;
					ros::spinOnce();
				}

			}
			case BAG_OFF:
			{
				ros::Duration(0.5).sleep();
				ros::spinOnce();
				break;
			}
			case BAG_RECOVERY:
			{
				for(auto line:topics_recovery)
				{
					if(stateCurr!=BAG_RECOVERY)
					{
						break;
					}
					std::cout<<"typei:"<<line[1]<<std::endl;
					dumpAnyTopic(bag,line[0],line[1]);
					std::cout<<"topicname:"<<line[0]<<std::endl;
					ros::spinOnce();

				}
			}
		}
	}

}
