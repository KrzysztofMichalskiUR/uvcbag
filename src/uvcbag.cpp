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

//#define PATH_MAX 1024

std::string getApplicationDirectory() 
{
    char result[ PATH_MAX ];
    ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
    std::string appPath = std::string( result, (count > 0) ? count : 0 );

    std::size_t found = appPath.find_last_of("/\\");
    return appPath.substr(0,found);
}
typedef enum BagState(BAG_NORMAL,BAG_RECOVERY,BAG_OFF) BagState;

	BagState stateCurr=BAG_NORMAL;
	BagState statePrev=BAG_NORMAL;

int main(int argc, char** argv)
{

	typedef std::array<std::string,2> _2str;
	std::vector<_2str> topics;

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

	ros::init(argc,argv,"uvcbag");
	ros::NodeHandle n;
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
					std::cout<<"typei:"<<line[1]<<std::endl;
					dumpAnyTopic(n,bag,line[0],line[1]);
					std::cout<<"topicname:"<<line[0]<<std::endl;
				}
			}
			case BAG_OFF:
			{
				ros::Duration(0.5).sleep();
				break;
			}
		}

			ros::spin();
	}

}
