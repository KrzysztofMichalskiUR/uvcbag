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
#define PATH_MAX 1024

std::string getApplicationDirectory() {
    char result[ PATH_MAX ];
    ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
    std::string appPath = std::string( result, (count > 0) ? count : 0 );

    std::size_t found = appPath.find_last_of("/\\");
    return appPath.substr(0,found);
}

int main(int argc, char** argv)
{
	std::cout<<"Path"<<std::endl;
	std::cout<<getApplicationDirectory()<<std::endl;
	ros::init(argc,argv,"uvcbag");
	rosbag::Bag bag;
	bag.open("/home/kmros/catkin_ws/src/uvcbag/uvcbag.bag",rosbag::bagmode::Write);
	while(ros::ok())
	{
		FILE* topicList=fopen("/home/kmros/catkin_ws/src/uvcbag/topics.txt","r");
		std::cout<<"Opened topics file"<<std::endl;
		char type[STRLEN],topicname[STRLEN];
		int fr=0;

		while(fr!=EOF)
		{
			fscanf(topicList,"%s %s",  type,topicname);
			std::cout<<"type"<<std::endl;
			std::cout<<std::string(type)<<std::endl;
			std::cout<<"topic"<<std::endl;
			std::cout<<std::string(topicname)<<std::endl;
			dumpAnyTopic(bag,std::string(type),std::string(topicname));
		}

		ros::spin();
	}

	bag.close();
}
