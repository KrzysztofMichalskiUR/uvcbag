#pragma once
#include<actionlib_msgs/GoalStatusArray.h>
#include<cartographer_ros_msgs/SubmapList.h>
#include<dynamic_reconfigure/Config.h>
#include<dynamic_reconfigure/ConfigDescription.h>
#include<gazebo_msgs/LinkStates.h>
#include<gazebo_msgs/ModelStates.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PolygonStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/Twist.h>
#include<map_msgs/OccupancyGridUpdate.h>
#include<move_base_msgs/MoveBaseActionFeedback.h>
#include<move_base_msgs/MoveBaseActionGoal.h>
#include<move_base_msgs/MoveBaseActionResult.h>
#include<nav_msgs/MapMetaData.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<rosgraph_msgs/Clock.h>
#include<rosgraph_msgs/Log.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud2.h>
#include<tf2_msgs/TFMessage.h>
#include<visualization_msgs/MarkerArray.h>
#include<string>
#include <actionlib_msgs/GoalStatusArray.h>
#include<map>
#include<queue>

#define actionlib_msgs_GoalStatusArray       	actionlib_msgs,GoalStatusArray,0
#define	cartographer_ros_msgs_SubmapList       	cartographer_ros_msgs,SubmapList,1
#define	dynamic_reconfigure_Config             	dynamic_reconfigure,Config,2
#define	dynamic_reconfigure_ConfigDescription  	dynamic_reconfigure,ConfigDescription,3
#define	gazebo_msgs_LinkStates                 	gazebo_msgs,LinkStates,4
#define	gazebo_msgs_ModelStates                	gazebo_msgs,ModelStates,5
#define	geometry_msgs_PointStamped             	geometry_msgs,PointStamped,6
#define	geometry_msgs_PolygonStamped           	geometry_msgs,PolygonStamped,7
#define	geometry_msgs_PoseStamped              	geometry_msgs,PoseStamped,8
#define	geometry_msgs_PoseWithCovarianceStamped	geometry_msgs,PoseWithCovarianceStamped,9
#define	geometry_msgs_Twist                    	geometry_msgs,Twist,10
#define	map_msgs_OccupancyGridUpdate           	map_msgs,OccupancyGridUpdate,11
#define	move_base_msgs_MoveBaseActionFeedback  	move_base_msgs,MoveBaseActionFeedback,13
#define	move_base_msgs_MoveBaseActionGoal      	move_base_msgs,MoveBaseActionGoal,14
#define	move_base_msgs_MoveBaseActionResult    	move_base_msgs,MoveBaseActionResult,15
#define	nav_msgs_MapMetaData                   	nav_msgs,MapMetaData,16
#define	nav_msgs_OccupancyGrid                 	nav_msgs,OccupancyGrid,17
#define	nav_msgs_Odometry                      	nav_msgs,Odometry,18
#define	nav_msgs_Path                          	nav_msgs,Path,19
#define	rosgraph_msgs_Clock                    	rosgraph_msgs,Clock,20
#define	rosgraph_msgs_Log                      	rosgraph_msgs,Log,21
#define	sensor_msgs_Imu                        	sensor_msgs,Imu,22
#define	sensor_msgs_JointState                 	sensor_msgs,JointState,23
#define	sensor_msgs_LaserScan                  	sensor_msgs,LaserScan,24
#define	sensor_msgs_PointCloud2                	sensor_msgs,PointCloud2,25
#define	tf2_msgs_TFMessage                     	tf2_msgs,TFMessage,26
#define	visualization_msgs_MarkerArray         	visualization_msgs,MarkerArray,27





#define STRFY(x) #x

#define AS_STRING(PRE,POST,NUM) STRFY(PRE/POST)
#define INV_AS_STRING(...) AS_STRING(__VA_ARGS__)

#define AS_TYPE(PRE,POST,NUM) PRE::POST
#define INV_AS_TYPE(...) AS_TYPE(__VA_ARGS__)

#define AS_NUM(PRE,POST,NUM) NUM
#define INV_AS_NUM(...) AS_NUM(__VA_ARGS__)

#define MSG_CHOICE(PRE,POST,NUM) case INV_AS_NUM(PRE,POST,NUM): {dumpTopic<INV_AS_TYPE(PRE,POST,NUM)>(bag,topic);break;}
#define INV_MSG_CHOICE(...)  MSG_CHOICE(__VA_ARGS__)

#define AS_MAP_REC(PRE,POST,NUM) {AS_STRING(PRE,POST,NUM),NUM}
#define INV_AS_MAP_REC(...) AS_MAP_REC(__VA_ARGS__)
std::map<std::string,int> typeLookup=
{
	INV_AS_MAP_REC(	actionlib_msgs_GoalStatusArray       	),
	INV_AS_MAP_REC(	cartographer_ros_msgs_SubmapList       	),
	INV_AS_MAP_REC(	dynamic_reconfigure_Config             	),
	INV_AS_MAP_REC(	dynamic_reconfigure_ConfigDescription  	),
	INV_AS_MAP_REC(	gazebo_msgs_LinkStates                 	),
	INV_AS_MAP_REC(	gazebo_msgs_ModelStates                	),
	INV_AS_MAP_REC(	geometry_msgs_PointStamped             	),
	INV_AS_MAP_REC(	geometry_msgs_PolygonStamped           	),
	INV_AS_MAP_REC(	geometry_msgs_PoseStamped              	),
	INV_AS_MAP_REC(	geometry_msgs_PoseWithCovarianceStamped	),
	INV_AS_MAP_REC(	geometry_msgs_Twist                    	),
	INV_AS_MAP_REC(	map_msgs_OccupancyGridUpdate           	),
	INV_AS_MAP_REC(	move_base_msgs_MoveBaseActionFeedback  	),
	INV_AS_MAP_REC(	move_base_msgs_MoveBaseActionGoal      	),
	INV_AS_MAP_REC(	move_base_msgs_MoveBaseActionResult    	),
	INV_AS_MAP_REC(	nav_msgs_MapMetaData                   	),
	INV_AS_MAP_REC(	nav_msgs_OccupancyGrid                 	),
	INV_AS_MAP_REC(	nav_msgs_Odometry                      	),
	INV_AS_MAP_REC(	nav_msgs_Path                          	),
	INV_AS_MAP_REC(	rosgraph_msgs_Clock                    	),
	INV_AS_MAP_REC(	rosgraph_msgs_Log                      	),
	INV_AS_MAP_REC(	sensor_msgs_Imu                        	),
	INV_AS_MAP_REC(	sensor_msgs_JointState                 	),
	INV_AS_MAP_REC(	sensor_msgs_LaserScan                  	),
	INV_AS_MAP_REC(	sensor_msgs_PointCloud2                	),
	INV_AS_MAP_REC(	tf2_msgs_TFMessage                     	),
	INV_AS_MAP_REC(	visualization_msgs_MarkerArray         	)
};

template <class T> void dumpTopic(rosbag::Bag& bag,std::string topic)
{
	int msgNum=10;
	std::queue<T> q;;
	bag.write(topic,ros::Time::now(),msg);

}
void dumpAnyTopic(rosbag::Bag& bag,std::string topic,std::string type)
{
	switch(typeLookup[type])
	{
		INV_MSG_CHOICE(	actionlib_msgs_GoalStatusArray);
		INV_MSG_CHOICE(	cartographer_ros_msgs_SubmapList       );
		INV_MSG_CHOICE(	dynamic_reconfigure_Config             );
		INV_MSG_CHOICE(	dynamic_reconfigure_ConfigDescription  );
		INV_MSG_CHOICE(	gazebo_msgs_LinkStates                 	);
		INV_MSG_CHOICE(	gazebo_msgs_ModelStates                );
		INV_MSG_CHOICE(	geometry_msgs_PointStamped             );
		INV_MSG_CHOICE(	geometry_msgs_PolygonStamped           );
		INV_MSG_CHOICE(	geometry_msgs_PoseStamped              );
		INV_MSG_CHOICE(	geometry_msgs_PoseWithCovarianceStamped );
		INV_MSG_CHOICE(	geometry_msgs_Twist                    );
		INV_MSG_CHOICE(	map_msgs_OccupancyGridUpdate           );
		INV_MSG_CHOICE(	move_base_msgs_MoveBaseActionFeedback  );
		INV_MSG_CHOICE(	move_base_msgs_MoveBaseActionGoal      );
		INV_MSG_CHOICE(	move_base_msgs_MoveBaseActionResult    );
		INV_MSG_CHOICE(	nav_msgs_MapMetaData                   );
		INV_MSG_CHOICE(	nav_msgs_OccupancyGrid                 );
		INV_MSG_CHOICE(	nav_msgs_Odometry                      );
		INV_MSG_CHOICE(	nav_msgs_Path                          );
		INV_MSG_CHOICE(	rosgraph_msgs_Clock                    );
		INV_MSG_CHOICE(	rosgraph_msgs_Log                      );
		INV_MSG_CHOICE(	sensor_msgs_Imu                        );
		INV_MSG_CHOICE(	sensor_msgs_JointState                 );
		INV_MSG_CHOICE(	sensor_msgs_LaserScan                  );
		INV_MSG_CHOICE(	sensor_msgs_PointCloud2                );
		INV_MSG_CHOICE(	tf2_msgs_TFMessage                     );
		INV_MSG_CHOICE(	visualization_msgs_MarkerArray         );
	}
}
typedef std::array<std::string,2> _2str;
std::vector<_2str> topicList=
{
	{"/map_metadata","nav_msgs/MapMetaData"},
	{"/move_base/global_costmap/costmap_updates","map_msgs/OccupancyGridUpdate"},
	{"/move_base/MyDWAPlannerROS/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/move_base/current_goal","geometry_msgs/PoseStamped"},
	{"/scan2","sensor_msgs/LaserScan"},
	{"/move_base/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/move_base/local_costmap/footprint","geometry_msgs/PolygonStamped"},
	{"/move_base/feedback","move_base_msgs/MoveBaseActionFeedback"},
	{"/move_base/result","move_base_msgs/MoveBaseActionResult"},
	{"/move_base/global_costmap/parameter_updates","dynamic_reconfigure/Config"},
	{"/tf","tf2_msgs/TFMessage"},
	{"/clicked_point","geometry_msgs/PointStamped"},
	{"/move_base/local_costmap/costmap","nav_msgs/OccupancyGrid"},
	{"/odom","nav_msgs/Odometry"},
	{"/move_base/global_costmap/static_layer/parameter_updates","dynamic_reconfigure/Config"},
	{"/scan","sensor_msgs/LaserScan"},
	{"/move_base/MyDWAPlannerROS/global_plan","nav_msgs/Path"},
	{"/move_base/local_costmap/Ginflater_layer/parameter_updates","dynamic_reconfigure/Config"},
	{"/move_base/parameter_updates","dynamic_reconfigure/Config"},
	{"/move_base/MyDWAPlannerROS/local_plan_best","nav_msgs/Path"},
	{"/move_base/MyNavfnROS/plan","nav_msgs/Path"},
	{"/move_base/local_costmap/Ginflater_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/move_base_simple/goal","geometry_msgs/PoseStamped"},
	{"/move_base/status","actionlib_msgs/GoalStatusArray"},
	{"/move_base/global_costmap/static_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/move_base/local_costmap/costmap_updates","map_msgs/OccupancyGridUpdate"},
	{"/flat_imu","sensor_msgs/Imu"},
	{"/move_base/local_costmap/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/gazebo_gui/parameter_updates","dynamic_reconfigure/Config"},
	{"/tf_static","tf2_msgs/TFMessage"},
	{"/move_base/global_costmap/inflation_layer/parameter_updates","dynamic_reconfigure/Config"},
	{"/imu","sensor_msgs/Imu"},
	{"/gazebo/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/map","nav_msgs/OccupancyGrid"},
	{"/move_base/MyDWAPlannerROS/parameter_updates","dynamic_reconfigure/Config"},
	{"/cmd_vel","geometry_msgs/Twist"},
	{"/landmark_poses_list","visualization_msgs/MarkerArray"},
	{"/move_base/local_costmap/Gobstacles_layer/parameter_updates","dynamic_reconfigure/Config"},
	{"/gazebo_gui/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/move_base/local_costmap/parameter_updates","dynamic_reconfigure/Config"},
	{"/submap_list","cartographer_ros_msgs/SubmapList"},
	{"/move_base/global_costmap/footprint","geometry_msgs/PolygonStamped"},
	{"/scan_matched_points2","sensor_msgs/PointCloud2"},
	{"/joint_states","sensor_msgs/JointState"},
	{"/rosout","rosgraph_msgs/Log"},
	{"/move_base/goal","move_base_msgs/MoveBaseActionGoal"},
	{"/initialpose","geometry_msgs/PoseWithCovarianceStamped"},
	{"/rosout_agg","rosgraph_msgs/Log"},
	{"/move_base/MyDWAPlannerROS/local_plan","nav_msgs/Path"},
	{"/recovery_sector_markers","visualization_msgs/MarkerArray"},
	{"/trajectory_node_list","visualization_msgs/MarkerArray"},
	{"/move_base/MyDWAPlannerROS/cost_cloud","sensor_msgs/PointCloud2"},
	{"/move_base/MyNavfnROS/visualization_marker","visualization_msgs/MarkerArray"},
	{"/move_base/global_costmap/inflation_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/move_base/global_costmap/obstacle_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/constraint_list","visualization_msgs/MarkerArray"},
	{"/move_base/local_costmap/static_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/gazebo/link_states","gazebo_msgs/LinkStates"},
	{"/move_base/MyDWAPlannerROS/trajectory_cloud","sensor_msgs/PointCloud2"},
	{"/gazebo/model_states","gazebo_msgs/ModelStates"},
	{"/clock","rosgraph_msgs/Clock"},
	{"/move_base/global_costmap/obstacle_layer/parameter_updates","dynamic_reconfigure/Config"},
	{"/move_base/local_costmap/Gobstacles_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/move_base/local_costmap/static_layer/parameter_updates","dynamic_reconfigure/Config"},
	{"/move_base/global_costmap/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
	{"/move_base/global_costmap/costmap","nav_msgs/OccupancyGrid"},
	{"/gazebo/parameter_updates","dynamic_reconfigure/Config"},
}
