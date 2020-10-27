#pragma once
//To see version with macros applied do: grep -v include topics.h >nocl.h; gcc -E nocl.h 
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
#include<functional>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <boost/type_index.hpp>
#include <array>
#include <experimental/array>
#include "frozen/unordered_map.h"
#include <type_traits>


unsigned constexpr const_hash(char const *input) {
    return *input ?
        static_cast<unsigned int>(*input) + 33 * const_hash(input + 1) :
        5381;
}

unsigned runtime_hash(char const *input) {
    return *input ?
        static_cast<unsigned int>(*input) + 33 * const_hash(input + 1) :
        5381;
}

template <typename V, typename... T>
constexpr auto array_of(T&&... t)
    -> std::array < V, sizeof...(T) >
{
    return {{ std::forward<T>(t)... }};
}
using _2strArr=std::array<const char*,2>;

template<unsigned> struct TopicType; //just generating type and linking it to its name hash( because we can't use char* to specialise templates)

#define TOPIC_TYPE_SPECIALISATION(NAMESPACE,TYPE) template<> struct TopicType< const_hash(#NAMESPACE"/"#TYPE) > 
  { static constexpr auto dumpFun=&dumpTopic<NAMESPACE::TYPE>;\
    using type=NAMESPACE::TYPE;\
    static const unsigned hash=const_hash(#NAMESPACE"/"#TYPE);  };

#define TOPIC_INSTANTIATION(NAME,TYPE)  Topic<typename TopicType<const_hash( TYPE )>::type >{NAME}



template <class T> void dumpTopic(rosbag::Bag& bag,std::string topic)
{
	int msgNum=10;
	ros::NodeHandle n;

	auto pn=boost::typeindex::type_id<T>().pretty_name();
	std::cout<<"T="<<pn<<std::endl;
	auto msgPtr=ros::topic::waitForMessage<T>(topic,n,ros::Duration(0.25));
	if(msgPtr==nullptr)
	{
		std::cout<<"msg is null"<<std::endl;
	}
	else
	{
		std::cout<<"msg cougth sucessfully"<<std::endl;
		bag.write(topic,std::max(ros::Time::now(),ros::TIME_MIN),*msgPtr);
	}
	std::cout<<std::endl;
};
//every type of ros message must be registered here,
//                        namespace/header      class
TOPIC_TYPE_SPECIALISATION(actionlib_msgs       ,GoalStatusArray          )
TOPIC_TYPE_SPECIALISATION(cartographer_ros_msgs,SubmapList               )
TOPIC_TYPE_SPECIALISATION(dynamic_reconfigure  ,Config                   )
TOPIC_TYPE_SPECIALISATION(dynamic_reconfigure  ,ConfigDescription        )
TOPIC_TYPE_SPECIALISATION(gazebo_msgs          ,LinkStates               )
TOPIC_TYPE_SPECIALISATION(gazebo_msgs          ,ModelStates              )
TOPIC_TYPE_SPECIALISATION(geometry_msgs        ,PointStamped             )
TOPIC_TYPE_SPECIALISATION(geometry_msgs        ,PolygonStamped           )
TOPIC_TYPE_SPECIALISATION(geometry_msgs        ,PoseStamped              )
TOPIC_TYPE_SPECIALISATION(geometry_msgs        ,PoseWithCovarianceStamped)
TOPIC_TYPE_SPECIALISATION(geometry_msgs        ,Twist                    )
TOPIC_TYPE_SPECIALISATION(map_msgs             ,OccupancyGridUpdate      )
TOPIC_TYPE_SPECIALISATION(move_base_msgs       ,MoveBaseActionFeedback   )
TOPIC_TYPE_SPECIALISATION(move_base_msgs       ,MoveBaseActionGoal       )
TOPIC_TYPE_SPECIALISATION(move_base_msgs       ,MoveBaseActionResult     )
TOPIC_TYPE_SPECIALISATION(nav_msgs             ,MapMetaData              )
TOPIC_TYPE_SPECIALISATION(nav_msgs             ,OccupancyGrid            )
TOPIC_TYPE_SPECIALISATION(nav_msgs             ,Odometry                 )
TOPIC_TYPE_SPECIALISATION(nav_msgs             ,Path                     )
TOPIC_TYPE_SPECIALISATION(rosgraph_msgs        ,Clock                    )
TOPIC_TYPE_SPECIALISATION(rosgraph_msgs        ,Log                      )
TOPIC_TYPE_SPECIALISATION(sensor_msgs          ,Imu                      )
TOPIC_TYPE_SPECIALISATION(sensor_msgs          ,JointState               )
TOPIC_TYPE_SPECIALISATION(sensor_msgs          ,LaserScan                )
TOPIC_TYPE_SPECIALISATION(sensor_msgs          ,PointCloud2              )
TOPIC_TYPE_SPECIALISATION(tf2_msgs             ,TFMessage                )

template <class T> struct Topic
{
  char* topicname;
  int size_limit=10;
  std::queue<T> 
//  CexprQ< T, 10>
    buffor;
  int putMsg( ros::NodeHandle& n)
  {
    if(buffor.size()==size_limit)
    {
      buffor.pop();
    }
    auto msgPtr=ros::topic::waitForMessage<T>(topicname,n,ros::Duration(0.25));
    buffor.push(*msgPtr);
    return 1;
  }
  int dump(rosbag::Bag& bag)
  {
    while(buffor.size()>0)
    {
        bag.write(topicname,std::max(ros::Time::now(),ros::TIME_MIN),buffor.front());
        buffor.pop();
    }
    return 1;
  }
  ~Topic()
  {
  }
};
    auto getTopicsDefault() //Here we set topic lists
  {
    return std::make_tuple
   (
  TOPIC_INSTANTIATION("/map_metadata",                                                   "nav_msgs/MapMetaData"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/costmap_updates",                       "map_msgs/OccupancyGridUpdate"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/parameter_descriptions",               "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/current_goal",                                         "geometry_msgs/PoseStamped"),
  TOPIC_INSTANTIATION("/scan2",                                                          "sensor_msgs/LaserScan"),
  TOPIC_INSTANTIATION("/move_base/parameter_descriptions",                               "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/footprint",                              "geometry_msgs/PolygonStamped"),
  TOPIC_INSTANTIATION("/move_base/feedback",                                             "move_base_msgs/MoveBaseActionFeedback"),
  TOPIC_INSTANTIATION("/move_base/result",                                               "move_base_msgs/MoveBaseActionResult"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/parameter_updates",                     "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/tf",                                                             "tf2_msgs/TFMessage"),
  TOPIC_INSTANTIATION("/clicked_point",                                                  "geometry_msgs/PointStamped"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/costmap",                                "nav_msgs/OccupancyGrid"),
  TOPIC_INSTANTIATION("/odom",                                                           "nav_msgs/Odometry"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/static_layer/parameter_updates",        "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/scan",                                                           "sensor_msgs/LaserScan"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/global_plan",                          "nav_msgs/Path"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/Ginflater_layer/parameter_updates",      "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/move_base/parameter_updates",                                    "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/local_plan_best",                      "nav_msgs/Path"),
  TOPIC_INSTANTIATION("/move_base/MyNavfnROS/plan",                                      "nav_msgs/Path"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/Ginflater_layer/parameter_descriptions", "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base_simple/goal",                                          "geometry_msgs/PoseStamped"),
  TOPIC_INSTANTIATION("/move_base/status",                                               "actionlib_msgs/GoalStatusArray"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/static_layer/parameter_descriptions",   "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/costmap_updates",                        "map_msgs/OccupancyGridUpdate"),
  TOPIC_INSTANTIATION("/flat_imu",                                                       "sensor_msgs/Imu"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/parameter_descriptions",                 "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/gazebo_gui/parameter_updates",                                   "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/tf_static",                                                      "tf2_msgs/TFMessage"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/inflation_layer/parameter_updates",     "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/imu",                                                            "sensor_msgs/Imu"),
  TOPIC_INSTANTIATION("/gazebo/parameter_descriptions",                                  "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/map",                                                            "nav_msgs/OccupancyGrid"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/parameter_updates",                    "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/cmd_vel",                                                        "geometry_msgs/Twist"),
  TOPIC_INSTANTIATION("/landmark_poses_list",                                            "visualization_msgs/MarkerArray"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/Gobstacles_layer/parameter_updates",     "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/gazebo_gui/parameter_descriptions",                              "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/parameter_updates",                      "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/submap_list",                                                    "cartographer_ros_msgs/SubmapList"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/footprint",                             "geometry_msgs/PolygonStamped"),
  TOPIC_INSTANTIATION("/scan_matched_points2",                                           "sensor_msgs/PointCloud2"),
  TOPIC_INSTANTIATION("/joint_states",                                                   "sensor_msgs/JointState"),
  TOPIC_INSTANTIATION("/rosout",                                                         "rosgraph_msgs/Log"),
  TOPIC_INSTANTIATION("/move_base/goal",                                                 "move_base_msgs/MoveBaseActionGoal"),
  TOPIC_INSTANTIATION("/initialpose",                                                    "geometry_msgs/PoseWithCovarianceStamped"),
  TOPIC_INSTANTIATION("/rosout_agg",                                                     "rosgraph_msgs/Log"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/local_plan",                           "nav_msgs/Path"),
  TOPIC_INSTANTIATION("/recovery_sector_markers",                                        "visualization_msgs/MarkerArray"),
  TOPIC_INSTANTIATION("/trajectory_node_list",                                           "visualization_msgs/MarkerArray"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/cost_cloud",                           "sensor_msgs/PointCloud2"),
  TOPIC_INSTANTIATION("/move_base/MyNavfnROS/visualization_marker",                      "visualization_msgs/MarkerArray"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/inflation_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/obstacle_layer/parameter_descriptions", "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/constraint_list",                                                "visualization_msgs/MarkerArray"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/static_layer/parameter_descriptions",    "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/gazebo/link_states",                                             "gazebo_msgs/LinkStates"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/trajectory_cloud",                     "sensor_msgs/PointCloud2"),
  TOPIC_INSTANTIATION("/gazebo/model_states",                                            "gazebo_msgs/ModelStates"),
  TOPIC_INSTANTIATION("/clock",                                                          "rosgraph_msgs/Clock"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/obstacle_layer/parameter_updates",      "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/Gobstacles_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/static_layer/parameter_updates",         "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/parameter_descriptions",                "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/costmap",                               "nav_msgs/OccupancyGrid"),
  TOPIC_INSTANTIATION("/gazebo/parameter_updates",                                       "dynamic_reconfigure/Config")
   );

  }
//  using DefTopicList=std::result_of<getTopicsDefault>::type;
//  using DefTopicList=typename std::result_of<getTopicsDefault()>::type;


//the other set of topics
   auto getTopicsRecovery()
{
  return std::make_tuple
   (
  TOPIC_INSTANTIATION("/map_metadata",                                                   "nav_msgs/MapMetaData"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/costmap_updates",                       "map_msgs/OccupancyGridUpdate"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/parameter_descriptions",               "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/current_goal",                                         "geometry_msgs/PoseStamped"),
  TOPIC_INSTANTIATION("/scan2",                                                          "sensor_msgs/LaserScan"),
  TOPIC_INSTANTIATION("/move_base/parameter_descriptions",                               "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/footprint",                              "geometry_msgs/PolygonStamped"),
  TOPIC_INSTANTIATION("/move_base/feedback",                                             "move_base_msgs/MoveBaseActionFeedback"),
  TOPIC_INSTANTIATION("/move_base/result",                                               "move_base_msgs/MoveBaseActionResult"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/parameter_updates",                     "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/tf",                                                             "tf2_msgs/TFMessage"),
  TOPIC_INSTANTIATION("/clicked_point",                                                  "geometry_msgs/PointStamped"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/costmap",                                "nav_msgs/OccupancyGrid"),
  TOPIC_INSTANTIATION("/odom",                                                           "nav_msgs/Odometry"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/static_layer/parameter_updates",        "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/scan",                                                           "sensor_msgs/LaserScan"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/global_plan",                          "nav_msgs/Path"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/Ginflater_layer/parameter_updates",      "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/move_base/parameter_updates",                                    "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/local_plan_best",                      "nav_msgs/Path"),
  TOPIC_INSTANTIATION("/move_base/MyNavfnROS/plan",                                      "nav_msgs/Path"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/Ginflater_layer/parameter_descriptions", "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base_simple/goal",                                          "geometry_msgs/PoseStamped"),
  TOPIC_INSTANTIATION("/move_base/status",                                               "actionlib_msgs/GoalStatusArray"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/static_layer/parameter_descriptions",   "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/costmap_updates",                        "map_msgs/OccupancyGridUpdate"),
  TOPIC_INSTANTIATION("/flat_imu",                                                       "sensor_msgs/Imu"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/parameter_descriptions",                 "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/gazebo_gui/parameter_updates",                                   "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/tf_static",                                                      "tf2_msgs/TFMessage"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/inflation_layer/parameter_updates",     "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/imu",                                                            "sensor_msgs/Imu"),
  TOPIC_INSTANTIATION("/gazebo/parameter_descriptions",                                  "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/map",                                                            "nav_msgs/OccupancyGrid"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/parameter_updates",                    "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/cmd_vel",                                                        "geometry_msgs/Twist"),
  TOPIC_INSTANTIATION("/landmark_poses_list",                                            "visualization_msgs/MarkerArray"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/Gobstacles_layer/parameter_updates",     "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/gazebo_gui/parameter_descriptions",                              "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/parameter_updates",                      "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/submap_list",                                                    "cartographer_ros_msgs/SubmapList"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/footprint",                             "geometry_msgs/PolygonStamped"),
  TOPIC_INSTANTIATION("/scan_matched_points2",                                           "sensor_msgs/PointCloud2"),
  TOPIC_INSTANTIATION("/joint_states",                                                   "sensor_msgs/JointState"),
  TOPIC_INSTANTIATION("/rosout",                                                         "rosgraph_msgs/Log"),
  TOPIC_INSTANTIATION("/move_base/goal",                                                 "move_base_msgs/MoveBaseActionGoal"),
  TOPIC_INSTANTIATION("/initialpose",                                                    "geometry_msgs/PoseWithCovarianceStamped"),
  TOPIC_INSTANTIATION("/rosout_agg",                                                     "rosgraph_msgs/Log"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/local_plan",                           "nav_msgs/Path"),
  TOPIC_INSTANTIATION("/recovery_sector_markers",                                        "visualization_msgs/MarkerArray"),
  TOPIC_INSTANTIATION("/trajectory_node_list",                                           "visualization_msgs/MarkerArray"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/cost_cloud",                           "sensor_msgs/PointCloud2"),
  TOPIC_INSTANTIATION("/move_base/MyNavfnROS/visualization_marker",                      "visualization_msgs/MarkerArray"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/inflation_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/obstacle_layer/parameter_descriptions", "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/constraint_list",                                                "visualization_msgs/MarkerArray"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/static_layer/parameter_descriptions",    "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/gazebo/link_states",                                             "gazebo_msgs/LinkStates"),
  TOPIC_INSTANTIATION("/move_base/MyDWAPlannerROS/trajectory_cloud",                     "sensor_msgs/PointCloud2"),
  TOPIC_INSTANTIATION("/gazebo/model_states",                                            "gazebo_msgs/ModelStates"),
  TOPIC_INSTANTIATION("/clock",                                                          "rosgraph_msgs/Clock"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/obstacle_layer/parameter_updates",      "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/Gobstacles_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/local_costmap/static_layer/parameter_updates",         "dynamic_reconfigure/Config"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/parameter_descriptions",                "dynamic_reconfigure/ConfigDescription"),
  TOPIC_INSTANTIATION("/move_base/global_costmap/costmap",                               "nav_msgs/OccupancyGrid"),
  TOPIC_INSTANTIATION("/gazebo/parameter_updates",                                       "dynamic_reconfigure/Config")
   );
}

struct TopicPack
{
  using dT= decltype(getTopicsDefault());
  dT defaultTopics=getTopicsDefault();
  using rT= decltype(getTopicsRecovery());
  rT recoveryTopics=getTopicsRecovery();
  using gT= decltype(getTopicsDefault());
  gT ghostingTopics=getTopicsDefault();  //etc for eventually others sets of topics...


  void recordAnyTopicDefault(rosbag::Bag& bag,ros::NodeHandle &n) //Must be copied and pasted for now. We need std::queue, so Topic is not constexpr and we can't use templates for these methods
  {
    std::apply([&n,&bag](auto&&... args) {((

          args.putMsg(n) 

            ), ...);}, defaultTopics);

  }

  void dumpAnyTopicDefault(rosbag::Bag& bag,ros::NodeHandle &n)
  {
    std::apply([&n,&bag](auto&&... args) {((

          args.dump(bag)

            ), ...);}, defaultTopics);
  }



  void recordAnyTopicRecovery(rosbag::Bag& bag,ros::NodeHandle &n) 
  {
    std::apply([&n,&bag](auto&&... args) {((

          args.putMsg(n) 

            ), ...);}, recoveryTopics);

  }

  void dumpAnyTopicRecovery(rosbag::Bag& bag,ros::NodeHandle &n)
  {
    std::apply([&n,&bag](auto&&... args) {((

          args.dump(bag)

            ), ...);}, recoveryTopics);
  }

};
