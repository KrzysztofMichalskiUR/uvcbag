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


//#include <node_handle.h>
//
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

/*
template<class Arr, Arr arr,int size, int i, unsigned cmpr0,unsigned cmpr1, bool> struct FindInArr
{
  static const unsigned ret=FindInArr<Arr,arr,size,i+1,cmpr0,cmpr1,  ( const_hash(arr[i][0]) == cmpr0 && const_hash(arr[i][1]) ==cmpr1 )  || i>=size>::ret;
}
template<>  struct FindInArr<class Arr, Arr arr, int size, int i, unsigned cmpr0, unsigned cmpr1, true>
{
  static const unsigned ret=i-1;
}
*/
template <class T> struct ITopicType{};
template<unsigned> struct TopicType;

#define TOPIC_TYPE_SPECIALISATION(NAMESPACE,TYPE) template<> struct TopicType< const_hash(#NAMESPACE"/"#TYPE) > : ITopicType< TopicType<const_hash(#NAMESPACE"/"#TYPE)> >\
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

/*
auto Types = array_of<_2strArr  > (

_2strArr{"actionlib_msgs"       ,"GoalStatusArray"}          ,
_2strArr{"cartographer_ros_msgs","SubmapList"}               ,
_2strArr{"dynamic_reconfigure"  ,"Config"}                   ,
_2strArr{"dynamic_reconfigure"  ,"ConfigDescription"}        ,
_2strArr{"gazebo_msgs"          ,"LinkStates"}               ,
_2strArr{"gazebo_msgs"          ,"ModelStates"}              ,
_2strArr{"geometry_msgs"        ,"PointStamped"}             ,
_2strArr{"geometry_msgs"        ,"PolygonStamped"}           ,
_2strArr{"geometry_msgs"        ,"PoseStamped"}              ,
_2strArr{"geometry_msgs"        ,"PoseWithCovarianceStamped"},
_2strArr{"geometry_msgs"        ,"Twist"}                    ,
_2strArr{"map_msgs"             ,"OccupancyGridUpdate"}      ,
_2strArr{"move_base_msgs"       ,"MoveBaseActionFeedback"}   ,
_2strArr{"move_base_msgs"       ,"MoveBaseActionGoal"}       ,
_2strArr{"move_base_msgs"       ,"MoveBaseActionResult"}     ,
_2strArr{"nav_msgs"             ,"MapMetaData"}              ,
_2strArr{"nav_msgs"             ,"OccupancyGrid"}            ,
_2strArr{"nav_msgs"             ,"Odometry"}                 ,
_2strArr{"nav_msgs"             ,"Path"}                     ,
_2strArr{"rosgraph_msgs"        ,"Clock"}                    ,
_2strArr{"rosgraph_msgs"        ,"Log"}                      ,
_2strArr{"sensor_msgs"          ,"Imu"}                      ,
_2strArr{"sensor_msgs"          ,"JointState"}               ,
_2strArr{"sensor_msgs"          ,"LaserScan"}                ,
_2strArr{"sensor_msgs"          ,"PointCloud2"}              ,
_2strArr{"tf2_msgs"             ,"TFMessage"}                ,
_2strArr{"visualization_msgs"   ,"MarkerArray"}              
    );
    */
/*
auto str_Types=std::experimental::make_array
//array_of<char*>
(
  "actionlib_msgs"       "/""GoalStatusArray"          ,
  "cartographer_ros_msgs""/""SubmapList"               ,
  "dynamic_reconfigure"  "/""Config"                   ,
  "dynamic_reconfigure"  "/""ConfigDescription"        ,
  "gazebo_msgs"          "/""LinkStates"               ,
  "gazebo_msgs"          "/""ModelStates"              ,
  "geometry_msgs"        "/""PointStamped"             ,
  "geometry_msgs"        "/""PolygonStamped"           ,
  "geometry_msgs"        "/""PoseStamped"              ,
  "geometry_msgs"        "/""PoseWithCovarianceStamped",
  "geometry_msgs"        "/""Twist"                    ,
  "map_msgs"             "/""OccupancyGridUpdate"      ,
  "move_base_msgs"       "/""MoveBaseActionFeedback"   ,
  "move_base_msgs"       "/""MoveBaseActionGoal"       ,
  "move_base_msgs"       "/""MoveBaseActionResult"     ,
  "nav_msgs"             "/""MapMetaData"              ,
  "nav_msgs"             "/""OccupancyGrid"            ,
  "nav_msgs"             "/""Odometry"                 ,
  "nav_msgs"             "/""Path"                     ,
  "rosgraph_msgs"        "/""Clock"                    ,
  "rosgraph_msgs"        "/""Log"                      ,
  "sensor_msgs"          "/""Imu"                      ,
  "sensor_msgs"          "/""JointState"               ,
  "sensor_msgs"          "/""LaserScan"                ,
  "sensor_msgs"          "/""PointCloud2"              ,
  "tf2_msgs"             "/""TFMessage"                ,
  "visualization_msgs"   "/""MarkerArray"              
);
*/

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
TOPIC_TYPE_SPECIALISATION(visualization_msgs   ,MarkerArray              )
/*
std::tuple <
  TopicType<  const_hash("actionlib_msgs"       "/""GoalStatusArray"           ) >,
  TopicType<  const_hash("cartographer_ros_msgs""/""SubmapList"                ) >,
  TopicType<  const_hash("dynamic_reconfigure"  "/""Config"                    ) >,
  TopicType<  const_hash("dynamic_reconfigure"  "/""ConfigDescription"         ) >,
  TopicType<  const_hash("gazebo_msgs"          "/""LinkStates"                ) >,
  TopicType<  const_hash("gazebo_msgs"          "/""ModelStates"               ) >,
  TopicType<  const_hash("geometry_msgs"        "/""PointStamped"              ) >,
  TopicType<  const_hash("geometry_msgs"        "/""PolygonStamped"            ) >,
  TopicType<  const_hash("geometry_msgs"        "/""PoseStamped"               ) >,
  TopicType<  const_hash("geometry_msgs"        "/""PoseWithCovarianceStamped" ) >,
  TopicType<  const_hash("geometry_msgs"        "/""Twist"                     ) >,
  TopicType<  const_hash("map_msgs"             "/""OccupancyGridUpdate"       ) >,
  TopicType<  const_hash("move_base_msgs"       "/""MoveBaseActionFeedback"    ) >,
  TopicType<  const_hash("move_base_msgs"       "/""MoveBaseActionGoal"        ) >,
  TopicType<  const_hash("move_base_msgs"       "/""MoveBaseActionResult"      ) >,
  TopicType<  const_hash("nav_msgs"             "/""MapMetaData"               ) >,
  TopicType<  const_hash("nav_msgs"             "/""OccupancyGrid"             ) >,
  TopicType<  const_hash("nav_msgs"             "/""Odometry"                  ) >,
  TopicType<  const_hash("nav_msgs"             "/""Path"                      ) >,
  TopicType<  const_hash("rosgraph_msgs"        "/""Clock"                     ) >,
  TopicType<  const_hash("rosgraph_msgs"        "/""Log"                       ) >,
  TopicType<  const_hash("sensor_msgs"          "/""Imu"                       ) >,
  TopicType<  const_hash("sensor_msgs"          "/""JointState"                ) >,
  TopicType<  const_hash("sensor_msgs"          "/""LaserScan"                 ) >,
  TopicType<  const_hash("sensor_msgs"          "/""PointCloud2"               ) >,
  TopicType<  const_hash("tf2_msgs"             "/""TFMessage"                 ) >,
  TopicType<  const_hash("visualization_msgs"   "/""MarkerArray"               ) >
> __Types;
*/


template <class T> struct Topic
{
  char* topicname;
  int size_limit=10;
  std::queue<T> buffor;
  int putMsg( ros::NodeHandle& n)
  {
    if(buffor.size()>=size_limit)
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
        bag.write(topicname,std::max(ros::Time::now(),ros::TIME_MIN),buffor.pop_front());
    }
    return 1;
  }
  ~Topic()
  {
  }
};
/*
template<auto a> struct TupleHashMap
{
  constexpr auto arr=std::transform(a.begin(),a.end(),[](auto aa)
      {
        return Topic<typename TopicType<const_hash(aa[0])>::type>{aa[1]};
      }
      );
  static constexpr frozen::unordered_map<int,unsigned,decltype(a)::size()> map=std::transform(a.begin(),a.end(),[](auto aa)
      {
        static int id=0;
        return std::pair<unsigned,int>{const_hash(aa[0]),id++};
      });
  static constexpr auto get(unsigned h)
  {
    return std::get<map[h]>(arr);
  }
};
*/
/*
template<auto input,auto fun> struct ConstComprehension
{
  static constexpr auto output=std::transform(input.begin(),input.end(),fun);
};
*/
/*
constexpr auto topics_arr=
//    array_of<_2strArr>
    std::make_tuple
    (
  _2strArr{"/map_metadata",                                                   "nav_msgs/MapMetaData"},
  _2strArr{"/move_base/global_costmap/costmap_updates",                       "map_msgs/OccupancyGridUpdate"},
  _2strArr{"/move_base/MyDWAPlannerROS/parameter_descriptions",               "dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/move_base/current_goal",                                         "geometry_msgs/PoseStamped"},
  _2strArr{"/scan2",                                                          "sensor_msgs/LaserScan"},
  _2strArr{"/move_base/parameter_descriptions",                               "dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/move_base/local_costmap/footprint",                              "geometry_msgs/PolygonStamped"},
  _2strArr{"/move_base/feedback",                                             "move_base_msgs/MoveBaseActionFeedback"},
  _2strArr{"/move_base/result",                                               "move_base_msgs/MoveBaseActionResult"},
  _2strArr{"/move_base/global_costmap/parameter_updates",                     "dynamic_reconfigure/Config"},
  _2strArr{"/tf",                                                             "tf2_msgs/TFMessage"},
  _2strArr{"/clicked_point",                                                  "geometry_msgs/PointStamped"},
  _2strArr{"/move_base/local_costmap/costmap",                                "nav_msgs/OccupancyGrid"},
  _2strArr{"/odom",                                                           "nav_msgs/Odometry"},
  _2strArr{"/move_base/global_costmap/static_layer/parameter_updates",        "dynamic_reconfigure/Config"},
  _2strArr{"/scan",                                                           "sensor_msgs/LaserScan"},
  _2strArr{"/move_base/MyDWAPlannerROS/global_plan",                          "nav_msgs/Path"},
  _2strArr{"/move_base/local_costmap/Ginflater_layer/parameter_updates",      "dynamic_reconfigure/Config"},
  _2strArr{"/move_base/parameter_updates",                                    "dynamic_reconfigure/Config"},
  _2strArr{"/move_base/MyDWAPlannerROS/local_plan_best",                      "nav_msgs/Path"},
  _2strArr{"/move_base/MyNavfnROS/plan",                                      "nav_msgs/Path"},
  _2strArr{"/move_base/local_costmap/Ginflater_layer/parameter_descriptions", "dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/move_base_simple/goal",                                          "geometry_msgs/PoseStamped"},
  _2strArr{"/move_base/status",                                               "actionlib_msgs/GoalStatusArray"},
  _2strArr{"/move_base/global_costmap/static_layer/parameter_descriptions",   "dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/move_base/local_costmap/costmap_updates",                        "map_msgs/OccupancyGridUpdate"},
  _2strArr{"/flat_imu",                                                       "sensor_msgs/Imu"},
  _2strArr{"/move_base/local_costmap/parameter_descriptions",                 "dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/gazebo_gui/parameter_updates",                                   "dynamic_reconfigure/Config"},
  _2strArr{"/tf_static",                                                      "tf2_msgs/TFMessage"},
  _2strArr{"/move_base/global_costmap/inflation_layer/parameter_updates",     "dynamic_reconfigure/Config"},
  _2strArr{"/imu",                                                            "sensor_msgs/Imu"},
  _2strArr{"/gazebo/parameter_descriptions",                                  "dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/map",                                                            "nav_msgs/OccupancyGrid"},
  _2strArr{"/move_base/MyDWAPlannerROS/parameter_updates",                    "dynamic_reconfigure/Config"},
  _2strArr{"/cmd_vel",                                                        "geometry_msgs/Twist"},
  _2strArr{"/landmark_poses_list",                                            "visualization_msgs/MarkerArray"},
  _2strArr{"/move_base/local_costmap/Gobstacles_layer/parameter_updates",     "dynamic_reconfigure/Config"},
  _2strArr{"/gazebo_gui/parameter_descriptions",                              "dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/move_base/local_costmap/parameter_updates",                      "dynamic_reconfigure/Config"},
  _2strArr{"/submap_list",                                                    "cartographer_ros_msgs/SubmapList"},
  _2strArr{"/move_base/global_costmap/footprint",                             "geometry_msgs/PolygonStamped"},
  _2strArr{"/scan_matched_points2",                                           "sensor_msgs/PointCloud2"},
  _2strArr{"/joint_states",                                                   "sensor_msgs/JointState"},
  _2strArr{"/rosout",                                                         "rosgraph_msgs/Log"},
  _2strArr{"/move_base/goal",                                                 "move_base_msgs/MoveBaseActionGoal"},
  _2strArr{"/initialpose",                                                    "geometry_msgs/PoseWithCovarianceStamped"},
  _2strArr{"/rosout_agg",                                                     "rosgraph_msgs/Log"},
  _2strArr{"/move_base/MyDWAPlannerROS/local_plan",                           "nav_msgs/Path"},
  _2strArr{"/recovery_sector_markers",                                        "visualization_msgs/MarkerArray"},
  _2strArr{"/trajectory_node_list",                                           "visualization_msgs/MarkerArray"},
  _2strArr{"/move_base/MyDWAPlannerROS/cost_cloud",                           "sensor_msgs/PointCloud2"},
  _2strArr{"/move_base/MyNavfnROS/visualization_marker",                      "visualization_msgs/MarkerArray"},
  _2strArr{"/move_base/global_costmap/inflation_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/move_base/global_costmap/obstacle_layer/parameter_descriptions", "dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/constraint_list",                                                "visualization_msgs/MarkerArray"},
  _2strArr{"/move_base/local_costmap/static_layer/parameter_descriptions",    "dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/gazebo/link_states",                                             "gazebo_msgs/LinkStates"},
  _2strArr{"/move_base/MyDWAPlannerROS/trajectory_cloud",                     "sensor_msgs/PointCloud2"},
  _2strArr{"/gazebo/model_states",                                            "gazebo_msgs/ModelStates"},
  _2strArr{"/clock",                                                          "rosgraph_msgs/Clock"},
  _2strArr{"/move_base/global_costmap/obstacle_layer/parameter_updates",      "dynamic_reconfigure/Config"},
  _2strArr{"/move_base/local_costmap/Gobstacles_layer/parameter_descriptions","dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/move_base/local_costmap/static_layer/parameter_updates",         "dynamic_reconfigure/Config"},
  _2strArr{"/move_base/global_costmap/parameter_descriptions",                "dynamic_reconfigure/ConfigDescription"},
  _2strArr{"/move_base/global_costmap/costmap",                               "nav_msgs/OccupancyGrid"},
  _2strArr{"/gazebo/parameter_updates",                                       "dynamic_reconfigure/Config"}
   );
*/

  // constexpr 
   auto TopicsDefault=std::make_tuple
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



   auto TopicsRecovery=std::make_tuple
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



   /*
    *
auto topicsDefault= std::transform(
    topics_arr.begin(),
    topics_arr.end(),
    [](auto _topic_type)
      {
        return Topic<typename TopicType<const_hash( _topic_type[1] )>::type >{_topic_type[0]};
      },
      topics_arr.size()
    );
    */
   /*
constexpr auto _2strToTopic=[](const _2strArr _topic_type)
{
        constexpr  Topic<typename TopicType<const_hash( _topic_type[1] )>::type >{_topic_type[0]} t;
        return t;
};*/
   /*
 template<auto _topic_type> struct Str2Topic 
{
  static auto topic=Topic<typename TopicType<const_hash( _topic_type[1] )>::type >{_topic_type[0]} t;

}
template<auto v,class fun,int i=0, bool _=true,auto ... args>
struct Comprehesion
{
  static auto output=std::tuple_cat( std::make_tuple( fun<std::get<i>(v)>::topic,args...) ),Comprehesion< v, fun, i+1, i<std::tuple_size(v), args ...>::output );
};
*/
   /*
template<auto v,auto fun,int i,auto ... args>
struct Comprehesion<v,fun,i, false,args...>
{
  static auto output=std::make_tuple( fun(std::get<i>(v),args...) );
};
*/

/*
auto topicsDefault= Comprehesion<topics_arr,
     [][](const _2strArr _topic_type)
     {
        return Topic<typename TopicType<const_hash( _topic_type[1] )>::type >{_topic_type[0]};
     }
     >::output;
     */
/*
std::vector<std::string> strSplit(std::string s,std::string token)
{
  std::vector<std::string> ret;

  std::stringstream ss(s);
  std::string ns;
  while (std::getline(ss, ns, token)) 
 {
    ret.push_back(ns);
 }
  return ret;

}
*/
/*
template< class Arr,class Fun,class ...Args> struct ConstexprForHeader
{
  static Arr arr;
  static Fun fun;
  static std::tuple<Args...> args;
};
template<class Header,int size,int i,bool> struct ConstexprFor
{
  static constexpr auto ret= std::tuple_cat(std::make_tuple(Header::fun(Header::arr[i],Header::args)),ConstexprFor<Header, size, i+1,i<size >::ret);
};
template <class Header,int size,int i>struct ConstexprFor<Header,size,i,false> 
{
*/
//  static constexpr auto ret= std::make_tuple(Header::fun(Header::arr[i],Header::args));
//};

/*
template<class _Arr,class _Fun,class... _Args> struct ComprehensionTypes
{
  using Arr=_Arr;
  using Fun=_Fun;
  using Args=std::tuple<_Args...>;

  static Arr arr;
  static Fun fun;
  static static Args args; 
};
template <class CompTypes,int i,int size,bool> struct Comprehension
{
  return std::tuple_cat(std::make_tuple(CompTypes::fun(Co

};
*/
/*
template<class Header,int size,int i,bool> struct ConstexprForTuple
{
  static constexpr auto ret= std::tuple_cat(std::make_tuple(Header::fun(std::get<i>(Header::arr),Header::args)),ConstexprFor<Header, size, i+1,i<size >::ret);
};
template <class Header,int size,int i>struct ConstexprForTuple<Header,size,i,false> 
{
  static constexpr auto ret= std::make_tuple(Header::fun(std::get<i>(Header::arr),Header::args));
};
class Doer
{
  template<class T,class A> void operator(T topic,A args)
  {
    static auto fun=[](T t,unsigned hash,rosbag::Bag& bag,std::string topic)
    {
      if(T::hash==hash)
      {
        dumpTopic<T>(bag,topic);
      }
    }
    std::apply(fun,std::cat_tuple(std::make_tuple(t),args));
  }
};
*/

void recordAnyTopic(rosbag::Bag& bag,ros::NodeHandle &n)
{
  /*
  for( auto& t :Topics)
  {
    t.putMsg(n);
    t.dump(bag);
  }
  */
  std::apply([&n,&bag](auto&&... args) {((

        args.putMsg(n) 
        //&&
//        args.dump(bag)

          ), ...);}, Topics);

}

void recordAnyTopic(rosbag::Bag& bag,ros::NodeHandle &n)
{
  /*
  for( auto& t :Topics)
  {
    t.putMsg(n);
    t.dump(bag);
  }
  */
  std::apply([&n,&bag](auto&&... args) {((

        args.putMsg(n) 
        //&&
//        args.dump(bag)

          ), ...);}, Topics);

}
