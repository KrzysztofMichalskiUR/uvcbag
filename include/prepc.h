# 1 "noncl.h"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 1 "<command-line>" 2
# 1 "noncl.h"
       
# 52 "noncl.h"
std::map<std::string,int> typeLookup=
{
 {"actionlib_msgs/GoalStatusArray",0},
 {"cartographer_ros_msgs/SubmapList",1},
 {"dynamic_reconfigure/Config",2},
 {"dynamic_reconfigure/ConfigDescription",3},
 {"gazebo_msgs/LinkStates",4},
 {"gazebo_msgs/ModelStates",5},
 {"geometry_msgs/PointStamped",6},
 {"geometry_msgs/PolygonStamped",7},
 {"geometry_msgs/PoseStamped",8},
 {"geometry_msgs/PoseWithCovarianceStamped",9},
 {"geometry_msgs/Twist",10},
 {"map_msgs/OccupancyGridUpdate",11},
 {"move_base_msgs/MoveBaseActionFeedback",13},
 {"move_base_msgs/MoveBaseActionGoal",14},
 {"move_base_msgs/MoveBaseActionResult",15},
 {"nav_msgs/MapMetaData",16},
 {"nav_msgs/OccupancyGrid",17},
 {"nav_msgs/Odometry",18},
 {"nav_msgs/Path",19},
 {"rosgraph_msgs/Clock",20},
 {"rosgraph_msgs/Log",21},
 {"sensor_msgs/Imu",22},
 {"sensor_msgs/JointState",23},
 {"sensor_msgs/LaserScan",24},
 {"sensor_msgs/PointCloud2",25},
 {"tf2_msgs/TFMessage",26},
 {"visualization_msgs/MarkerArray",27}
};
template <class T> class StatefullCb
{
 public:
 std::deque<T> msgs;
 void operator()(const T & msg)
 {
  msgs.push_back(msg);
 }
 void _cb(const T & msg)
 {
  std::cout<<"Went into inner callback"<<std::endl;
  msgs.push_back(msg);
 }
};

template <class T> void dumpTopic(ros::NodeHandle &n,rosbag::Bag& bag,std::string topic)
{
 int msgNum=10;

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
  bag.write(topic,ros::Time::now(),*msgPtr);
 }
 std::cout<<std::endl;
}

void dumpAnyTopic(ros::NodeHandle& n,rosbag::Bag& bag,std::string topic,std::string type)
{
 std::cout<<"typeLookup["<<type<<"]="<<typeLookup[type]<<std::endl;
 switch(typeLookup[type])
 {

  case 1: {dumpTopic<cartographer_ros_msgs::SubmapList>(n,bag,topic);break;};
  case 2: {dumpTopic<dynamic_reconfigure::Config>(n,bag,topic);break;};
  case 3: {dumpTopic<dynamic_reconfigure::ConfigDescription>(n,bag,topic);break;};
  case 4: {dumpTopic<gazebo_msgs::LinkStates>(n,bag,topic);break;};
  case 5: {dumpTopic<gazebo_msgs::ModelStates>(n,bag,topic);break;};
  case 6: {dumpTopic<geometry_msgs::PointStamped>(n,bag,topic);break;};
  case 7: {dumpTopic<geometry_msgs::PolygonStamped>(n,bag,topic);break;};
  case 8: {dumpTopic<geometry_msgs::PoseStamped>(n,bag,topic);break;};
  case 9: {dumpTopic<geometry_msgs::PoseWithCovarianceStamped>(n,bag,topic);break;};
  case 10: {dumpTopic<geometry_msgs::Twist>(n,bag,topic);break;};
  case 11: {dumpTopic<map_msgs::OccupancyGridUpdate>(n,bag,topic);break;};
  case 13: {dumpTopic<move_base_msgs::MoveBaseActionFeedback>(n,bag,topic);break;};
  case 14: {dumpTopic<move_base_msgs::MoveBaseActionGoal>(n,bag,topic);break;};
  case 15: {dumpTopic<move_base_msgs::MoveBaseActionResult>(n,bag,topic);break;};
  case 16: {dumpTopic<nav_msgs::MapMetaData>(n,bag,topic);break;};
  case 17: {dumpTopic<nav_msgs::OccupancyGrid>(n,bag,topic);break;};
  case 18: {dumpTopic<nav_msgs::Odometry>(n,bag,topic);break;};
  case 19: {dumpTopic<nav_msgs::Path>(n,bag,topic);break;};
  case 20: {dumpTopic<rosgraph_msgs::Clock>(n,bag,topic);break;};
  case 21: {dumpTopic<rosgraph_msgs::Log>(n,bag,topic);break;};
  case 22: {dumpTopic<sensor_msgs::Imu>(n,bag,topic);break;};
  case 23: {dumpTopic<sensor_msgs::JointState>(n,bag,topic);break;};
  case 24: {dumpTopic<sensor_msgs::LaserScan>(n,bag,topic);break;};
  case 25: {dumpTopic<sensor_msgs::PointCloud2>(n,bag,topic);break;};
  case 26: {dumpTopic<tf2_msgs::TFMessage>(n,bag,topic);break;};
  case 27: {dumpTopic<visualization_msgs::MarkerArray>(n,bag,topic);break;};
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
};
