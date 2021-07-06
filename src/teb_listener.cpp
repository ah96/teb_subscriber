// rosbag record /mobile_base_controller/odom /mobile_base_controller/cmd_vel /move_base/TebLocalPlannerROS/global_plan /move_base/TebLocalPlannerROS/local_plan /move_base/GlobalPlanner/plan /move_base/current_goal /amcl_pose /move_base/local_costmap/costmap /move_base/global_costmap/costmap /map /tf /move_base/local_costmap/footprint
// rosrun --prefix 'gdb -ex run --args' teb_subscriber teb_listener


// Ulazi se upisuju u svoje .csv fajlove samo kad se cmd_vel ispisuje u svoj .csv fajl

#include <ros/ros.h>

#include <rosgraph_msgs/Clock.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Twist.h>

#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseArray.h>

#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>

#include <costmap_converter/ObstacleArrayMsg.h>

#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

//#include <tf/transform_listener.h>

#include <tf2_ros/transform_listener.h>

#include <iostream>

#include <fstream>

#include <string>

#include <geometry_msgs/PolygonStamped.h>

#include <geometry_msgs/Polygon.h>


// ofstream writers global variables
std::ofstream writer1;
std::ofstream writer2;
std::ofstream writer3;
std::ofstream writer4;
std::ofstream writer5;
std::ofstream writer6;
std::ofstream writer7;
std::ofstream writer8;
std::ofstream writer9;
std::ofstream writer10;
std::ofstream writer11;
std::ofstream writer12;
std::ofstream writer13;
std::ofstream writer14;
std::ofstream writer15;
std::ofstream writer16;
std::ofstream writer17;
std::ofstream writer18;
std::ofstream writer19;
std::ofstream writer20;
std::ofstream writer21;

// output .csv files' names global variables
std::string fn1 = "~/amar_ws/Dataset/clock.csv";
std::string fn2 = "~/amar_ws/Dataset/odom.csv";
std::string fn3 = "~/amar_ws/Dataset/cmd_vel.csv";
std::string fn4 = "~/amar_ws/Dataset/teb_global_plan.csv";
std::string fn5 = "~/amar_ws/Dataset/teb_local_plan.csv";
std::string fn6 = "~/amar_ws/Dataset/teb_poses.csv";
std::string fn7 = "~/amar_ws/Dataset/teb_markers.csv";
std::string fn8 = "~/amar_ws/Dataset/current_goal.csv";
std::string fn9 = "~/amar_ws/Dataset/obstacles.csv";
std::string fn10 = "~/amar_ws/Dataset/via_points.csv";
std::string fn11 = "~/amar_ws/Dataset/local_costmap_data.csv";
std::string fn12 = "~/amar_ws/Dataset/local_costmap_info.csv";
std::string fn13 = "~/amar_ws/Dataset/plan.csv";
std::string fn14 = "~/amar_ws/Dataset/amcl_pose.csv";
std::string fn15 = "~/amar_ws/Dataset/tf_odom_map.csv";
std::string fn16 = "~/amar_ws/Dataset/tf_map_odom.csv";
std::string fn17 = "~/amar_ws/Dataset/global_costmap_info.csv";
std::string fn18 = "~/amar_ws/Dataset/global_costmap_data.csv";
std::string fn19 = "~/amar_ws/Dataset/map_info.csv";
std::string fn20 = "~/amar_ws/Dataset/map_data.csv";
std::string fn21 = "~/amar_ws/Dataset/footprints.csv";

// odometry global variables
float odomPositionX = 0.0, odomPositionY = 0.0, odomOrientationZ = 0.0, odomOrientationW = 0.0, odomVelLinX = 0.0, odomVelAngZ = 0.0;
std::string odomFrame = "None", odomChildFrame = "None";

// amcl global variables
float amclPositionX = 0.0, amclPositionY = 0.0, amclOrientationZ = 0.0, amclOrientationW = 0.0;
std::string amclFrame = "None";

// local_costmap global variables
float resolution = 0.0;
uint32_t width = 0, height = 0;
float originPosX = 0.0, originPosY = 0.0, originOrientationZ = 0.0, originOrientationW = 0.0;
std::vector<signed char> data(1600);
std::string localCostmapFrame = "None";

// global_costmap global variables
float global_resolution = 0.0;
uint32_t global_width = 0, global_height = 0;
float global_originPosX = 0.0, global_originPosY = 0.0, global_originOrientationZ = 0.0, global_originOrientationW = 0.0;
std::vector<signed char> global_data(60588);
std::string globalCostmapFrame = "None";

// map global variables
float map_resolution = 0.0;
uint32_t map_width = 0, map_height = 0;
float map_originPosX = 0.0, map_originPosY = 0.0, map_originOrientationZ = 0.0, map_originOrientationW = 0.0;
std::vector<signed char> map_data(60588);
std::string mapFrame = "None";

// plans' global variables
std::vector<std::vector<float>> plan, localPlan, globalPlan;
std::string planFrame = "None", localPlanFrame = "None", globalPlanFrame = "None";

// plans' counter global variable
int planCounter = 0;

// transform listener and buffer global variables
//tf::TransformListener *tf_;
tf2_ros::Buffer *tfBuffer;
tf2_ros::TransformListener *tfListener;

// footprint global variables
std::string	footprintFrame = "None";
geometry_msgs::Polygon	polygon;	



// function for writing headers to .csv files
// ovdje se po potrebi moze dodati upis naslova/headera/imena kolona u costmap_data
void UpisiNaslove(void)
{
  // cmd_vel	
  writer3.open(fn3, std::ios::app);

  std::string info = "cmd_vel_lin_x, cmd_vel_ang_z";
  writer3 << info << std::endl;
  
  writer3.close();
  
  
  // odometry
  writer2.open(fn2, std::ios::app);

  info = "frame,child_frame,odom_position_x,odom_position_y,odom_orientation_z,odom_orientation_w,odom_linear_x,odom_angular_z";
  writer2 << info << std::endl;
  
  writer2.close();
  
  
  // global plan
  writer4.open(fn4, std::ios::app);

  info = "frame,global_plan_position_x,global_plan_position_y,global_plan_orientation_z,global_plan_orientation_w,ID";
  writer4 << info << std::endl;
  
  writer4.close();
  
  
  // local plan
  writer5.open(fn5, std::ios::app);

  info = "frame,local_plan_position_x,local_plan_position_y,local_plan_orientation_z,local_plan_orientation_w,ID";
  writer5 << info << std::endl;
  
  writer5.close();
 
  // current_goal
  writer8.open(fn8, std::ios::app);

  info =  "frame,current_goal_position_x,current_goal_position_y,current_goal_orientation_z,current_goal_orientation_w";
  writer8 << info << std::endl;
  
  writer8.close();

  
  // local_costmap_info
  writer12.open(fn12, std::ios::app);

  info =  "frame,local_costmap_resolution,local_costmap_width,local_costmap_height,local_costmap_origin_pos_x,local_costmap_origin_pos_y,local_costmap_origin_orientation_z,local_costmap_origin_orientation_w";
  writer12 << info << std::endl;
  
  writer12.close();
  
  
  // plan from global_planner
  writer13.open(fn13, std::ios::app);

  info = "frame,plan_position_x,plan_position_y,plan_orientation_z,plan_orientation_w,ID";
  writer13 << info << std::endl;
  
  writer13.close();
  
  
  // amcl (global) pose
  writer14.open(fn14, std::ios::app);

  info = "frame,amcl_position_x,amcl_position_y,amcl_orientation_z,amcl_orientation_w";
  writer14 << info << std::endl;
  
  writer14.close();
  
  // tf_odom_map
  writer15.open(fn15, std::ios::app);

  info = "tf_odom_map_tran_x,tf_odom_map_tran_y,tf_odom_map_tran_z,tf_odom_map_rot_x,tf_odom_map_rot_y,tf_odom_map_rot_z,tf_odom_map_rot_w";
  writer15 << info << std::endl;
  
  writer15.close();
  
  // tf_map_odom
  writer16.open(fn16, std::ios::app);

  info = "tf_map_odom_tran_x,tf_map_odom_tran_y,tf_map_odom_tran_z,tf_map_odom_rot_x,tf_map_odom_rot_y,tf_map_odom_rot_z,tf_map_odom_rot_w";
  writer16 << info << std::endl;
  
  writer16.close();
  
  //global_costmap_info
  writer17.open(fn17, std::ios::app);

  info =  "frame,global_costmap_resolution,global_costmap_width,global_costmap_height,global_costmap_origin_pos_x,global_costmap_origin_pos_y,global_costmap_origin_orientation_z,global_costmap_origin_orientation_w";
  writer17 << info << std::endl;
  
  writer17.close();
  
  // map_info
  writer19.open(fn19, std::ios::app);

  info =  "frame,map_costmap_resolution,map_costmap_width,map_costmap_height,map_costmap_origin_pos_x,map_costmap_origin_pos_y,map_costmap_origin_orientation_z,map_costmap_origin_orientation_w";
  writer19 << info << std::endl;
  
  writer19.close();
  
  // footprints
  writer21.open(fn21, std::ios::app);

  info = "frame,footprint_x,footprint_y,footprint_z,ID";
  writer21 << info << std::endl;
  
  writer21.close();
}


void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // getting transformations between map and odom frames	
  geometry_msgs::TransformStamped transformStamped1;
  geometry_msgs::TransformStamped transformStamped2;

  transformStamped1 = tfBuffer->lookupTransform("map", "odom", ros::Time(0));
  transformStamped2 = tfBuffer->lookupTransform("odom", "map", ros::Time(0));

  //std::cout << transformStamped.transform.translation.x << " " << transformStamped.transform.translation.y << " " << transformStamped.transform.rotation << std::endl;
  
  // tf_odom_map
  writer15.open(fn15, std::ios::app);

  std::string info = std::to_string(transformStamped1.transform.translation.x) + "," + std::to_string(transformStamped1.transform.translation.y) + "," + std::to_string(transformStamped1.transform.translation.z) + "," 
							+ std::to_string(transformStamped1.transform.rotation.x) + "," + std::to_string(transformStamped1.transform.rotation.y) + "," 
							+ std::to_string(transformStamped1.transform.rotation.z) + "," + std::to_string(transformStamped1.transform.rotation.w);
  writer15 << info << std::endl;
  
  writer15.close();
  
  // tf_map_odom
  writer16.open(fn16, std::ios::app);

  info = std::to_string(transformStamped2.transform.translation.x) + "," + std::to_string(transformStamped2.transform.translation.y) + "," + std::to_string(transformStamped2.transform.translation.z) + "," 
							+ std::to_string(transformStamped2.transform.rotation.x) + "," + std::to_string(transformStamped2.transform.rotation.y) + "," 
							+ std::to_string(transformStamped2.transform.rotation.z) + "," + std::to_string(transformStamped2.transform.rotation.w);
  writer16 << info << std::endl;
  
  writer16.close();

  // cmd_vel
  writer3.open(fn3, std::ios::app);

  info = std::to_string(msg->linear.x) + "," + std::to_string(msg->angular.z);
  writer3 << info << std::endl;
  
  writer3.close();
  
  
  // odometry
  writer2.open(fn2, std::ios::app);

  info = odomFrame + "," + odomChildFrame + "," + std::to_string(odomPositionX) + "," + std::to_string(odomPositionY) + "," + std::to_string(odomOrientationZ) + "," 
							+ std::to_string(odomOrientationW) + "," + std::to_string(odomVelLinX) + "," + std::to_string(odomVelAngZ);
  writer2 << info << std::endl;
  
  writer2.close();
  
  
  // global plan
  writer4.open(fn4, std::ios::app);

  for(int i = 0; i < globalPlan.size(); i++)
  {
		info = globalPlanFrame + "," + std::to_string(globalPlan[i][0]) + "," + std::to_string(globalPlan[i][1]) + "," + std::to_string(globalPlan[i][2]) + "," + std::to_string(globalPlan[i][3]) + ',' + std::to_string(planCounter);
		writer4 << info << std::endl;
  }
  writer4 << std::endl;
   
  writer4.close();
  
  
  // local plan
  writer5.open(fn5, std::ios::app);

  for(int i = 0; i < localPlan.size(); i++)
  {
		info = localPlanFrame + "," + std::to_string(localPlan[i][0]) + "," + std::to_string(localPlan[i][1]) + "," + std::to_string(localPlan[i][2]) + "," + std::to_string(localPlan[i][3]) + ',' + std::to_string(planCounter);
		writer5 << info << std::endl;
  }
  writer5 << std::endl;
  
  writer5.close();
      
   
  // plan from global_planner
  writer13.open(fn13, std::ios::app);

  for(int i = 0; i < plan.size(); i++)
  {
		info = planFrame + "," + std::to_string(plan[i][0]) + "," + std::to_string(plan[i][1]) + "," + std::to_string(plan[i][2]) + "," + std::to_string(plan[i][3]) + ',' + std::to_string(planCounter);
		writer13 << info << std::endl;
  }
  writer13 << std::endl;
  
  writer13.close();
  
  // footprint
  writer21.open(fn21, std::ios::app);

  for(int i = 0; i < polygon.points.size(); i++)
  {
		info = footprintFrame + "," + std::to_string(polygon.points[i].x) + "," + std::to_string(polygon.points[i].y) + "," + std::to_string(polygon.points[i].z) + "," + std::to_string(planCounter);
		writer21 << info << std::endl;
  }
  writer21 << std::endl;
  
  writer21.close();
  
  
  planCounter++;
  

  // amcl pose
  writer14.open(fn14, std::ios::app);

  info = amclFrame + "," + std::to_string(amclPositionX) + "," + std::to_string(amclPositionY) + "," + std::to_string(amclOrientationZ) + "," 
							+ std::to_string(amclOrientationW);
  writer14 << info << std::endl;
  
  writer14.close();

 
  // local_costmap_info
  std::cout << "local: " << height << " " << width << std::endl;
  
  writer12.open(fn12, std::ios::app);

  info = localCostmapFrame + "," + std::to_string(resolution) + "," + std::to_string(width) + "," + std::to_string(height) + "," + 
			std::to_string(originPosX) + "," + std::to_string(originPosY) + "," +
			std::to_string(originOrientationZ) + "," + std::to_string(originOrientationW);
  writer12 << info << std::endl;
  
  writer12.close();
 
  // local_costmap_data
  writer11.open(fn11, std::ios::app);

  for(int i = 0; i < height; i++)
  {
	  for(int j = 0; j < width; j++)
	  {
		  writer11 << static_cast<int>(data.at(i*width + j)) << ","; // ovdje se moze ispraviti pisanje zarez iza posljednjeg elementa u svakom redu
	  }
	  writer11 << std::endl;
  }
  writer11 << std::endl; // ovo dodje kao pomoc: u slucaju kada nema spremne costmape ispisuje se prazan red
  
  writer11.close();
}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odomFrame = msg->header.frame_id; 
	odomChildFrame = msg->child_frame_id;
	odomPositionX = msg->pose.pose.position.x; 
	odomPositionY = msg->pose.pose.position.y; 
	odomOrientationZ = msg->pose.pose.orientation.z; 
	odomOrientationW = msg->pose.pose.orientation.w; 
	odomVelLinX = msg->twist.twist.linear.x;
	odomVelAngZ = msg->twist.twist.angular.z;
}


void TebGlobalPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
	globalPlan.clear();
	
	globalPlanFrame = msg->poses.begin()->header.frame_id;
	
	int i=0;
    for(std::vector<geometry_msgs::PoseStamped>::const_iterator it = msg->poses.begin(); it!= msg->poses.end();  ++it)
    {
		std::vector<float> temp;
		geometry_msgs::PoseStamped tmp = *it;
		temp.push_back(tmp.pose.position.x);
		temp.push_back(tmp.pose.position.y);
		temp.push_back(tmp.pose.orientation.z);
		temp.push_back(tmp.pose.orientation.w);
		globalPlan.push_back(temp);
		i++;
    } 
}


void TebLocalPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
	localPlan.clear();
	
	localPlanFrame = msg->poses.begin()->header.frame_id;
	
	int i=0;
    for(std::vector<geometry_msgs::PoseStamped>::const_iterator it = msg->poses.begin(); it!= msg->poses.end();  ++it)
    {
		std::vector<float> temp;
		geometry_msgs::PoseStamped tmp = *it;
		temp.push_back(tmp.pose.position.x);
		temp.push_back(tmp.pose.position.y);
		temp.push_back(tmp.pose.orientation.z);
		temp.push_back(tmp.pose.orientation.w);
		localPlan.push_back(temp);
		i++;
    }
}


void CurrentGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	  // current_goal	
	  writer8.open(fn8, std::ios::app);

	  std::string info = msg->header.frame_id + "," + std::to_string(msg->pose.position.x) + "," + std::to_string(msg->pose.position.y) + ','
								+ std::to_string(msg->pose.orientation.z) + "," + std::to_string(msg->pose.orientation.w);
	  writer8 << info << std::endl;
	  
	  writer8.close();
}


void PlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
	plan.clear();
	
	planFrame = msg->poses.begin()->header.frame_id;
	
	int i=0;
    for(std::vector<geometry_msgs::PoseStamped>::const_iterator it = msg->poses.begin(); it!= msg->poses.end();  ++it)
    {
		std::vector<float> temp;
		geometry_msgs::PoseStamped tmp = *it;
		temp.push_back(tmp.pose.position.x);
		temp.push_back(tmp.pose.position.y);
		temp.push_back(tmp.pose.orientation.z);
		temp.push_back(tmp.pose.orientation.w);
		plan.push_back(temp);
		i++;
    }
}


void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	amclFrame = msg->header.frame_id;
	amclPositionX = msg->pose.pose.position.x; 
	amclPositionY = msg->pose.pose.position.y; 
	amclOrientationZ = msg->pose.pose.orientation.z; 
	amclOrientationW = msg->pose.pose.orientation.w;
}


void LocalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	localCostmapFrame = msg->header.frame_id;
	resolution = msg->info.resolution;
	width = msg->info.width; 
	height = msg->info.height;
	originPosX = msg->info.origin.position.x; 
	originPosY = msg->info.origin.position.y;
	originOrientationZ = msg->info.origin.orientation.z;
	originOrientationW = msg->info.origin.orientation.w;
	
	data = msg->data;
}


void GlobalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	globalCostmapFrame = msg->header.frame_id;
	global_resolution = msg->info.resolution;
	global_width = msg->info.width; 
	global_height = msg->info.height;
	global_originPosX = msg->info.origin.position.x; 
	global_originPosY = msg->info.origin.position.y;
	global_originOrientationZ = msg->info.origin.orientation.z;
	global_originOrientationW = msg->info.origin.orientation.w;
	
	global_data = msg->data;
	
	
	// global_costmap_info  
	std::cout << "global: " << global_height << " " << global_width << std::endl;
   
	writer17.open(fn17, std::ios::app);

	std::string  info = globalCostmapFrame + "," + std::to_string(global_resolution) + "," + std::to_string(global_width) + "," + std::to_string(global_height) + "," + 
				std::to_string(global_originPosX) + "," + std::to_string(global_originPosY) + "," +
				std::to_string(global_originOrientationZ) + "," + std::to_string(global_originOrientationW);
	writer17 << info << std::endl;
  
	writer17.close();
  
	// global_costmap_data 
	writer18.open(fn18, std::ios::app);

	for(int i = 0; i < global_height; i++)
	{
		for(int j = 0; j < global_width; j++)
		{
			writer18 << static_cast<int>(global_data.at(i*global_width + j)) << ",";
		}
		writer18 << std::endl;
	}
	writer18 << std::endl;
  
	writer18.close();
}

void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapFrame = msg->header.frame_id;
	map_resolution = msg->info.resolution;
	map_width = msg->info.width; 
	map_height = msg->info.height;
	map_originPosX = msg->info.origin.position.x; 
	map_originPosY = msg->info.origin.position.y;
	map_originOrientationZ = msg->info.origin.orientation.z;
	map_originOrientationW = msg->info.origin.orientation.w;
	
	map_data = msg->data;
	
	// map_costmap_info
	std::cout << "map: " << map_height << " " << map_width << std::endl;
  
	writer19.open(fn19, std::ios::app);

	std::string info = mapFrame + "," + std::to_string(map_resolution) + "," + std::to_string(map_width) + "," + std::to_string(map_height) + "," + 
				std::to_string(map_originPosX) + "," + std::to_string(map_originPosY) + "," +
				std::to_string(map_originOrientationZ) + "," + std::to_string(map_originOrientationW);
	writer19 << info << std::endl;
  
	writer19.close();  
  
	// map_costmap_data
	writer20.open(fn20, std::ios::app);

	for(int i = 0; i < map_height; i++)
	{
		for(int j = 0; j < map_width; j++)
		{
			writer20 << static_cast<int>(map_data.at(i*map_width + j)) << ",";
		}
		writer20 << std::endl;
	}
	writer20 << std::endl;
  
	writer20.close();	
}

void FootprintCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
	footprintFrame = msg->header.frame_id;
	
	polygon = msg->polygon;		
}


// main function
int main(int argc, char **argv)
{
  // initialize ROS node 'teb_listener_1'
  ros::init(argc, argv, "teb_listener_1");
  
  // output headers' names in output files
  UpisiNaslove();

  // define ROS node handle
  ros::NodeHandle n;
  
  // Initialize tranform buffer and transform listener
  //*tf_ = new tf::TransformListener;
  tf2_ros::Buffer tfBuff;
  tfBuffer = &tfBuff;
  tf2_ros::TransformListener tfList(*tfBuffer);
  tfListener = &tfList;

  // odometry subscriber
  ros::Subscriber odom = n.subscribe("/mobile_base_controller/odom", 1000, odomCallback);

  // control velocities subscriber	
  ros::Subscriber cmd_vel = n.subscribe("/mobile_base_controller/cmd_vel", 1000, CmdVelCallback);
  
  // global_plan subscriber
  ros::Subscriber teb_global_plan = n.subscribe("/move_base/TebLocalPlannerROS/global_plan", 1000, TebGlobalPlanCallback);

  // local_plan subscriber
  ros::Subscriber teb_local_plan = n.subscribe("/move_base/TebLocalPlannerROS/local_plan", 1000, TebLocalPlanCallback);
  
  // global planner's plan subscriber
  ros::Subscriber plan = n.subscribe("/move_base/GlobalPlanner/plan", 1000, PlanCallback);
  
  // current goal subscriber
  ros::Subscriber current_goal = n.subscribe("/move_base/current_goal", 1000, CurrentGoalCallback);
            
  // amcl pose subscriber
  ros::Subscriber amcl = n.subscribe("/amcl_pose", 1000, amclCallback);

  // local_costmap subscriber
  ros::Subscriber local_costmap = n.subscribe("/move_base/local_costmap/costmap", 1000, LocalCostmapCallback);
  
  // global_costmap subscriber
  ros::Subscriber global_costmap = n.subscribe("/move_base/global_costmap/costmap", 1000, GlobalCostmapCallback);
  
  // map subscriber
  ros::Subscriber map = n.subscribe("/map", 1000, MapCallback);
  
  // footprint subscriber
  ros::Subscriber footprint = n.subscribe("/move_base/local_costmap/footprint", 1000, FootprintCallback);

  ros::spin();

  return 0;
}















// subscribers not currently used
// teb_poses subscriber
//ros::Subscriber teb_poses = n.subscribe("/move_base/TebLocalPlannerROS/teb_poses", 1000, TebPosesCallback);

// teb_markers subscriber
//ros::Subscriber teb_markers = n.subscribe("/move_base/TebLocalPlannerROS/teb_markers", 1000, TebMarkersCallback);

// obstacles subscriber
//ros::Subscriber obstacles = n.subscribe("/move_base/TebLocalPlannerROS/obstacles", 1000, ObstaclesCallback);

// via_points subscriber
//ros::Subscriber via_points = n.subscribe("/move_base/TebLocalPlannerROS/via_points", 1000, ViaPointsCallback);


// other global variables (currently not used)
//float globalPlanPositionX = 0.0, globalPlanPositionY = 0.0, globalPlanOrientationZ = 0.0, globalPlanOrientationW = 0.0;
//float localPlanPositionX = 0.0, localPlanPositionY = 0.0, localPlanOrientationZ = 0.0, localPlanOrientationW = 0.0;
//float posesPlanPositionX = 0.0, posesPlanPositionY = 0.0, posesPlanOrientationZ = 0.0, posesPlanOrientationW = 0.0;
//float markersPlanPositionX = 0.0, markersPlanPositionY = 0.0, markersPlanOrientationZ = 0.0, markersPlanOrientationW = 0.0;
//float obstaclePosX = 0.0, obstaclePosY = 0.0;
//float viaPointsPosX = 0.0, viaPointsPosY = 0.0, viaPointsOrientationZ = 0.0, viaPointsOrientationW = 0.0;


// callback functions for subscribers currently not used
/*
void TebPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{  
	posesPlanPositionX = msg->poses[0].position.x; 
	posesPlanPositionY = msg->poses[0].position.y; 
	posesPlanOrientationZ = msg->poses[0].orientation.z; 
	posesPlanOrientationW = msg->poses[0].orientation.w;
}

void TebMarkersCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
	markersPlanPositionX = msg->pose.position.x; 
	markersPlanPositionY = msg->pose.position.y; 
	markersPlanOrientationZ = msg->pose.orientation.z; 
	markersPlanOrientationW = msg->pose.orientation.w;
}

void ObstaclesCallback(const costmap_converter::ObstacleArrayMsg::ConstPtr& msg)
{
	obstaclePosX = msg->obstacles[0].polygon.points[0].x;
	obstaclePosY = msg->obstacles[0].polygon.points[0].y;
}

void ViaPointsCallback(const nav_msgs::Path::ConstPtr& msg)
{
	viaPointsPosX = msg->poses[0].pose.position.x;
	viaPointsPosY = msg->poses[0].pose.position.y;
	viaPointsOrientationZ = msg->poses[0].pose.orientation.z;
	viaPointsOrientationW = msg->poses[0].pose.orientation.w;
}
*/


// header writers currently not used
 /*
  // poses 
  writer6.open(fn6, std::ios::app);

  info =  "pose_position_x,pose_position_y,pose_orientation_z,pose_orientation_w";
  writer6 << info << std::endl;
  
  writer6.close();
  
  // markers
  writer7.open(fn7, std::ios::app);

  info =  "marker_position_x,marker_position_y,marker_orientation_z,marker_orientation_w";
  writer7 << info << std::endl;
  
  writer7.close();
  
  // obstacles
  writer9.open(fn9, std::ios::app);

  info =  "current_goal_position_x,current_goal_position_y,current_goal_orientation_z,current_goal_orientation_w";
  writer9 << info << std::endl;
  
  writer9.close();

  // via_points
  writer10.open(fn10, std::ios::app);

  info =  "current_goal_position_x,current_goal_position_y,current_goal_orientation_z,current_goal_orientation_w";
  writer10 << info << std::endl;
  
  writer10.close();
 */
 
 // data writers currently not used
 /*
  // poses 
  writer6.open(fn6, std::ios::app);

  info =  std::to_string(posesPlanPositionX) + "," + std::to_string(posesPlanPositionY) + ","
			+ std::to_string(posesPlanOrientationZ) + "," + std::to_string(posesPlanOrientationW);
  writer6 << info << std::endl;
  
  writer6.close();
  
  // markers
  writer7.open(fn7, std::ios::app);

  info =  std::to_string(markersPlanPositionX) + "," + std::to_string(markersPlanPositionY) + ","
			+ std::to_string(markersPlanOrientationZ) + "," + std::to_string(markersPlanOrientationW);
  writer7 << info << std::endl;
  
  writer7.close();
  */
  /*
  // obstacles
  writer9.open(fn9, std::ios::app);

  info =  std::to_string(obstaclePosX) + "," + std::to_string(obstaclePosY);
  writer9 << info << std::endl;
  
  writer9.close();

  // via points
  writer10.open(fn10, std::ios::app);

  info = std::to_string(viaPointsPosX) + "," + std::to_string(viaPointsPosY) + ","
			+ std::to_string(viaPointsOrientationZ) + "," + std::to_string(viaPointsOrientationW);
  writer10 << info << std::endl;
  
  writer10.close();
  */
