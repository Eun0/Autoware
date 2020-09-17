/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OP_TRAJECTORY_GENERATOR_CORE
#define OP_TRAJECTORY_GENERATOR_CORE

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_can_msgs/CANInfo.h>

#include "op_planner/PlannerH.h"
#include "op_planner/PlannerCommonDef.h"

//eun
#include "op_planner/MappingHelpers.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/LineArray.h"
#include "vector_map_msgs/AreaArray.h"
#include "vector_map_msgs/SignalArray.h"
#include "vector_map_msgs/StopLine.h"
#include "vector_map_msgs/VectorArray.h"
#include <autoware_msgs/LaneInfo.h>


namespace TrajectoryGeneratorNS
{

class TrajectoryGen
{
protected:

  //eun
  bool bWayGlobalPathLogs;
  int current_lane_num;
  int total_lane_num;
  ros::Publisher pub_lane_info;
  PlannerHNS::MAP_SOURCE_TYPE m_MapType;
  std::string m_MapPath;
  PlannerHNS::RoadNetwork m_Map;
  bool bMap;
  UtilityHNS::MapRaw m_MapRaw;
  ros::Subscriber sub_lanes;
  ros::Subscriber sub_points;
  ros::Subscriber sub_dt_lanes;
  ros::Subscriber sub_intersect;
  ros::Subscriber sup_area;
  ros::Subscriber sub_lines;
  ros::Subscriber sub_stop_line;
  ros::Subscriber sub_signals;
  ros::Subscriber sub_vectors;
  ros::Subscriber sub_curbs;
  ros::Subscriber sub_edges;
  ros::Subscriber sub_way_areas;
  ros::Subscriber sub_cross_walk;
  ros::Subscriber sub_nodes;
  void callbackGetVMLanes(const vector_map_msgs::LaneArray& msg);
  void callbackGetVMPoints(const vector_map_msgs::PointArray& msg);
  void callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg);
  void callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg);
  void callbackGetVMAreas(const vector_map_msgs::AreaArray& msg);
  void callbackGetVMLines(const vector_map_msgs::LineArray& msg);
  void callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg);
  void callbackGetVMSignal(const vector_map_msgs::SignalArray& msg);
  void callbackGetVMVectors(const vector_map_msgs::VectorArray& msg);
  void callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg);
  void callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg);
  void callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg);
  void callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg);
  void callbackGetVMNodes(const vector_map_msgs::NodeArray& msg);

  //eun end

  PlannerHNS::PlannerH m_Planner;
  geometry_msgs::Pose m_OriginPos;
  PlannerHNS::WayPoint m_InitPos;
  bool bInitPos;

  PlannerHNS::WayPoint m_CurrentPos;
  bool bNewCurrentPos;

  PlannerHNS::VehicleState m_VehicleStatus;
  bool bVehicleStatus;

  std::vector<PlannerHNS::WayPoint> m_temp_path;
  std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
  std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathSections;
  std::vector<PlannerHNS::WayPoint> t_centerTrajectorySmoothed;
  std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_RollOuts;
  bool bWayGlobalPath;
  struct timespec m_PlanningTimer;
    std::vector<std::string>    m_LogData;
    PlannerHNS::PlanningParams m_PlanningParams;
    PlannerHNS::CAR_BASIC_INFO m_CarInfo;


    //ROS messages (topics)
  ros::NodeHandle nh;

  //define publishers
  ros::Publisher pub_LocalTrajectories;
  ros::Publisher pub_LocalTrajectoriesRviz;

  // define subscribers.
  ros::Subscriber sub_initialpose;
  ros::Subscriber sub_current_pose;
  ros::Subscriber sub_current_velocity;
  ros::Subscriber sub_robot_odom;
  ros::Subscriber sub_can_info;
  ros::Subscriber sub_GlobalPlannerPaths;


  // Callback function for subscriber.
  void callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
  void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
  void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
  void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
  void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
  void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);

  //Helper Functions
  void UpdatePlanningParams(ros::NodeHandle& _nh);

public:
  TrajectoryGen();
  ~TrajectoryGen();
  void MainLoop();
};

}

#endif  // OP_TRAJECTORY_GENERATOR_CORE
