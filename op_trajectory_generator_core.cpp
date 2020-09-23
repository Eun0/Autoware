/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include "op_trajectory_generator_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"


namespace TrajectoryGeneratorNS
{

TrajectoryGen::TrajectoryGen()
{
  bInitPos = false;
  bNewCurrentPos = false;
  bVehicleStatus = false;
  bWayGlobalPath = false;

  ros::NodeHandle _nh;
  UpdatePlanningParams(_nh);

  //eun
  bMap = false;
  nh.getParam("/op_global_planner/mapFileName" , m_MapPath);

  int iSource = 0;
  nh.getParam("/op_global_planner/mapSource" , iSource);
  if(iSource == 0)
    m_MapType = PlannerHNS::MAP_AUTOWARE;
  else if (iSource == 1)
    m_MapType = PlannerHNS::MAP_FOLDER;
  else if(iSource == 2)
    m_MapType = PlannerHNS::MAP_KML_FILE;
  //eun end

  tf::StampedTransform transform;
  PlannerHNS::ROSHelpers::GetTransformFromTF("map", "world", transform);
  m_OriginPos.position.x  = transform.getOrigin().x();
  m_OriginPos.position.y  = transform.getOrigin().y();
  m_OriginPos.position.z  = transform.getOrigin().z();

  pub_LocalTrajectories = nh.advertise<autoware_msgs::LaneArray>("local_trajectories", 1);
  pub_LocalTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories_gen_rviz", 1);

  sub_initialpose = nh.subscribe("/initialpose", 1, &TrajectoryGen::callbackGetInitPose, this);
  sub_current_pose = nh.subscribe("/current_pose", 10, &TrajectoryGen::callbackGetCurrentPose, this);

  int bVelSource = 1;
  _nh.getParam("/op_trajectory_generator/velocitySource", bVelSource);
  if(bVelSource == 0)
    sub_robot_odom = nh.subscribe("/odom", 10,  &TrajectoryGen::callbackGetRobotOdom, this);
  else if(bVelSource == 1)
    sub_current_velocity = nh.subscribe("/current_velocity", 10, &TrajectoryGen::callbackGetVehicleStatus, this);
  else if(bVelSource == 2)
    sub_can_info = nh.subscribe("/can_info", 10, &TrajectoryGen::callbackGetCANInfo, this);

  sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &TrajectoryGen::callbackGetGlobalPlannerPath, this);

  //eun
  pub_lane_info = nh.advertise<autoware_msgs::LaneInfo>("lane_info", 1);
  sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &TrajectoryGen::callbackGetVMLanes,  this);
  sub_points = nh.subscribe("/vector_map_info/point", 1, &TrajectoryGen::callbackGetVMPoints,  this);
  sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &TrajectoryGen::callbackGetVMdtLanes,  this);
  sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &TrajectoryGen::callbackGetVMIntersections,  this);
  sup_area = nh.subscribe("/vector_map_info/area", 1, &TrajectoryGen::callbackGetVMAreas,  this);
  sub_lines = nh.subscribe("/vector_map_info/line", 1, &TrajectoryGen::callbackGetVMLines,  this);
  sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &TrajectoryGen::callbackGetVMStopLines,  this);
  sub_signals = nh.subscribe("/vector_map_info/signal", 1, &TrajectoryGen::callbackGetVMSignal,  this);
  sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &TrajectoryGen::callbackGetVMVectors,  this);
  sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &TrajectoryGen::callbackGetVMCurbs,  this);
  sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &TrajectoryGen::callbackGetVMRoadEdges,  this);
  sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &TrajectoryGen::callbackGetVMWayAreas,  this);
  sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &TrajectoryGen::callbackGetVMCrossWalks,  this);
  sub_nodes = nh.subscribe("/vector_map_info/node", 1, &TrajectoryGen::callbackGetVMNodes,  this);

}

TrajectoryGen::~TrajectoryGen()
{
}

void TrajectoryGen::UpdatePlanningParams(ros::NodeHandle& _nh)
{
  _nh.getParam("/op_trajectory_generator/samplingTipMargin", m_PlanningParams.carTipMargin);
  _nh.getParam("/op_trajectory_generator/samplingOutMargin", m_PlanningParams.rollInMargin);
  _nh.getParam("/op_trajectory_generator/samplingSpeedFactor", m_PlanningParams.rollInSpeedFactor);
  _nh.getParam("/op_trajectory_generator/enableHeadingSmoothing", m_PlanningParams.enableHeadingSmoothing);

  _nh.getParam("/op_common_params/enableSwerving", m_PlanningParams.enableSwerving);
  if(m_PlanningParams.enableSwerving)
    m_PlanningParams.enableFollowing = true;
  else
    _nh.getParam("/op_common_params/enableFollowing", m_PlanningParams.enableFollowing);

  _nh.getParam("/op_common_params/enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
  _nh.getParam("/op_common_params/enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);

  _nh.getParam("/op_common_params/maxVelocity", m_PlanningParams.maxSpeed);
  _nh.getParam("/op_common_params/minVelocity", m_PlanningParams.minSpeed);
  _nh.getParam("/op_common_params/maxLocalPlanDistance", m_PlanningParams.microPlanDistance);

  _nh.getParam("/op_common_params/pathDensity", m_PlanningParams.pathDensity);
  _nh.getParam("/op_common_params/rollOutDensity", m_PlanningParams.rollOutDensity);
  if(m_PlanningParams.enableSwerving)
    _nh.getParam("/op_common_params/rollOutsNumber", m_PlanningParams.rollOutNumber);
  else
    m_PlanningParams.rollOutNumber = 0;

  //eun
  _nh.getParam("/op_common_params/bordersNumber",m_PlanningParams.borderNumber);

  _nh.getParam("/op_common_params/horizonDistance", m_PlanningParams.horizonDistance);
  _nh.getParam("/op_common_params/minFollowingDistance", m_PlanningParams.minFollowingDistance);
  _nh.getParam("/op_common_params/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
  _nh.getParam("/op_common_params/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
  _nh.getParam("/op_common_params/speedProfileFactor", m_PlanningParams.speedProfileFactor);

  _nh.getParam("/op_common_params/smoothingDataWeight", m_PlanningParams.smoothingDataWeight);
  _nh.getParam("/op_common_params/smoothingSmoothWeight", m_PlanningParams.smoothingSmoothWeight);

  _nh.getParam("/op_common_params/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
  _nh.getParam("/op_common_params/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);

  _nh.getParam("/op_common_params/enableLaneChange", m_PlanningParams.enableLaneChange);

  _nh.getParam("/op_common_params/width", m_CarInfo.width);
  _nh.getParam("/op_common_params/length", m_CarInfo.length);
  _nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
  _nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
  _nh.getParam("/op_common_params/maxSteerAngle", m_CarInfo.max_steer_angle);
  _nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
  _nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);

  m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
  m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;

}

//eun
void TrajectoryGen::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
  PlannerHNS::WayPoint* p = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(m_CurrentPos,m_Map);
  m_InitPos = m_CurrentPos;
  bNewCurrentPos = true;
  bInitPos = true;
}

void TrajectoryGen::callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
  if(msg->lanes.size() > 0 && bMap)
  {

    bool bOldGlobalPath = m_GlobalPaths.size() == msg->lanes.size();

    m_GlobalPaths.clear();

    for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
    {
      PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), m_temp_path);
      PlannerHNS::Lane* pPrevValid = 0;
      for(unsigned int j = 0 ; j < m_temp_path.size(); j++)
      {
        PlannerHNS::Lane* pLane = 0;
        pLane = PlannerHNS::MappingHelpers::GetLaneById(m_temp_path.at(j).laneId, m_Map);
        if(!pLane)
        {
          pLane = PlannerHNS::MappingHelpers::GetClosestLaneFromMapDirectionBased(m_temp_path.at(j), m_Map, 1);

          if(!pLane && !pPrevValid)
          {
            ROS_ERROR("Map inconsistency between Global Path and Local Planer Map, Can't identify current lane.");
            return;
          }

          if(!pLane)
            m_temp_path.at(j).pLane = pPrevValid;
          else
          {
            m_temp_path.at(j).pLane = pLane;
            pPrevValid = pLane ;
          }

          m_temp_path.at(j).laneId = m_temp_path.at(j).pLane->id;
        }
        else
          m_temp_path.at(j).pLane = pLane;


        //std::cout << "StopLineInGlobalPath: " << m_temp_path.at(j).stopLineID << std::endl;
      }

      PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_temp_path);
      m_GlobalPaths.push_back(m_temp_path);

      if(bOldGlobalPath)
      {
        bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(m_temp_path, m_GlobalPaths.at(i));
      }
    }

    if(!bOldGlobalPath)
    {
      bWayGlobalPath = true;
      bWayGlobalPathLogs = true;
      for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
      {
        PlannerHNS::PlanningHelpers::FixPathDensity(m_GlobalPaths.at(i), m_PlanningParams.pathDensity);
        PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.35, 0.4, 0.05);

        PlannerHNS::PlanningHelpers::GenerateRecommendedSpeed(m_GlobalPaths.at(i), m_CarInfo.max_speed_forward, m_PlanningParams.speedProfileFactor);
        m_GlobalPaths.at(i).at(m_GlobalPaths.at(i).size()-1).v = 0;
      }

      std::cout << "Received New Global Path Selector! " << std::endl;
    }
    else
    {
      m_GlobalPaths.clear();
    }
  }
}

void TrajectoryGen::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
  std::cout << "Received Lanes" << msg.data.size() << endl;
  if(m_MapRaw.pLanes == nullptr)
  { 
    m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
  }
}

void TrajectoryGen::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
  std::cout << "Received Points" << msg.data.size() << endl;
  if(m_MapRaw.pPoints  == nullptr)
    m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
}

void TrajectoryGen::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
  std::cout << "Received dtLanes" << msg.data.size() << endl;
  if(m_MapRaw.pCenterLines == nullptr)
    m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void TrajectoryGen::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
  std::cout << "Received CrossRoads" << msg.data.size() << endl;
  if(m_MapRaw.pIntersections == nullptr)
    m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void TrajectoryGen::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
  std::cout << "Received Areas" << msg.data.size() << endl;
  if(m_MapRaw.pAreas == nullptr)
    m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void TrajectoryGen::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
  std::cout << "Received Lines" << msg.data.size() << endl;
  if(m_MapRaw.pLines == nullptr)
    m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
}

void TrajectoryGen::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
  std::cout << "Received StopLines" << msg.data.size() << endl;
  if(m_MapRaw.pStopLines == nullptr)
    m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void TrajectoryGen::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
  std::cout << "Received Signals" << msg.data.size() << endl;
  if(m_MapRaw.pSignals  == nullptr)
    m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void TrajectoryGen::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
  std::cout << "Received Vectors" << msg.data.size() << endl;
  if(m_MapRaw.pVectors  == nullptr)
    m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void TrajectoryGen::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
  std::cout << "Received Curbs" << msg.data.size() << endl;
  if(m_MapRaw.pCurbs == nullptr)
    m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void TrajectoryGen::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
  std::cout << "Received Edges" << msg.data.size() << endl;
  if(m_MapRaw.pRoadedges  == nullptr)
    m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
}

void TrajectoryGen::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
  std::cout << "Received Wayareas" << msg.data.size() << endl;
  if(m_MapRaw.pWayAreas  == nullptr)
    m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void TrajectoryGen::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
  std::cout << "Received CrossWalks" << msg.data.size() << endl;
  if(m_MapRaw.pCrossWalks == nullptr)
    m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void TrajectoryGen::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
  std::cout << "Received Nodes" << msg.data.size() << endl;
  if(m_MapRaw.pNodes == nullptr)
    m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}
//eun end

void TrajectoryGen::callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  if(!bInitPos)
  {
    m_InitPos = PlannerHNS::WayPoint(msg->pose.pose.position.x+m_OriginPos.position.x,
        msg->pose.pose.position.y+m_OriginPos.position.y,
        msg->pose.pose.position.z+m_OriginPos.position.z,
        tf::getYaw(msg->pose.pose.orientation));
    m_CurrentPos = m_InitPos;
    bInitPos = true;
  }
}


void TrajectoryGen::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
  m_VehicleStatus.speed = msg->twist.linear.x;
  m_CurrentPos.v = m_VehicleStatus.speed;
  if(fabs(msg->twist.linear.x) > 0.25)
    m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
  UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
  bVehicleStatus = true;
}

void TrajectoryGen::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
  m_VehicleStatus.speed = msg->speed/3.6;
  m_VehicleStatus.steer = msg->angle * m_CarInfo.max_steer_angle / m_CarInfo.max_steer_value;
  UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
  bVehicleStatus = true;
}

void TrajectoryGen::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
  m_VehicleStatus.speed = msg->twist.twist.linear.x;
  m_VehicleStatus.steer += atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
  UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
  bVehicleStatus = true;
}


 //eun
void TrajectoryGen::MainLoop()
{
  ros::Rate loop_rate(100);

  PlannerHNS::WayPoint prevState, state_change;


  autoware_msgs::LaneInfo lane_info;

  while (ros::ok())
  {
    ros::spinOnce();

    if(!bMap)
    {
      std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;;

      if(m_MapRaw.GetVersion()==2)
      {
        std::cout << "Map Version 2" << endl;
        bMap = true;
        PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessageV2(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
            m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
            m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,  m_MapRaw.pSignals->m_data_list,
            m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
            m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,
            m_MapRaw.pLanes, m_MapRaw.pPoints, m_MapRaw.pNodes, m_MapRaw.pLines, PlannerHNS::GPSPoint(), m_Map, true, true, false);
      }
      
    }

    if(bInitPos && m_GlobalPaths.size()>0)
    {
      m_GlobalPathSections.clear();

      for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
      {
        t_centerTrajectorySmoothed.clear();
        PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_GlobalPaths.at(i), m_CurrentPos, m_PlanningParams.horizonDistance ,
            m_PlanningParams.pathDensity ,t_centerTrajectorySmoothed);

        m_GlobalPathSections.push_back(t_centerTrajectorySmoothed);
      }

      m_Planner.FindLaneNum(m_CurrentPos, m_Map, current_lane_num, total_lane_num);
      lane_info.current_lane = current_lane_num;
      lane_info.total_lane = total_lane_num;
      pub_lane_info.publish(lane_info);
      std::vector<PlannerHNS::WayPoint> sampledPoints_debug;
      if(bMap) {
        m_Planner.GenerateRunoffTrajectoryV2(m_GlobalPathSections, m_CurrentPos,
                  true,
                  m_VehicleStatus.speed,
                  m_PlanningParams.microPlanDistance,
                  m_PlanningParams.maxSpeed,
                  m_PlanningParams.minSpeed,
                  m_PlanningParams.carTipMargin,
                  m_PlanningParams.rollInMargin,
                  m_PlanningParams.rollInSpeedFactor,
                  m_PlanningParams.pathDensity,
                  m_PlanningParams.rollOutDensity,
                  m_PlanningParams.borderNumber,
                  m_PlanningParams.smoothingDataWeight,
                  m_PlanningParams.smoothingSmoothWeight,
                  m_PlanningParams.smoothingToleranceError,
                  m_PlanningParams.speedProfileFactor,
                  m_PlanningParams.enableHeadingSmoothing,
                  -1 , -1,
                  m_RollOuts, sampledPoints_debug,m_Map);
      }
      else
        return;
      
      autoware_msgs::LaneArray local_lanes;
      for(unsigned int i=0; i < m_RollOuts.size(); i++)
      {
        for(unsigned int j=0; j < m_RollOuts.at(i).size(); j++)
        {
          autoware_msgs::Lane lane;
          PlannerHNS::PlanningHelpers::PredictConstantTimeCostForTrajectory(m_RollOuts.at(i).at(j), m_CurrentPos, m_PlanningParams.minSpeed, m_PlanningParams.microPlanDistance);
          PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_RollOuts.at(i).at(j), lane);
          lane.closest_object_distance = 0;
          lane.closest_object_velocity = 0;
          lane.cost = 0;
          lane.is_blocked = false;
          lane.lane_index = i;
          local_lanes.lanes.push_back(lane);
        }
      }
      pub_LocalTrajectories.publish(local_lanes);
    }
    else
      sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array",   1,    &TrajectoryGen::callbackGetGlobalPlannerPath,   this);

    visualization_msgs::MarkerArray all_rollOuts;
    PlannerHNS::ROSHelpers::TrajectoriesToMarkers(m_RollOuts, all_rollOuts);
    pub_LocalTrajectoriesRviz.publish(all_rollOuts);

    loop_rate.sleep();
  }
}

}
