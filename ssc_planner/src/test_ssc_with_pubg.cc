/**
 * @file test_ssc_with_pubg.cc
 * @author HKUST Aerial Robotics Group
 * @brief test ssc planner with pubg behavior planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#include <stdlib.h>
#include <chrono>
#include <iostream>

#include <ros/ros.h>
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"

#include "pubg_planner/pubg_server_ros.h"
#include "onlane_motion_planner/onlane_server_ros.h"
#include "ssc_planner/ssc_server_ros.h"

DECLARE_BACKWARD;
double ssc_planner_work_rate = 20.0;
double bp_work_rate = 20.0;
double onlane_mp_work_rate = 20.0;

planning::SscPlannerServer* p_ssc_server_{nullptr};
planning::PubgPlannerServer* p_bp_server_{nullptr};

int BehaviorUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ssc_server_) p_ssc_server_->PushSemanticMap(smm);
  return 0;
}

int SemanticMapUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) p_bp_server_->PushSemanticMap(smm);
  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  int ego_id;
  if (!nh.getParam("ego_id", ego_id)) {
    ROS_ERROR("Failed to get param %d", ego_id);
    assert(false);
  }
  std::string agent_config_path;
  if (!nh.getParam("agent_config_path", agent_config_path)) {
    ROS_ERROR("Failed to get param %s", agent_config_path.c_str());
    assert(false);
  }

  semantic_map_manager::SemanticMapManager semantic_map_manager(
      ego_id, agent_config_path);
  semantic_map_manager::RosAdapter smm_ros_adapter(nh, &semantic_map_manager);
  smm_ros_adapter.BindMapUpdateCallback(SemanticMapUpdateCallback);

  double desired_vel;
  nh.param("desired_vel", desired_vel, 6.0);
  // Declare bp
  p_bp_server_ = new planning::PubgPlannerServer(nh, bp_work_rate, ego_id);
  p_bp_server_->set_user_desired_velocity(desired_vel);
  p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);
  p_bp_server_->set_autonomous_level(3);

  p_ssc_server_ =
      new planning::SscPlannerServer(nh, ssc_planner_work_rate, ego_id);

  p_ssc_server_->Init();
  p_bp_server_->Init();
  smm_ros_adapter.Init();

  p_bp_server_->Start();
  p_ssc_server_->Start();

  // TicToc timer;
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
