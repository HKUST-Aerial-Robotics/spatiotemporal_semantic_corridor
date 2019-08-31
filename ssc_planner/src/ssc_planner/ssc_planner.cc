/**
 * @file ssc_planner.cc
 * @author HKUST Aerial Robotics Group
 * @brief implementation for ssc planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */

#include "ssc_planner/ssc_planner.h"
#include "ssc_planner/ssc_map_utils.h"

#include <omp.h>

#define USE_OPENMP 1

namespace planning {

SscPlanner::SscPlanner() { p_ssc_map_ = new SscMap(SscMap::Config()); }

SscPlanner::SscPlanner(const Config& cfg) : config_(cfg) {
  p_ssc_map_ = new SscMap(SscMap::Config());
}

std::string SscPlanner::Name() { return std::string("ssc_planner"); }

ErrorType SscPlanner::Init(const std::string config) { return kSuccess; }

ErrorType SscPlanner::set_init_state(const State& state) {
  init_state_ = state;
  has_init_state_ = true;
  return kSuccess;
}

ErrorType SscPlanner::RunOnce() {
  if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    printf("[SscPlanner]fail to get ego vehicle info.\n");
    return kWrongStatus;
  }

  if (!has_init_state_) {
    init_state_ = ego_vehicle_.state();
  }
  has_init_state_ = false;

  if (map_itf_->GetLocalNavigationLane(&nav_lane_local_) != kSuccess) {
    printf("[SscPlanner]fail to find ego lane.\n");
    return kWrongStatus;
  }
  stf_ = common::StateTransformer(nav_lane_local_);

  common::FrenetState fs0;
  if (stf_.GetFrenetStateFromState(init_state_, &fs0) != kSuccess) {
    printf("[SscPlanner]fail to get init state frenet state.\n");
    return kWrongStatus;
  }

  if (map_itf_->GetEgoDiscretBehavior(&ego_behavior_) != kSuccess) {
    printf("[SscPlanner]fail to get ego behavior.\n");
    return kWrongStatus;
  }

  if (map_itf_->GetSemanticVehicleSet(&semantic_vehicle_set_) != kSuccess) {
    printf("[SscPlanner]fail to get semantic vehicle set.\n");
    return kWrongStatus;
  }

  if (map_itf_->GetObstacleMap(&grid_map_) != kSuccess) {
    printf("[SscPlanner]fail to get obstacle map.\n");
    return kWrongStatus;
  }

  if (map_itf_->GetObstacleGrids(&obstacle_grids_) != kSuccess) {
    printf("[SscPlanner]fail to get obstacle grids.\n");
    return kWrongStatus;
  }

  if (map_itf_->GetForwardTrajectories(&forward_behaviors_, &forward_trajs_,
                                       &surround_forward_trajs_) != kSuccess) {
    printf("[SscPlanner]fail to get forward trajectories.\n");
    return kWrongStatus;
  }

  // Traffic signal
  vec_E<common::SpeedLimit> speed_limit_list;
  if (map_itf_->GetSpeedLimit(&speed_limit_list) != kSuccess) {
    printf("[SscPlanner]fail to get speed limit.\n");
    return kWrongStatus;
  }
  // p_ssc_map_->set_speed_limit_list(speed_limit_list);

  vec_E<common::TrafficLight> traffic_light_list;
  if (map_itf_->GetTrafficLight(&traffic_light_list) != kSuccess) {
    printf("[SscPlanner]fail to get traffic light.\n");
    return kWrongStatus;
  }
  // p_ssc_map_->set_traffic_light_list(traffic_light_list);

  TicToc timer_stf;
  if (StateTransformForInputData() != kSuccess) {
    return kWrongStatus;
  }
  printf("[SscPlanner] Transform time cost: %lf ms\n", timer_stf.toc());

  std::vector<common::AxisAlignedsCubeNd<decimal_t, 3>> cubes;
  GetSemanticCubeList(speed_limit_list, traffic_light_list, &cubes);
  p_ssc_map_->set_semantic_cube_set(cubes);
  p_ssc_map_->set_desired_fs(fs0);

  if (!has_last_state_s_) {
    last_lane_s_ = std::make_pair(init_state_, fs0.vec_s[0]);
    global_s_offset_ = -fs0.vec_s[0];
    has_last_state_s_ = true;
  } else {
    common::FrenetState new_fs0;
    stf_.GetFrenetStateFromState(last_lane_s_.first, &new_fs0);
    global_s_offset_ += last_lane_s_.second - new_fs0.vec_s[0];
    last_lane_s_ = std::make_pair(init_state_, fs0.vec_s[0]);
  }

  // decimal_t global_s = global_s_offset_ + fs0.vec_s[0];
  // decimal_t global_s_lb = std::floor(global_s / 0.25) * 0.25;
  // decimal_t local_s_lb = global_s_lb - global_s_offset_;

  p_ssc_map_->p_3d_grid()->clear_data();
  p_ssc_map_->p_3d_inflated_grid()->clear_data();

  std::array<decimal_t, 3> map_origin = {fs0.vec_s[0] - 20, -10.0, 0};
  p_ssc_map_->p_3d_grid()->set_origin(map_origin);
  p_ssc_map_->p_3d_inflated_grid()->set_origin(map_origin);

  time_origin_ = init_state_.time_stamp;
  p_ssc_map_->set_start_time(time_origin_);

  // ~ For naive prediction
  // TicToc timer_con;
  // if (p_ssc_map_->ConstructSscMap(sur_vehicle_trajs_fs_, obstacle_grids_fs_))
  // {
  //   return kWrongStatus;
  // }
  // printf("[SscPlanner] ConstructSscMap time cost: %lf ms\n",
  // timer_con.toc());
  // TicToc timer_infl;
  // p_ssc_map_->InflateObstacleGrid(ego_vehicle_.param());
  // printf("[SscPlanner] InflateObstacleGrid time cost: %lf ms\n",
  // timer_infl.toc());
  // TicToc timer_corridor;
  // if (p_ssc_map_->ConstructCorridorsUsingInitialTrajectories(
  //         p_ssc_map_->p_3d_inflated_grid(), forward_trajs_fs_) != kSuccess) {
  //   return kWrongStatus;
  // }
  // printf("[SscPlanner] Corridor generation time cost: %lf ms\n",
  //        timer_corridor.toc());

  // ~ For MPDM prediction
  p_ssc_map_->ClearDrivingCorridor();
  int num_behaviors = forward_behaviors_.size();
  for (int i = 0; i < num_behaviors; ++i) {
    if (p_ssc_map_->ConstructSscMap(surround_forward_trajs_fs_[i],
                                    obstacle_grids_fs_)) {
      return kWrongStatus;
    }
    if (p_ssc_map_->ConstructCorridorUsingInitialTrajectoryWithAssignedBehavior(
            p_ssc_map_->p_3d_inflated_grid(), forward_trajs_fs_[i]) !=
        kSuccess) {
      return kWrongStatus;
    }
  }
  p_ssc_map_->GetFinalGlobalMetricCubesList();

  if (RunQpOptimization() != kSuccess) {
    printf("[SscQP]fail to optimize qp trajectories.\n");
    return kWrongStatus;
  }

  BezierSpline bezier_spline;
  if (GetBezierSplineWithCurrentBehavior(qp_trajs_, valid_behaviors_,
                                         &bezier_spline) != kSuccess) {
    printf("[SscQP]BezierToTrajectory: current behavior %d not valid.\n",
           static_cast<int>(ego_behavior_));
    return kWrongStatus;
  }

  if (BezierToTrajectory(bezier_spline, &trajectory_) != kSuccess) {
    printf("[SscQP]fail to transform bezier to trajectory.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType SscPlanner::RunQpOptimization() {
  vec_E<vec_E<common::AxisAlignedsCubeNd<decimal_t, 3>>> cube_list =
      p_ssc_map_->final_cubes_list();
  std::vector<int> if_corridor_valid = p_ssc_map_->if_corridor_valid();
  if (cube_list.size() < 1) return kWrongStatus;
  if (cube_list.size() != forward_behaviors_.size()) {
    printf(
        "[SscQP]cube list %d not consist with behavior size: %d, forward traj "
        "%d, flag size %d\n",
        (int)cube_list.size(), (int)forward_behaviors_.size(),
        (int)forward_trajs_.size(), (int)if_corridor_valid.size());
    return kWrongStatus;
  }

  qp_trajs_.clear();
  valid_behaviors_.clear();
  for (int i = 0; i < static_cast<int>(cube_list.size()); i++) {
    if (if_corridor_valid[i] == 0) {
      printf("[error]**** for behavior %s has no valid corridor. ****\n",
             static_cast<int>(forward_behaviors_[i]));
      continue;
    }

    auto fs_vehicle_traj = forward_trajs_fs_[i];
    int num_states = static_cast<int>(fs_vehicle_traj.size());

    vec_E<Vecf<2>> start_constraints;
    start_constraints.push_back(
        Vecf<2>(ego_frenet_state_.vec_s[0], ego_frenet_state_.vec_dt[0]));
    start_constraints.push_back(
        Vecf<2>(ego_frenet_state_.vec_s[1], ego_frenet_state_.vec_dt[1]));
    start_constraints.push_back(
        Vecf<2>(ego_frenet_state_.vec_s[2], ego_frenet_state_.vec_dt[2]));

    // printf("[Inconsist]Start sd position (%lf, %lf).\n",
    // start_constraints[0](0),
    //        start_constraints[0](1));
    vec_E<Vecf<2>> end_constraints;
    end_constraints.push_back(
        Vecf<2>(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[0],
                fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[0]));
    end_constraints.push_back(
        Vecf<2>(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[1],
                fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[1]));
    common::SplineGenerator<5, 2> spline_generator;
    BezierSpline bezier_spline;

    if (CorridorFeasibilityCheck(cube_list[i]) != kSuccess) {
      printf("[SscQP]corridor not valid for optimization.\n");
      continue;
    }

    std::vector<decimal_t> ref_stamps;
    vec_E<Vecf<2>> ref_points;

    for (int n = 0; n < num_states; n++) {
      ref_stamps.push_back(fs_vehicle_traj[n].frenet_state.time_stamp);
      ref_points.push_back(Vecf<2>(fs_vehicle_traj[n].frenet_state.vec_s[0],
                                   fs_vehicle_traj[n].frenet_state.vec_dt[0]));
    }

    if (spline_generator.GetBezierSplineUsingCorridor(
            cube_list[i], start_constraints, end_constraints, ref_stamps,
            ref_points, &bezier_spline) != kSuccess) {
      printf("[error]******** solver error for behavior %d ********.\n",
             static_cast<int>(forward_behaviors_[i]));
      for (auto& cube : cube_list[i]) {
        printf(
            "[error] x_lb = %f, x_ub = %f, y_lb = %f, y_ub = %f, z_lb = %f, "
            "z_ub = %f, d_z = %f\n",
            cube.pos_lb[0], cube.pos_ub[0], cube.pos_lb[1], cube.pos_ub[1],
            cube.pos_lb[2], cube.pos_ub[2], cube.pos_ub[2] - cube.pos_lb[2]);
      }

      printf("[error]Start sd velocity (%lf, %lf).\n", start_constraints[1](0),
             start_constraints[1](1));
      printf("[error]Start sd acceleration (%lf, %lf).\n",
             start_constraints[2](0), start_constraints[2](1));
      printf("[error]End sd position (%lf, %lf).\n", end_constraints[0](0),
             end_constraints[0](1));
      printf("[error]End sd velocity (%lf, %lf).\n", end_constraints[1](0),
             end_constraints[1](1));
      printf("[error]End state stamp: %lf.\n",
             fs_vehicle_traj[num_states - 1].frenet_state.time_stamp);
      continue;
    }
    // printf("[SscQP]spline begin stamp: %lf.\n", bezier_spline.begin());
    qp_trajs_.push_back(bezier_spline);
    valid_behaviors_.push_back(forward_behaviors_[i]);
  }
  return kSuccess;
}

ErrorType SscPlanner::GetBezierSplineWithCurrentBehavior(
    const vec_E<BezierSpline>& trajs,
    const std::vector<LateralBehavior>& behaviors,
    BezierSpline* bezier_spline) {
  int num_valid_behaviors = static_cast<int>(behaviors.size());
  if (num_valid_behaviors < 1) {
    return kWrongStatus;
  }
  bool find_exact_match_behavior = false;
  int index = 0;
  for (int i = 0; i < num_valid_behaviors; i++) {
    if (behaviors[i] == ego_behavior_) {
      find_exact_match_behavior = true;
      index = i;
    }
  }
  bool find_candidate_behavior = false;
  LateralBehavior candidate_bahavior = common::LateralBehavior::kLaneKeeping;
  if (!find_exact_match_behavior) {
    for (int i = 0; i < num_valid_behaviors; i++) {
      if (behaviors[i] == candidate_bahavior) {
        find_candidate_behavior = true;
        index = i;
      }
    }
  }
  if (!find_exact_match_behavior && !find_candidate_behavior)
    return kWrongStatus;
  // if (!find_exact_match_behavior) return kWrongStatus;

  *bezier_spline = trajs[index];
  return kSuccess;
}

ErrorType SscPlanner::BezierToTrajectory(const BezierSpline& bezier_spline,
                                         Trajectory* traj) {
  using SplineType = Trajectory::SplineType;
  using SplineGeneratorType = Trajectory::SplineGeneratorType;
  const SplineGeneratorType generator;

  std::vector<decimal_t> para;
  vec_E<State> state_vec;

  FrenetState fs;
  State s;

  Vecf<2> pos, vel, acc;
  for (decimal_t t = bezier_spline.begin(); t < bezier_spline.end() + kEPS;
       t += config_.kSampleBezierStep) {
    bezier_spline.evaluate(t, 0, &pos);
    bezier_spline.evaluate(t, 1, &vel);
    bezier_spline.evaluate(t, 2, &acc);
    fs.Load(Vec3f(pos[0], vel[0], acc[0]), Vec3f(pos[1], vel[1], acc[1]),
            FrenetState::kInitWithDt);
    if (stf_.GetStateFromFrenetState(fs, &s) == kSuccess) {
      para.push_back(t);
      state_vec.push_back(s);
    } else {
      fs.Load(Vec3f(pos[0], 0.0, 0.0), Vec3f(pos[1], 0.0, 0.0),
              FrenetState::kInitWithDs);
      stf_.GetStateFromFrenetState(fs, &s);
      para.push_back(t);
      state_vec.push_back(s);
    }
  }

  SplineType spline;
  if (generator.GetSplineFromStateVec(para, state_vec, &spline) != kSuccess) {
    printf("[SscQP]fail to generate spline from state vec.\n");
    return kWrongStatus;
  }
  *traj = Trajectory(spline);
  return kSuccess;
}

ErrorType SscPlanner::CorridorFeasibilityCheck(
    const vec_E<common::AxisAlignedsCubeNd<decimal_t, 3>>& cubes) {
  int num_cubes = static_cast<int>(cubes.size());
  if (num_cubes < 1) {
    printf("[SscQP]number of cubes not enough.\n");
    return kWrongStatus;
  }
  for (int i = 1; i < num_cubes; i++) {
    if (cubes[i].pos_lb[2] != cubes[i - 1].pos_ub[2]) {
      printf("[SscQP]corridor error.\n");
      printf(
          "[SscQP]Err- x_lb = %f, x_ub = %f, y_lb = %f, y_ub = %f, z_lb = %f, "
          "z_ub = %f\n",
          cubes[i - 1].pos_lb[0], cubes[i - 1].pos_ub[0],
          cubes[i - 1].pos_lb[1], cubes[i - 1].pos_ub[1],
          cubes[i - 1].pos_lb[2], cubes[i - 1].pos_ub[2]);
      printf(
          "[SscQP]Err- x_lb = %f, x_ub = %f, y_lb = %f, y_ub = %f, z_lb = %f, "
          "z_ub = %f\n",
          cubes[i].pos_lb[0], cubes[i].pos_ub[0], cubes[i].pos_lb[1],
          cubes[i].pos_ub[1], cubes[i].pos_lb[2], cubes[i].pos_ub[2]);
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType SscPlanner::StateTransformForInputData() {
  vec_E<State> global_state_vec;
  vec_E<Vec2f> global_point_vec;
  int num_v;

  // ~ Stage I. Package states and points
  // * Ego vehicle state and vertices
  {
    global_state_vec.push_back(init_state_);
    vec_E<Vec2f> v_vec;
    common::SemanticsUtils::GetVehicleVertices(ego_vehicle_.param(),
                                               init_state_, &v_vec);
    num_v = v_vec.size();
    global_point_vec.insert(global_point_vec.end(), v_vec.begin(), v_vec.end());
  }

  // * Ego forward simulation trajs states and vertices
  {
    common::VehicleParam ego_param = ego_vehicle_.param();
    for (int i = 0; i < (int)forward_trajs_.size(); ++i) {
      if (forward_trajs_[i].size() < 1) continue;
      for (int k = 0; k < (int)forward_trajs_[i].size(); ++k) {
        // states
        State traj_state = forward_trajs_[i][k].state();
        global_state_vec.push_back(traj_state);
        // vertices
        vec_E<Vec2f> v_vec;
        SscMapUtils::RetVehicleVerticesUsingState(traj_state, ego_param,
                                                  &v_vec);
        global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                v_vec.end());
      }
    }
  }

  // * Surrounding vehicle trajs states and vertices
  {
    for (auto it = semantic_vehicle_set_.semantic_vehicles.begin();
         it != semantic_vehicle_set_.semantic_vehicles.end(); ++it) {
      if (it->second.pred_traj.size() < 1) continue;
      common::VehicleParam v_param = it->second.vehicle.param();
      for (int i = 0; i < (int)it->second.pred_traj.size(); ++i) {
        // states
        State traj_state = it->second.pred_traj[i];
        global_state_vec.push_back(traj_state);
        // vertices
        vec_E<Vec2f> v_vec;
        SscMapUtils::RetVehicleVerticesUsingState(traj_state, v_param, &v_vec);
        global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                v_vec.end());
      }
    }
  }

  // * Surrounding vehicle trajs from MPDM
  {
    for (int i = 0; i < surround_forward_trajs_.size(); ++i) {
      for (auto it = surround_forward_trajs_[i].begin();
           it != surround_forward_trajs_[i].end(); ++it) {
        for (int k = 0; k < it->second.size(); ++k) {
          // states
          State traj_state = it->second[k].state();
          global_state_vec.push_back(traj_state);
          // vertices
          vec_E<Vec2f> v_vec;
          SscMapUtils::RetVehicleVerticesUsingState(
              traj_state, it->second[k].param(), &v_vec);
          global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                  v_vec.end());
        }
      }
    }
  }

  // * Obstacle grids
  {
    for (auto it = obstacle_grids_.begin(); it != obstacle_grids_.end(); ++it) {
      Vec2f pt((*it)[0], (*it)[1]);
      global_point_vec.push_back(pt);
    }
  }

  vec_E<FrenetState> frenet_state_vec(global_state_vec.size());
  vec_E<Vec2f> fs_point_vec(global_point_vec.size());

  // ~ Stage II. Do transformation in multi-thread flavor
#if USE_OPENMP
  TicToc timer_stf;
  StateTransformUsingOpenMp(global_state_vec, global_point_vec,
                            &frenet_state_vec, &fs_point_vec);
  printf("[SscPlanner] OpenMp transform time cost: %lf ms\n", timer_stf.toc());
#else
  TicToc timer_stf;
  StateTransformSingleThread(global_state_vec, global_point_vec,
                             &frenet_state_vec, &fs_point_vec);
  printf("[SscPlanner] Single thread transform time cost: %lf ms\n",
         timer_stf.toc());
#endif

  // ~ Stage III. Retrieve states and points
  int offset = 0;
  // * Ego vehicle state and vertices
  {
    fs_ego_vehicle_.frenet_state = frenet_state_vec[offset];
    fs_ego_vehicle_.vertices.clear();
    for (int i = 0; i < num_v; ++i) {
      fs_ego_vehicle_.vertices.push_back(fs_point_vec[offset * num_v + i]);
    }
    offset++;
  }

  // * Ego forward simulation trajs states and vertices
  {
    forward_trajs_fs_.clear();
    if (forward_trajs_.size() < 1) return kWrongStatus;
    for (int j = 0; j < (int)forward_trajs_.size(); ++j) {
      if (forward_trajs_[j].size() < 1) assert(false);
      vec_E<common::FsVehicle> traj_fs;
      for (int k = 0; k < (int)forward_trajs_[j].size(); ++k) {
        common::FsVehicle fs_v;
        fs_v.frenet_state = frenet_state_vec[offset];
        for (int i = 0; i < num_v; ++i) {
          fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
        }
        traj_fs.emplace_back(fs_v);
        offset++;
      }
      forward_trajs_fs_.emplace_back(traj_fs);
    }
  }

  // * Surrounding vehicle trajs states and vertices
  {
    sur_vehicle_trajs_fs_.clear();
    // if (semantic_vehicle_set_.semantic_vehicles.size() < 1) return
    // kWrongStatus;
    for (auto it = semantic_vehicle_set_.semantic_vehicles.begin();
         it != semantic_vehicle_set_.semantic_vehicles.end(); ++it) {
      if (it->second.pred_traj.size() < 1) continue;
      int v_id = it->first;
      vec_E<common::FsVehicle> traj_fs;
      for (int k = 0; k < (int)it->second.pred_traj.size(); ++k) {
        common::FsVehicle fs_v;
        fs_v.frenet_state = frenet_state_vec[offset];
        for (int i = 0; i < num_v; ++i) {
          fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
        }
        traj_fs.emplace_back(fs_v);
        offset++;
      }
      sur_vehicle_trajs_fs_.insert(
          std::pair<int, vec_E<common::FsVehicle>>(v_id, traj_fs));
    }
  }

  // // ! Surrounding vehicle trajs from MPDM
  // {
  //   for (int i = 0; i < surround_forward_trajs_.size(); ++i) {
  //     for (auto it = surround_forward_trajs_[i].begin();
  //          it != surround_forward_trajs_[i].end(); ++it) {
  //       for (int k = 0; k < it->second.size(); ++k) {
  //         // states
  //         State traj_state = it->second[k].state();
  //         global_state_vec.push_back(traj_state);
  //         // vertices
  //         vec_E<Vec2f> v_vec;
  //         SscMapUtils::RetVehicleVerticesUsingState(
  //             traj_state, it->second[k].param(), &v_vec);
  //         global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
  //                                 v_vec.end());
  //       }
  //     }
  //   }
  // }

  // * Surrounding vehicle trajs from MPDM
  {
    surround_forward_trajs_fs_.clear();
    for (int j = 0; j < surround_forward_trajs_.size(); ++j) {
      std::unordered_map<int, vec_E<common::FsVehicle>> sur_trajs;
      for (auto it = surround_forward_trajs_[j].begin();
           it != surround_forward_trajs_[j].end(); ++it) {
        int v_id = it->first;
        vec_E<common::FsVehicle> traj_fs;
        for (int k = 0; k < it->second.size(); ++k) {
          common::FsVehicle fs_v;
          fs_v.frenet_state = frenet_state_vec[offset];
          for (int i = 0; i < num_v; ++i) {
            fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
          }
          traj_fs.emplace_back(fs_v);
          offset++;
        }
        sur_trajs.insert(
            std::pair<int, vec_E<common::FsVehicle>>(v_id, traj_fs));
      }
      surround_forward_trajs_fs_.emplace_back(sur_trajs);
    }
  }

  // * Obstacle grids
  {
    obstacle_grids_fs_.clear();
    for (int i = 0; i < static_cast<int>(obstacle_grids_.size()); ++i) {
      obstacle_grids_fs_.push_back(fs_point_vec[offset * num_v + i]);
    }
  }

  ego_frenet_state_ = fs_ego_vehicle_.frenet_state;
  return kSuccess;
}

ErrorType SscPlanner::StateTransformUsingOpenMp(
    const vec_E<State>& global_state_vec, const vec_E<Vec2f>& global_point_vec,
    vec_E<FrenetState>* frenet_state_vec, vec_E<Vec2f>* fs_point_vec) const {
  int state_num = global_state_vec.size();
  int point_num = global_point_vec.size();

  auto ptr_state_vec = frenet_state_vec->data();
  auto ptr_point_vec = fs_point_vec->data();

  printf("[OpenMp]Total number of queries: %d.\n", state_num + point_num);
  omp_set_num_threads(4);
  {
#pragma omp parallel for
    for (int i = 0; i < state_num; ++i) {
      FrenetState fs;
      if (kSuccess != stf_.GetFrenetStateFromState(global_state_vec[i], &fs)) {
        fs.time_stamp = global_state_vec[i].time_stamp;
      }
      *(ptr_state_vec + i) = fs;
    }
  }
  {
#pragma omp parallel for
    for (int i = 0; i < point_num; ++i) {
      Vec2f fs_pt;
      stf_.GetFrenetPointFromPoint(global_point_vec[i], &fs_pt);
      *(ptr_point_vec + i) = fs_pt;
    }
  }
  return kSuccess;
}

ErrorType SscPlanner::StateTransformSingleThread(
    const vec_E<State>& global_state_vec, const vec_E<Vec2f>& global_point_vec,
    vec_E<FrenetState>* frenet_state_vec, vec_E<Vec2f>* fs_point_vec) const {
  int state_num = global_state_vec.size();
  int point_num = global_point_vec.size();
  auto ptr_state_vec = frenet_state_vec->data();
  auto ptr_point_vec = fs_point_vec->data();
  {
    for (int i = 0; i < state_num; ++i) {
      FrenetState fs;
      stf_.GetFrenetStateFromState(global_state_vec[i], &fs);
      *(ptr_state_vec + i) = fs;
    }
  }
  {
    for (int i = 0; i < point_num; ++i) {
      Vec2f fs_pt;
      stf_.GetFrenetPointFromPoint(global_point_vec[i], &fs_pt);
      *(ptr_point_vec + i) = fs_pt;
    }
  }
  return kSuccess;
}

ErrorType SscPlanner::set_map_interface(SscPlannerMapItf* map_itf) {
  if (map_itf == nullptr) return kIllegalInput;
  map_itf_ = map_itf;
  map_valid_ = true;
  return kSuccess;
}

ErrorType SscPlanner::GetSemanticCubeList(
    const vec_E<common::SpeedLimit>& speed_limit,
    const vec_E<common::TrafficLight>& traffic_light,
    std::vector<common::AxisAlignedsCubeNd<decimal_t, 3>>* cubes) {
  // ~ hard code some lateral boundaries (lane)
  // std::vector<decimal_t> lat_boundary_pos = {
  //     {-1.75 - 3.5, -1.75, 1.75, 1.75 + 3.5, 1.75 + 7}};
  std::vector<decimal_t> lat_boundary_pos = {};

  // ~ Convert rules to frenet frame
  vec_E<common::SpeedLimit> speed_limit_fs;
  for (const auto& rule : speed_limit) {
    Vec2f fp_0;
    stf_.GetFrenetPointFromPoint(rule.start_point(), &fp_0);
    Vec2f fp_1;
    stf_.GetFrenetPointFromPoint(rule.end_point(), &fp_1);
    common::SpeedLimit sl_fs(fp_0, fp_1, rule.vel_range());
    speed_limit_fs.push_back(sl_fs);
  }

  int rule_num = speed_limit_fs.size();
  std::set<decimal_t> lon_boundary_pos_set;
  for (int i = 0; i < rule_num; ++i) {
    lon_boundary_pos_set.insert(speed_limit_fs[i].start_point()(0));
    lon_boundary_pos_set.insert(speed_limit_fs[i].end_point()(0));
  }
  lon_boundary_pos_set.insert(0);
  lon_boundary_pos_set.insert(p_ssc_map_->config().map_size[0] *
                              p_ssc_map_->config().map_resolution[0]);

  std::vector<decimal_t> lon_boundary_pos;
  lon_boundary_pos.assign(lon_boundary_pos_set.begin(),
                          lon_boundary_pos_set.end());

  // printf("[Corridor] Lon: ");
  // for (const auto& lon : lon_boundary_pos) {
  //   printf("%f ", lon);
  // }
  // printf("\n");
  // printf("[Corridor] Lat: ");
  // for (const auto& lat : lat_boundary_pos) {
  //   printf("%f ", lat);
  // }
  // printf("\n");

  for (int i = 0; i < static_cast<int>(lon_boundary_pos.size()) - 1; ++i) {
    for (int j = 0; j < static_cast<int>(lat_boundary_pos.size()) - 1; ++j) {
      std::vector<decimal_t> ub = {lon_boundary_pos[i + 1],
                                   lat_boundary_pos[j + 1], 10};
      std::vector<decimal_t> lb = {lon_boundary_pos[i], lat_boundary_pos[j], 0};
      common::AxisAlignedsCubeNd<decimal_t, 3> cube(ub, lb);
      cubes->push_back(cube);
    }
  }

  for (int i = 0; i < cubes->size(); ++i) {
    for (int j = 0; j < rule_num; ++j) {
      if ((*cubes)[i].pos_ub[0] <= speed_limit_fs[j].end_point()(0) &&
          (*cubes)[i].pos_lb[0] >= speed_limit_fs[j].start_point()(0)) {
        (*cubes)[i].v_lb[0] = speed_limit_fs[j].vel_range()(0);
        (*cubes)[i].v_ub[0] = speed_limit_fs[j].vel_range()(1);
      }
    }
  }

  // printf("[Corridor]cubes->size() = %d\n", cubes->size());
  // for (int i = 0; i < cubes->size(); ++i) {
  //   printf("[Corridor] %d - s: [%f, %f], d: [%f, %f], v: [%f, %f]\n", i,
  //          (*cubes)[i].pos_lb[0], (*cubes)[i].pos_ub[0],
  //          (*cubes)[i].pos_lb[1],
  //          (*cubes)[i].pos_ub[1], (*cubes)[i].v_lb[0], (*cubes)[i].v_ub[0]);
  // }

  return kSuccess;
}

}  // namespace planning