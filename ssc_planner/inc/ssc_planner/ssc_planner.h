/**
 * @file ssc_planner.h
 * @author HKUST Aerial Robotics Group
 * @brief planner using spatio-temporal corridor
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_SEARCH_H_
#define _UTIL_SSC_PLANNER_INC_SSC_SEARCH_H_

#include <memory>
#include <set>
#include <string>
#include <thread>

#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/spline/spline_generator.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/trajectory.h"
#include "ssc_planner/map_interface.h"
#include "ssc_planner/ssc_map.h"

namespace planning {

class SscPlanner : public Planner {
 public:
  using ObstacleMapType = uint8_t;
  using SscMapDataType = uint8_t;

  using Lane = common::Lane;
  using State = common::State;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;
  using FrenetState = common::FrenetState;
  using Trajectory = common::Trajectory;
  using SemanticVehicleSet = common::SemanticVehicleSet;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;

  typedef common::BezierSpline<5, 2> BezierSpline;
  struct Config {
    decimal_t kSampleBezierStep{0.05};
    decimal_t kMaxCurvature{0.3};
    decimal_t kMaxTangentAcceleration{2.0};
    decimal_t kMaxNormalAcceleration{2.0};
    decimal_t kMaxTangentDeceleration{2.0};
    decimal_t kMaxVelocity{28.0};
  };

  SscPlanner();
  SscPlanner(const Config& cfg);

  /**
   * @brief setters
   */
  ErrorType set_map_interface(SscPlannerMapItf* map_itf);

  ErrorType set_init_state(const State& state);
  /**
   * @brief getters
   */
  SscMap* p_ssc_map() const { return p_ssc_map_; }

  FrenetState ego_frenet_state() const { return ego_frenet_state_; }

  Vehicle ego_vehicle() const { return ego_vehicle_; }

  vec_E<vec_E<Vehicle>> forward_trajs() const { return forward_trajs_; }

  vec_E<BezierSpline> qp_trajs() const { return qp_trajs_; }

  decimal_t time_origin() const { return time_origin_; }

  std::unordered_map<int, vec_E<common::FsVehicle>> sur_vehicle_trajs_fs()
      const {
    return sur_vehicle_trajs_fs_;
  }

  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs() const {
    return forward_trajs_fs_;
  }

  vec_E<Vec2f> ego_vehicle_contour_fs() const {
    return fs_ego_vehicle_.vertices;
  }

  Trajectory trajectory() const { return trajectory_; }

  /**
   * @brief Initialize the planner with config path
   */
  std::string Name() override;

  /**
   * @brief Initialize the planner with config path
   */
  ErrorType Init(const std::string config) override;

  /**
   * @brief Run one planning round with given states
   */
  ErrorType RunOnce() override;

 private:
  ErrorType CorridorFeasibilityCheck(
      const vec_E<common::AxisAlignedsCubeNd<decimal_t, 3>>& cubes);
  /**
   * @brief transform all the states in a batch
   */
  ErrorType StateTransformForInputData();

  ErrorType RunQpOptimization();
  /**
   * @brief transform all the states using openmp
   */
  ErrorType StateTransformUsingOpenMp(const vec_E<State>& global_state_vec,
                                      const vec_E<Vec2f>& global_point_vec,
                                      vec_E<FrenetState>* frenet_state_vec,
                                      vec_E<Vec2f>* fs_point_vec) const;

  ErrorType StateTransformSingleThread(const vec_E<State>& global_state_vec,
                                       const vec_E<Vec2f>& global_point_vec,
                                       vec_E<FrenetState>* frenet_state_vec,
                                       vec_E<Vec2f>* fs_point_vec) const;

  ErrorType GetBezierSplineWithCurrentBehavior(
      const vec_E<BezierSpline>& trajs,
      const std::vector<LateralBehavior>& behaviors,
      BezierSpline* bezier_spline);

  ErrorType BezierToTrajectory(const BezierSpline& bezier_spline,
                               Trajectory* traj);

  ErrorType GetSemanticCubeList(
      const vec_E<common::SpeedLimit>& speed_limit,
      const vec_E<common::TrafficLight>& traffic_light,
      std::vector<common::AxisAlignedsCubeNd<decimal_t, 3>>* cubes);

  Vehicle ego_vehicle_;
  LateralBehavior ego_behavior_;
  FrenetState ego_frenet_state_;
  Lane nav_lane_local_;
  decimal_t time_origin_{0.0};

  State init_state_;
  bool has_init_state_ = false;

  GridMap2D grid_map_;
  std::set<std::array<decimal_t, 2>> obstacle_grids_;
  SemanticVehicleSet semantic_vehicle_set_;
  vec_E<vec_E<Vehicle>> forward_trajs_;
  std::vector<LateralBehavior> forward_behaviors_;
  vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_forward_trajs_;

  vec_E<Vec2f> obstacle_grids_fs_;

  // Initial solution for optimization
  common::FsVehicle fs_ego_vehicle_;
  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs_;
  std::unordered_map<int, vec_E<common::FsVehicle>> sur_vehicle_trajs_fs_;
  vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>>
      surround_forward_trajs_fs_;

  vec_E<BezierSpline> qp_trajs_;
  std::vector<LateralBehavior> valid_behaviors_;

  Trajectory trajectory_;

  common::StateTransformer stf_;
  // Map
  SscPlannerMapItf* map_itf_;
  bool map_valid_ = false;
  SscMap* p_ssc_map_;

  std::pair<State, decimal_t> last_lane_s_;
  bool has_last_state_s_ = false;
  decimal_t global_s_offset_ = 0.0;

  TicToc timer_;
  Config config_;
};

}  // namespace planning

#endif