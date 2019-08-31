/**
 * @file ssc_map.h
 * @author HKUST Aerial Robotics Group
 * @brief maintain the spatial temporal map for the planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_MAP_H_
#define _UTIL_SSC_PLANNER_INC_SSC_MAP_H_

#include <assert.h>
#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "common/basics/semantics.h"
#include "common/state/state.h"

namespace planning {

using ObstacleMapType = uint8_t;
using SscMapDataType = uint8_t;

class SscMap {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using SemanticVehicleSet = common::SemanticVehicleSet;
  using GridMap3D = common::GridMapND<ObstacleMapType, 3>;

  struct Config {
    std::array<int, 3> map_size = {{1000, 100, 51}};                // s, d, t
    std::array<decimal_t, 3> map_resolution = {{0.25, 0.2, 0.15}};  // m, m, s
    std::array<std::string, 3> axis_name = {{"s", "d", "t"}};
    decimal_t s_back_len = 0.0;
    decimal_t kMaxLongitudinalVel = 30.0;
    decimal_t kMinLongitudinalVel = 0.0;
    decimal_t kMaxLateralVel = 2.0;
    decimal_t kMaxLongitudinalAcc = 2.0;
    decimal_t kMaxLongitudinalDecel = -3.0;  // Avg. driver max
    decimal_t kMaxLateralAcc = 2.0;
    decimal_t kMaxLateralDecel = -2.0;
  };

  SscMap() {}
  SscMap(const Config &config);
  ~SscMap() {}

  GridMap3D *p_3d_grid() const { return p_3d_grid_; }
  GridMap3D *p_3d_inflated_grid() const { return p_3d_inflated_grid_; }

  Config config() const { return config_; }

  vec_E<common::DrivingCorridor3D> driving_corridor_vec() const {
    return driving_corridor_vec_;
  }
  vec_E<vec_E<common::AxisAlignedsCubeNd<decimal_t, 3>>> final_cubes_list()
      const {
    return final_cubes_list_;
  };
  std::vector<common::AxisAlignedsCubeNd<int, 3>> semantic_voxel_set() const {
    return semantic_voxel_set_;
  }

  std::vector<int> if_corridor_valid() const { return if_corridor_valid_; }

  void set_start_time(const decimal_t &t) { start_time_ = t; }
  void set_desired_fs(const common::FrenetState &fs) { desired_fs_ = fs; }

  void set_semantic_cube_set(
      const std::vector<common::AxisAlignedsCubeNd<decimal_t, 3>> &in) {
    semantic_cube_set_ = in;
  }

  void set_speed_limit_list(const vec_E<common::SpeedLimit> &in) {
    speed_limit_list_ = in;
  }

  void set_traffic_light_list(const vec_E<common::TrafficLight> &in) {
    traffic_light_list_ = in;
  }

  ErrorType ConstructSscMap(
      const std::unordered_map<int, vec_E<common::FsVehicle>>
          &sur_vehicle_trajs_fs,
      const vec_E<Vec2f> &obstacle_grids);

  ErrorType InflateObstacleGrid(const common::VehicleParam &param);

  ErrorType ConstructCorridorsUsingInitialTrajectories(
      GridMap3D *p_grid, const vec_E<vec_E<common::FsVehicle>> &trajs);

  ErrorType ConstructCorridorUsingInitialTrajectoryWithAssignedBehavior(
      GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs);

  ErrorType GetDtUsingTimeStamp(const decimal_t &time_stamp,
                                decimal_t *dt) const;

  ErrorType ClearDrivingCorridor();

  ErrorType GetFinalGlobalMetricCubesList();

 private:
  bool CheckIfCubeIsFree(GridMap3D *p_grid,
                         const common::AxisAlignedsCubeNd<int, 3> &cube) const;

  bool CheckIfPlaneIsFreeOnXAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedsCubeNd<int, 3> &cube,
                                 const int &z) const;

  bool CheckIfPlaneIsFreeOnYAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedsCubeNd<int, 3> &cube,
                                 const int &z) const;

  bool CheckIfPlaneIsFreeOnZAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedsCubeNd<int, 3> &cube,
                                 const int &z) const;

  bool CheckIfCubeContainsSeed(const common::AxisAlignedsCubeNd<int, 3> &cube_a,
                               const Vec3i &seed) const;

  bool CheckIfIntersectWithSemanticVoxels(
      const common::AxisAlignedsCubeNd<int, 3> &cube,
      std::unordered_map<int, std::array<bool, 6>> *inters_on_sem_voxels,
      std::unordered_map<int, std::array<bool, 6>> *inters_on_cube) const;

  bool CheckIfIntersectionsIgnorable(
      const std::unordered_map<int, std::array<bool, 6>> &intersections) const;

  ErrorType GetSemanticVoxelSet(GridMap3D *p_grid);

  ErrorType GetInitialCubeUsingSeed(const vec_E<Vec3i> &seeds, const int &i,
                                    common::AxisAlignedsCubeNd<int, 3> *cube);

  ErrorType GetInitialCubeUsingSeed(
      const Vec3i &seed_0, const Vec3i &seed_1,
      common::AxisAlignedsCubeNd<int, 3> *cube) const;

  ErrorType GetTimeCoveredCubeIndices(
      const common::DrivingCorridor3D *p_corridor, const int &start_id,
      const int &dir, const int &t_trans, std::vector<int> *idx_list) const;

  ErrorType CorridorRelaxation(GridMap3D *p_grid,
                               common::DrivingCorridor3D *p_corridor);

  ErrorType FillSemanticInfoInCorridor(GridMap3D *p_grid,
                                       common::DrivingCorridor3D *p_corridor);

  ErrorType InflateCubeIn3dGrid(GridMap3D *p_grid,
                                const std::array<bool, 6> &dir_disabled,
                                const std::array<int, 6> &dir_step,
                                common::AxisAlignedsCubeNd<int, 3> *cube);

  ErrorType GetInflationDirections(const bool &if_inter_with_sv,
                                   const bool &if_first_cube,
                                   const Vec3i &seed_0, const Vec3i &seed_1,
                                   std::array<bool, 6> *dirs_disabled);

  bool InflateCubeOnXPosAxis(GridMap3D *p_grid, const int &n_step,
                             const bool &skip_semantic_cube,
                             common::AxisAlignedsCubeNd<int, 3> *cube);
  bool InflateCubeOnXNegAxis(GridMap3D *p_grid, const int &n_step,
                             const bool &skip_semantic_cube,
                             common::AxisAlignedsCubeNd<int, 3> *cube);
  bool InflateCubeOnYPosAxis(GridMap3D *p_grid, const int &n_step,
                             const bool &skip_semantic_cube,
                             common::AxisAlignedsCubeNd<int, 3> *cube);
  bool InflateCubeOnYNegAxis(GridMap3D *p_grid, const int &n_step,
                             const bool &skip_semantic_cube,
                             common::AxisAlignedsCubeNd<int, 3> *cube);
  bool InflateCubeOnZPosAxis(GridMap3D *p_grid, const int &n_step,
                             const bool &skip_semantic_cube,
                             common::AxisAlignedsCubeNd<int, 3> *cube);
  bool InflateCubeOnZNegAxis(GridMap3D *p_grid, const int &n_step,
                             const bool &skip_semantic_cube,
                             common::AxisAlignedsCubeNd<int, 3> *cube);

  ErrorType FillStaticPart(const vec_E<Vec2f> &obs_grid_fs);

  ErrorType FillDynamicPart(
      const std::unordered_map<int, vec_E<common::FsVehicle>>
          &sur_vehicle_trajs_fs);

  ErrorType FillMapWithFsVehicleTraj(const vec_E<common::FsVehicle> traj);

  common::GridMapND<SscMapDataType, 3> *p_3d_grid_;
  common::GridMapND<SscMapDataType, 3> *p_3d_inflated_grid_;

  std::unordered_map<int, std::array<bool, 6>> inters_for_sem_voxels_;
  std::unordered_map<int, std::array<bool, 6>> inters_for_cube_;

  Config config_;

  decimal_t start_time_;

  common::FrenetState desired_fs_;

  bool map_valid_ = false;

  std::vector<common::AxisAlignedsCubeNd<decimal_t, 3>> semantic_cube_set_;
  std::vector<common::AxisAlignedsCubeNd<int, 3>> semantic_voxel_set_;

  vec_E<common::DrivingCorridor3D> driving_corridor_vec_;

  std::vector<int> if_corridor_valid_;
  vec_E<vec_E<common::AxisAlignedsCubeNd<decimal_t, 3>>> final_cubes_list_;

  // Traffic signal
  vec_E<common::SpeedLimit> speed_limit_list_;
  vec_E<common::TrafficLight> traffic_light_list_;
};

}  // namespace planning
#endif  // _UTIL_SSC_PLANNER_INC_SSC_MAP_H_