/**
 * @file ssc_map_utils.h
 * @author HKUST Aerial Robotics Group
 * @brief utils for map maintenance
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_MAP_UTILS_H__
#define _UTIL_SSC_PLANNER_INC_SSC_MAP_UTILS_H__

#include "ssc_planner/ssc_map.h"

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"

namespace planning {

class SscMapUtils {
 public:
  static ErrorType RetVehicleVerticesUsingState(
      const common::State& state, const common::VehicleParam& param,
      vec_E<Vec2f>* vertices) {
    decimal_t angle = state.angle;
    decimal_t x = state.vec_position(0);
    decimal_t y = state.vec_position(1);

    decimal_t cos_theta = cos(angle);
    decimal_t sin_theta = sin(angle);

    decimal_t c_x = x + param.d_cr() * cos_theta;
    decimal_t c_y = y + param.d_cr() * sin_theta;

    decimal_t d_wx = param.width() / 2 * sin_theta;
    decimal_t d_wy = param.width() / 2 * cos_theta;
    decimal_t d_lx = param.length() / 2 * cos_theta;
    decimal_t d_ly = param.length() / 2 * sin_theta;

    // Counterclockwise from left-front vertex
    vertices->push_back(Vec2f(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
    // vertices->push_back(Vec2f(c_x - d_wx, c_y + d_wy));
    vertices->push_back(Vec2f(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
    // vertices->push_back(Vec2f(c_x - d_lx, c_y - d_ly));
    vertices->push_back(Vec2f(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
    // vertices->push_back(Vec2f(c_x + d_wx, c_y - d_wy));
    vertices->push_back(Vec2f(c_x + d_wx + d_lx, c_y + d_ly - d_wy));
    // vertices->push_back(Vec2f(c_x + d_lx, c_y + d_ly));

    return kSuccess;
  }
};  // SscMapUtils

}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_SSC_MAP_UTILS_H__