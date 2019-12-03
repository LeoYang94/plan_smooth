/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file plan_path_smoother.cc
 **/

#include "plan_path_smoother.h"

#include <vector>

#include "common/line_segment2d.h"
#include "common/point.h"
#include "optimizer_param.h"
#include "optimizer_state.h"
#include "smoother_model/adolc_smoother_model.h"

PlanPathSmoother::PlanPathSmoother(const bool use_ipopt)
    : use_ipopt_(use_ipopt) {
  smooth_point_step_ = 0.4;
  smoother_model_ = std::make_shared<AdolcSmootherModel>();
}

bool PlanPathSmoother::Smooth(const std::vector<Point> &raw_path_line,
                              std::vector<Point> *const smoothed_path_line) {

  return MakePathParams(raw_path_line, smoothed_path_line);
}

bool PlanPathSmoother::MakePathParams(
    const std::vector<Point> &raw_path_line,
    std::vector<Point> *const smooth_path_line) {
  std::vector<OptimizerState> path_state;
  for (uint32_t i = 0; i < raw_path_line.size(); ++i) {
    Point curr_point(raw_path_line[i]);
    double theta = 0.0;
    if (i != raw_path_line.size() - 1) {
      Point next_point(raw_path_line[i + 1]);
      Point p_error(next_point.x_ - curr_point.x_,
                    next_point.y_ - curr_point.y_);
      theta = atan2(p_error.y_, p_error.x_);
    } else {
      theta = path_state.back().theta;
    }
    OptimizerState state;
    state.x = curr_point.x_;
    state.y = curr_point.y_;
    state.theta = theta;
    state.v = 0.8;
    path_state.emplace_back(state);
  }
  OptimizerParam optimizer_param;
  optimizer_param.wheelbase = wheel_base_;
  optimizer_param.bound_a = max_acceleration_;
  optimizer_param.bound_v = max_speed_;
  optimizer_param.bound_phy = max_steer_angle_;
  optimizer_param.bound_w = max_steer_angle_rate_;
  optimizer_param.smooth_weight = smooth_weight_; //未找到，暂时值
  optimizer_param.fixed_point_num = 4.0;          //未找到，暂时值
  optimizer_param.states.clear();
  optimizer_param.states = path_state;
  return Solve(optimizer_param, smooth_path_line);
}

bool PlanPathSmoother::Solve(const OptimizerParam &op,
                             std::vector<Point> *const smooth_path_line) {
  origin_x_ = op.states.front().x;
  origin_y_ = op.states.front().y;
  ne_ = static_cast<int32_t>(op.states.size());
  OptimizerParam dealed_op;
  ParamSepecialProcessing(op, &dealed_op);
  std::vector<double> smoothed_result;
  smoother_model_->Run(dealed_op,&smoothed_result);
  return true;
}

void PlanPathSmoother::ParamSepecialProcessing(
    const OptimizerParam &path_params,
    OptimizerParam *const dealed_path_params_ptr) const {
  *dealed_path_params_ptr = path_params;
  // go to zero
  for (auto &state : dealed_path_params_ptr->states) {
    state.x -= origin_x_;
    state.y -= origin_y_;
  }
  // angle processing
  for (uint32_t i = 1; i < dealed_path_params_ptr->states.size(); ++i) {
    auto &state = dealed_path_params_ptr->states[i];
    auto &last_state = dealed_path_params_ptr->states[i - 1];
    double delta_theta = state.theta - last_state.theta;
    while (std::fabs(delta_theta) > M_PI) {
      if (delta_theta > 0) {
        state.theta -= 2 * M_PI;
      } else {
        state.theta += 2 * M_PI;
      }
      delta_theta = state.theta - last_state.theta;
    }
  }
  // vertical line constrain
  std::vector<std::array<double, 4>> line_params;
  auto states_cp = dealed_path_params_ptr->states;
  states_cp.insert(states_cp.begin(), dealed_path_params_ptr->states.front());
  states_cp.emplace_back(dealed_path_params_ptr->states.back());
  for (uint32_t i = 1; i + 1 < states_cp.size(); ++i) {
    Point hor_line_start;
    Point ver_line_start;
    Point ver_line_end;
    Point hor_line_end;
    const auto &curr_state = states_cp[i];
    const auto &next_state = states_cp[i + 1];
    const auto &last_state = states_cp[i - 1];
    hor_line_start = Point(curr_state.x, curr_state.y);
    ver_line_start = Point(last_state.x, last_state.y);
    ver_line_end = Point(next_state.x, next_state.y);

    LineSegment2d ver_line(ver_line_start, ver_line_end);
    // vertical line in right
    Point hor_line_direct =
        Point(ver_line.direction().y_, -ver_line.direction().x_);
    Point hor_line_end_temp = hor_line_direct + ver_line.start();
    LineSegment2d hor_line_temp(ver_line.start(), hor_line_end_temp);
    hor_line_end = hor_line_temp.direction() + hor_line_start;

    double line_a = (hor_line_end.y() - hor_line_start.y());
    double line_b = (hor_line_start.x() - hor_line_end.x());
    double libe_c = (hor_line_end.x() * hor_line_start.y() -
                     hor_line_start.x() * hor_line_end.y());
    std::array<double, 4> line_param{line_a, line_b, libe_c, 1};
    line_params.emplace_back(line_param);
  }
  dealed_path_params_ptr->line_param = line_params;
}
