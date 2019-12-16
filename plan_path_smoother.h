/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file plan_path_smoother.h
 **/

#pragma once

#include <memory>
#include <vector>

#include "optimizer_param.h"
#include "common/point.h"
#include "smoother_model/smoother_model.h"

class PlanPathSmoother {
public:
  PlanPathSmoother(const bool use_ipopt);
  bool Smooth(const std::vector<Point> &raw_path_line,
              std::vector<Point> *const smoothed_path_line);

private:
  bool MakePathParams(const std::vector<Point> &path_line_points,
                      std::vector<Point> *const smooth_path_line);

  bool Solve(const OptimizerParam &op,
             std::vector<Point> *const smooth_path_line);

  void ParamSepecialProcessing(const OptimizerParam& op,OptimizerParam* const dealed_op_ptr)const;

  bool use_ipopt_ = true;
  double wheel_base_ = 1.2;
  double max_acceleration_ = 4.0;
  double max_speed_ = 5.0;
  double max_steer_angle_ = 3.14;
  double max_steer_angle_rate_ = 1.5;
  double smooth_weight_ = 1000;
  // double fixed_point_num_ = 4.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double smooth_point_step_ = 0.0;
  uint32_t fixed_point_num_ = 0;
  int32_t ne_ = 0;
  const double min_angle_error_ = 0.05;
  const double remain_len_ = 20.0; // curve remain length
  std::shared_ptr<SmootherModel> smoother_model_;
};
