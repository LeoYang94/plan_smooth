/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file optimizer_param.h
 **/
#include <array>
#include <vector>

#include "optimizer_state.h"

#pragma once

struct OptimizerParam {
  double bound_a = 0;
  double bound_v = 0;
  double bound_phy = 0;
  double bound_w = 0;
  double wheelbase = 0;
  double smooth_weight = 1000;
  double fixed_point_num = 4;
  std::vector<OptimizerState> states;
  std::vector<std::array<double, 4>> line_param;
};