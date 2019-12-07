/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file smoother_model.h
 **/

#pragma once

#include "optimizer_param.h"

class SmootherModel {
public:
  SmootherModel() {}
  virtual ~SmootherModel() = default;
  virtual bool Run(const OptimizerParam &dealed_op,
                   std::vector<double> *const smoother_results) const = 0;
};
