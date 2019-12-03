/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file adolc_smoother_model.h
 **/

#pragma once
#include "smoother_model.h"

class AdolcSmootherModel final : public SmootherModel {
public:
  AdolcSmootherModel() = default;
  ~AdolcSmootherModel() = default;
  bool Run(const OptimizerParam &dealed_op,
           std::vector<double> *const smoother_results) const override;
};