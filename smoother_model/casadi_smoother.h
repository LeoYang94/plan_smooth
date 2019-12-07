/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file casadi_smoother.h
 **/

#pragma once

#include <vector>

#include <casadi/casadi.hpp>
#include <casadi/core/sparsity.hpp>
#include <casadi/core/sparsity_interface.hpp>

#include "optimizer_param.h"
#include "smoother_model.h"

class CasadiSmoother final : public SmootherModel {
public:
  CasadiSmoother() = default;
  ~CasadiSmoother() = default;
  bool Run(const OptimizerParam &dealed_op,
           std::vector<double> *const smoother_results) const override;

private:
  bool EvalF(const OptimizerParam &param, const int32_t ne, const casadi::SX &x,
             casadi::SX *const f) const;
  bool EvalConstraints(const OptimizerParam &param, const int32_t ne,
                       const casadi::SX &x,
                       std::vector<casadi::SX> *const fg) const;
  casadi::Dict AddOptions() const;

  bool GetBoundsInfo(const OptimizerParam &param, const int32_t ne,
                     const int32_t nx, const int32_t ng,
                     std::vector<double> *const x_l,
                     std::vector<double> *const x_u,
                     std::vector<double> *const g_l,
                     std::vector<double> *const g_u) const;

  bool GetStartingPoint(const OptimizerParam &param, const int32_t ne,
                        std::vector<double> *const xi) const;
  void GetPathPoints(const casadi::DM result, int32_t nx,
                     std::vector<double> *const results_ptr) const;
};