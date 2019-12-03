/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file cg_smoother.h
 **/

#pragma once

#include "smoother_model.h"

class CgSmoother final : public SmootherModel{
public:
     CgSmoother() = default;
    ~CgSmoother() = default;
    bool Run(const OptimizerParam &dealed_op,
             std::vector<double> *const smoother_results) const override;


};