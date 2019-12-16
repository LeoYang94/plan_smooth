/***************************************************************************
 *
 * Copyright (c) 2019 JD.com, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file linear_interpolation.h
 **/

#include <vector>
#include "point.h"

#pragma once

Point Interpolate(const Point &p1, const Point &p2,
                  double distance);

void LinearInterpolation(const std::vector<Point> &points_series,
                         const double step,
                         std::vector<Point> *const interpolation_points_ptr);
