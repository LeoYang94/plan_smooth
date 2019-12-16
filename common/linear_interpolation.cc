/***************************************************************************
 *
 * Copyright (c) 2019 JD.com, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file linear_interpolation.cpp
 **/
#include "linear_interpolation.h"
#include <cmath>
#include <vector>

Point Interpolate(const Point &p1, const Point &p2, double distance) {
  Point error_p(p2.x_ - p1.x_,p2.y_ - p1.y_);
  Point p3;
  p3.set_x(error_p.x_* (distance / error_p.Length()) + p1.x_);
  p3.set_y(error_p.y_ * (distance / error_p.Length()) + p1.y_);
  return p3;
}

void LinearInterpolation(const std::vector<Point> &points_series,
                         const double step,
                         std::vector<Point> *const interpolation_points_ptr) {
  bool first_flag = true;
  Point last_point;
  double remain_step = 0;
  for (const auto &point_iter : points_series) {
    if (first_flag) {
      first_flag = false;
      last_point = point_iter;
      continue;
    }
    Point point_temp(point_iter.x_ - last_point.x_,point_iter.y_ - last_point.y_);
    double line_seg_norm = point_temp.Length() + kMathEpsilon;
    double sum_step_in_this_line = remain_step;
    while (sum_step_in_this_line < line_seg_norm) {
      Point interpolation_point =
          Interpolate(last_point, point_iter, sum_step_in_this_line);
      interpolation_points_ptr->emplace_back(interpolation_point);
      sum_step_in_this_line = sum_step_in_this_line + step;
    }
    remain_step = sum_step_in_this_line - line_seg_norm;
    last_point = point_iter;
  }

  if (interpolation_points_ptr->back().x_ != points_series.back().x_ ||
      interpolation_points_ptr->back().y_ != points_series.back().y_) {
    interpolation_points_ptr->emplace_back(points_series.back());
  }
}
