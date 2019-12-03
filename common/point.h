/***************************************************************************
 *
 * Copyright (c) 2019 JD.com, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file point.h
 **/
#pragma once

#include <cmath>
#include <iostream>

constexpr double kMathEpsilon = 1e-6;
struct Point {
  double x_;
  double y_;
  Point() {}
  Point(double x, double y) : x_(x), y_(y) {}
  double x() { return x_; }
  double y() { return y_; }
  void set_x(const double x) { x_ = x; }
  void set_y(const double y) { y_ = y; }
  double Length() { return std::hypot(x_, y_); }
  Point operator+(const Point &other) const {
    return Point(x_ + other.x_, y_ + other.y_);
  }
  Point operator-(const Point &other) const {
    return Point(x_ - other.x_, y_ - other.y_);
  }

  Point operator*(const double ratio) const {
    return Point(x_ * ratio, y_ * ratio);
  }

  Point operator/(const double ratio) const {
    if (ratio <= kMathEpsilon) {
      std::cout << "ratio is 0,wrong!" << std::endl;
    }
    return Point(x_ / ratio, y_ / ratio);
  }
  double Angle() const { return std::atan2(y_, x_); }
};
