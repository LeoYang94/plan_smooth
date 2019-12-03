/***************************************************************************
 *
 * Copyright (c) 2019 JD.com, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file line_segment2d.cc
 **/
#include "line_segment2d.h"
#include <cmath>
#include "point.h"

LineSegment2d::LineSegment2d() {
    unit_direction_ = Point(1,0);
}

LineSegment2d::LineSegment2d(const Point& start, const Point& end) : start_(start), end_(end) {
    const double dx = end_.x() - start_.x();
    const double dy = end_.y() - start_.y();
    length_ = std::hypot(dx, dy);
    direction_ = (length_ <= kMathEpsilon ? Point(0, 0) : end - start);

    unit_direction_ = (length_ <= kMathEpsilon ? Point(0, 0) : Point(dx / length_, dy / length_));
    heading_ = unit_direction_.Angle();
    line_a_ = (end.y_ - start.y_);
    line_b_ = (start.x_ - end.x_);
    line_c_ = (end.x_ * start.y_ - start.x_ * end.y_);
}

