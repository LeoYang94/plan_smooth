/***************************************************************************
 *
 * Copyright (c) 2019 JD.com, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file linear_segment2d.h
 **/
#pragma once

#include "point.h"
class LineSegment2d{
public:
    LineSegment2d();
    LineSegment2d(const Point& start, const Point& end);

    const Point& start() const {
        return start_;
    }

    /**
     * @brief Get the end point.
     * @return The end point of the line segment.
     */
    const Point& end() const {
        return end_;
    }

    /**
     * @brief Get the direction from the start point to the end point.
     * @return The direction of the line segment.
     */
    const Point& direction() const {
        return direction_;
    }

    /**
     * @brief Get the unit direction from the start point to the end point.
     * @return The start point of the line segment.
     */
    const Point& unit_direction() const {
        return unit_direction_;
    }

private:
    Point start_;
    Point end_;
    Point direction_;
    Point unit_direction_;
    double line_a_ = 0.0;
    double line_b_ = 0.0;
    double line_c_ = 0.0;
    double heading_ = 0.0;
    double length_ = 0.0;

};