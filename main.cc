/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file main.cc
 **/

#include <fstream>
#include <memory>
#include <string>

#include "bezier/Bezier.hpp"
#include "common/gnu_draw.h"
#include "common/point.h"
#include "plan_path_smoother.h"

using pair = std::vector<std::pair<double, double>>;
using Bezier = robotics::Bezier<double, 2>;

std::vector<std::pair<double, double>>
PolyLineToPlotLine(const std::vector<Point> &points) {
  std::vector<std::pair<double, double>> plot_line;
  for (const auto &point : points) {
    plot_line.emplace_back(point.x_, point.y_);
  }
  return plot_line;
}

std::vector<std::pair<double, double>>
PolyLineToPlotLine(const Bezier::VecPointType &points) {
  std::vector<std::pair<double, double>> plot_line;
  for (const auto &point : points) {
    plot_line.emplace_back(point[0], point[1]);
  }
  return plot_line;
}

bool ReadDataFromFile(const std::string &x_file_name,
                      const std::string &y_file_name,
                      std::vector<Point> *const raw_path) {
  std::ifstream x_file(x_file_name);
  std::ifstream y_file(y_file_name);
  std::string x_str, y_str;
  std::vector<double> x;
  std::vector<double> y;
  while (x_file >> x_str) {
    x.emplace_back(std::stod(x_str));
  }
  while (y_file >> y_str) {
    y.emplace_back(std::stod(y_str));
  }
  x_file.close();
  y_file.close();
  if (x.size() != y.size()) {
    std::cerr << "x y size error" << std::endl;
    return false;
  }
  for (uint32_t i = 0; i < x.size(); ++i) {
    raw_path->emplace_back(Point(x[i], y[i]));
  }
  return true;
}

void PrintDistance(std::vector<Point> path) {
  for (uint32_t i = 0; i < path.size() - 1; i++) {
    double distance = std::hypot(std::abs(path[i + 1].x() - path[i].x()),
                                 std::abs(path[i + 1].y() - path[i].y()));
    std::cout << "distance: " << distance << std::endl;
  }
}

void PrintPlanResult(const std::vector<std::pair<double, double>> &plan,
                     const int32_t counter) {
  std::ofstream outfile_x("/home/leo/code/plan_smooth/data/data_out" +
                              std::to_string(counter) + "x.txt",
                          std::ios::trunc);
  std::ofstream outfile_y("/home/leo/code/plan_smooth/data/data_out" +
                              std::to_string(counter) + "y.txt",
                          std::ios::trunc);
  for (uint32_t i = 0; i < plan.size(); ++i) {
    std::cout << "Plan result:"
              << ": (" << plan[i].first << "," << plan[i].second << ")"
              << std::endl;
    outfile_x << plan[i].first << "\n";
    outfile_y << plan[i].second << "\n";
  }
  outfile_x.close();
  outfile_y.close();
  std::cout << "write file!" << std::endl;
}

bool MakeBezierCurve(const pair &raw_path, pair *const res) {
  std::vector<pair> SegmentSix;
  for (uint32_t i = 0; i < 501;) {
    pair p;
    int begin_index = i;
    if (i != 0) {
      p.emplace_back(raw_path[begin_index - 1].first,
                     raw_path[begin_index - 1].second);
    } else {
      p.emplace_back(raw_path[0].first, raw_path[0].second);
    }
    for (size_t k = 0; k < 5; ++k) {
      p.emplace_back(raw_path[begin_index].first, raw_path[begin_index].second);
      begin_index++;
    }
    SegmentSix.emplace_back(p);
    i += 5;
  }

  for (uint32_t i = 0; i < SegmentSix.size(); ++i) {
    Bezier::PointType p1(SegmentSix[i][0].first, SegmentSix[i][0].second);
    Bezier::PointType p2(SegmentSix[i][1].first, SegmentSix[i][1].second);
    Bezier::PointType p3(SegmentSix[i][2].first, SegmentSix[i][2].second);
    Bezier::PointType p4(SegmentSix[i][3].first, SegmentSix[i][3].second);
    Bezier::PointType p5(SegmentSix[i][4].first, SegmentSix[i][4].second);
    Bezier::PointType p6(SegmentSix[i][5].first, SegmentSix[i][5].second);
    Bezier::VecPointType pV{p1, p2, p3, p4, p5, p6};
    Bezier::Ptr bezier = std::make_shared<Bezier>(5, pV);
    auto coeffV = bezier->binomialCoeffs();

    const Bezier::Tangent tan = bezier->tangent(0.7);
    const Bezier::Normal nor = bezier->normal(0.7);
    double curvature = bezier->curvature(0.7);
    std::cout << "tangent vector: \n" << tan << "\n";
    std::cout << "normal vector: \n" << nor << "\n";
    std::cout << "dot product: " << tan.dot(nor) << "\n";
    std::cout << "curvature: " << curvature << "\n";
    std::cout << "Trajectory: \n";
    auto current_control_points = PolyLineToPlotLine(bezier->controlPoints());
    Bezier::VecPointType trajectory = bezier->trajectory(20);
    auto plot_line_smoothed = PolyLineToPlotLine(trajectory);
    if (i != 0) {
      plot_line_smoothed.pop_back();
    }
    res->insert(res->end(), plot_line_smoothed.begin(),
                plot_line_smoothed.end());
  }
  return true;
}

int main() {
  std::vector<Point> raw_path;
  std::vector<Point> smoothed_path0;
  std::vector<Point> smoothed_path1;
  std::cout << "Start read data..." << std::endl;
  if (!ReadDataFromFile("/home/leo/code/plan_smooth/data/data1x.txt",
                        "/home/leo/code/plan_smooth/data/data1y.txt",
                        &raw_path)) {
    std::cout << "wrong data!" << std::endl;
    return -1;
  }

  // PrintDistance(raw_path);
  PlanPathSmoother smoother(true);
  std::vector<Point> smoothed_path;
  smoother.Smooth(raw_path, &smoothed_path);
  // draw plot
  auto plot_line_origin = PolyLineToPlotLine(raw_path);
  auto plot_line_smoothed = PolyLineToPlotLine(smoothed_path);

  std::cout << "Start bezier..." << std::endl;
  pair res;
  // pair control_points;
  MakeBezierCurve(plot_line_origin, &res);
  std::cout << "Start draw raw data..." << std::endl;
  Gnuplot gp;
  GnuDraw::GnuPlotInit(&gp);
  GnuDraw::Plot(&gp);
  GnuDraw::DrawOrietedLine(plot_line_origin, 1, BLUE, &gp, "origin path");
  GnuDraw::DrawOrietedLine(plot_line_smoothed, 1, MAGENTA, &gp, "ipopt path");
  GnuDraw::DrawOrietedLine(res, 1, GREEN, &gp);
  GnuDraw::EndPlot(&gp);
  sleep(10000);
  return 0;
}
