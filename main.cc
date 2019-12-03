/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file main.cc
 **/
#include <fstream>
#include <string>

#include "common/gnu_draw.h"
#include "common/point.h"
#include "plan_path_smoother.h"

std::vector<std::pair<double, double>>
PolyLineToPlotLine(const std::vector<Point> &points) {
  std::vector<std::pair<double, double>> plot_line;
  for (const auto &point : points) {
    plot_line.emplace_back(point.x_, point.y_);
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
int main() {
  std::vector<Point> raw_path;
  std::cout << "Start read data..." << std::endl;
  if (!ReadDataFromFile("/home/leo/code/plan_smooth/data/data1x.txt",
                        "/home/leo/code/plan_smooth/data/data1y.txt",
                        &raw_path)) {
    std::cout << "wrong data!" << std::endl;
    return -1;
  }
  std::cout << "Start draw raw data..." << std::endl;
  // PrintDistance(raw_path);
  //
  // draw plot
  // auto plot_line_origin = PolyLineToPlotLine(raw_path);
  //  Gnuplot gp;
  //  GnuDraw::GnuPlotInit(&gp);
  //  GnuDraw::Plot(&gp);
  //  GnuDraw::DrawOrietedLine(plot_line_origin, 1, BLUE, &gp);
  //  // GnuDraw::DrawOrietedLine(plot_line, 1, MAGENTA, &gp);
  //  GnuDraw::EndPlot(&gp);
  //  sleep(10000);
  PlanPathSmoother smoother(true);
  std::vector<Point> smoothed_path;
  smoother.Smooth(raw_path, &smoothed_path);
  return 0;
}
