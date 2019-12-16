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
  //  if (!ReadDataFromFile("/home/leo/code/plan_smooth/data/data_out0x.txt",
  //                        "/home/leo/code/plan_smooth/data/data_out0y.txt",
  //                        &smoothed_path0)) {
  //    std::cout << "wrong data 0 !" << std::endl;
  //  }
  //  if (!ReadDataFromFile("/home/leo/code/plan_smooth/data/data_out1x.txt",
  //                        "/home/leo/code/plan_smooth/data/data_out1y.txt",
  //                        &smoothed_path1)) {
  //    std::cout << "wrong data 1 !" << std::endl;
  //  }
  std::cout << "Start draw raw data..." << std::endl;
  // PrintDistance(raw_path);
  PlanPathSmoother smoother(true);
  std::vector<Point> smoothed_path;
  smoother.Smooth(raw_path, &smoothed_path);
  // draw plot
  auto plot_line_origin = PolyLineToPlotLine(raw_path);
  auto plot_line_smoothed = PolyLineToPlotLine(smoothed_path);
  //auto plot_line_smoothed1 = PolyLineToPlotLine(smoothed_path1);
  // PrintPlanResult(plot_line_smoothed, 1);
  Gnuplot gp;
  GnuDraw::GnuPlotInit(&gp);
  GnuDraw::Plot(&gp);
  GnuDraw::DrawOrietedLine(plot_line_origin, 1, BLUE, &gp, "origin path" );
  GnuDraw::DrawOrietedLine(plot_line_smoothed, 1, MAGENTA, &gp,"ipopt path");
  // GnuDraw::DrawOrietedLine(plot_line_smoothed1, 1, LIGHTBLUE, &gp);
  GnuDraw::EndPlot(&gp);
  sleep(10000);
  return 0;
}
