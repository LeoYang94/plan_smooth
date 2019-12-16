/***************************************************************************
 *
 * Copyright (c) 2019 cubot.cumt, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file casadi_smoother.cc
 **/

#include "casadi_smoother.h"

bool CasadiSmoother::EvalF(const OptimizerParam &param, const int32_t ne,
                           const casadi::SX &x, casadi::SX *const f) const {
  casadi::SX sum_dist = 0.0;
  for (int32_t i = 0; i < ne; i++) {
    casadi::SX dx = param.states[i].x - x(i + 1);
    casadi::SX dy = param.states[i].y - x(ne + i + 1);
    sum_dist += pow(dx, 2) + pow(dy, 2);
  }
  casadi::SX sum_e = 0;
  for (int32_t i = 2; i <= ne; i++) {
    casadi::SX dphy = x(3 * ne + i) - x(3 * ne + i - 1);
    sum_e += pow(dphy, 2);
  }
  // the evaluate object
  *f = (1.0 / param.smooth_weight) * sum_dist + sum_e;
  return true;
}

bool CasadiSmoother::EvalConstraints(const OptimizerParam &param,
                                     const int32_t ne, const casadi::SX &x,
                                     std::vector<casadi::SX> *const fg) const {
  fg->resize(4 * ne + 1);
  fg->at(0) = 0.0;
  int32_t counter = 0;
  // xy constrain : sin(theta) * dx = cos(theta) * dy
  for (int32_t i = 1; i < ne; ++i) {
    fg->at(++counter) = (x(i + 1) - x(i)) * sin(x(2 * ne + i)) -
                        (x(ne + i + 1) - x(ne + i)) * cos(x(2 * ne + i));
  }
  //    for (int32_t i = 1; i < ne; ++i) {
  //      casadi::SX dx = (x(i + 1) - x(i));
  //      casadi::SX dy = (x(ne + i + 1) - x(ne + i));
  //      casadi::SX theta = (x(2 * ne + i));
  //      casadi::SX vx =
  //          (x(4 * ne + 1) + x(5 * ne + 1) + x(6 * ne + 1) + x(7 * ne + 1)) /
  //          4;
  //      casadi::SX vy =
  //          (-x(4 * ne + 1) + x(5 * ne + 1) - x(6 * ne + 1) + x(7 * ne + 1)) /
  //          4;
  //      fg->at(++counter) = dx * (vx * sin(theta) + vy * cos(theta)) -
  //                          dy * (vx * cos(theta) - vy * sin(theta));
  //    }
  fg->at(++counter) = 0.0;
  // vertical line constrain: the state must on the vertical line
  fg->at(++counter) = 0.0;
  for (int32_t i = 2; i < ne; ++i) {
    casadi::SX a = param.line_param[i - 1][0];
    casadi::SX b = param.line_param[i - 1][1];
    casadi::SX c = param.line_param[i - 1][2];
    fg->at(++counter) = (a * x(i) + b * x(ne + i) + c);
  }
  fg->at(++counter) = 0.0;
  // dtheta constrain: dtheta * wheel_base = tan(phy) * ds
  //对全向麦克纳姆轮机器人来说，怎样才是合理的约束？？
  //
  for (int32_t i = 1; i < ne; ++i) {
    casadi::SX dx = (x(i + 1) - x(i));
    casadi::SX dy = (x(ne + i + 1) - x(ne + i));
    fg->at(++counter) = sqrt(pow(dx, 2) + pow(dy, 2)) * tan(x(3 * ne + i)) -
                        param.wheelbase * (x(2 * ne + i + 1) - x(2 * ne + i));
  }
  //  for (int32_t i = 1; i < ne; ++i) {
  //    casadi::SX dx = (x(i + 1) - x(i));
  //    casadi::SX dy = (x(ne + i + 1) - x(ne + i));
  //    casadi::SX theta = (x(2 * ne + i));
  //    casadi::SX vx =
  //        (x(4 * ne + 1) + x(5 * ne + 1) + x(6 * ne + 1) + x(7 * ne + 1)) / 4;
  //    casadi::SX vy =
  //        (-x(4 * ne + 1) + x(5 * ne + 1) - x(6 * ne + 1) + x(7 * ne + 1)) /
  //        4;
  //    casadi::SX w =
  //        (-x(4 * ne + 1) + x(5 * ne + 1) + x(6 * ne + 1) - x(7 * ne + 1)) /
  //        (4 * (0.17 + 0.20));
  //    fg->at(++counter) = (x(2 * ne + i + 1) - x(2 * ne + i)) -
  //                        w * dx / (vx * cos(theta) - vy * sin(theta));
  //  }
  fg->at(++counter) = 0.0;
  // the next state at the front of current state
  for (int32_t i = 1; i < ne - 1; i++) {
    casadi::SX dx_0 = x(i + 1) - x(i);
    casadi::SX dy_0 = x(ne + i + 1) - x(ne + i);
    casadi::SX dx_1 = x(i + 2) - x(i + 1);
    casadi::SX dy_1 = x(ne + i + 2) - x(ne + i + 1);
    casadi::SX dot = dx_0 * dx_1 + dy_0 * dy_1;
    fg->at(++counter) = dot;
  }
  fg->at(++counter) = 1;
  fg->at(++counter) = 1;
  return true;
}

casadi::Dict CasadiSmoother::AddOptions() const {
  casadi::Dict opts;
  opts["expand"] = false;
  opts["ipopt.print_level"] = 0;
  opts["verbose"] = false;
  opts["print_time"] = false;
  opts["ipopt.linear_solver"] = "mumps";
  return opts;
}

bool CasadiSmoother::GetBoundsInfo(const OptimizerParam &param,
                                   const int32_t ne, const int32_t nx,
                                   const int32_t ng,
                                   std::vector<double> *const x_l,
                                   std::vector<double> *const x_u,
                                   std::vector<double> *const g_l,
                                   std::vector<double> *const g_u) const {
  x_l->resize(nx);
  x_u->resize(nx);
  g_l->resize(ng);
  g_u->resize(ng);
  x_l->at(0) = -1e20;
  x_u->at(0) = 1e20;
  // set x,y,theat low up bound
  for (int32_t i = 1; i <= 3 * ne; i++) {
    x_l->at(i) = -1e20;
    x_u->at(i) = 1e20;
  }
  //  // the phy low and bound
  for (int32_t i = 3 * ne + 1; i <= 4 * ne; i++) {
    x_l->at(i) = -param.bound_phy;
    x_u->at(i) = param.bound_phy;
  }
  for (int32_t i = 4 * ne + 1; i <= 8 * ne; i++) {
    x_l->at(i) = -param.bound_vx;
    x_u->at(i) = param.bound_vx;
  }
  // fix the start points and the end point
  for (int32_t i = 0; i < param.fixed_point_num; ++i) {
    x_l->at(i + 1) = param.states[i].x;
    x_u->at(i + 1) = param.states[i].x;
    x_l->at(ne + i + 1) = param.states[i].y;
    x_u->at(ne + i + 1) = param.states[i].y;
  }
  x_l->at(ne) = param.states.back().x;
  x_u->at(ne) = param.states.back().x;
  x_l->at(2 * ne) = param.states.back().y;
  x_u->at(2 * ne) = param.states.back().y;

  // theta
  x_l->at(2 * ne + 1) = param.states.front().theta;
  x_u->at(2 * ne + 1) = param.states.front().theta;
  x_l->at(3 * ne) = param.states.back().theta;
  x_u->at(3 * ne) = param.states.back().theta;
  // end phy
  x_l->at(4 * ne) = 0;
  x_u->at(4 * ne) = 0;

  x_l->at(5 * ne) = 0;
  x_u->at(5 * ne) = 0;

  x_l->at(6 * ne) = 0;
  x_u->at(6 * ne) = 0;

  x_l->at(7 * ne) = 0;
  x_u->at(7 * ne) = 0;

  x_l->at(8 * ne) = 0;
  x_u->at(8 * ne) = 0;

  for (int32_t i = 0; i <= 3 * ne; i++) {
    g_l->at(i) = 0;
    g_u->at(i) = 0;
  }
  for (int32_t i = 3 * ne + 1; i < ng; i++) {
    g_l->at(i) = 0;
    g_u->at(i) = 1e20;
  }
  return true;
}

bool CasadiSmoother::GetStartingPoint(const OptimizerParam &param,
                                      const int32_t ne,
                                      std::vector<double> *const xi) const {
  xi->resize(8 * param.states.size() + 1);
  for (int32_t i = 1; i <= ne; ++i) {
    xi->at(i) = param.states[i - 1].x;
    xi->at(ne + i) = param.states[i - 1].y;
    xi->at(2 * ne + i) = param.states[i - 1].theta;
    xi->at(3 * ne + i) = param.states[i - 1].phy;
    xi->at(4 * ne + i) = param.states[i - 1].v1;
    xi->at(5 * ne + i) = param.states[i - 1].v2;
    xi->at(6 * ne + i) = param.states[i - 1].v3;
    xi->at(7 * ne + i) = param.states[i - 1].v4;
  }

  double path_length = 0.0;
  for (int32_t i = 2; i <= ne; i++) {
    path_length += hypot((xi->at(i) - xi->at(i - 1)),
                         (xi->at(ne + i) - xi->at(ne + i - 1)));
  }
  xi->at(0) = path_length;
  return true;
}

bool CasadiSmoother::Run(const OptimizerParam &dealed_op,
                         std::vector<double> *const smoother_results) const {
  const auto &ne =
      static_cast<int32_t>(static_cast<int32_t>(dealed_op.states.size()));
  int32_t nx = 8 * ne + 1;
  int32_t ng = 4 * ne + 1;
  casadi::SX x = casadi::SX::sym("x", nx);
  casadi::SX f;
  EvalF(dealed_op, ne, x, &f);
  std::vector<casadi::SX> fg;
  EvalConstraints(dealed_op, ne, x, &fg);
  casadi::SX g = vertcat(fg);
  // add options
  casadi::Dict opts = AddOptions();
  casadi::SXDict nlp = {{"x", x}, {"f", f}, {"g", g}};
  casadi::Function solver = nlpsol("solver", "ipopt", nlp, opts);
  // add bound info and start point
  std::vector<double> g_l, g_u, x_l, x_u, x_init;
  GetBoundsInfo(dealed_op, ne, nx, ng, &x_l, &x_u, &g_l, &g_u);
  GetStartingPoint(dealed_op, ne, &x_init);
  // set arg
  std::map<std::string, casadi::DM> arg, res;
  arg["lbx"] = x_l;
  arg["ubx"] = x_u;
  arg["lbg"] = g_l;
  arg["ubg"] = g_u;
  arg["x0"] = x_init;
  // Solve the NLP
  res = solver(arg);
  casadi::DM result;
  result = res.at("x");
  // get result
  GetPathPoints(result, nx, smoother_results);
  return true;
}

void CasadiSmoother::GetPathPoints(
    const casadi::DM result, int32_t nx,
    std::vector<double> *const results_ptr) const {
  for (int32_t i = 0; i < nx; ++i) {
    results_ptr->emplace_back(result(i, 0).scalar());
  }
}
