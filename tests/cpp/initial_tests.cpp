// Acceptance tests: the initial state. Redundant constraints (an overconstrained mechanism),
// the assembly of positions that violate the constraints, the static equilibrium.
// Each test prints "RESULT <test> <metric> <value> tol <tol>" (fdtest::report).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "test_support.hpp"

using namespace fdtest;

namespace {

// Channel `name` (first component) of the outputs y of sys.
double channel(fd::System& sys, const Vec& y, const std::string& name, int k = 0) {
  const int i = sys.output_index(name);
  if (i < 0) throw std::runtime_error("no output " + name);
  return y[i + k];
}

// 1. Parallelogram of four revolute joints (examples/parallelogram.yaml): a planar loop is
// overconstrained in space by 3 constraints. They are found and deactivated; the motion is that
// of the parallelogram: the coupler translates (its orientation stays), both cranks turn alike,
// the energy is kept as for the pendulum at alpha = -0.01.
void test_redundant() {
  const char* T = "redundant";
  fd::Model m = example("parallelogram");
  m.solver.end_time = 2.0;
  fd::System sys(m);
  fd::HhtIntegrator hht(sys, m.solver);
  hht.initialize();
  const fd::InitialReport& r = hht.initial_report();
  report(T, "redundant constraints found - 3", std::abs(double(r.redundant.size()) - 3.0), 0.0);
  for (int c : r.redundant) std::printf("INFO %s deactivated: %s\n", T, sys.constraint_name(c).c_str());
  report(T, "assembly corrections (consistent model)", r.assembly_iterations, 0.0);
  Vec y, th0(4);
  sys.outputs(y);
  for (int k = 0; k < 4; ++k) th0[k] = channel(sys, y, "coupler.theta", k);
  auto energy = [&] {
    double E = 0.0;
    for (const char* b : {"crank_a", "crank_b", "coupler"}) {
      const std::string n(b);
      const double mass = n == "coupler" ? 2.0 : 1.0;
      E += channel(sys, y, n + ".T") + mass * 9.81 * channel(sys, y, n + ".R", 2);
    }
    return E;
  };
  const double E0 = energy();
  double dE = 0.0, T_max = 0.0, dth = 0.0, dz = 0.0, lam = 0.0;
  const long n = std::lround(m.solver.end_time / m.solver.step);
  for (long k = 0; k < n; ++k) {
    hht.step();
    sys.outputs(y);
    dE = std::max(dE, std::abs(energy() - E0));
    T_max = std::max(T_max, channel(sys, y, "coupler.T"));
    for (int j = 0; j < 4; ++j) dth = std::max(dth, std::abs(channel(sys, y, "coupler.theta", j) - th0[j]));
    dz = std::max(dz, std::abs(channel(sys, y, "crank_a.R", 2) - channel(sys, y, "crank_b.R", 2)));
    for (int c : r.redundant) lam = std::max(lam, std::abs(sys.lam[c]));
  }
  report(T, "coupler orientation change", dth, 1e-10);
  report(T, "|z_a - z_b| of the cranks", dz, 1e-10);
  report(T, "multipliers of the deactivated constraints", lam, 1e-20);
  report(T, "max |E - E0| / T_max", dE / T_max, 1e-4);
}

// 2. Assembly: the pendulum's rod moved off its hinge (2 cm, 1 cm, 3 cm) is put back onto it by
// the smallest correction in the mass metric; the hinge point is then at the origin.
void test_assembly() {
  const char* T = "assembly";
  fd::Model m = example("pendulum");
  const double off[3] = {0.02, -0.01, 0.03};
  for (int i = 0; i < 3; ++i) m.elements[0].q[i] += off[i];
  fd::System sys(m);
  fd::HhtIntegrator hht(sys, m.solver);
  hht.initialize();
  const fd::InitialReport& r = hht.initial_report();
  info(T, "Newton corrections", r.assembly_iterations);
  report(T, "violation before - |offset| (largest component)", std::abs(r.violation - 0.03), 1e-12);
  report(T, "no assembly needed (0) / needed (1): 1 - needed", r.assembly_iterations > 0 ? 0.0 : 1.0, 0.0);
  Vec y;
  sys.outputs(y);
  // hinge point u = (-0.5, 0, 0) of the rod, global
  double A[3][3], P[3];
  rotation(&y[sys.output_index("rod.theta")], A);
  const int iR = sys.output_index("rod.R");
  for (int i = 0; i < 3; ++i) P[i] = y[iR + i] - 0.5 * A[i][0];
  report(T, "|hinge point| after the assembly", std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]), 1e-10);
  // a revolute joint about y lets the rod turn about y: the translation is undone, the x offset
  // partly by a rotation; the run goes on from there
  for (int k = 0; k < 100; ++k) hht.step();
  info(T, "time after 100 steps", hht.time());
}

// 3. Static equilibrium (SolverSettings::statics): the oscillator hangs at -m g / k, the pendulum
// straight down, the parallelogram with its cranks down; all stay at rest. A free body under
// gravity has no equilibrium: an error.
void test_statics() {
  const char* T = "statics";
  struct Case {
    const char* model;
    const char* channel;
    int component;
    double value;
  };
  for (const Case& c : {Case{"oscillator", "mass.R", 2, -2.0 * 9.81 / 200.0}, Case{"pendulum", "rod.R", 2, -0.5},
                        Case{"pendulum", "rod.R", 0, 0.0}, Case{"parallelogram", "coupler.R", 2, -1.0},
                        Case{"parallelogram", "crank_a.R", 0, 0.0}}) {
    fd::Model m = example(c.model);
    m.solver.statics = true;
    m.solver.end_time = 1.0;
    fd::System sys(m);
    fd::HhtIntegrator hht(sys, m.solver);
    hht.initialize();
    Vec y;
    sys.outputs(y);
    const std::string what = std::string(c.model) + " " + c.channel + "[" + std::to_string(c.component) + "]";
    report(T, what + " in equilibrium", std::abs(channel(sys, y, c.channel, c.component) - c.value), 1e-10);
    info(T, what + ": corrections", hht.initial_report().static_iterations);
    double drift = 0.0;
    const long n = std::lround(m.solver.end_time / m.solver.step);
    for (long k = 0; k < n; ++k) {
      hht.step();
      sys.outputs(y);
      drift = std::max(drift, std::abs(channel(sys, y, c.channel, c.component) - c.value));
    }
    report(T, what + " after 1 s at rest", drift, 1e-9);
  }
  fd::Model fall = body_model({1, 2, 3}, {1, 0, 0, 0}, {0, 0, 0}, -0.1, 1e-3, 1.0);
  fall.elements[0].p[6] = -9.81;
  fall.solver.statics = true;
  fd::System sys(fall);
  fd::HhtIntegrator hht(sys, fall.solver);
  bool threw = false;
  try {
    hht.initialize();
  } catch (const std::exception& e) {
    threw = true;
    std::printf("INFO %s free body under gravity: %s\n", T, e.what());
  }
  report(T, "free body under gravity: no error (1)", threw ? 0.0 : 1.0, 0.0);
}

}  // namespace

void add_initial_tests(Tests& tests) {
  tests["redundant"] = test_redundant;
  tests["assembly"] = test_assembly;
  tests["statics"] = test_statics;
}
