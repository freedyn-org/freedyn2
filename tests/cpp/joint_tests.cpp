// Acceptance tests: joints (pendulum, double pendulum).
// Each test prints "RESULT <test> <metric> <value> tol <tol>" (fdtest::report).

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "test_support.hpp"

using namespace fdtest;

namespace {

// 8. Joints. Compound pendulum: a rod (m = 1, L = 1, COM at the middle) hinged at one end,
// released from the horizontal. Exact period 4 sqrt(I_p / (m g l)) K(sin(pi/4)) with
// I_p = J2 + m l^2, l = L/2 (amplitude pi/2 about the bottom).
struct Pivot {
  double energy = 0, pivot = 0, planar = 0, period = 0;
};

// Point of a body at body-frame offset u, global.
void point(const Vec& y, int iR, int it, const double u[3], double P[3]) {
  double A[3][3];
  rotation(&y[it], A);
  for (int i = 0; i < 3; ++i) P[i] = y[iR + i] + A[i][0] * u[0] + A[i][1] * u[1] + A[i][2] * u[2];
}

Pivot pendulum(double alpha, double h) {
  fd::Model m = example("pendulum");
  m.solver.alpha = alpha;
  m.solver.step = h;
  const double mass = m.elements[0].p[0], g = 9.81;
  int iR = -1, it = -1, iT = -1;
  double E0 = 0, T_max = 0, prev = 1, t_prev = 0, t_first = -1, t_last = -1;
  int crossings = 0;
  Pivot r;
  const double u[3] = {-0.5, 0, 0};
  simulate(m, [&](double t, const Vec& y, fd::System& sys) {
    if (iR < 0) {
      iR = sys.output_index("rod.R");
      it = sys.output_index("rod.theta");
      iT = sys.output_index("rod.T");
    }
    const double E = y[iT] + mass * g * y[iR + 2];
    if (t == 0) E0 = E;
    T_max = std::max(T_max, y[iT]);
    r.energy = std::max(r.energy, std::abs(E - E0));
    double P[3];
    point(y, iR, it, u, P);
    r.pivot = std::max(r.pivot, std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]));
    r.planar = std::max(r.planar, std::abs(y[iR + 1]));
    // Rod direction x = cos(phi): zero crossings at the bottom, where phi'' = 0 (linear
    // interpolation exact to O(h^3)); two per period.
    const double c = y[iR] / 0.5;
    if (t > 0 && (prev > 0) != (c > 0)) {
      const double tc = t_prev + (t - t_prev) * prev / (prev - c);
      if (t_first < 0) t_first = tc;
      t_last = tc;
      ++crossings;
    }
    prev = c, t_prev = t;
  });
  r.energy /= T_max;
  const double Ip = m.elements[0].p[2] + mass * 0.25, K = 1.8540746773013719;  // K(1/sqrt(2))
  const double T_ex = 4 * std::sqrt(Ip / (mass * g * 0.5)) * K;
  r.period = std::abs(2 * (t_last - t_first) / (crossings - 1) / T_ex - 1);
  return r;
}

void test_pendulum() {
  const char* T = "pendulum";
  // alpha = 0 (trapezoidal rule) is unstable for index-3 constraints (Cardona and Geradin 1989):
  // a (-1)^n mode of lambda and the accelerations grows until the run breaks down (legacy
  // FreeDyn crashes on the same model after 2.75 s). A little numerical damping removes it.
  const Pivot a = pendulum(-0.01, 1e-3), b = pendulum(-0.1, 1e-3), b2 = pendulum(-0.1, 5e-4);
  try {  // the (-1)^n mode grows from round-off: the run may break down before its end
    info(T, "alpha=0: max |E - E0| / T_max (unstable)", pendulum(0.0, 1e-3).energy);
  } catch (const std::exception& e) {
    std::printf("INFO %s alpha=0: run broke down (unstable): %s\n", T, e.what());
  }
  // Index-3 constraints hold at every step to the Newton tolerance.
  report(T, "alpha=-0.01: max |pivot| [m]", a.pivot, 1e-10);
  report(T, "alpha=-0.01: max |R_y| [m]", a.planar, 1e-12);
  // Energy: O(alpha h^2) numerical damping plus O(h^2) oscillation.
  report(T, "alpha=-0.01: max |E - E0| / T_max", a.energy, 2e-5);
  report(T, "alpha=-0.01: period vs elliptic integral", a.period, 1e-5);
  report(T, "alpha=-0.1: max |pivot| [m]", b.pivot, 1e-10);
  report(T, "alpha=-0.1: max |E - E0| / T_max", b.energy, 5e-5);
  report(T, "alpha=-0.1: period vs elliptic integral", b.period, 1e-5);
  info(T, "alpha=-0.1: order of period error", std::log2(b.period / b2.period));
  report(T, "alpha=-0.1: order of period error: |2 - p|", std::abs(2 - std::log2(b.period / b2.period)), 0.2);
}

// Double pendulum (chaotic): both hinges hold, energy is conserved up to 3 s (small damping).
void test_double_pendulum() {
  const char* T = "dbl_pendulum";
  fd::Model m = example("double_pendulum");
  m.solver.alpha = -0.01;
  m.solver.end_time = 3.0;
  const double g = 9.81;
  int iR1 = -1, it1 = -1, iT1 = -1, iR2 = -1, it2 = -1, iT2 = -1;
  double E0 = 0, T_max = 0, dE = 0, hinge1 = 0, hinge2 = 0, planar = 0;
  const double u1[3] = {-0.5, 0, 0}, u12[3] = {0.5, 0, 0};
  simulate(m, [&](double t, const Vec& y, fd::System& sys) {
    if (iR1 < 0) {
      iR1 = sys.output_index("rod1.R"), it1 = sys.output_index("rod1.theta"), iT1 = sys.output_index("rod1.T");
      iR2 = sys.output_index("rod2.R"), it2 = sys.output_index("rod2.theta"), iT2 = sys.output_index("rod2.T");
    }
    const double E = y[iT1] + y[iT2] + g * (y[iR1 + 2] + y[iR2 + 2]);  // m = 1
    if (t == 0) E0 = E;
    T_max = std::max(T_max, y[iT1] + y[iT2]);
    dE = std::max(dE, std::abs(E - E0));
    double P[3], Q1[3], Q2[3];
    point(y, iR1, it1, u1, P);
    point(y, iR1, it1, u12, Q1);
    point(y, iR2, it2, u1, Q2);
    hinge1 = std::max(hinge1, std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]));
    hinge2 = std::max(hinge2, std::hypot(Q1[0] - Q2[0], Q1[1] - Q2[1], Q1[2] - Q2[2]));
    planar = std::max({planar, std::abs(y[iR1 + 1]), std::abs(y[iR2 + 1])});
  });
  report(T, "max |hinge 1 point| [m]", hinge1, 1e-10);
  report(T, "max |hinge 2 gap| [m]", hinge2, 1e-10);
  report(T, "max |R_y| [m]", planar, 1e-12);
  report(T, "alpha=-0.01: max |E - E0| / T_max", dE / T_max, 2e-4);
}

// Real-time mode (newton fixed): exactly N corrections in every step, the pivot order of the
// first factorization kept, and the result close to converged Newton on the chaotic double
// pendulum (1 s, before the chaos separates nearby trajectories).
void test_newton_fixed() {
  const char* T = "newton_fixed";
  auto run = [](const std::string& newton, int n, long* pivots, int* iter_min, int* iter_max) {
    fd::Model m = example("double_pendulum");
    m.solver.alpha = -0.1;
    m.solver.end_time = 1.0;
    m.solver.newton = newton;
    m.solver.newton_iterations = n;
    fd::System sys(m);
    fd::HhtIntegrator hht(sys, m.solver, fd::default_linear_solver());
    hht.initialize();
    *iter_min = 1 << 30, *iter_max = 0;
    for (long k = 0; k < std::lround(m.solver.end_time / m.solver.step); ++k) {
      const int it = hht.step().iterations;
      *iter_min = std::min(*iter_min, it), *iter_max = std::max(*iter_max, it);
    }
    *pivots = hht.full_factorizations();
    return sys.q;
  };
  long pivots;
  int lo, hi;
  const Vec conv = run("velocity", 0, &pivots, &lo, &hi);
  for (int n : {1, 2}) {
    const Vec q = run("fixed", n, &pivots, &lo, &hi);
    double d = 0;
    for (size_t i = 0; i < q.size(); ++i) d = std::max(d, std::abs(q[i] - conv[i]));
    const std::string N = "N=" + std::to_string(n) + ": ";
    report(T, N + "Newton iterations per step - N", std::max(std::abs(lo - n), std::abs(hi - n)), 0.0);
    if (std::string(fd::default_linear_solver()) == "klu")
      report(T, N + "new pivot orders after the first", static_cast<double>(pivots - 1), 0.0);
    report(T, N + "max |q - q_converged| at 1 s", d, n == 1 ? 1e-5 : 1e-9);
  }
}

}  // namespace

void add_joint_tests(Tests& tests) {
  tests["newton_fixed"] = test_newton_fixed;
  tests["pendulum"] = test_pendulum;
  tests["double_pendulum"] = test_double_pendulum;
}
