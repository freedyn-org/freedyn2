// Acceptance tests: generalized-alpha (SolverSettings::rho_inf) and the PI step size control.
// Each test prints "RESULT <test> <metric> <value> tol <tol>" (fdtest::report).

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "test_support.hpp"

using namespace fdtest;

namespace {

fd::ElementSpec& element(fd::Model& m, const std::string& name) {
  for (auto& e : m.elements)
    if (e.name == name) return e;
  throw std::runtime_error("no element " + name);
}

// 1. Spectral radius, measured: a block on a stiff undamped bushing along z (the bushing force is
// linear in the displacement, so the steps are linear), no gravity, Omega = omega h. Its
// positions z_n follow a linear recurrence of the integrator's state (q, v, a, qdd_n): fitted to
// the run (least squares), the largest root of the recurrence is the spectral radius. It must be
// that of the amplification matrix of the textbook method (Chung and Hulbert 1993, written
// independently below; for linear systems the Arnold-Bruls form gives the same solution), and
// tends to rho_inf for Omega -> infinity. Omega = 10 keeps the roots apart (at infinity they
// coincide, and a fit cannot resolve a multiple root).
double measured_rho(double rho_inf, double alpha, double Omega) {
  fd::Model m = example("bushing_mount");
  fd::ElementSpec& body = element(m, "block");
  body.q[2] = 0.01;
  std::fill(body.u.begin(), body.u.end(), 0.0);
  fd::ElementSpec& mount = element(m, "mount");
  const double h = 0.1, omega = Omega / h;
  mount.p[26] = body.p[0] * omega * omega;         // kt_z (p: u1, K1, r2, K2, kt, dt, kr, dr)
  for (int i = 27; i < 30; ++i) mount.p[i] = 0.0;  // dt
  for (int i = 33; i < 36; ++i) mount.p[i] = 0.0;  // dr
  m.solver.step = h;
  m.solver.end_time = 60 * h;
  m.solver.rho_inf = rho_inf;
  m.solver.alpha = alpha;
  m.solver.newton_tol = 1e-15;  // linear steps solved to round-off (the fit sees the decay only)
  std::vector<double> z;
  simulate(m, [&](double, const Vec& y, fd::System& sys) { z.push_back(y[sys.output_index("block.R") + 2]); });
  const double z0 = std::abs(z[0]);
  size_t n = 0;  // samples until they decay to round-off of the start
  while (n < z.size() && std::abs(z[n]) > 1e-8 * z0) ++n;
  // the order of the recurrence: the largest (at most 4) with a design matrix of full rank
  // (HHT has 3: its pseudo-acceleration is a combination of qdd_n and qdd_n-1)
  for (int order = 4; order >= 2; --order) {
    const int rows = static_cast<int>(n) - order;
    if (rows < order + 2) throw std::runtime_error("measured_rho: too few samples");
    Eigen::MatrixXd A(rows, order);
    Eigen::VectorXd b(rows);
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < order; ++j) A(i, j) = z[i + j];
      b(i) = z[i + order];
    }
    const Eigen::VectorXd sv = Eigen::JacobiSVD<Eigen::MatrixXd>(A).singularValues();
    if (sv(order - 1) < 1e-9 * sv(0)) continue;
    const Eigen::VectorXd c = A.colPivHouseholderQr().solve(b);  // z_k+order = sum c_j z_k+j
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(order, order);       // companion matrix
    for (int j = 0; j < order; ++j) C(order - 1, j) = c(j);
    for (int j = 0; j + 1 < order; ++j) C(j, j + 1) = 1.0;
    return Eigen::EigenSolver<Eigen::MatrixXd>(C).eigenvalues().cwiseAbs().maxCoeff();
  }
  throw std::runtime_error("measured_rho: no recurrence");
}

// Spectral radius of the generalized-alpha method (alpha_m, alpha_f, beta, gamma) on
// d'' + w^2 d = 0 at Omega = w h: L x_n+1 = R x_n, x = (d, h v, h^2 a).
double textbook_rho(double am, double af, double beta, double gamma, double Omega) {
  const double W = Omega * Omega;
  Eigen::Matrix3d L, R;
  L << 1, 0, -beta, 0, 1, -gamma, W * (1 - af), 0, 1 - am;
  R << 1, 1, 0.5 - beta, 0, 1, 1 - gamma, -W * af, 0, -am;
  return Eigen::EigenSolver<Eigen::Matrix3d>(L.inverse() * R).eigenvalues().cwiseAbs().maxCoeff();
}

void test_spectral_radius() {
  const char* T = "spectral_radius";
  auto check = [&](const char* name, double rho_inf, double alpha, double am, double af, double beta, double gamma) {
    char what[96];
    const double expect = textbook_rho(am, af, beta, gamma, 10.0);
    std::snprintf(what, sizeof what, "%s: |rho - textbook| at Omega = 10 (%.4f)", name, expect);
    report(T, what, std::abs(measured_rho(rho_inf, alpha, 10.0) - expect), 1e-6);
    std::snprintf(what, sizeof what, "%s: textbook rho at Omega = 1e6 - rho_inf", name);
    const double r_inf = rho_inf >= 0 ? rho_inf : (1 + alpha) / (1 - alpha);
    report(T, what, std::abs(textbook_rho(am, af, beta, gamma, 1e6) - r_inf), 1e-3);
  };
  for (double rho : {0.0, 0.3, 0.6, 0.9, 1.0}) {
    const double am = (2 * rho - 1) / (rho + 1), af = rho / (rho + 1), gamma = 0.5 + af - am;
    char name[48];
    std::snprintf(name, sizeof name, "generalized-alpha rho_inf = %.1f", rho);
    check(name, rho, -0.1, am, af, 0.25 * (gamma + 0.5) * (gamma + 0.5), gamma);
  }
  for (double alpha : {-0.1, -1.0 / 3.0}) {
    char name[48];
    std::snprintf(name, sizeof name, "HHT alpha = %.3f", alpha);
    check(name, -1.0, alpha, 0.0, -alpha, 0.25 * (1 - alpha) * (1 - alpha), 0.5 - alpha);
  }
}

// Pendulum (examples/pendulum.yaml) at t = 1 s: rod.R with step h.
Vec pendulum_at_1s(double h, double rho_inf, double alpha = -0.1) {
  fd::Model m = example("pendulum");
  m.solver.step = h;
  m.solver.end_time = 1.0;
  m.solver.rho_inf = rho_inf;
  m.solver.alpha = alpha;
  Vec R(3);
  simulate(m, [&](double, const Vec& y, fd::System& sys) {
    for (int i = 0; i < 3; ++i) R[i] = y[sys.output_index("rod.R") + i];
  });
  return R;
}

double dist(const Vec& a, const Vec& b) {
  double s = 0;
  for (size_t i = 0; i < a.size(); ++i) s += (a[i] - b[i]) * (a[i] - b[i]);
  return std::sqrt(s);
}

// 2. Generalized-alpha is second order (pendulum, against a run at h = 1e-5), and at the same
// rho_inf it damps the pendulum's motion less than HHT (its point: high frequencies damped as
// much, low ones less).
void test_generalized_alpha() {
  const char* T = "generalized_alpha";
  const double rho = 0.8;
  const Vec ref = pendulum_at_1s(1e-5, rho);
  const double e1 = dist(pendulum_at_1s(4e-3, rho), ref), e2 = dist(pendulum_at_1s(2e-3, rho), ref);
  info(T, "error at h = 4e-3", e1);
  info(T, "error at h = 2e-3", e2);
  report(T, "observed order - 2 (>= -0.1)", std::min(0.0, std::log2(e1 / e2) - 2.0 + 0.1), 0.0);

  // energy after 5 s, released from the horizontal, h = 5 ms (damping of the pendulum mode)
  auto energy_loss = [](double rho_inf, double alpha) {
    fd::Model m = example("pendulum");
    m.solver.step = 5e-3;
    m.solver.end_time = 5.0;
    m.solver.rho_inf = rho_inf;
    m.solver.alpha = alpha;
    double E0 = 0, E = 0;
    simulate(m, [&](double t, const Vec& y, fd::System& sys) {
      E = y[sys.output_index("rod.T")] + 9.81 * y[sys.output_index("rod.R") + 2];
      if (t == 0) E0 = E;
    });
    return std::abs(E - E0) / 4.905;  // relative to m g l (the swing's energy scale)
  };
  const double alpha_same = (rho - 1) / (rho + 1);  // HHT with the same rho_inf
  const double ga = energy_loss(rho, -0.1), hht = energy_loss(-1.0, alpha_same);
  info(T, "energy change after 5 s, generalized-alpha rho_inf = 0.8", ga);
  info(T, "energy change after 5 s, HHT alpha = -0.111 (rho_inf 0.8)", hht);
  report(T, "generalized-alpha damps less than HHT: ga / hht", ga / hht, 1.0);
}

// 3. PI step size control: the pendulum at tolerances 1e-4 .. 1e-8 (largest step 0.05 s): the
// error at the outputs against a run at h = 1e-5 follows the tolerance (as eps^(2/3)), the
// outputs are exactly at k output_step.
void test_pi_controller() {
  const char* T = "pi_controller";
  fd::Model base = example("pendulum");
  base.solver.end_time = 2.0;
  base.solver.output_step = 0.1;
  // reference: fixed step 1e-5, outputs every 0.1 s
  std::vector<Vec> ref;
  {
    fd::Model m = base;
    m.solver.step = 1e-5;
    long k = 0;
    simulate(m, [&](double, const Vec& y, fd::System& sys) {
      if (k++ % 10000 == 0) ref.push_back(Vec(y.begin() + sys.output_index("rod.R"), y.begin() + sys.output_index("rod.R") + 3));
    });
  }
  double prev_err = -1.0;
  for (double eps : {1e-4, 1e-6, 1e-8}) {
    fd::Model m = base;
    m.solver.adaptive = true;
    m.solver.controller = "pi";
    m.solver.eps = eps;
    m.solver.step = 0.05;
    fd::System sys(m);
    fd::HhtIntegrator hht(sys, m.solver);
    fd::PiTimeStepper pi(hht, m.solver);
    Vec y;
    double err = 0.0, t_err = 0.0;
    size_t k = 0;
    const long steps = pi.run([&](double t) {
      sys.outputs(y);
      t_err = std::max(t_err, std::abs(t - (k == 0 ? 0.0 : std::min(k * 0.1, 2.0))));
      if (k < ref.size()) err = std::max(err, dist(Vec(y.begin() + sys.output_index("rod.R"), y.begin() + sys.output_index("rod.R") + 3), ref[k]));
      ++k;
    });
    char what[96];
    std::snprintf(what, sizeof what, "eps %.0e: steps %ld, rejected %ld, max error", eps, steps, pi.rejected_steps());
    info(T, what, err);
    std::snprintf(what, sizeof what, "eps %.0e: outputs (21) - %zu", eps, k);
    report(T, what, std::abs(21.0 - k), 0.0);
    std::snprintf(what, sizeof what, "eps %.0e: output times off k 0.1", eps);
    report(T, what, t_err, 1e-12);
    // local error per step O(h^3), global O(h^2): the global error goes with eps^(2/3)
    std::snprintf(what, sizeof what, "eps %.0e: error / (10 eps^(2/3))", eps);
    report(T, what, err / (10 * std::pow(eps, 2.0 / 3.0)), 1.0);
    if (prev_err > 0) {
      std::snprintf(what, sizeof what, "eps %.0e: error / error at 100 eps (100^(-2/3) = 0.046)", eps);
      report(T, what, err / prev_err, 0.1);
    }
    prev_err = err;
  }
}

}  // namespace

void add_integrator_tests(Tests& tests) {
  tests["spectral_radius"] = test_spectral_radius;
  tests["generalized_alpha"] = test_generalized_alpha;
  tests["pi_controller"] = test_pi_controller;
}
