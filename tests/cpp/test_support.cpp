#include "test_support.hpp"

#include <algorithm>
#include <cstdio>

namespace fdtest {

namespace {
bool g_ok = true;
}  // namespace

bool all_ok() { return g_ok; }

void report(const char* test, const std::string& metric, double value, double tol) {
  const bool ok = std::isfinite(value) && value <= tol;
  std::printf("RESULT %-12s %-34s %.3e tol %.1e %s\n", test, metric.c_str(), value, tol,
              ok ? "ok" : "FAIL");
  g_ok = g_ok && ok;
}

void info(const char* test, const std::string& metric, double value) {
  std::printf("RESULT %-12s %-34s %.3e (info)\n", test, metric.c_str(), value);
}

double norm_inf(const Vec& v) {
  double n = 0;
  for (double x : v) n = std::max(n, std::abs(x));
  return n;
}

// Shabana 2005 Eq. 2.13, written independently of the generator.
void rotation(const double* t, double A[3][3]) {
  const double t0 = t[0], t1 = t[1], t2 = t[2], t3 = t[3];
  const double R[3][3] = {{1 - 2 * (t2 * t2 + t3 * t3), 2 * (t1 * t2 - t0 * t3), 2 * (t1 * t3 + t0 * t2)},
                          {2 * (t1 * t2 + t0 * t3), 1 - 2 * (t1 * t1 + t3 * t3), 2 * (t2 * t3 - t0 * t1)},
                          {2 * (t1 * t3 - t0 * t2), 2 * (t2 * t3 + t0 * t1), 1 - 2 * (t1 * t1 + t2 * t2)}};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) A[i][j] = R[i][j];
}

// Rigid body element: Lie group (rigid_body: 7 coordinates, 6 velocity dofs) or Euler
// parameters (rigid_body_euler: 7 coordinates = dofs, normalization constraint).
fd::ElementSpec rigid_spec(const std::string& type, const std::string& name, const Vec& p, const Vec& q, const Vec& u) {
  fd::ElementSpec e;
  e.type = type;
  e.name = name;
  e.coords = {0, 1, 2, 3, 4, 5, 6};
  e.dofs = type == "rigid_body" ? std::vector<int>{0, 1, 2, 3, 4, 5} : e.coords;
  if (type != "rigid_body") e.cons = {0};
  e.p = p;
  e.q = q;
  e.u = u;
  return e;
}

const char* element_for(const std::string& form) { return form == "negrut" ? "rigid_body_euler" : "rigid_body"; }

// One rigid body with body-frame initial angular velocity w_body and orientation theta.
fd::Model body_model(const Vec& J, const Vec& theta, const Vec& w_body, double alpha, double h,
                     double end_time, const std::string& form) {
  double A[3][3];
  rotation(theta.data(), A);
  Vec w(3, 0.0);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) w[i] += A[i][j] * w_body[j];
  fd::Model m;
  m.name = "test_body";
  m.solver.alpha = alpha;
  m.solver.step = h;
  m.solver.end_time = end_time;
  m.solver.hht_form = form;
  m.elements.push_back(rigid_spec(element_for(form), "body", {2.0, J[0], J[1], J[2], 0.0, 0.0, 0.0},
                                  {0.3, -0.2, 0.1, theta[0], theta[1], theta[2], theta[3]},
                                  {0.1, 0.0, -0.05, w[0], w[1], w[2]}));
  return m;
}

// The same model with Euler-parameter bodies (for the negrut form, the legacy profile).
fd::Model with_element(fd::Model m, const std::string& type) {
  for (auto& e : m.elements) e = rigid_spec(type, e.name, e.p, e.q, e.u);
  return m;
}

fd::Model example(const std::string& name) { return fd::load_model(std::string(FD_GEN_DIR) + "/" + name + ".fdm"); }

// Integrate a model; call `f(t, y)` with the output channels at t = 0 and after every step.
void simulate(const fd::Model& m, const std::function<void(double, const Vec&, fd::System&)>& f,
              const std::string& ls) {
  fd::System sys(m);
  fd::HhtIntegrator hht(sys, m.solver, ls);
  hht.initialize();
  Vec y;
  sys.outputs(y);
  f(0.0, y, sys);
  const long n = std::lround(m.solver.end_time / m.solver.step);
  for (long k = 0; k < n; ++k) {
    hht.step();
    sys.outputs(y);
    f(hht.time(), y, sys);
  }
}

Vec dense(const fd::Sparsity& sp, const double* nz) {
  Vec D(sp.nrow * sp.ncol, 0.0);  // row major
  for (int j = 0; j < sp.ncol; ++j)
    for (int k = sp.colind[j]; k < sp.colind[j + 1]; ++k) D[sp.row[k] * sp.ncol + j] = nz[k];
  return D;
}

Run run(const std::string& name, double h) {
  fd::Model m = example(name);
  m.solver.step = h;
  Run r;
  simulate(m, [&](double t, const Vec& y, fd::System& sys) {
    r.t.push_back(t);
    const auto& names = sys.output_names();
    for (size_t i = 0; i < names.size(); ++i) r.y[names[i]].push_back(y[i]);
  });
  return r;
}

}  // namespace fdtest
