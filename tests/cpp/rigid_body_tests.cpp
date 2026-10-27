// Acceptance tests: rigid bodies (generated functions vs FD, free body, precession, Dzhanibekov,
// fast rotor, linear solvers).
// Each test prints "RESULT <test> <metric> <value> tol <tol>" (fdtest::report).

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "test_support.hpp"

using namespace fdtest;

namespace {

// 1a. Generated element functions vs central finite differences.
void test_fd_element() {
  const char* T = "fd_element";
  const fd_element_type& et = fd::find_element_type("rigid_body_euler");
  fd::GenFunction res(et.res), iter(et.iter), ic(et.ic);
  // Arbitrary state, deliberately off |theta| = 1 so that all terms of the Jacobian are tested.
  Vec q = {0.3, -0.2, 0.1, 0.8, 0.3, -0.4, 0.25};
  Vec qd = {0.1, 0.2, -0.3, 0.5, -0.7, 0.2, 0.9};
  Vec qdd = {1.0, -0.5, 0.3, 0.4, 0.2, -1.1, 0.6};
  Vec lam = {0.7}, p = {2.0, 0.1, 0.2, 0.3, 0.0, 0.0, -9.81};  // mass, inertia, gravity
  if (static_cast<int>(p.size()) != res.sparsity_in(4).nnz())  // the generated code reads all of p
    report(T, "parameter vector length", 1.0, 0.0);
  const double alpha = -0.1, beta = 0.3025, gamma = 0.6, h = 0.1;  // large h: all blocks matter
  Vec coef = {alpha, beta, gamma, h};
  const int nx = 8;
  const double eps = 1e-6;

  auto eval_r = [&](const Vec& a, const Vec& b, const Vec& c, const Vec& l) {
    Vec r(res.sparsity_out(0).nnz()), f(res.sparsity_out(1).nnz());
    const double* in[] = {a.data(), b.data(), c.data(), l.data(), p.data(), coef.data(), nullptr, nullptr};
    double* out[] = {r.data(), f.data()};
    res.eval(in, out);
    return dense(res.sparsity_out(0), r.data());
  };
  // Central FD of r with respect to one input vector (index which: 0 q, 1 qd, 2 qdd, 3 lam).
  auto fd_jac = [&](int which) {
    Vec* v[] = {&q, &qd, &qdd, &lam};
    const int n = static_cast<int>(v[which]->size());
    Vec D(nx * n);
    for (int j = 0; j < n; ++j) {
      const double v0 = (*v[which])[j];
      (*v[which])[j] = v0 + eps;
      Vec rp = eval_r(q, qd, qdd, lam);
      (*v[which])[j] = v0 - eps;
      Vec rm = eval_r(q, qd, qdd, lam);
      (*v[which])[j] = v0;
      for (int i = 0; i < nx; ++i) D[i * n + j] = (rp[i] - rm[i]) / (2 * eps);
    }
    return D;
  };
  Vec Dq = fd_jac(0), Dqd = fd_jac(1), Dqdd = fd_jac(2), Dlam = fd_jac(3);

  Vec Jnz(iter.sparsity_out(0).nnz());
  {
    const double* in[] = {q.data(), qd.data(), qdd.data(), lam.data(), p.data(), coef.data(), nullptr, nullptr};
    double* out[] = {Jnz.data()};
    iter.eval(in, out);
  }
  Vec J = dense(iter.sparsity_out(0), Jnz.data());
  double err = 0, scale = 1;
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < nx; ++j) {
      const double fdv = j < 7 ? Dqdd[i * 7 + j] + gamma * h * Dqd[i * 7 + j] + beta * h * h * Dq[i * 7 + j]
                               : Dlam[i];
      err = std::max(err, std::abs(J[i * nx + j] - fdv));
      scale = std::max(scale, std::abs(fdv));
    }
  report(T, "iteration matrix vs FD (rel)", err / scale, 1e-7);

  // Structural sparsity: every FD entry outside the generated pattern must vanish.
  Vec Jpat = dense(iter.sparsity_out(0), Vec(Jnz.size(), 1.0).data());
  double outside = 0;
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < nx; ++j)
      if (Jpat[i * nx + j] == 0.0) {
        const double fdv = j < 7 ? std::abs(Dqdd[i * 7 + j]) + std::abs(Dqd[i * 7 + j]) + std::abs(Dq[i * 7 + j])
                                 : std::abs(Dlam[i]);
        outside = std::max(outside, fdv);
      }
  report(T, "FD entries outside sparsity pattern", outside, 1e-8);
  info(T, "iteration matrix structural nnz", static_cast<double>(Jnz.size()));

  // IC system: A = [M Phi_q^T; Phi_q 0] with M = dr/dqdd at alpha = 0, b = [Q - h; -(Phi_q qd)_q qd].
  coef = {0.0, beta, gamma, h};
  Vec Dqdd0 = fd_jac(2), Dlam0 = fd_jac(3);
  Vec Dq0 = fd_jac(0);
  Vec nz[4];
  for (int i = 0; i < 4; ++i) nz[i].resize(ic.sparsity_out(i).nnz());
  auto eval_ic = [&](const Vec& a, const Vec& b) {
    const double* in[] = {a.data(), b.data(), nullptr, nullptr, p.data(), nullptr, nullptr, nullptr};
    double* out[] = {nz[0].data(), nz[1].data(), nz[2].data(), nz[3].data()};
    ic.eval(in, out);
  };
  eval_ic(q, qd);
  Vec A = dense(ic.sparsity_out(0), nz[0].data()), b = dense(ic.sparsity_out(1), nz[1].data());
  err = 0;
  scale = 1;
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < nx; ++j) {
      double ref;
      if (i < 7 && j < 7) ref = Dqdd0[i * 7 + j];
      else if (i < 7) ref = Dlam0[i];
      else if (j < 7) ref = Dq0[7 * 7 + j] * beta * h * h;  // r_con = Phi / (beta h^2)
      else ref = 0.0;
      err = std::max(err, std::abs(A[i * nx + j] - ref));
      scale = std::max(scale, std::abs(ref));
    }
  report(T, "IC matrix vs FD (rel)", err / scale, 1e-7);

  // b_dyn = Q - h = -r_dyn(qdd = 0, lam = 0, alpha = 0).
  Vec r0 = eval_r(q, qd, Vec(7, 0.0), Vec(1, 0.0));
  err = 0;
  for (int i = 0; i < 7; ++i) err = std::max(err, std::abs(b[i] + r0[i]));
  // b_con = -d/ds [Phi_q(q + s qd) qd] at s = 0, by central FD of the phid output.
  auto phid_at = [&](double s) {
    Vec qs(q);
    for (int i = 0; i < 7; ++i) qs[i] += s * qd[i];
    eval_ic(qs, qd);
    return dense(ic.sparsity_out(3), nz[3].data())[7];
  };
  const double bcon_fd = -(phid_at(eps) - phid_at(-eps)) / (2 * eps);
  err = std::max(err, std::abs(b[7] - bcon_fd));
  report(T, "IC right-hand side vs FD", err / std::max(1.0, norm_inf(b)), 1e-7);
}

// 1b. Assembled global HHT residual (scatter, Newmark or Lie group update) vs assembled
// iteration matrix, by central FD with respect to x = [qdd; lam].
double fd_global_error(const fd::Model& m, int* nnz = nullptr) {
  fd::System sys(m);
  fd::HhtIntegrator hht(sys, m.solver);
  hht.initialize();
  hht.step();
  hht.begin_step(m.solver.step);
  const int nx = sys.nx(), nv = sys.nv();
  Vec x(nx);
  for (int i = 0; i < nx; ++i) x[i] = (i < nv ? sys.qdd[i] : sys.lam[i - nv]) + 0.1 * std::sin(i + 1.0);
  Vec r0;
  hht.residual_at(x, r0);  // also sets the state for the matrix evaluation
  const fd::CsrMatrix& J = sys.iteration_matrix(hht.coef());
  if (nnz) *nnz = J.nnz();
  Vec Jd(nx * nx, 0.0);
  for (int i = 0; i < nx; ++i)
    for (int k = J.rowptr[i]; k < J.rowptr[i + 1]; ++k) Jd[i * nx + J.col[k]] = J.val[k];
  const double eps = 1e-6;
  double err = 0, scale = 1;
  for (int j = 0; j < nx; ++j) {
    Vec xp(x), xm(x), rp, rm;
    xp[j] += eps;
    xm[j] -= eps;
    hht.residual_at(xp, rp);
    hht.residual_at(xm, rm);
    for (int i = 0; i < nx; ++i) {
      const double fdv = (rp[i] - rm[i]) / (2 * eps);
      err = std::max(err, std::abs(Jd[i * nx + j] - fdv));
      scale = std::max(scale, std::abs(fdv));
    }
  }
  return err / scale;
}

void test_fd_global() {
  const char* T = "fd_global";
  // Lie group body, Arnold-Bruls: tests the exponential update and the tangent operator.
  fd::Model m = example("free_body");
  m.solver.alpha = -0.1;
  m.solver.step = 0.05;
  int nnz = 0;
  report(T, "Lie group, Arnold-Bruls (rel)", fd_global_error(m, &nnz), 1e-7);
  info(T, "global CSR nnz, Lie group body", nnz);
  // Euler-parameter body, Negrut (legacy profile), with and without modified Newmark.
  fd::Model e = with_element(m, "rigid_body_euler");
  e.solver.hht_form = "negrut";
  report(T, "Euler parameters, Negrut (rel)", fd_global_error(e, &nnz), 1e-7);
  info(T, "global CSR nnz, Euler-parameter body", nnz);
  e.elements[0].newmark = "modified";
  report(T, "Euler parameters, Negrut, modified Newmark (rel)", fd_global_error(e), 1e-7);
}

// 2-4. Default example: uniform COM motion, |theta| = 1, conservation of T and H.
struct Drift {
  double R = 0, norm = 0, T = 0, H = 0, T2 = 0, H2 = 0, T_end = 0, H_end = 0;
};

Drift free_body_drift(double alpha, double h, const char* form = "arnold_bruls") {
  fd::Model m = with_element(example("free_body"), element_for(form));
  m.solver.alpha = alpha;
  m.solver.step = h;
  m.solver.hht_form = form;
  const auto& e = m.elements[0];
  Drift d;
  double T0 = 0, H0[3] = {0, 0, 0}, Hn0 = 0;
  int iR = -1, iT = -1, iH = -1, iN = -1;
  simulate(m, [&](double t, const Vec& y, fd::System& sys) {
    if (iR < 0) {
      iR = sys.output_index("body1.R");
      iT = sys.output_index("body1.T");
      iH = sys.output_index("body1.H");
      iN = sys.output_index("body1.theta_norm");
      T0 = y[iT];
      for (int k = 0; k < 3; ++k) H0[k] = y[iH + k];
      Hn0 = std::sqrt(H0[0] * H0[0] + H0[1] * H0[1] + H0[2] * H0[2]);
    }
    double dH = 0;
    for (int k = 0; k < 3; ++k) {
      const double ref = e.q[k] + e.u[k] * t;
      d.R = std::max(d.R, std::abs(y[iR + k] - ref) / std::max(1.0, std::abs(ref)));
      dH += (y[iH + k] - H0[k]) * (y[iH + k] - H0[k]);
    }
    dH = std::sqrt(dH) / Hn0;
    const double dT = std::abs(y[iT] - T0) / T0;
    d.norm = std::max(d.norm, std::abs(y[iN] - 1.0));
    d.T = std::max(d.T, dT);
    d.H = std::max(d.H, dH);
    if (t <= 2.0 + 1e-12) {
      d.T2 = d.T;
      d.H2 = d.H;
    }
    d.T_end = y[iT] / T0 - 1.0;
    double hn = 0;
    for (int k = 0; k < 3; ++k) hn += y[iH + k] * y[iH + k];
    d.H_end = std::sqrt(hn) / Hn0 - 1.0;
  });
  return d;
}

void test_free_body_alpha0() {
  const char* T = "alpha=0";
  const Drift d = free_body_drift(0.0, 1e-3), d2 = free_body_drift(0.0, 5e-4);
  report(T, "COM |R - (R0 + v0 t)| (rel)", d.R, 1e-12);
  report(T, "max ||theta| - 1|", d.norm, 1e-12);
  // Up to 2 s the drift is the O(h^2) truncation error of the trapezoidal rule.
  report(T, "max |T - T0| / T0, t <= 2 s", d.T2, 1e-6);
  report(T, "max |H - H0| / |H0|, t <= 2 s", d.H2, 1e-6);
  info(T, "same T, h=5e-4", d2.T2);
  info(T, "observed order of T drift, t <= 2 s", std::log2(d.T2 / d2.T2));
  // Lie group body: no normalization constraint, so the weakly unstable (-1)^n mode of
  // theta.thetadot of the Euler-parameter formulation does not exist; still O(h^2) after 10 s.
  report(T, "max |T - T0| / T0, t <= 10 s", d.T, 1e-6);
  report(T, "max |H - H0| / |H0|, t <= 10 s", d.H, 1e-6);
  // Euler parameters (legacy profile, alpha = 0): the parasitic mode dominates over 10 s.
  const Drift e = free_body_drift(0.0, 1e-3, "negrut");
  info(T, "Euler parameters: max |T - T0| / T0, t <= 10 s", e.T);
}

void test_free_body_alpha01() {
  const char* T = "alpha=-0.1";
  const Drift d = free_body_drift(-0.1, 1e-3), d2 = free_body_drift(-0.1, 5e-4);
  report(T, "COM |R - (R0 + v0 t)| (rel)", d.R, 1e-12);
  report(T, "max ||theta| - 1|", d.norm, 1e-12);
  // Arnold-Bruls form: second order, the numerical damping is O(h^2) and the parasitic
  // theta.thetadot mode of alpha = 0 is damped.
  report(T, "max |T - T0| / T0, t <= 10 s", d.T, 2e-6);
  report(T, "max |H - H0| / |H0|, t <= 10 s", d.H, 2e-6);
  info(T, "T(10) / T0 - 1", d.T_end);
  info(T, "|H(10)| / |H0| - 1", d.H_end);
  info(T, "T(10) / T0 - 1, h=5e-4", d2.T_end);
  // At least second order (the Lie group form shows about third order here).
  info(T, "order of energy loss p", std::log2(d.T_end / d2.T_end));
  report(T, "order of energy loss: max(0, 2 - p)", std::max(0.0, 2.0 - std::log2(d.T_end / d2.T_end)), 0.2);
  // Negrut form (legacy FreeDyn): first order, for comparison.
  const Drift n = free_body_drift(-0.1, 1e-3, "negrut"), n2 = free_body_drift(-0.1, 5e-4, "negrut");
  info(T, "negrut: T(10) / T0 - 1", n.T_end);
  info(T, "negrut: order of energy loss", std::log2(n.T_end / n2.T_end));
}

// 5. Symmetric body: body-frame angular velocity vs analytic torque-free precession.
double precession_error(double h, double alpha, double* err_w3 = nullptr, const char* form = "arnold_bruls") {
  const double J1 = 0.1, J3 = 0.3;
  const Vec w0 = {0.5, 0.3, 2.0};
  const double s = 1.0 / std::sqrt(1 + 0.04 + 0.09 + 0.16);
  const Vec theta = {s, 0.2 * s, -0.3 * s, 0.4 * s};  // non-trivial orientation: tests global input
  fd::Model m = body_model({J1, J1, J3}, theta, w0, alpha, h, 10.0, form);
  const double Omega = (J3 - J1) / J1 * w0[2];
  const double wperp = std::hypot(w0[0], w0[1]);
  double err = 0, e3 = 0;
  int iw = -1;
  simulate(m, [&](double t, const Vec& y, fd::System& sys) {
    if (iw < 0) iw = sys.output_index("body.omega_body");
    const double c = std::cos(Omega * t), sn = std::sin(Omega * t);
    const double w1 = w0[0] * c - w0[1] * sn, w2 = w0[0] * sn + w0[1] * c;
    err = std::max(err, std::hypot(y[iw] - w1, y[iw + 1] - w2) / wperp);
    e3 = std::max(e3, std::abs(y[iw + 2] - w0[2]) / std::abs(w0[2]));
  });
  if (err_w3) *err_w3 = e3;
  return err;
}

void test_precession() {
  const char* T = "precession";
  double e3;
  const double e1 = precession_error(1e-3, 0.0, &e3);
  const double e2 = precession_error(5e-4, 0.0);
  report(T, "max |w_perp - analytic| / |w_perp|, h=1e-3", e1, 2e-4);
  // Euler's equations conserve w3 of a symmetric body exactly, also discretely in body velocities.
  report(T, "max |w3 - w3(0)| / |w3|, h=1e-3", e3, 1e-13);
  info(T, "same, h=5e-4", e2);
  const double order = std::log2(e1 / e2);
  report(T, "observed order |2 - p|", std::abs(2.0 - order), 0.1);
  const double ea1 = precession_error(1e-3, -0.1), ea2 = precession_error(5e-4, -0.1);
  report(T, "alpha=-0.1: error, h=1e-3", ea1, 3e-4);
  info(T, "alpha=-0.1: error, h=5e-4", ea2);
  report(T, "alpha=-0.1: observed order |2 - p|", std::abs(2.0 - std::log2(ea1 / ea2)), 0.1);
  const double en1 = precession_error(1e-3, -0.1, nullptr, "negrut"), en2 = precession_error(5e-4, -0.1, nullptr, "negrut");
  info(T, "negrut alpha=-0.1: error, h=1e-3", en1);
  info(T, "negrut alpha=-0.1: observed order p", std::log2(en1 / en2));
}

// 6. Intermediate axis (Dzhanibekov) instability, compared with RK4 on Euler's equations.
void euler_rhs(const double J[3], const double w[3], double dw[3]) {
  dw[0] = (J[1] - J[2]) / J[0] * w[1] * w[2];
  dw[1] = (J[2] - J[0]) / J[1] * w[2] * w[0];
  dw[2] = (J[0] - J[1]) / J[2] * w[0] * w[1];
}

// Time of the first sign change of w2 for Euler's equations (RK4, h = 1e-5).
double euler_first_flip(const double J[3], const double w0[3]) {
  double w[3] = {w0[0], w0[1], w0[2]};
  const double h = 1e-5;
  for (long n = 0; n < 4000000; ++n) {
    double k[4][3], tmp[3];
    euler_rhs(J, w, k[0]);
    for (int i = 0; i < 3; ++i) tmp[i] = w[i] + 0.5 * h * k[0][i];
    euler_rhs(J, tmp, k[1]);
    for (int i = 0; i < 3; ++i) tmp[i] = w[i] + 0.5 * h * k[1][i];
    euler_rhs(J, tmp, k[2]);
    for (int i = 0; i < 3; ++i) tmp[i] = w[i] + h * k[2][i];
    euler_rhs(J, tmp, k[3]);
    const double prev = w[1];
    for (int i = 0; i < 3; ++i) w[i] += h / 6 * (k[0][i] + 2 * k[1][i] + 2 * k[2][i] + k[3][i]);
    if (prev > 0 && w[1] <= 0) return (n + 1) * h;
  }
  return -1;
}

void test_dzhanibekov() {
  const char* T = "dzhanibekov";
  fd::Model m = example("dzhanibekov");
  const auto& e = m.elements[0];
  const double J[3] = {e.p[1], e.p[2], e.p[3]}, w0[3] = {e.u[3], e.u[4], e.u[5]};  // theta0 = identity
  double w2_min = 1e9, t_flip = -1, T0 = -1;
  int flips = 0, iw = -1, iT = -1;
  double prev = w0[1];
  simulate(m, [&](double t, const Vec& y, fd::System& sys) {
    if (iw < 0) {
      iw = sys.output_index("wingnut.omega_body");
      iT = sys.output_index("wingnut.T");
      T0 = y[iT];
    }
    const double w2 = y[iw + 1];
    w2_min = std::min(w2_min, w2);
    if ((prev > 0) != (w2 > 0)) {
      ++flips;
      if (t_flip < 0) t_flip = t;
    }
    prev = w2;
  });
  const double t_ref = euler_first_flip(J, w0);
  report(T, "flip: w2_min / w2(0) + 1", w2_min / w0[1] + 1.0, 0.05);
  report(T, "first flip time vs RK4 reference (rel)", std::abs(t_flip - t_ref) / t_ref, 1e-3);
  info(T, "first flip time [s]", t_flip);
  info(T, "RK4 reference first flip time [s]", t_ref);
  info(T, "sign changes of w2 in 20 s", flips);
}

// 7. Fast rotor: spin about a principal axis must neither decay nor drift in orientation.
// Euler parameters oscillate with omega/2 and HHT damps that oscillation (the legacy saturation
// problem); the Lie group body integrates the constant body angular velocity instead.
void spin(double w, const std::string& form, double* loss, double* angle_err) {
  const Vec theta = {1, 0, 0, 0};
  fd::Model m = body_model({0.1, 0.1, 0.2}, theta, {0, 0, w}, -0.1, 1e-3, 1.0, form);
  m.elements[0].u = {0, 0, 0, 0, 0, w};
  int iw = -1, it = -1;
  *loss = 0;
  *angle_err = 0;
  try {
    simulate(m, [&](double t, const Vec& y, fd::System& sys) {
      if (iw < 0) {
        iw = sys.output_index("body.omega_body");
        it = sys.output_index("body.theta");
      }
      *loss = std::max(*loss, std::abs(y[iw + 2] / w - 1.0));
      // Exact orientation: rotation by w t about z, theta_ex = [cos(w t / 2), 0, 0, sin(w t / 2)].
      // Error quaternion conj(theta_ex) o theta; its vector part is sin(angle error / 2)
      // (better conditioned than acos of the scalar part).
      const double c = std::cos(0.5 * w * t), sn = std::sin(0.5 * w * t);
      const double t0 = y[it], t1 = y[it + 1], t2 = y[it + 2], t3 = y[it + 3];
      const double e1 = c * t1 + sn * t2, e2 = c * t2 - sn * t1, e3 = c * t3 - sn * t0;
      *angle_err = std::max(*angle_err, 2 * std::asin(std::min(1.0, std::sqrt(e1 * e1 + e2 * e2 + e3 * e3))));
    });
  } catch (const std::exception&) {
    *loss = *angle_err = INFINITY;
  }
}

void test_fast_rotor() {
  const char* T = "fast_rotor";
  for (double w : {300.0, 1000.0, 3000.0}) {  // omega h = 0.3, 1, 3
    double loss, ang;
    spin(w, "arnold_bruls", &loss, &ang);
    const std::string tag = "w=" + std::to_string(static_cast<int>(w)) + ": ";
    report(T, tag + "Lie group spin loss", loss, 1e-12);
    report(T, tag + "Lie group orientation error [rad]", ang, 1e-11);
    spin(w, "negrut", &loss, &ang);
    info(T, tag + "Euler parameters (negrut) spin loss", loss);
  }
}

// Linear solvers behind the interface give the same trajectory.
void test_linear_solvers() {
  const char* T = "lin_solvers";
  fd::Model m = example("free_body");
  m.solver.end_time = 1.0;
  Vec ya, yb;
  simulate(m, [&](double, const Vec& y, fd::System&) { ya = y; }, "sparselu");
  simulate(m, [&](double, const Vec& y, fd::System&) { yb = y; }, "dense");
  double d = 0;
  for (size_t i = 0; i < ya.size(); ++i) d = std::max(d, std::abs(ya[i] - yb[i]));
  report(T, "sparselu vs dense, max diff at t=1", d, 1e-12);
#ifdef FD_WITH_KLU
  // KLU (refactorization with kept pivots) on a model with joints, against the dense LU:
  // other round-off in the Newton corrections, so the trajectories agree to the Newton
  // tolerance (sparselu vs dense: 1.9e-8 on this model), not to round-off.
  fd::Model dp = example("pendulum");
  dp.solver.end_time = 1.0;
  simulate(dp, [&](double, const Vec& y, fd::System&) { ya = y; }, "klu");
  simulate(dp, [&](double, const Vec& y, fd::System&) { yb = y; }, "dense");
  d = 0;
  for (size_t i = 0; i < ya.size(); ++i) d = std::max(d, std::abs(ya[i] - yb[i]) / std::max(1.0, std::abs(yb[i])));
  report(T, "klu vs dense (pendulum), max rel diff at t=1", d, 1e-7);
#endif
}

}  // namespace

void add_rigid_body_tests(Tests& tests) {
  tests["fd_element"] = test_fd_element;
  tests["fd_global"] = test_fd_global;
  tests["free_body_alpha0"] = test_free_body_alpha0;
  tests["free_body_alpha01"] = test_free_body_alpha01;
  tests["precession"] = test_precession;
  tests["dzhanibekov"] = test_dzhanibekov;
  tests["fast_rotor"] = test_fast_rotor;
  tests["linear_solvers"] = test_linear_solvers;
}
