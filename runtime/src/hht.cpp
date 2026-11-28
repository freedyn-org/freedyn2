#include "freedyn2/hht.hpp"

#include <algorithm>
#include <cfloat>
#include <cstdio>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fd {

namespace {
double norm_inf(const std::vector<double>& v) {
  double n = 0.0;
  for (double x : v) n = std::max(n, std::abs(x));
  return n;
}

double norm_2(const double* v, int n) {
  double s = 0.0;
  for (int i = 0; i < n; ++i) s += v[i] * v[i];
  return std::sqrt(s);
}

// Constants of the legacy solver (SOL_DynSolverHHT.h).
constexpr double kLegacyEps = 1e-16;
constexpr double kNoiseFactor = 1e3;
// Newton (velocity criterion): a correction that stagnates below this times the tolerance is round-off.
constexpr double kStagnationFactor = 1e3;
constexpr double kAdaptivity = 1e-3;  // c in Negrut et al. Eq. 40
}  // namespace

const char* to_string(StepStatus s) {
  switch (s) {
    case StepStatus::ok: return "ok";
    case StepStatus::newton_not_converged: return "Newton not converged";
    case StepStatus::residual_increased: return "residual increased";
    case StepStatus::local_error: return "local error too large";
    case StepStatus::linear_solver_failed: return "linear solver failed";
    case StepStatus::event: return "event within the step";
  }
  return "?";
}

HhtIntegrator::HhtIntegrator(System& sys, const SolverSettings& s, const std::string& linear_solver)
    : sys_(sys), s_(s) {
  if (s.alpha < -1.0 / 3.0 - 1e-15 || s.alpha > 0.0)
    throw std::runtime_error("HHT: alpha must be in [-1/3, 0]");
  if (s.hht_form != "negrut" && s.hht_form != "arnold_bruls")
    throw std::runtime_error("HHT: form must be negrut or arnold_bruls");
  ab_ = s.hht_form == "arnold_bruls";
  legacy_error_norm_ = s.error_norm == "legacy";
  if (s.controller != "legacy" && s.controller != "pi")
    throw std::runtime_error("HHT: controller must be legacy or pi");
  // Same expressions as SOL_DynSolverHHTParameter.
  alpha_ = s.alpha;
  beta_ = 0.25 * (1. - s.alpha) * (1. - s.alpha);
  gamma_ = 0.5 * (1. - 2. * s.alpha);
  // Generalized-alpha (Chung and Hulbert 1993) on the Lie group (Arnold and Bruls 2007; Bruls,
  // Cardona, Arnold 2012) from the spectral radius at infinity rho_inf: the equations at t_n+1,
  // the pseudo-acceleration (1 - am) a_n+1 + am a_n = (1 - af) qdd_n+1 + af qdd_n, i.e.
  // a_n+1 = c1 qdd_n+1 + c2 qdd_n + c3 a_n. HHT (Arnold-Bruls form) is the case am = 0,
  // af = -alpha; it keeps its own expressions below (the same results to the last bit).
  gen_alpha_ = s.rho_inf >= 0.0;
  if (gen_alpha_) {
    const double rho = s.rho_inf;
    if (rho > 1.0) throw std::runtime_error("HHT: rho_inf must be in [0, 1]");
    if (s.hht_form != "arnold_bruls") throw std::runtime_error("HHT: rho_inf (generalized-alpha) needs the arnold_bruls form");
    if (s.newton == "legacy") throw std::runtime_error("HHT: rho_inf (generalized-alpha) does not go with newton legacy");
    am_ = (2. * rho - 1.) / (rho + 1.);
    af_ = rho / (rho + 1.);
    gamma_ = 0.5 + af_ - am_;
    beta_ = 0.25 * (gamma_ + 0.5) * (gamma_ + 0.5);
    alpha_ = -af_;  // only for the error constant of the legacy controller and the reports
    c1_ = (1. - af_) / (1. - am_);
    c2_ = af_ / (1. - am_);
    c3_ = -am_ / (1. - am_);
  } else {
    am_ = 0.0;
    af_ = -alpha_;
  }
  // Coefficients passed to the generated functions. Negrut: r_dyn = M qdd/(1+alpha) + f
  // - alpha/(1+alpha) f_n. Arnold-Bruls: r_dyn = M qdd + f at t_{n+1} with q, qd depending on
  // qdd through the pseudo-acceleration a = (1+alpha) qdd - alpha qdd_n, i.e. with
  // dq/dqdd = (1+alpha) beta h^2 and dqd/dqdd = (1+alpha) gamma h. The same generated
  // iteration matrix serves both forms.
  c_.alpha = ab_ ? 0. : alpha_;
  c_.beta = ab_ ? (1. + alpha_) * beta_ : beta_;
  c_.gamma = ab_ ? (1. + alpha_) * gamma_ : gamma_;
  if (gen_alpha_) c_.beta = c1_ * beta_, c_.gamma = c1_ * gamma_;  // dq/dqdd = c1 beta h^2, dqd/dqdd = c1 gamma h
  c_.h = s.step;
  if (!ab_ && !sys.vector_space())
    throw std::runtime_error("HHT: the negrut form needs vector-space elements (e.g. rigid_body_euler)");
  if (s.newton == "fixed" && s.adaptive)
    throw std::runtime_error("HHT: newton fixed is meant for a fixed step size (adaptive off)");
  if (ab_ && sys.modified_newmark())
    throw std::runtime_error("HHT: modified Newmark formulas are a legacy option of the negrut form");
  // Local error of a step: the Newmark truncation term C h^2 (qdd_n+1 - qdd_n) per velocity dof.
  // Negrut et al. (2007) Eq. 37a: C = beta - 1/(6 (1 + alpha)), for the force-weighted form,
  // kept with the legacy controller of HHT. For the equations at t_n+1 (Arnold-Bruls HHT,
  // generalized-alpha) the position increment h v_n + h^2 ((1/2 - beta) a_n + beta a_n+1), with
  // a_n = qdd(t_n + (am - af) h) for accelerations linear in time, leaves
  // C = beta + (am - af)/2 - 1/6 (Newmark: beta - 1/6, Zienkiewicz and Xie 1991).
  const double c_newmark = beta_ - 1. / (6. * (1. + alpha_));
  const double c_err = ab_ && (gen_alpha_ || s.controller == "pi") ? beta_ + 0.5 * (am_ - af_) - 1. / 6. : c_newmark;
  psi_ = sys.nv() * std::pow(s.eps, 2) / std::pow(c_err, 2);
  psi_newton_ = sys.nv() * std::pow(s.eps, 2) / std::pow(c_newmark, 2);
  ls_name_ = linear_solver;
  make_solvers();
  const int nx = sys.nx(), nq = sys.nq(), nv = sys.nv();
  for (auto* v : {&q_n_, &q_pred_}) v->resize(nq);
  for (auto* v : {&qd_n_, &qdd_n_, &a_n_, &qd_pred_, &incr_pred_}) v->resize(nv);
  lam_n_.resize(sys.nc());
  q_max_.assign(nv, 1.0);
  scale_new_.resize(nv);
  for (auto* v : {&f_n_, &x_, &dx_, &r_, &f_, &rhs_}) v->resize(nx);
  sys.set_newmark_reference(&q_n_, &qd_n_, &qdd_n_);
}

void HhtIntegrator::make_solvers() {
  ls_ = make_linear_solver(ls_name_);
  ls_->set_fixed_pivots(s_.newton == "fixed");  // real-time: the same work in every step
  ls_ic_ = make_linear_solver(ls_name_);
}

namespace {
// Largest |v| over the constraint rows (x space) of the active constraints; its row in *at.
double active_inf(const System& sys, const std::vector<double>& v, int* at = nullptr) {
  double n = 0.0;
  for (int c = 0; c < sys.nc(); ++c)
    if (sys.constraint_active(c) && std::abs(v[sys.nv() + c]) > n) {
      n = std::abs(v[sys.nv() + c]);
      if (at) *at = c;
    }
  return n;
}
double active_norm_2(const System& sys, const std::vector<double>& v) {
  double s = 0.0;
  for (int c = 0; c < sys.nc(); ++c)
    if (sys.constraint_active(c)) s += v[sys.nv() + c] * v[sys.nv() + c];
  return std::sqrt(s);
}
// Tolerance of the constraint positions at t = 0 (absolute, as before the assembly existed).
constexpr double kPositionTol = 1e-10;
}  // namespace

void HhtIntegrator::assemble_positions() {
  std::vector<double> moved(sys_.nv(), 0.0);
  report_.assembly_iterations = project_positions(moved, &report_.violation);
  for (int i = 0; i < sys_.nv(); ++i)
    if (std::abs(moved[i]) > report_.largest_move) {
      report_.largest_move = std::abs(moved[i]);
      report_.largest_move_dof = i;
    }
}

int HhtIntegrator::project_positions(std::vector<double>& moved, double* violation) {
  const int nx = sys_.nx(), nv = sys_.nv();
  std::vector<double> b(nx), phi(nx), phid(nx), rhs(nx, 0.0), dx(nx), q_n;
  const CsrMatrix* A = &sys_.ic_system(b.data(), phi.data(), phid.data());
  if (violation) *violation = active_inf(sys_, phi);
  double prev = std::numeric_limits<double>::infinity();
  int corrections = 0;
  for (int it = 0;; ++it) {
    const double err = active_inf(sys_, phi);
    if (err <= kPositionTol) break;
    // Newton converges quadratically; no progress, or too many corrections: the constraints
    // cannot be satisfied (bodies too far apart for the joints, a driver beyond its range).
    if (it == 25 || (it >= 3 && !(err < prev))) {
      int c = -1;
      active_inf(sys_, phi, &c);
      throw std::runtime_error("HHT: initial positions violate the constraints and cannot be assembled (|Phi| = " +
                               std::to_string(err) + " at " + sys_.constraint_name(c) + ")");
    }
    prev = err;
    for (int c = 0; c < sys_.nc(); ++c) rhs[nv + c] = sys_.constraint_active(c) ? -phi[nv + c] : 0.0;
    ls_ic_->factorize(*A);
    ls_ic_->solve(rhs.data(), dx.data());
    q_n = sys_.q;
    std::copy(dx.begin(), dx.begin() + nv, sys_.incr.begin());
    sys_.update_configuration(q_n);
    for (int i = 0; i < nv; ++i) moved[i] += dx[i];
    ++corrections;
    A = &sys_.ic_system(b.data(), phi.data(), phid.data());
  }
  std::fill(sys_.incr.begin(), sys_.incr.end(), 0.0);
  return corrections;
}

// Static equilibrium f(q, 0, lam) = h - Q + B^T lam = 0, Phi(q) = 0 at rest, by pseudo-transient
// continuation with the generated iteration matrix: with the coefficients (alpha, beta, gamma,
// h) = (0, beta, 0, 1) and vd = 0 its rows are [M + beta K T, B^T; B, 0] for the unknowns
// (dvd, dlam), the configuration moves by delta = beta dvd, so
//   (M / beta + K) delta + B^T dlam = -f,  B delta = -Phi:
// a Newton step in the mass metric regularized by M / beta (beta: a pseudo-time step squared).
// Pseudo-transient continuation (Kelley and Keyes 1998, SIAM J. Numer. Anal. 35) with the
// switched evolution relaxation rule beta <- beta |a|_old / |a|_new (at most tenfold per step):
// small beta at the start makes the steps a gradient flow in the mass metric towards a minimum
// of the potential (a stable equilibrium from far away, through positions where the loads grow
// first), large beta near the equilibrium makes them Newton steps. A step whose positions cannot
// be projected onto the constraints is undone and beta divided by ten.
// After every step the positions are projected back onto Phi = 0 (project_positions): the
// constraint error of a large step is second order, and a correction of it inside the step
// would act through M / beta as a large inertia force at small beta. The multipliers are those
// at rest, [M B^T; B 0] [a; lam] = [Q - h; 0] (the initial-condition matrix), and the merit is
// |a|_M = sqrt(a^T M a) = sqrt(-a^T f), the acceleration the system would start with if released
// there (the M^-1 norm of the force the constraints do not take; massless dofs have none).
void HhtIntegrator::static_equilibrium() {
  const int nx = sys_.nx(), nv = sys_.nv();
  std::fill(sys_.qd.begin(), sys_.qd.end(), 0.0);
  std::fill(sys_.qdd.begin(), sys_.qdd.end(), 0.0);
  std::fill(sys_.incr.begin(), sys_.incr.end(), 0.0);
  std::unique_ptr<LinearSolver> ls = make_linear_solver(ls_name_);  // own pivots: another matrix
  std::vector<double> r(nx), f(nx), rhs(nx), dx(nx), b(nx), phi(nx), phid(nx), xa(nx), moved(nv, 0.0);
  std::vector<double> moved_prev, q_prev, q_n;
  constexpr double kBetaNewton = 1e8, kBetaMax = 1e14, kBetaMin = 1e-14;
  HhtCoef c{0.0, 1e-2, 0.0, 1.0};
  auto at_rest = [&] {  // lam at rest, the residual with it (r_dyn = f at vd = 0), the merit
    const CsrMatrix& A = sys_.ic_system(b.data(), phi.data(), phid.data());
    for (int i = nv; i < nx; ++i) b[i] = 0.0;  // no constraint accelerations (drivers: at rest)
    ls_ic_->factorize(A);
    ls_ic_->solve(b.data(), xa.data());
    std::copy(xa.begin() + nv, xa.end(), sys_.lam.begin());
    sys_.residual(c, r.data(), f.data());
    double m2 = 0.0;
    for (int i = 0; i < nv; ++i) m2 -= xa[i] * r[i];
    return std::sqrt(std::max(m2, 0.0));
  };
  double merit = at_rest();
  const double tiny = 1e-12 * (1.0 + merit);  // round-off (relative to the loads at the start)
  for (int it = 0;; ++it) {
    if (merit <= tiny) break;
    if (it == 400 || c.beta < kBetaMin) {
      char msg[256];
      std::snprintf(msg, sizeof msg,
                    "HHT: static equilibrium not found (|a| = %.3g at rest): a body free to fall or to drift, or a "
                    "mechanism in a singular position (e.g. a parallelogram stretched out)?",
                    merit);
      throw std::runtime_error(msg);
    }
    try {
      ls->factorize(sys_.iteration_matrix(c));
    } catch (const std::exception&) {  // singular in the Newton regime: smaller pseudo-time step
      c.beta /= 10.0;
      sys_.residual(c, r.data(), f.data());
      continue;
    }
    for (int i = 0; i < nx; ++i) rhs[i] = -r[i];
    ls->solve(rhs.data(), dx.data());
    q_prev = sys_.q, moved_prev = moved, q_n = sys_.q;
    double step = 0.0;
    for (int i = 0; i < nv; ++i) {
      sys_.incr[i] = c.beta * dx[i];
      moved[i] += sys_.incr[i];
      step = std::max(step, std::abs(sys_.incr[i]));
    }
    sys_.update_configuration(q_n);
    std::fill(sys_.incr.begin(), sys_.incr.end(), 0.0);
    double m = std::numeric_limits<double>::infinity();
    try {
      project_positions(moved);
      m = at_rest();
    } catch (const std::exception&) {
    }
    if (!std::isfinite(m)) {  // undo, smaller pseudo-time step
      sys_.q = q_prev, moved = moved_prev;
      c.beta /= 10.0;
      merit = at_rest();
      continue;
    }
    const double ratio = m > 0.0 ? std::min(merit / m, 10.0) : 10.0;
    merit = m;
    ++report_.static_iterations;
    double qmax = 0.0;
    for (double x : sys_.q) qmax = std::max(qmax, std::abs(x));
    if (c.beta >= kBetaNewton && step <= 1e-10 * (1.0 + qmax)) break;
    c.beta = std::min(ratio * c.beta, kBetaMax);
    sys_.residual(c, r.data(), f.data());  // r_con = Phi / beta with the new beta
  }
  for (int i = 0; i < nv; ++i)
    if (std::abs(moved[i]) > report_.static_move) {
      report_.static_move = std::abs(moved[i]);
      report_.static_move_dof = i;
    }
}

void HhtIntegrator::initialize() {
  sys_.set_initial_state();
  t_ = 0.0;
  sys_.t = 0.0;
  n_steps_ = 0;
  report_ = InitialReport();
  event_log_.clear();
  event_probes_ = grid_steps_ = subdivided_steps_ = 0;
  const int nx = sys_.nx(), nv = sys_.nv();
  std::fill(sys_.incr.begin(), sys_.incr.end(), 0.0);
  // Redundant constraints: their equations would make every matrix singular. Deactivated in
  // the given positions (the rank of an assembled mechanism is the same unless it sits in a
  // singular position); the new pattern needs new linear solvers.
  sys_.deactivate_constraints(sys_.find_redundant_constraints());
  report_.redundant = sys_.inactive_constraints();
  candidates_.clear();
  rank_log_.clear();
  make_solvers();
  if (!report_.redundant.empty() && s_.newton != "fixed") classify_redundant();
  assemble_positions();
  if (s_.statics) {
    static_equilibrium();
    // The equilibrium can be a singular position (a linkage hanging stretched): its rank is
    // checked again there (as after a failed step); what is redundant now is so by the position.
    if (s_.newton != "fixed") {
      const std::vector<int> found = sys_.find_redundant_constraints(1e-6);
      if (!found.empty()) {
        if (sys_.deactivate_constraints(found)) make_solvers();
        for (int c : found) {
          report_.redundant.push_back(c);
          report_.positional.push_back(c);
          candidates_.push_back(c);
        }
      }
    }
  }
  std::vector<double> b(nx), phi(nx), phid(nx), rhs(nx, 0.0), dv(nx);
  sys_.ic_system(b.data(), phi.data(), phid.data());
  // A redundant constraint that its independent ones do not satisfy: the overconstrained
  // mechanism does not fit together (e.g. hinge axes of a closed loop not parallel).
  for (int c : sys_.inactive_constraints())
    if (std::abs(phi[nv + c]) > kPositionTol)
      throw std::runtime_error("HHT: the redundant constraint " + sys_.constraint_name(c) +
                               " is violated in the assembled positions (|Phi| = " + std::to_string(std::abs(phi[nv + c])) +
                               "): the overconstrained mechanism does not fit together");
  // Project the velocities onto the constraints, [M B^T; B 0] [dv; mu] = [0; -B v], until
  // |B v| <= tol (as SOL_InitialConditionManager::adaptVelocities, at most 10 iterations).
  for (int it = 0; active_norm_2(sys_, phid) > s_.vproj_tol; ++it) {
    if (it == 10) throw std::runtime_error("HHT: velocity projection did not converge");
    for (int c = 0; c < sys_.nc(); ++c) rhs[nv + c] = sys_.constraint_active(c) ? -phid[nv + c] : 0.0;
    ls_ic_->factorize(sys_.ic_system(b.data(), phi.data(), phid.data()));
    ls_ic_->solve(rhs.data(), dv.data());
    for (int i = 0; i < nv; ++i) sys_.qd[i] += dv[i];
    sys_.ic_system(b.data(), phi.data(), phid.data());
  }
  if (norm_inf(phid) > 1e-5) {
    int c = -1;
    for (int k = 0; k < sys_.nc(); ++k)
      if (c < 0 || std::abs(phid[nv + k]) > std::abs(phid[nv + c])) c = k;
    throw std::runtime_error("HHT: initial velocities violate the constraints (" + sys_.constraint_name(c) +
                             (sys_.constraint_active(c) ? "" : ", redundant") + ")");
  }
  // Consistent accelerations and Lagrange multipliers.
  const CsrMatrix& A = sys_.ic_system(b.data(), phi.data(), phid.data());
  ls_ic_->factorize(A);
  ls_ic_->solve(b.data(), x_.data());
  std::copy(x_.begin(), x_.begin() + nv, sys_.qdd.begin());
  std::copy(x_.begin() + nv, x_.end(), sys_.lam.begin());
  sys_.residual(c_, r_.data(), f_n_.data());  // f_0
  q_max_.assign(nv, 1.0);
  save_accepted();
  a_n_ = qdd_n_;  // pseudo-acceleration a_0 = qdd_0
}

void HhtIntegrator::save_accepted() {
  sys_.post_step();  // element state of the accepted state
  if (sys_.n_events()) sys_.events(g_n_);
  q_n_ = sys_.q;
  qd_n_ = sys_.qd;
  qdd_n_ = sys_.qdd;
  lam_n_ = sys_.lam;
  // Largest magnitude per velocity dof so far (|q_i| for vector-space elements, as legacy):
  // weights of the error estimate, not needed with newton fixed (fixed step size).
  if (s_.newton == "fixed") return;
  sys_.scales(scale_);
  for (int i = 0; i < sys_.nv(); ++i) q_max_[i] = std::max(q_max_[i], scale_[i]);
}

void HhtIntegrator::restore_accepted() {
  sys_.q = q_n_;
  sys_.qd = qd_n_;
  sys_.qdd = qdd_n_;
  sys_.lam = lam_n_;
  sys_.t = t_;
}

void HhtIntegrator::begin_step(double h) {
  c_.h = h;
  if (ab_) {
    // Lie group form: q = q_n o exp(incr), incr = h (qd_n + h (1/2 - beta) a_n + h beta a_{n+1})
    // with the pseudo-acceleration a_{n+1} = (1+alpha) qdd - alpha qdd_n; qd alike.
    if (gen_alpha_)
      for (int i = 0; i < sys_.nv(); ++i) {
        const double a_known = c2_ * qdd_n_[i] + c3_ * a_n_[i];  // a_n+1 = c1 qdd + a_known
        incr_pred_[i] = h * (qd_n_[i] + h * ((0.5 - beta_) * a_n_[i] + beta_ * a_known));
        qd_pred_[i] = qd_n_[i] + h * ((1. - gamma_) * a_n_[i] + gamma_ * a_known);
      }
    else
      for (int i = 0; i < sys_.nv(); ++i) {
        incr_pred_[i] = h * (qd_n_[i] + h * ((0.5 - beta_) * a_n_[i] - beta_ * alpha_ * qdd_n_[i]));
        qd_pred_[i] = qd_n_[i] + h * ((1. - gamma_) * a_n_[i] - gamma_ * alpha_ * qdd_n_[i]);
      }
  } else {
    // Newmark predictor in the operation order of SOL_DynSolverHHT::applyNewmark.
    const double hh2 = 0.5 * h * h, hg = h * c_.gamma;
    for (int i = 0; i < sys_.nq(); ++i) {
      q_pred_[i] = (q_n_[i] + h * qd_n_[i]) + hh2 * (1. - 2. * c_.beta) * qdd_n_[i];
      qd_pred_[i] = qd_n_[i] + (h - hg) * qdd_n_[i];
    }
  }
  std::copy(qdd_n_.begin(), qdd_n_.end(), x_.begin());
  std::copy(lam_n_.begin(), lam_n_.end(), x_.begin() + sys_.nv());
}

void HhtIntegrator::update_from_x(const double* x) {
  const int nv = sys_.nv();
  const double h = c_.h, bh2 = 0.5 * h * h * c_.beta * 2., gh = h * c_.gamma;
  if (ab_) {
    for (int i = 0; i < nv; ++i) {
      sys_.qdd[i] = x[i];
      sys_.qd[i] = qd_pred_[i] + gh * x[i];
      sys_.incr[i] = incr_pred_[i] + h * h * c_.beta * x[i];
    }
    sys_.update_configuration(q_n_);
  } else {
    for (int i = 0; i < nv; ++i) {
      sys_.qdd[i] = x[i];
      sys_.q[i] = q_pred_[i] + bh2 * x[i];
      sys_.qd[i] = qd_pred_[i] + gh * x[i];
    }
    sys_.apply_modified_newmark(c_);  // legacy option, no-op otherwise
  }
  std::copy(x + nv, x + sys_.nx(), sys_.lam.begin());
}

void HhtIntegrator::residual_at(const std::vector<double>& x, std::vector<double>& r) {
  update_from_x(x.data());
  r.resize(sys_.nx());
  sys_.residual(c_, r.data(), f_.data());
  const double w = c_.alpha / (1.0 + c_.alpha);
  for (int i = 0; i < sys_.nx(); ++i) r[i] -= w * f_n_[i];
}

double HhtIntegrator::q_norm(const double* dqdd) const {
  double n = 0.0;
  for (int i = 0; i < sys_.nv(); ++i) n += std::pow(dqdd[i] / q_max_[i], 2);
  return std::sqrt(n);
}

// SOL_DynSolverHHT::isCorrectionConverged (Negrut et al. Eq. 40).
bool HhtIntegrator::newton_converged_legacy(double h) const {
  const double xi = norm_dx_prev_ > kLegacyEps ? norm_dx_ / norm_dx_prev_ : 1.;
  if (std::abs(xi - 1.) <= kLegacyEps) return true;
  return std::pow(xi / (1. - xi), 2.) * std::pow(norm_dx_, 2.) <=
         std::pow(kAdaptivity, 2.) * psi_newton_ / std::pow(h, 4.);
}

StepResult HhtIntegrator::try_step(double h, double t_new) {
  StepResult res;
  const int nq = sys_.nq(), nv = sys_.nv(), nx = sys_.nx();
  const double bh2 = c_.beta * h * h;
  const bool legacy = s_.newton == "legacy", fixed = s_.newton == "fixed";
  const int max_iterations = fixed ? s_.newton_iterations : s_.max_newton;
  sys_.t = t_new;  // equations at t_{n+1} (restore_accepted resets it)
  begin_step(h);
  residual_at(x_, r_);

  // Residual norms of the predictor: every correction must stay below them (legacy).
  auto e1_norm = [&] { return norm_2(r_.data(), nv); };
  auto e2_norm = [&] {
    double s = 0.0;
    for (int i = nv; i < nx; ++i) s += std::pow(r_[i] * bh2, 2);
    return std::sqrt(s);
  };
  const double e1p = e1_norm(), e2p = e2_norm();
  const double e1_noise = kNoiseFactor * DBL_EPSILON * std::max(1., norm_2(sys_.lam.data(), sys_.nc()));
  const double e2_noise = kNoiseFactor * DBL_EPSILON * std::max(1., norm_2(sys_.q.data(), nq));

  bool converged = false;
  double prev_correction = std::numeric_limits<double>::infinity();
  while (res.iterations < max_iterations && !converged) {
    ++res.iterations;
    try {
      ls_->factorize(sys_.iteration_matrix(c_));
    } catch (const std::exception&) {
      res.status = StepStatus::linear_solver_failed;
      break;
    }
    // Right-hand side as in SOL_DynSolverHHT::constructRightHandSideForUpdate: constraint
    // residuals below machine epsilon are treated as zero.
    for (int i = 0; i < nv; ++i) rhs_[i] = -r_[i];
    for (int i = nv; i < nx; ++i) rhs_[i] = std::abs(r_[i] * bh2) < DBL_EPSILON ? 0. : -r_[i];
    ls_->solve(rhs_.data(), dx_.data());
    for (int i = 0; i < nx; ++i) x_[i] += dx_[i];
    norm_dx_prev_ = norm_dx_;
    norm_dx_ = q_norm(dx_.data());
    if (fixed && res.iterations == max_iterations && c_.alpha == 0.0)
      update_from_x(x_.data());  // last correction: the residual is not needed (no f_n term)
    else
      residual_at(x_, r_);
    if (legacy) {
      const double e1 = e1_norm(), e2 = e2_norm();
      if ((e1 > e1p && e1 > e1_noise) || (e2 > e2p && e2 > e2_noise)) {
        res.status = StepStatus::residual_increased;
        break;
      }
      // At least two corrections, as in the legacy solver.
      converged = res.iterations > 1 && newton_converged_legacy(h);
    } else {
      // The last correction changes qd = qd_pred + gamma h qdd by less than tol relative.
      double dv = 0.0;  // largest velocity change of the correction (accelerations only, not lam)
      for (int i = 0; i < nv; ++i) dv = std::max(dv, std::abs(dx_[i]));
      const double qd_scale = 1.0 + norm_inf(sys_.qd);
      res.correction = c_.gamma * h * dv / qd_scale;
      // fixed: always max_iterations corrections, the work does not depend on the state
      converged = fixed ? res.iterations == max_iterations : c_.gamma * h * dv <= s_.newton_tol * qd_scale;
      // Round-off floor: corrections that stop getting smaller (after 3 iterations) while already
      // below kStagnationFactor * tol are round-off, not divergence (badly scaled models: mm
      // units, massless bodies), and the state is converged as far as it can be.
      if (!fixed && !converged && res.iterations >= 3 && res.correction <= kStagnationFactor * s_.newton_tol &&
          res.correction >= 0.5 * prev_correction)
        converged = true;
      prev_correction = res.correction;
    }
  }
  if (res.status == StepStatus::ok && !converged) res.status = StepStatus::newton_not_converged;

  if (res.status == StepStatus::ok && !fixed) {
    // Local integration error, Negrut et al. Eq. 37a: the Newmark truncation term
    // (beta - 1/(6(1+alpha))) h^2 (qdd - qdd_n) per velocity dof, in the RMS norm with weights
    // eps w_i (psi_ holds nv eps^2): theta <= 1 accepts the step.
    double xn = 0.0;
    if (legacy_error_norm_) {
      // Eq. 35b as legacy FreeDyn: w_i = largest magnitude so far, at least 1.
      for (int i = 0; i < nv; ++i) xn += std::pow((sys_.qdd[i] - qdd_n_[i]) / q_max_[i], 2);
    } else {
      // Absolute and relative tolerance eps: w_i = 1 + max(|s_n|, |s_n+1|) (Hairer, Norsett,
      // Wanner, Solving ODEs I, Sec. II.4), so a coordinate that has once been large (a
      // translation far from the origin) does not loosen its tolerance for the rest of the run.
      sys_.scales(scale_new_);
      for (int i = 0; i < nv; ++i)
        xn += std::pow((sys_.qdd[i] - qdd_n_[i]) / (1.0 + std::max(scale_[i], scale_new_[i])), 2);
    }
    res.theta = xn * std::pow(h, 4.0) / psi_;
    if (s_.adaptive && !(res.theta <= 1.0)) res.status = StepStatus::local_error;
  }
  if (res.status == StepStatus::ok && check_events_) {
    sys_.events(g_trial_);
    bool crossed = false;
    for (size_t i = 0; i < g_trial_.size(); ++i) crossed = crossed || (g_trial_[i] > 0.0) != (g_n_[i] > 0.0);
    if (crossed && !allow_crossing_) res.status = StepStatus::event;
    if (probe_ && res.status == StepStatus::ok) {  // a probe of the event location: never accepted
      restore_accepted();
      return res;
    }
  }
  if (res.status != StepStatus::ok) {
    restore_accepted();
    const bool solver = res.status == StepStatus::newton_not_converged || res.status == StepStatus::linear_solver_failed ||
                        res.status == StepStatus::residual_increased;
    if (solver && !probe_ && !rank_retry_ && s_.newton != "fixed" && recheck_rank()) {
      rank_retry_ = true;
      res = try_step(h, t_new);
      rank_retry_ = false;
    }
    return res;
  }
  // f_{n+1} of the converged state (newton fixed with alpha = 0: stale, and not used)
  std::copy(f_.begin(), f_.end(), f_n_.begin());
  if (ab_)
    for (int i = 0; i < nv; ++i)
      a_n_[i] = gen_alpha_ ? c1_ * sys_.qdd[i] + c2_ * qdd_n_[i] + c3_ * a_n_[i]
                           : (1. + alpha_) * sys_.qdd[i] - alpha_ * qdd_n_[i];
  save_accepted();
  newton_last_ = res.iterations;
  newton_total_ += res.iterations;
  ++n_steps_;
  t_ = t_new;
  reactivate_constraints();
  return res;
}

// Redundant in this position only (a singular position: a stretched linkage), or by the
// structure (a planar loop in space)? The mechanism is moved a little along a motion it can make
// (delta in the null space of the active constraints, [M B^T; B 0] [delta; mu] = [r; 0], at most
// 1e-3 per dof) and assembled there exactly (project_positions): rows redundant by the structure
// stay redundant on the constraint manifold, those of a singular position become independent.
// The latter are made active again once the mechanism has left the position.
void HhtIntegrator::classify_redundant() {
  const int nx = sys_.nx(), nv = sys_.nv();
  const std::vector<double> q0 = sys_.q;
  std::vector<double> b(nx), phi(nx), phid(nx), rhs(nx, 0.0), x(nx), moved(nv, 0.0);
  for (int i = 0; i < nv; ++i) rhs[i] = std::sin(1.0 + 2.4 * i);  // a fixed, generic direction
  ls_ic_->factorize(sys_.ic_system(b.data(), phi.data(), phid.data()));
  ls_ic_->solve(rhs.data(), x.data());
  double big = 0.0;
  for (int i = 0; i < nv; ++i) big = std::max(big, std::abs(x[i]));
  std::vector<int> nearby = report_.redundant;  // no motion (driven entirely): nothing positional
  if (big > 0.0) {
    for (int i = 0; i < nv; ++i) sys_.incr[i] = 1e-3 * x[i] / big;
    sys_.update_configuration(q0);
    std::fill(sys_.incr.begin(), sys_.incr.end(), 0.0);
    try {
      project_positions(moved);
      nearby = sys_.find_redundant_constraints(1e-8, true);
    } catch (const std::exception&) {  // cannot be assembled there: keep them all as structural
    }
    sys_.q = q0;
  }
  for (int c : report_.redundant)
    if (std::find(nearby.begin(), nearby.end(), c) == nearby.end()) candidates_.push_back(c);
  report_.positional = candidates_;
}

bool HhtIntegrator::recheck_rank() {
  const std::vector<int> found = sys_.find_redundant_constraints(1e-6);
  if (found.empty()) return false;
  if (sys_.deactivate_constraints(found)) make_solvers();
  for (int c : found) {
    if (std::find(candidates_.begin(), candidates_.end(), c) == candidates_.end()) candidates_.push_back(c);
    rank_log_.push_back({t_, c, -1, 0.0});
  }
  return true;
}

void HhtIntegrator::reactivate_constraints() {
  std::vector<int> off;
  for (int c : candidates_)
    if (!sys_.constraint_active(c)) off.push_back(c);
  if (off.empty()) return;
  const std::vector<int> dependent = sys_.find_redundant_constraints(1e-3, true);
  std::vector<int> back;
  for (int c : off)
    if (std::find(dependent.begin(), dependent.end(), c) == dependent.end()) back.push_back(c);
  if (back.empty()) return;
  const int nx = sys_.nx();
  std::vector<double> b(nx), phi(nx), phid(nx);
  sys_.ic_system(b.data(), phi.data(), phid.data());
  sys_.activate_constraints(back);
  for (int c : back) rank_log_.push_back({t_, c, 1, std::abs(phi[sys_.nv() + c])});
}

StepResult HhtIntegrator::try_step_to_event(double h, double t_new, double snap) {
  if (sys_.n_events() == 0 || s_.newton == "fixed") return try_step(h, t_new);
  const double t0 = t_;
  const std::vector<double> g0 = g_n_;
  struct Mode {  // the modes of try_step, reset on every way out
    HhtIntegrator& i;
    ~Mode() { i.check_events_ = i.probe_ = i.allow_crossing_ = false; }
  } mode{*this};
  check_events_ = true;
  StepResult r = try_step(h, t_new);
  if (r.status != StepStatus::event) return r;  // accepted without a crossing, or failed
  // Bracket [lo, hi] of step lengths from t0: no crossing at lo, a crossing at hi. Regula falsi
  // on the earliest crossing, with the Illinois modification (the value at an end kept twice is
  // halved), so the bracket shrinks from both sides.
  double lo = 0.0, hi = h, w_lo = 1.0, w_hi = 1.0;
  std::vector<double> g_lo = g0, g_hi = g_trial_;
  enum { NONE, LO, HI } last = NONE;
  probe_ = true;
  for (int it = 0; hi - lo > s_.event_tol; ++it) {
    if (it == 60) throw std::runtime_error("HHT: event location did not converge at t = " + std::to_string(t0));
    double tau = hi;
    for (size_t i = 0; i < g0.size(); ++i)
      if ((g_lo[i] > 0.0) != (g_hi[i] > 0.0)) {
        const double a = w_lo * g_lo[i], b = w_hi * g_hi[i];
        tau = std::min(tau, lo + (hi - lo) * a / (a - b));
      }
    tau = std::min(std::max(tau, lo + 1e-3 * (hi - lo)), hi - 1e-3 * (hi - lo));
    ++event_probes_;
    r = try_step(tau, t0 + tau);
    if (r.status == StepStatus::event) {
      hi = tau, g_hi = g_trial_, w_hi = 1.0;
      if (last == HI) w_lo *= 0.5;
      last = HI;
    } else if (r.status == StepStatus::ok) {
      lo = tau, g_lo = g_trial_, w_lo = 1.0;
      if (last == LO) w_hi *= 0.5;
      last = LO;
    } else {
      return r;  // a probe failed (restored to t0)
    }
  }
  if (h - hi <= snap) hi = h;
  probe_ = false;
  allow_crossing_ = true;
  r = try_step(hi, hi == h ? t_new : t0 + hi);
  if (r.status == StepStatus::ok)
    for (size_t i = 0; i < g0.size(); ++i)
      if ((g0[i] > 0.0) != (g_n_[i] > 0.0)) event_log_.push_back({t_, static_cast<int>(i), g_n_[i] > 0.0 ? 1 : -1});
  return r;
}

StepResult HhtIntegrator::advance_to(double t_target) {
  StepResult r;
  int iterations = 0;
  double h_sub = s_.step;
  bool subdivided = false;
  const double tiny = 1e-9 * s_.step;
  // steps of settings.step, the last one to t_target (one that is up to 1 % longer rather than a
  // very short one after it). Events end steps early; the rest follows (a crossing in the last
  // percent of a step keeps the full step). A step Newton does not converge is halved (down to
  // step / 1024; not in the real-time mode newton fixed, whose work per step is fixed): a
  // friction contact that reverses its slip within a step.
  while (t_ < t_target - tiny) {
    const bool last = t_ + 1.01 * h_sub >= t_target - tiny;
    const double h = last ? t_target - t_ : h_sub;
    r = try_step_to_event(h, last ? t_target : t_ + h, 1e-2 * h);
    if (r.status != StepStatus::ok) {
      if (s_.newton == "fixed" || h_sub < s_.step / 1024.0)
        throw std::runtime_error(std::string("HHT: step failed at t = ") + std::to_string(t_ + h) + ": " +
                                 to_string(r.status));
      h_sub *= 0.5, subdivided = true;
      continue;
    }
    iterations += r.iterations;
  }
  subdivided_steps_ += subdivided;
  r.iterations = iterations;
  return r;
}

StepResult HhtIntegrator::step() {
  const StepResult r = advance_to(static_cast<double>(grid_steps_ + 1) * s_.step);
  ++grid_steps_;
  return r;
}

long LegacyTimeStepper::run(const std::function<void(double)>& on_output,
                            const std::function<void(double, double, const StepResult&)>& on_attempt,
                            const std::function<void(double)>& before_step) {
  // Output times as in SIM_SimulationObject::setEndTime / computeNextStep.
  const double t_final = s_.end_time, dt_out = s_.output_step;
  double t_out = 0. + dt_out;
  if (t_out < s_.output_begin) t_out = s_.output_begin;
  if (std::abs(t_out - t_final) < s_.h_min) t_out = t_final;

  if (before_step) before_step(0.0);
  hht_.initialize();
  on_output(0.0);
  // SOL_SimTimeManagerHHT: m_dH = hmax, m_dTheta = 1, m_dS = 0.9.
  double t = 0.0, h = s_.step, theta = 1.0;
  bool last_ok = true, first_interval = true;
  long accepted = 0;
  rejected_ = 0;
  for (;;) {
    if (last_ok)
      h = theta > 0 ? 0.9 * h / std::pow(theta, 1.0 / 6.0) : s_.step;
    else
      h = h / 2.0;
    if (h > s_.step) h = s_.step;
    if (s_.exact_output && (1.1 * h) > (t_out - t)) {
      h = t_out - t;
      if (h < s_.h_min) h = s_.h_min;
    }
    t += h;
    const bool last_in_interval = t >= t_out;

    if (before_step) before_step(t);
    const StepResult r = hht_.try_step(h, t);
    if (on_attempt) on_attempt(t, h, r);
    last_ok = r.status == StepStatus::ok;
    if (!last_ok) {
      ++rejected_;
      t = t - h;  // resetToLastTimeStep
    } else {
      theta = r.theta;
      ++accepted;
      if (last_in_interval) {
        on_output(t);
        // SIM_SimulationObject::computeNextStep. After create() the output time holds a
        // sentinel, so the second output time is based on the actual solver time.
        if (!(t + 1e-14 < t_final)) return accepted;
        const double base = first_interval ? t : t_out;
        first_interval = false;
        if ((1.1 * dt_out) > (t_final - base))
          t_out = t_final;
        else
          t_out = base + dt_out;
        if (t_out > t_final) t_out = t_final;
      }
    }
    if (h < s_.h_min)
      throw std::runtime_error("HHT: step size below the minimum step size at t = " + std::to_string(t));
  }
}

long PiTimeStepper::run(const std::function<void(double)>& on_output,
                        const std::function<void(double, double, const StepResult&)>& on_attempt,
                        const std::function<void(double)>& before_step) {
  constexpr double kOrder = 3.0, kSafety = 0.9, kMin = 0.2, kMax = 5.0;
  const double t_final = s_.end_time, dt_out = s_.output_step, h_max = s_.step;
  const double t_tiny = 1e-12 * std::max(1.0, t_final);
  // output times k dt_out (from output_begin on) and t_final, each computed from k (no drift)
  long k_out = 1;
  while (k_out * dt_out < s_.output_begin - t_tiny) ++k_out;
  auto next_output = [&] { return std::min(k_out * dt_out, t_final); };

  if (before_step) before_step(0.0);
  hht_.initialize();
  on_output(0.0);
  double t = 0.0, h = h_max, err_prev = 1.0;
  bool rejected_last = false;
  long accepted = 0;
  rejected_ = 0;
  for (;;) {
    const double t_out = next_output();
    double h_try = std::min(h, h_max);
    bool lands = false;
    if (t + h_try >= t_out - t_tiny) {
      h_try = t_out - t, lands = true;
    } else if (t + 2.0 * h_try > t_out) {
      h_try = 0.5 * (t_out - t);  // two equal steps instead of a long and a very short one
    }
    if (h_try < s_.h_min)
      throw std::runtime_error("HHT: step size below the minimum step size at t = " + std::to_string(t));
    const double t_new = lands ? t_out : t + h_try;
    if (before_step) before_step(t_new);
    const StepResult r = hht_.try_step_to_event(h_try, t_new);
    if (on_attempt) on_attempt(hht_.time(), h_try, r);
    if (r.status != StepStatus::ok) {
      ++rejected_;
      const double err = std::sqrt(r.theta);
      h = h_try * (r.status == StepStatus::local_error ? std::max(kMin, kSafety * std::pow(err, -1.0 / kOrder)) : 0.25);
      rejected_last = true;
      continue;
    }
    ++accepted;
    const double err = std::max(std::sqrt(r.theta), 1e-10);
    double factor = kSafety * std::pow(err, -0.7 / kOrder) * std::pow(err_prev, 0.4 / kOrder);
    factor = std::min(std::max(factor, kMin), rejected_last ? 1.0 : kMax);
    err_prev = err;
    rejected_last = false;
    // a step that ended at an event (try_step_to_event) is shorter than tried, and did not land
    const double taken = hht_.time() - t;
    const bool early = taken < h_try - t_tiny;
    lands = lands && !early;
    // a step shortened to land on an output time, or ended at an event, does not shorten the next
    h = lands || early ? std::max(h, taken * factor) : h_try * factor;
    t = lands ? t_new : hht_.time();
    if (lands) {
      on_output(t);
      if (t >= t_final - t_tiny) return accepted;
      ++k_out;
    }
  }
}

}  // namespace fd
