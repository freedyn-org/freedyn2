#include "freedyn2/hht.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
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
constexpr double kAdaptivity = 1e-3;  // c in Negrut et al. Eq. 40
}  // namespace

const char* to_string(StepStatus s) {
  switch (s) {
    case StepStatus::ok: return "ok";
    case StepStatus::newton_not_converged: return "Newton not converged";
    case StepStatus::residual_increased: return "residual increased";
    case StepStatus::local_error: return "local error too large";
    case StepStatus::linear_solver_failed: return "linear solver failed";
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
  // Same expressions as SOL_DynSolverHHTParameter.
  alpha_ = s.alpha;
  beta_ = 0.25 * (1. - s.alpha) * (1. - s.alpha);
  gamma_ = 0.5 * (1. - 2. * s.alpha);
  // Coefficients passed to the generated functions. Negrut: r_dyn = M qdd/(1+alpha) + f
  // - alpha/(1+alpha) f_n. Arnold-Bruls: r_dyn = M qdd + f at t_{n+1} with q, qd depending on
  // qdd through the pseudo-acceleration a = (1+alpha) qdd - alpha qdd_n, i.e. with
  // dq/dqdd = (1+alpha) beta h^2 and dqd/dqdd = (1+alpha) gamma h. The same generated
  // iteration matrix serves both forms.
  c_.alpha = ab_ ? 0. : alpha_;
  c_.beta = ab_ ? (1. + alpha_) * beta_ : beta_;
  c_.gamma = ab_ ? (1. + alpha_) * gamma_ : gamma_;
  c_.h = s.step;
  if (!ab_ && !sys.vector_space())
    throw std::runtime_error("HHT: the negrut form needs vector-space elements (e.g. rigid_body_euler)");
  if (s.newton == "fixed" && s.adaptive)
    throw std::runtime_error("HHT: newton fixed is meant for a fixed step size (adaptive off)");
  if (ab_ && sys.modified_newmark())
    throw std::runtime_error("HHT: modified Newmark formulas are a legacy option of the negrut form");
  psi_ = sys.nv() * std::pow(s.eps, 2) / std::pow(beta_ - 1. / (6. * (1. + alpha_)), 2);
  ls_ = make_linear_solver(linear_solver);
  ls_->set_fixed_pivots(s.newton == "fixed");  // real-time: the same work in every step
  ls_ic_ = make_linear_solver(linear_solver);
  const int nx = sys.nx(), nq = sys.nq(), nv = sys.nv();
  for (auto* v : {&q_n_, &q_pred_}) v->resize(nq);
  for (auto* v : {&qd_n_, &qdd_n_, &a_n_, &qd_pred_, &incr_pred_}) v->resize(nv);
  lam_n_.resize(sys.nc());
  q_max_.assign(nv, 1.0);
  scale_new_.resize(nv);
  for (auto* v : {&f_n_, &x_, &dx_, &r_, &f_, &rhs_}) v->resize(nx);
  sys.set_newmark_reference(&q_n_, &qd_n_, &qdd_n_);
}

void HhtIntegrator::initialize() {
  sys_.set_initial_state();
  t_ = 0.0;
  sys_.t = 0.0;
  n_steps_ = 0;
  const int nx = sys_.nx(), nv = sys_.nv();
  std::fill(sys_.incr.begin(), sys_.incr.end(), 0.0);
  std::vector<double> b(nx), phi(nx), phid(nx), rhs(nx, 0.0), dv(nx);
  sys_.ic_system(b.data(), phi.data(), phid.data());
  if (norm_inf(phi) > 1e-10)
    throw std::runtime_error("HHT: initial positions violate the constraints");
  // Project the velocities onto the constraints, [M B^T; B 0] [dv; mu] = [0; -B v], until
  // |B v| <= tol (as SOL_InitialConditionManager::adaptVelocities, at most 10 iterations).
  for (int it = 0; norm_2(phid.data(), nx) > s_.vproj_tol; ++it) {
    if (it == 10) throw std::runtime_error("HHT: velocity projection did not converge");
    for (int i = nv; i < nx; ++i) rhs[i] = -phid[i];
    ls_ic_->factorize(sys_.ic_system(b.data(), phi.data(), phid.data()));
    ls_ic_->solve(rhs.data(), dv.data());
    for (int i = 0; i < nv; ++i) sys_.qd[i] += dv[i];
    sys_.ic_system(b.data(), phi.data(), phid.data());
  }
  if (norm_inf(phid) > 1e-5) throw std::runtime_error("HHT: initial velocities violate the constraints");
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
         std::pow(kAdaptivity, 2.) * psi_ / std::pow(h, 4.);
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
  if (res.status != StepStatus::ok) {
    restore_accepted();
    return res;
  }
  // f_{n+1} of the converged state (newton fixed with alpha = 0: stale, and not used)
  std::copy(f_.begin(), f_.end(), f_n_.begin());
  if (ab_)
    for (int i = 0; i < nv; ++i) a_n_[i] = (1. + alpha_) * sys_.qdd[i] - alpha_ * qdd_n_[i];
  save_accepted();
  newton_last_ = res.iterations;
  newton_total_ += res.iterations;
  ++n_steps_;
  t_ = t_new;
  return res;
}

StepResult HhtIntegrator::step() {
  const StepResult r = try_step(s_.step, static_cast<double>(n_steps_ + 1) * s_.step);
  if (r.status != StepStatus::ok)
    throw std::runtime_error(std::string("HHT: step failed at t = ") + std::to_string(t_ + s_.step) +
                             ": " + to_string(r.status));
  return r;
}

long LegacyTimeStepper::run(const std::function<void(double)>& on_output,
                            const std::function<void(double, double, const StepResult&)>& on_attempt) {
  // Output times as in SIM_SimulationObject::setEndTime / computeNextStep.
  const double t_final = s_.end_time, dt_out = s_.output_step;
  double t_out = 0. + dt_out;
  if (t_out < s_.output_begin) t_out = s_.output_begin;
  if (std::abs(t_out - t_final) < s_.h_min) t_out = t_final;

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

}  // namespace fd
