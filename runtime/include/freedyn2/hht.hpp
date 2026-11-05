#pragma once
// HHT-alpha integrator for index-3 DAEs of multibody dynamics with Newton correction
// using the generated iteration matrix, in two forms:
// - arnold_bruls (default): equations of motion at t_{n+1}, alpha only in the Newmark
//   pseudo-acceleration (generalized-alpha with alpha_m = 0, Arnold & Bruls 2007), second order,
//   on Lie groups: q_{n+1} = q_n o exp(incr) with the elements' update functions
//   (Bruls, Cardona, Arnold 2012), so rotations are integrated with the angular velocity;
// - negrut: force-weighted form of Negrut, Rampalli, Ottarsson, Sandu 2007, as in the legacy
//   FreeDyn kernel; first order for configuration-dependent mass matrices when alpha < 0.
// One step of a given size is tried at a time; the step size control lives in the drivers
// (fixed step, LegacyTimeStepper).

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "freedyn2/linear_solver.hpp"
#include "freedyn2/system.hpp"

namespace fd {

enum class StepStatus { ok, newton_not_converged, residual_increased, local_error, linear_solver_failed };
const char* to_string(StepStatus s);

struct StepResult {
  StepStatus status = StepStatus::ok;
  int iterations = 0;
  double theta = 0.0;  // local error estimate (Negrut et al. Eq. 37a), <= 1 accepted
  // Velocity change of the last Newton correction, gamma h |dqdd|_inf / (1 + |qd|_inf):
  // how far from converged the step ended (newton = fixed: the quantity to monitor).
  double correction = 0.0;
};

class HhtIntegrator {
 public:
  HhtIntegrator(System& sys, const SolverSettings& s, const std::string& linear_solver = "sparselu");

  // Set q, qd from the model and solve for consistent qdd and lam at t = 0.
  void initialize();
  // Try one step of size h from the last accepted state, ending at time t_new. On
  // success the new state is accepted, otherwise the last accepted state is restored.
  StepResult try_step(double h, double t_new);
  // Fixed step of size settings.step (throws if the step fails).
  StepResult step();

  double time() const { return t_; }
  const HhtCoef& coef() const { return c_; }
  long newton_iterations() const { return newton_total_; }
  int last_newton_iterations() const { return newton_last_; }
  long full_factorizations() const { return ls_->full_factorizations(); }  // new pivot orders so far

  // Scaled HHT residual for given x (used by tests): sets qdd, lam from x and q, qd
  // from the Newmark formulas of the step prepared by begin_step(h).
  void residual_at(const std::vector<double>& x, std::vector<double>& r);
  void begin_step(double h);

 private:
  void update_from_x(const double* x);
  void save_accepted();
  void restore_accepted();
  bool newton_converged_legacy(double h) const;
  double q_norm(const double* dqdd) const;

  System& sys_;
  SolverSettings s_;
  bool ab_ = true;                          // Arnold-Bruls form, else Negrut
  bool legacy_error_norm_ = false;          // error weights of legacy FreeDyn (SolverSettings::error_norm)
  double alpha_ = 0, beta_ = 0, gamma_ = 0;  // HHT parameters
  HhtCoef c_;                               // effective coefficients of the generated functions
  double psi_ = 0.0;  // Negrut's error scale n eps^2 / (beta - 1/(6(1+alpha)))^2
  std::unique_ptr<LinearSolver> ls_, ls_ic_;
  double t_ = 0.0;
  long n_steps_ = 0;
  long newton_total_ = 0;
  int newton_last_ = 0;
  // Last accepted state, Newmark predictors, f_n for the alpha term, max |q_i| so far.
  std::vector<double> q_n_, qd_n_, qdd_n_, lam_n_, f_n_, q_max_;
  std::vector<double> a_n_;  // Newmark pseudo-acceleration (Arnold-Bruls)
  std::vector<double> q_pred_, qd_pred_, incr_pred_;
  std::vector<double> scale_, scale_new_;  // element scales at the accepted and the trial state
  std::vector<double> x_, dx_, r_, f_, rhs_;
  double norm_dx_ = 0.0, norm_dx_prev_ = 0.0;
};

// Step size and output time control of the legacy FreeDyn HHT solver
// (SOL_SimTimeManagerHHT and SIM_SimulationObject), reproduced operation by operation
// so that both programs take the same steps.
class LegacyTimeStepper {
 public:
  LegacyTimeStepper(HhtIntegrator& hht, const SolverSettings& s) : hht_(hht), s_(s) {}
  // Integrate to end_time, calling on_output(t) at t = 0 and at every output time and
  // on_attempt(t, h, result) after every step attempt (accepted or rejected).
  // Returns the number of accepted steps; throws if the step size gets too small.
  // before_step(t) is called with t = 0 before the initial state is solved and with the end
  // time of every step attempt before it (runtime inputs: System::set_input).
  long run(const std::function<void(double)>& on_output,
           const std::function<void(double, double, const StepResult&)>& on_attempt = nullptr,
           const std::function<void(double)>& before_step = nullptr);
  long rejected_steps() const { return rejected_; }

 private:
  HhtIntegrator& hht_;
  SolverSettings s_;
  long rejected_ = 0;
};

}  // namespace fd
