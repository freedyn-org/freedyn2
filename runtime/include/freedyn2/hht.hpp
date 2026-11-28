#pragma once
// HHT-alpha / generalized-alpha integrator for index-3 DAEs of multibody dynamics with Newton
// correction using the generated iteration matrix. Generalized-alpha (SolverSettings::rho_inf,
// Chung and Hulbert 1993) runs in the arnold_bruls form; HHT in two forms:
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

// event: an event function changed sign within the step (only inside try_step_to_event).
enum class StepStatus { ok, newton_not_converged, residual_increased, local_error, linear_solver_failed, event };

// A located event: event function `index` (System::event_name) changed sign at time t, to
// positive (direction +1: a contact begins) or to negative (-1: it ends).
// A constraint switched during the run because of its position (a singular position of a
// mechanism): deactivated (-1) where a step failed and it was redundant, or active again (+1)
// once independent; violation: |Phi| when it became active again.
struct RankChange {
  double t;
  int constraint, direction;
  double violation;
};

struct EventRecord {
  double t;
  int index, direction;
};
const char* to_string(StepStatus s);

struct StepResult {
  StepStatus status = StepStatus::ok;
  int iterations = 0;
  double theta = 0.0;  // local error estimate (Negrut et al. Eq. 37a), <= 1 accepted
  // Velocity change of the last Newton correction, gamma h |dqdd|_inf / (1 + |qd|_inf):
  // how far from converged the step ended (newton = fixed: the quantity to monitor).
  double correction = 0.0;
};

// What initialize() did to reach a consistent initial state.
struct InitialReport {
  std::vector<int> redundant;   // constraints found redundant and deactivated (System::constraint_name)
  std::vector<int> positional;  // of them, redundant in this position only (independent nearby)
  double violation = 0.0;       // largest |Phi| of the active constraints in the given positions
  int assembly_iterations = 0;  // Newton corrections of the positions (0: consistent as given)
  double largest_move = 0.0;    // largest change of a coordinate by the assembly (velocity dof units)
  int largest_move_dof = -1;    // its velocity dof
  int static_iterations = 0;    // corrections of the static equilibrium (SolverSettings::statics)
  double static_move = 0.0;     // largest change of a coordinate by it
  int static_move_dof = -1;
};

class HhtIntegrator {
 public:
  HhtIntegrator(System& sys, const SolverSettings& s, const std::string& linear_solver = "sparselu");

  // Set q, qd from the model and solve for consistent qdd and lam at t = 0: redundant
  // constraints are deactivated (lam = 0, the others carry the load), positions that violate
  // the constraints are assembled (Newton, the smallest correction in the mass metric:
  // [M B^T; B 0] [delta; mu] = [0; -Phi]), with SolverSettings::statics moved into the static
  // equilibrium at rest, velocities projected onto the constraints.
  void initialize();
  const InitialReport& initial_report() const { return report_; }
  // Constraints switched during the run (singular positions): see try_step.
  const std::vector<RankChange>& rank_changes() const { return rank_log_; }
  // Try one step of size h from the last accepted state, ending at time t_new. On
  // success the new state is accepted, otherwise the last accepted state is restored.
  // Singular positions (not with newton fixed): a step that fails in Newton or the linear solver
  // has the rank of the constraints checked again at the accepted state (tolerance 1e-6); newly
  // redundant constraints are deactivated and the step is tried once more. Constraints that are
  // redundant in a position only (at the start, or deactivated so) are active again after the
  // accepted step at which they are clearly independent (1e-3: a hysteresis).
  StepResult try_step(double h, double t_new);
  // The step of size h to t_new with event location (System::events): a step in which an event
  // function changes sign is not accepted; the time of the sign change is located by the
  // Illinois method (regula falsi) on probe steps from the last accepted state, to
  // SolverSettings::event_tol, and the step to just past it is accepted, so the step ends
  // early (time()). A crossing within `snap` of the end keeps the full step. Without event
  // functions, and with newton fixed (the same work in every step), it is try_step.
  StepResult try_step_to_event(double h, double t_new, double snap = 0.0);
  const std::vector<EventRecord>& event_log() const { return event_log_; }
  long event_probes() const { return event_probes_; }  // probe steps of the event location
  long subdivided_steps() const { return subdivided_steps_; }  // grid steps halved (step())
  // Steps of at most settings.step from time() to t_target, the last one ending there (a
  // communication step of a co-simulation, fmu/); events and failed steps as for step().
  StepResult advance_to(double t_target);
  // Fixed step of size settings.step to the next point of the grid k step; events end steps
  // early, then the rest of the grid step follows; a step Newton does not converge is halved
  // (down to step / 1024, then it throws; with newton fixed it throws at once).
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
  double alpha_ = 0, beta_ = 0, gamma_ = 0;  // HHT parameters (generalized-alpha: its beta, gamma)
  bool gen_alpha_ = false;                   // generalized-alpha (SolverSettings::rho_inf)
  double am_ = 0, af_ = 0;                   // its alpha_m, alpha_f (HHT: 0, -alpha)
  double c1_ = 1, c2_ = 0, c3_ = 0;          // a_n+1 = c1 qdd_n+1 + c2 qdd_n + c3 a_n
  HhtCoef c_{};                               // effective coefficients of the generated functions
  double psi_ = 0.0;         // error scale nv eps^2 / C^2 of the local error C h^2 (qdd - qdd_n)
  double psi_newton_ = 0.0;  // Negrut's, for the legacy Newton rule (Eq. 40)
  std::unique_ptr<LinearSolver> ls_, ls_ic_;
  std::string ls_name_;
  InitialReport report_;
  void make_solvers();
  void assemble_positions();
  void classify_redundant();    // initialize: redundant by the position, not the structure
  bool recheck_rank();          // after a failed step: deactivate newly redundant constraints
  void reactivate_constraints();  // after an accepted step: positional ones independent again
  std::vector<int> candidates_;   // constraints redundant in a position only
  std::vector<RankChange> rank_log_;
  bool rank_retry_ = false;
  // Newton corrections of the positions onto Phi = 0 in the mass metric (active constraints),
  // accumulated per velocity dof in moved; |Phi| before in *violation. Throws if they fail.
  int project_positions(std::vector<double>& moved, double* violation = nullptr);
  void static_equilibrium();
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
  // events: values at the accepted state and at the end of the trial step; modes of try_step
  std::vector<double> g_n_, g_trial_;
  bool check_events_ = false, probe_ = false, allow_crossing_ = false;
  std::vector<EventRecord> event_log_;
  long event_probes_ = 0, grid_steps_ = 0, subdivided_steps_ = 0;
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

// Step size control for accuracy (controller = pi): the PI controller of Gustafsson (1991) as in
// Hairer and Wanner, Solving ODEs II, IV.8, on the local error estimate of the integrator
// (err = sqrt(theta) <= 1 accepts): h_new = h 0.9 err_n^(-0.7/k) err_n-1^(0.4/k), k = 3 (the
// local error is O(h^3)), the factor within [0.2, 5] and not above 1 after a rejected step. A
// step that Newton does not converge is retried with a quarter. Steps end exactly at the output
// times (k output_step from output_begin on, and end_time); a step shortened to reach one does
// not shorten the next. step is the largest step, h_min the smallest.
class PiTimeStepper {
 public:
  PiTimeStepper(HhtIntegrator& hht, const SolverSettings& s) : hht_(hht), s_(s) {}
  // As LegacyTimeStepper::run.
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
