#pragma once
// Global multibody system: state, element instances and scatter assembly.
//
// Configuration q is stored with nq numbers (e.g. R and a unit quaternion per Lie group
// rigid body), velocities qd and accelerations qdd have nv entries (velocity dofs, e.g.
// Rdot and the body angular velocity). For vector-space elements nq = nv and qd = dq/dt.

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "freedyn2/crg.hpp"
#include "freedyn2/generated_function.hpp"
#include "freedyn2/model.hpp"
#include "freedyn2/scatter.hpp"

namespace fd {

// Integration coefficients passed to generated functions, order fixed by the generator.
struct HhtCoef {
  double alpha, beta, gamma, h;
};

class System {
 public:
  explicit System(const Model& model);

  int nq() const { return nq_; }        // stored configuration numbers
  int nv() const { return nv_; }        // velocity dofs
  int nc() const { return nc_; }
  int nx() const { return nv_ + nc_; }  // unknowns x = [qdd; lam]
  bool vector_space() const { return vector_space_; }  // all elements have nq = nv, qd = dq/dt
  bool modified_newmark() const { return modified_newmark_; }

  // State. Unknowns of the Newton iteration are qdd and lam. incr is the configuration
  // increment of the current step (q = q_n o exp(incr)), an input of the iteration matrix.
  std::vector<double> q, qd, qdd, lam, incr;
  double t = 0.0;  // time of the state, input of all res / iter / ic functions

  // q from the model, qd from each element's init function.
  void set_initial_state();
  // q = q_n o exp(incr) with the elements' generated update functions.
  void update_configuration(const std::vector<double>& q_n);
  // Element state after an accepted step (generated post functions update their parameters).
  void post_step();
  // Magnitude per velocity dof (generated scale functions), for weighted norms.
  void scales(std::vector<double>& s);

  // r: scaled HHT residual (without the f_n term), f = h + Phi_q^T lam - Q. Both size nx.
  void residual(const HhtCoef& c, double* r, double* f);
  // HHT Newton iteration matrix with respect to x.
  const CsrMatrix& iteration_matrix(const HhtCoef& c);
  // Consistent acceleration system A x = b, plus Phi and Phi_q qd (rows of x space).
  const CsrMatrix& ic_system(double* b, double* phi, double* phid);

  // Legacy modified Newmark formulas (vector space, elements with newmark = modified): the
  // last accepted state the formulas start from, set by the integrator.
  void set_newmark_reference(const std::vector<double>* q_n, const std::vector<double>* qd_n,
                             const std::vector<double>* qdd_n) {
    ref_q_n_ = q_n, ref_qd_n_ = qd_n, ref_qdd_n_ = qdd_n;
  }
  // Overwrite qd of those elements by the modified formulas at the current q, qdd.
  void apply_modified_newmark(const HhtCoef& c);

  // Runtime inputs (Model::inputs): parameters of the elements that use them, changed between
  // two steps (constant inside a step, also inside its Newton iterations). An input no element
  // uses can be set but has no effect.
  int n_inputs() const { return static_cast<int>(input_names_.size()); }
  const std::vector<std::string>& input_names() const { return input_names_; }
  // Index of the input, -1 if the model has no input of that name.
  int input_index(const std::string& name) const;
  void set_input(int index, double value);
  double input(int index) const { return input_values_.at(index); }

  // Output channels of all elements (generated out functions).
  const std::vector<std::string>& output_names() const { return out_names_; }
  void outputs(std::vector<double>& y);
  // Index of the first channel named "<element>.<output>", -1 if absent.
  int output_index(const std::string& name) const;

 private:
  struct Instance {
    Instance(const ElementSpec& s, const fd_element_type& t);
    ElementSpec spec;  // copy, the System does not keep a reference to the Model
    GenFunction res, iter, ic;
    // Elements that own their coordinates (bodies) have init, update and scale; joints only
    // constrain coordinates of other elements. out is optional (else lambda is output).
    std::unique_ptr<GenFunction> init, out, update, scale;
    std::unique_ptr<GenFunction> mn_qd, jac_qd;  // modified Newmark only
    std::unique_ptr<GenFunction> post;           // element state update (e.g. continuous angles)
    bool owner() const { return static_cast<bool>(init); }
    std::vector<int> coords;  // local stored coordinate -> global q index
    std::vector<int> map;     // local x index -> global x index
    int nq = 0, nv = 0, nc = 0;
    std::vector<double> q, qd, qdd, lam, incr, q_n, qd_n, qdd_n;
    std::vector<std::vector<double>> res_buf, ic_buf, out_buf;
    std::vector<int> res_rows[2], ic_rows[4];  // scatter maps of vector outputs
    // Work buffers, allocated once: a time step allocates no memory (except the legacy
    // modified Newmark option).
    std::vector<double> iter_buf, mn_qd_buf, mn_d_buf, jac_qd_buf, post_buf, scale_buf;
    std::vector<double*> out_ptr;
    int iter_block = -1, ic_block = -1;
  };

  void gather(Instance& e);
  // Road plane of the *_local tires at their current wheel position (before res / ic / out).
  void update_roads();
  // Inputs of res / iter in the order fixed by the generator (fdgen.lagrange.STD_INPUTS).
  std::array<const double*, 8> std_inputs(const Instance& e, const double* coef) const {
    return {e.q.data(), e.qd.data(), e.qdd.data(), e.lam.data(), e.spec.p.data(), coef, e.incr.data(), &t};
  }
  // Modified Newmark formulas of e from the reference state at its gathered q, qdd.
  void eval_modified_newmark(Instance& e, const double* coef);
  static std::unique_ptr<GenFunction> optional(const fd_function& f) {
    return f.eval ? std::make_unique<GenFunction>(f) : nullptr;
  }
  void modified_jacobian(Instance& e, const HhtCoef& c);
  static void alloc(GenFunction& f, std::vector<std::vector<double>>& buf);

  int nq_, nv_, nc_;
  bool vector_space_ = true, modified_newmark_ = false;
  std::vector<Instance> elems_;
  MatrixAssembler iter_asm_, ic_asm_;
  std::vector<std::string> out_names_, out_base_;
  const std::vector<double>*ref_q_n_ = nullptr, *ref_qd_n_ = nullptr, *ref_qdd_n_ = nullptr;
  struct RoadUser {
    int elem, p;  // element, offset of its road plane in p
    int point;    // query point of fd::Road (search history of this wheel)
  };
  std::unique_ptr<Road> road_;
  std::vector<RoadUser> road_users_;
  struct InputSlot {
    int elem, p;  // element, offset in its p
  };
  std::vector<std::string> input_names_;
  std::vector<double> input_values_;
  std::vector<std::vector<InputSlot>> input_slots_;  // per input
};

}  // namespace fd
