#include "freedyn2/system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fd {

namespace {
// Input order of res / iter / ic, fixed by the generator (fdgen.lagrange.STD_INPUTS).
enum { IN_Q, IN_QD, IN_QDD, IN_LAM, IN_P };
const char* const AXES = "xyz";

const fd_function& required(const fd_function& f, const std::string& type, const char* role) {
  if (!f.eval) throw std::runtime_error("element type " + type + " has no " + role + " function");
  return f;
}
}  // namespace

System::Instance::Instance(const ElementSpec& s, const fd_element_type& t)
    : spec(s),
      res(required(t.res, s.type, "res")),
      iter(required(s.jacobian == "no_qvv" ? t.iter_noqv : t.iter, s.type, s.jacobian == "no_qvv" ? "iter_noqv" : "iter")),
      ic(required(t.ic, s.type, "ic")),
      init(optional(t.init)),
      out(optional(t.out)),
      update(optional(t.update)),
      scale(optional(t.scale)),
      post(optional(t.post)),
      spline(optional(t.spline)),
      event(optional(t.event)) {
  if (init && !(update && scale))
    throw std::runtime_error("element type " + s.type + ": init without update and scale functions");
  if (s.newmark == "modified") {
    mn_qd = std::make_unique<GenFunction>(required(t.mn_qd, s.type, "mn_qd (modified Newmark)"));
    // With the iteration matrix without derivatives of h there is no qd derivative to correct.
    if (s.jacobian != "no_qvv") jac_qd = std::make_unique<GenFunction>(required(t.jac_qd, s.type, "jac_qd"));
  }
}

void System::alloc(GenFunction& f, std::vector<std::vector<double>>& buf) {
  buf.resize(f.n_out());
  for (int i = 0; i < f.n_out(); ++i) buf[i].assign(f.sparsity_out(i).nnz(), 0.0);
}

System::System(const Model& model) : nq_(model.nq()), nv_(model.nv()), nc_(model.nc()) {
  for (const InputSpec& in : model.inputs) {
    input_names_.push_back(in.name);
    input_values_.push_back(in.value);
  }
  input_slots_.resize(model.inputs.size());
  elems_.reserve(model.elements.size());
  for (const ElementSpec& s : model.elements) {
    const fd_element_type& t = find_element_type(s.type);
    elems_.emplace_back(s, t);
    Instance& e = elems_.back();
    e.nq = e.res.sparsity_in(IN_Q).nrow;
    e.nv = e.res.sparsity_in(IN_QD).nrow;
    e.nc = e.res.sparsity_in(IN_LAM).nrow;
    auto check = [&](size_t got, int want, const char* what) {
      if (static_cast<int>(got) != want)
        throw std::runtime_error("element " + s.name + ": expected " + std::to_string(want) +
                                 " " + what + ", got " + std::to_string(got));
    };
    check(s.coords.size(), e.nq, "coordinates");
    check(s.dofs.size(), e.nv, "velocity dofs");
    check(s.cons.size(), e.nc, "constraints");
    check(s.p.size(), e.res.sparsity_in(IN_P).nrow, "parameters");
    check(s.q.size(), e.owner() ? e.nq : 0, "initial coordinates");
    check(s.u.size(), e.owner() ? e.init->sparsity_in(1).nrow : 0, "initial velocities");
    if (s.newmark != "standard" && s.newmark != "modified")
      throw std::runtime_error("element " + s.name + ": newmark must be standard or modified");
    vector_space_ = vector_space_ && e.nq == e.nv && s.coords == s.dofs;
    modified_newmark_ = modified_newmark_ || e.mn_qd;

    e.coords = s.coords;
    e.map = s.dofs;
    for (int c : s.cons) e.map.push_back(nv_ + c);
    e.q.resize(e.nq);
    e.q_n.resize(e.nq);
    for (auto* v : {&e.qd, &e.qdd, &e.incr, &e.qd_n, &e.qdd_n}) v->resize(e.nv);
    e.lam.resize(e.nc);
    alloc(e.res, e.res_buf);
    alloc(e.ic, e.ic_buf);
    if (e.out) {
      // q, qd, qdd, p, t (fdgen.kinematics.OUT_INPUTS); a plugin from before the accelerations has 4
      if (e.out->n_in() != 5)
        throw std::runtime_error("element " + s.name + ": output function of an older generator, build the model again");
      alloc(*e.out, e.out_buf);
      for (auto& b : e.out_buf) e.out_ptr.push_back(b.data());
    }
    if (e.scale) e.scale_buf.resize(e.scale->sparsity_out(0).nnz());
    e.iter_buf.resize(e.iter.sparsity_out(0).nnz());
    if (e.mn_qd) {
      e.mn_qd_buf.resize(e.mn_qd->sparsity_out(0).nnz());
      e.mn_d_buf.resize(e.mn_qd->sparsity_out(1).nnz());
    }
    if (e.jac_qd) e.jac_qd_buf.resize(e.jac_qd->sparsity_out(0).nnz());
    if (e.post) {
      check(s.p.size(), e.post->sparsity_out(0).nrow, "post function output (parameters)");
      e.post_buf.resize(e.post->sparsity_out(0).nnz());
    }
    if (e.spline || !s.splines.empty()) {
      if (!e.spline || e.spline->sparsity_out(0).nrow != static_cast<int>(s.splines.size()))
        throw std::runtime_error("element " + s.name + ": spline calls and spline function do not match");
      for (const auto& c : s.splines)
        if (c.segment < 0 || c.segment + 5 > static_cast<int>(s.p.size()))
          throw std::runtime_error("element " + s.name + ": spline segment outside p");
      e.spline_buf.resize(s.splines.size());
      spline_users_.push_back(static_cast<int>(elems_.size()) - 1);
    }
    if (e.event) {
      if (e.event->n_in() != 4) throw std::runtime_error("element " + s.name + ": event function with other inputs");
      e.event_offset = n_events_;
      e.event_buf.resize(e.event->sparsity_out(0).nnz());
      n_events_ += e.event->sparsity_out(0).nrow;
      event_users_.push_back(static_cast<int>(elems_.size()) - 1);
    }
    for (const auto& [index, offset] : s.inputs) {
      if (index < 0 || index >= n_inputs() || offset < 0 || offset >= static_cast<int>(s.p.size()))
        throw std::runtime_error("element " + s.name + ": input slot outside the inputs or p");
      input_slots_[index].push_back({static_cast<int>(elems_.size()) - 1, offset});
      e.spec.p[offset] = input_values_[index];  // the declared default
    }
    if (s.road >= 0) {  // tire on the runtime road: the plane (x0, y0, h0, gx, gy, mu) in p
      if (model.road.crg.empty()) throw std::runtime_error("element " + s.name + ": no road_crg in the model");
      if (s.road + 6 > static_cast<int>(s.p.size()) || e.nq < 2)
        throw std::runtime_error("element " + s.name + ": road parameters outside p");
      if (!road_) {
        const Road::Placement pl{model.road.x, model.road.y, model.road.z, model.road.phi};
        road_ = std::make_unique<Road>(model.road.crg, pl, model.road.friction, model.road.friction_crg);
      }
      road_users_.push_back({static_cast<int>(elems_.size()) - 1, s.road, road_->add_point()});
    }
    for (int i = 0; i < 2; ++i) e.res_rows[i] = vector_scatter_map(e.res.sparsity_out(i), e.map);
    for (int i = 1; i < 4; ++i) e.ic_rows[i] = vector_scatter_map(e.ic.sparsity_out(i), e.map);
    e.iter_block = iter_asm_.add_block(e.iter.sparsity_out(0), e.map);
    e.ic_block = ic_asm_.add_block(e.ic.sparsity_out(0), e.map);

    auto add_channels = [&](const std::string& base, int n) {
      for (int k = 0; k < n; ++k) {
        out_names_.push_back(n == 1 ? base : n == 3 ? base + "_" + AXES[k] : base + std::to_string(k));
        out_base_.push_back(base);
      }
    };
    if (e.out)
      for (int i = 0; i < e.out->n_out(); ++i) add_channels(s.name + "." + e.out->name_out(i), e.out->sparsity_out(i).nrow);
    else
      add_channels(s.name + ".lambda", e.nc);
  }
  iter_asm_.finalize(nx());
  ic_asm_.finalize(nx());
  q.assign(nq_, 0.0);
  for (auto* v : {&qd, &qdd, &incr}) v->assign(nv_, 0.0);
  lam.assign(nc_, 0.0);
  inactive_mask_.assign(nc_, 0);
  has_diagonal_.assign(nc_, 0);
}

void System::gather(Instance& e) {
  for (int i = 0; i < e.nq; ++i) e.q[i] = q[e.coords[i]];
  for (int i = 0; i < e.nv; ++i) {
    const int g = e.map[i];
    e.qd[i] = qd[g];
    e.qdd[i] = qdd[g];
    e.incr[i] = incr[g];
  }
  for (int i = 0; i < e.nc; ++i) e.lam[i] = lam[e.map[e.nv + i] - nv_];
}

void System::update_roads() {
  for (RoadUser& r : road_users_) {
    Instance& e = elems_[r.elem];
    const double x = q[e.coords[0]], y = q[e.coords[1]];  // center of the wheel (its first body)
    const RoadSample s = road_->eval(r.point, x, y);
    double* p = e.spec.p.data() + r.p;
    p[0] = x, p[1] = y, p[2] = s.z, p[3] = s.dzdx, p[4] = s.dzdy, p[5] = s.mu;
  }
}

void System::update_splines() {
  for (int k : spline_users_) {
    Instance& e = elems_[k];
    gather(e);
    const std::vector<ElementSpec::SplineCall>& calls = e.spec.splines;
    const Sparsity& sp = e.spline->sparsity_out(0);
    // calls in order; a call whose argument holds another spline needs that segment first
    for (size_t pass = 0; pass < calls.size(); ++pass) {
      const double* in[] = {e.q.data(), e.qd.data(), e.spec.p.data(), &t};
      double* out[] = {e.spline_buf.data()};
      e.spline->eval(in, out);
      bool changed = false;
      for (int j = 0; j < sp.nnz(); ++j) {
        const ElementSpec::SplineCall& c = calls[sp.row[j]];
        const double* x = c.table.data();
        const double xv = e.spline_buf[j];
        // the largest i with x[i] <= xv, 0 below the first point (fdgen.splines.segment)
        const int i = std::max(static_cast<int>(std::upper_bound(x, x + c.n, xv) - x) - 1, 0);
        double* seg = e.spec.p.data() + c.segment;
        if (seg[0] == x[i] && seg[1] == x[c.n + i]) continue;
        for (int m = 0; m < 5; ++m) seg[m] = x[m * c.n + i];
        changed = true;
      }
      if (!changed) break;
    }
  }
}

void System::set_initial_state() {
  for (Instance& e : elems_) {
    if (!e.owner()) continue;
    const double* in[] = {e.spec.q.data(), e.spec.u.data(), e.spec.p.data()};
    double* out[] = {e.qd.data()};
    e.init->eval(in, out);
    for (int i = 0; i < e.nq; ++i) q[e.coords[i]] = e.spec.q[i];
    for (int i = 0; i < e.nv; ++i) qd[e.map[i]] = e.qd[i];
  }
}

void System::update_configuration(const std::vector<double>& q_n) {
  for (Instance& e : elems_) {
    if (!e.owner()) continue;
    for (int i = 0; i < e.nq; ++i) e.q_n[i] = q_n[e.coords[i]];
    for (int i = 0; i < e.nv; ++i) e.incr[i] = incr[e.map[i]];
    const double* in[] = {e.q_n.data(), e.incr.data()};
    double* out[] = {e.q.data()};
    e.update->eval(in, out);
    for (int i = 0; i < e.nq; ++i) q[e.coords[i]] = e.q[i];
  }
}

void System::scales(std::vector<double>& s) {
  s.assign(nv_, 0.0);
  for (Instance& e : elems_) {
    if (!e.owner()) continue;
    gather(e);
    const double* in[] = {e.q.data()};
    double* out[] = {e.scale_buf.data()};
    e.scale->eval(in, out);
    const Sparsity& sp = e.scale->sparsity_out(0);
    for (int k = 0; k < sp.nnz(); ++k) s[e.map[sp.row[k]]] = e.scale_buf[k];
  }
}

int System::input_index(const std::string& name) const {
  for (int i = 0; i < n_inputs(); ++i)
    if (input_names_[i] == name) return i;
  return -1;
}

void System::set_input(int index, double value) {
  if (index < 0 || index >= n_inputs()) throw std::out_of_range("input index " + std::to_string(index));
  input_values_[index] = value;
  for (const InputSlot& slot : input_slots_[index]) elems_[slot.elem].spec.p[slot.p] = value;
}

void System::post_step() {
  for (Instance& e : elems_) {
    if (!e.post) continue;
    gather(e);
    const double* in[] = {e.q.data(), e.qd.data(), e.spec.p.data(), &t};
    double* out[] = {e.post_buf.data()};
    e.post->eval(in, out);
    const Sparsity& sp = e.post->sparsity_out(0);  // the state entries; the rest of p stays
    for (int k = 0; k < sp.nnz(); ++k) e.spec.p[sp.row[k]] = e.post_buf[k];
  }
}

void System::residual(const HhtCoef& c, double* r, double* f) {
  update_roads();
  update_splines();  // the iteration matrix that follows is evaluated at the same state
  std::fill(r, r + nx(), 0.0);
  std::fill(f, f + nx(), 0.0);
  const double coef[] = {c.alpha, c.beta, c.gamma, c.h};
  for (Instance& e : elems_) {
    gather(e);
    double* out[] = {e.res_buf[0].data(), e.res_buf[1].data()};
    e.res.eval(std_inputs(e, coef).data(), out);
    scatter_add(e.res_rows[0], e.res_buf[0].data(), r);
    scatter_add(e.res_rows[1], e.res_buf[1].data(), f);
  }
  for (int c : inactive_) r[nv_ + c] = lam[c];  // lam_c = 0; f has no constraint rows
}

const CsrMatrix& System::iteration_matrix(const HhtCoef& c) {
  iter_asm_.zero();
  const double coef[] = {c.alpha, c.beta, c.gamma, c.h};
  for (Instance& e : elems_) {
    gather(e);
    double* out[] = {e.iter_buf.data()};
    e.iter.eval(std_inputs(e, coef).data(), out);
    if (e.jac_qd) modified_jacobian(e, c);
    iter_asm_.add(e.iter_block, e.iter_buf.data());
  }
  pin_inactive(iter_asm_);
  return iter_asm_.matrix();
}

void System::eval_modified_newmark(Instance& e, const double* coef) {
  for (int i = 0; i < e.nq; ++i) e.q_n[i] = (*ref_q_n_)[e.coords[i]];
  for (int i = 0; i < e.nv; ++i) {
    e.qd_n[i] = (*ref_qd_n_)[e.map[i]];
    e.qdd_n[i] = (*ref_qdd_n_)[e.map[i]];
  }
  const double* in[] = {e.q.data(), e.q_n.data(), e.qd_n.data(), e.qdd_n.data(), e.qdd.data(), coef};
  double* out[] = {e.mn_qd_buf.data(), e.mn_d_buf.data()};
  e.mn_qd->eval(in, out);
}

void System::apply_modified_newmark(const HhtCoef& c) {
  if (!modified_newmark_) return;
  if (!ref_q_n_) throw std::runtime_error("modified Newmark: no reference state set");
  const double coef[] = {c.alpha, c.beta, c.gamma, c.h};
  for (Instance& e : elems_) {
    if (!e.mn_qd) continue;
    gather(e);
    eval_modified_newmark(e, coef);
    const Sparsity& sp = e.mn_qd->sparsity_out(0);
    for (int k = 0; k < sp.nnz(); ++k) qd[e.map[sp.row[k]]] = e.mn_qd_buf[k];
  }
}

// J += (dr/dqd) (D - gamma h I) with D = dqd/dqdd of the modified formulas (exact chain rule,
// as MBS_PhysicalDOF_x_PhysicalDOF_Map_Procedure::correctScalingValueOfJacEntriesDueToModifiedNewmarkFormulas).
void System::modified_jacobian(Instance& e, const HhtCoef& c) {
  const double coef[] = {c.alpha, c.beta, c.gamma, c.h};
  // D at the current state (gather has been done; reference state set by the integrator).
  eval_modified_newmark(e, coef);
  double* out[] = {e.jac_qd_buf.data()};
  e.jac_qd->eval(std_inputs(e, coef).data(), out);

  // Dense local products (element sizes are small).
  const int nr = e.nv + e.nc, nv = e.nv;
  auto dense = [](const Sparsity& sp, const std::vector<double>& nz) {
    std::vector<double> d(static_cast<size_t>(sp.nrow) * sp.ncol, 0.0);  // column major
    for (int j = 0; j < sp.ncol; ++j)
      for (int k = sp.colind[j]; k < sp.colind[j + 1]; ++k) d[j * sp.nrow + sp.row[k]] = nz[k];
    return d;
  };
  const std::vector<double> Jqd = dense(e.jac_qd->sparsity_out(0), e.jac_qd_buf);  // nr x nv
  std::vector<double> D = dense(e.mn_qd->sparsity_out(1), e.mn_d_buf);              // nv x nv
  for (int i = 0; i < nv; ++i) D[i * nv + i] -= c.gamma * c.h;
  const Sparsity& sp = e.iter.sparsity_out(0);
  std::vector<double> corr(static_cast<size_t>(nr) * nv, 0.0);
  for (int j = 0; j < nv; ++j)
    for (int k = 0; k < nv; ++k) {
      const double d = D[j * nv + k];
      if (d != 0.0)
        for (int i = 0; i < nr; ++i) corr[j * nr + i] += Jqd[k * nr + i] * d;
    }
  // Add into the nonzeros of the iteration matrix; the correction must lie in its pattern.
  std::vector<char> seen(static_cast<size_t>(nr) * nv, 0);
  for (int j = 0; j < nv; ++j)
    for (int k = sp.colind[j]; k < sp.colind[j + 1]; ++k) {
      e.iter_buf[k] += corr[j * nr + sp.row[k]];
      seen[j * nr + sp.row[k]] = 1;
    }
  for (int i = 0; i < nr * nv; ++i)
    if (!seen[i] && corr[i] != 0.0)
      throw std::runtime_error("modified Newmark: Jacobian correction outside the iteration matrix pattern");
}

const CsrMatrix& System::ic_system(double* b, double* phi, double* phid) {
  update_roads();
  update_splines();
  ic_asm_.zero();
  double* vec[] = {nullptr, b, phi, phid};
  for (int i = 1; i < 4; ++i) std::fill(vec[i], vec[i] + nx(), 0.0);
  for (Instance& e : elems_) {
    gather(e);
    const double* in[] = {e.q.data(), e.qd.data(), nullptr, nullptr, e.spec.p.data(), nullptr, nullptr, &t};
    double* out[] = {e.ic_buf[0].data(), e.ic_buf[1].data(), e.ic_buf[2].data(), e.ic_buf[3].data()};
    e.ic.eval(in, out);
    ic_asm_.add(e.ic_block, e.ic_buf[0].data());
    for (int i = 1; i < 4; ++i) scatter_add(e.ic_rows[i], e.ic_buf[i].data(), vec[i]);
  }
  pin_inactive(ic_asm_);
  if (!raw_)
    for (int c : inactive_) b[nv_ + c] = 0.0;
  return ic_asm_.matrix();
}

void System::outputs(std::vector<double>& y) {
  update_roads();
  update_splines();
  y.clear();
  for (Instance& e : elems_) {
    gather(e);
    if (!e.out) {
      y.insert(y.end(), e.lam.begin(), e.lam.end());
      continue;
    }
    const double* in[] = {e.q.data(), e.qd.data(), e.qdd.data(), e.spec.p.data(), &t};
    e.out->eval(in, e.out_ptr.data());
    for (int i = 0; i < e.out->n_out(); ++i) {
      const Sparsity& sp = e.out->sparsity_out(i);
      const size_t base = y.size();
      y.resize(base + sp.nrow, 0.0);
      for (int k = 0; k < sp.nnz(); ++k) y[base + sp.row[k]] = e.out_buf[i][k];
    }
  }
}

void System::pin_inactive(MatrixAssembler& a) {
  if (raw_) return;
  for (int c : inactive_) a.pin_row(nv_ + c);
}

std::vector<int> System::find_redundant_constraints(double tol, bool all) {
  std::vector<double> b(nx()), phi(nx()), phid(nx());
  raw_ = all;
  const CsrMatrix& A = ic_system(b.data(), phi.data(), phid.data());
  raw_ = false;
  // B (rows nv.., columns < nv) dense, columns equilibrated (translations and rotations, mm
  // and m models alike), rows of unit length.
  std::vector<double> B(static_cast<size_t>(nc_) * nv_, 0.0), colmax(nv_, 0.0);
  for (int c = 0; c < nc_; ++c)
    for (int k = A.rowptr[nv_ + c]; k < A.rowptr[nv_ + c + 1]; ++k)
      if (A.col[k] < nv_) {
        B[static_cast<size_t>(c) * nv_ + A.col[k]] = A.val[k];
        colmax[A.col[k]] = std::max(colmax[A.col[k]], std::abs(A.val[k]));
      }
  // Modified Gram-Schmidt in model order with reorthogonalization: a row is redundant when
  // its part orthogonal to the kept rows is below tol.
  std::vector<double> basis;  // kept rows, orthonormal, nv each
  std::vector<int> redundant;
  std::vector<double> v(nv_);
  for (int c = 0; c < nc_; ++c) {
    if (inactive_mask_[c] && !all) continue;
    double n0 = 0.0;
    for (int j = 0; j < nv_; ++j) {
      v[j] = colmax[j] > 0.0 ? B[static_cast<size_t>(c) * nv_ + j] / colmax[j] : 0.0;
      n0 += v[j] * v[j];
    }
    n0 = std::sqrt(n0);
    if (n0 > 0.0)
      for (double& x : v) x /= n0;
    for (int pass = 0; pass < 2 && n0 > 0.0; ++pass)
      for (size_t k = 0; k < basis.size(); k += nv_) {
        double d = 0.0;
        for (int j = 0; j < nv_; ++j) d += v[j] * basis[k + j];
        for (int j = 0; j < nv_; ++j) v[j] -= d * basis[k + j];
      }
    double n = 0.0;
    for (double x : v) n += x * x;
    n = std::sqrt(n);
    if (!(n0 > 0.0) || n < tol) {
      redundant.push_back(c);
      continue;
    }
    for (double x : v) basis.push_back(x / n);
  }
  return redundant;
}

bool System::deactivate_constraints(const std::vector<int>& cons) {
  bool added = false;
  for (int c : cons) {
    if (c < 0 || c >= nc_) throw std::out_of_range("constraint " + std::to_string(c));
    if (inactive_mask_[c]) continue;
    inactive_mask_[c] = 1;
    inactive_.push_back(c);
    lam[c] = 0.0;
    if (has_diagonal_[c]) continue;  // deactivated before: its diagonal is in the pattern
    has_diagonal_[c] = 1;
    iter_asm_.add_diagonal(nv_ + c);
    ic_asm_.add_diagonal(nv_ + c);
    added = true;
  }
  std::sort(inactive_.begin(), inactive_.end());
  if (!added) return false;
  iter_asm_.finalize(nx());
  ic_asm_.finalize(nx());
  return true;
}

void System::activate_constraints(const std::vector<int>& cons) {
  for (int c : cons) {
    if (c < 0 || c >= nc_) throw std::out_of_range("constraint " + std::to_string(c));
    if (!inactive_mask_[c]) continue;
    inactive_mask_[c] = 0;
    inactive_.erase(std::find(inactive_.begin(), inactive_.end(), c));
  }
}

std::string System::constraint_name(int c) const {
  for (const Instance& e : elems_)
    for (size_t k = 0; k < e.spec.cons.size(); ++k)
      if (e.spec.cons[k] == c)
        return e.spec.cons.size() == 1 ? e.spec.name
                                       : e.spec.name + " (constraint " + std::to_string(k + 1) + " of " +
                                             std::to_string(e.spec.cons.size()) + ")";
  return "constraint " + std::to_string(c);
}

std::string System::dof_name(int i) const {
  for (const Instance& e : elems_)
    if (e.owner())
      for (int k = 0; k < e.nv; ++k)
        if (e.map[k] == i) return e.spec.name + " (velocity dof " + std::to_string(k + 1) + " of " + std::to_string(e.nv) + ")";
  return "velocity dof " + std::to_string(i);
}

void System::events(std::vector<double>& g) {
  g.assign(n_events_, 0.0);  // no allocation once g has its size
  if (!n_events_) return;
  update_roads();
  update_splines();
  for (int k : event_users_) {
    Instance& e = elems_[k];
    gather(e);
    const Sparsity& sp = e.event->sparsity_out(0);
    const double* in[] = {e.q.data(), e.qd.data(), e.spec.p.data(), &t};
    double* out[] = {e.event_buf.data()};
    e.event->eval(in, out);
    for (int j = 0; j < sp.nnz(); ++j) g[e.event_offset + sp.row[j]] = e.event_buf[j];
  }
}

std::string System::event_name(int i) const {
  for (int k : event_users_) {
    const Instance& e = elems_[k];
    const int n = e.event->sparsity_out(0).nrow;
    if (i >= e.event_offset && i < e.event_offset + n)
      return n == 1 ? e.spec.name : e.spec.name + "[" + std::to_string(i - e.event_offset) + "]";
  }
  return "event " + std::to_string(i);
}

int System::output_index(const std::string& name) const {
  for (size_t i = 0; i < out_base_.size(); ++i)
    if (out_base_[i] == name) return static_cast<int>(i);
  return -1;
}

}  // namespace fd
