#pragma once
// Model description read from the generator's .fdm file (or built in code).

#include <string>
#include <vector>

#include "fd_registry.h"

namespace fd {

struct SolverSettings {
  double alpha = -0.1;
  std::string hht_form = "arnold_bruls";  // or "negrut" (legacy FreeDyn)
  double end_time = 1.0;
  double step = 1e-3;          // fixed step size, maximum step size if adaptive
  double output_step = 1e-2;
  double output_begin = 0.0;
  double newton_tol = 1e-10;   // newton = "velocity" only
  int max_newton = 25;
  // "velocity" (until the velocity change is below newton_tol), "legacy" (Negrut Eq. 40, as
  // legacy FreeDyn) or "fixed" (exactly newton_iterations corrections: the same work in every
  // step, for real-time use with a fixed step size)
  std::string newton = "velocity";
  int newton_iterations = 2;
  bool adaptive = false;       // legacy FreeDyn step size control (LegacyTimeStepper)
  double h_min = 1e-14;
  double eps = 1e-6;           // GlobalErrorTolerance of the legacy solver
  // Weights of the local error norm (adaptive): "mixed" eps (1 + max(|s_n|, |s_n+1|)) per
  // velocity dof (absolute and relative tolerance eps, Hairer, Norsett, Wanner I, II.4);
  // "legacy" eps max(1, largest |s| so far), as legacy FreeDyn. s: element scale functions.
  std::string error_norm = "mixed";
  bool exact_output = true;    // isExactOutputTimeEnforced
  double vproj_tol = 1e-12;    // initial velocity projection when |Phi_q qd| > tol (legacy: 1e-5)
};

struct ElementSpec {
  std::string type, name;
  std::vector<int> coords;  // global indices of the stored coordinates q (default: dofs)
  std::vector<int> dofs;    // global velocity dof indices (qd, qdd)
  std::vector<int> cons;  // global constraint indices
  std::vector<double> p;  // parameters
  std::vector<double> q;  // initial coordinates
  std::vector<double> u;  // initial user velocities, mapped to qd by the element's init function
  std::string jacobian = "full";     // "no_qvv": iteration matrix without derivatives of h (iter_noqv)
  std::string newmark = "standard";  // "modified": legacy modified Newmark formulas (mn_qd)
  // Tires on the runtime road (*_local types): offset of the local road plane (x0, y0, h0, gx,
  // gy, mu) in p, set from Model::road at the position of the element's first body; -1: none.
  int road = -1;
};

// Road surface of the *_local tire elements (fd::Road): OpenCRG height file, its placement in
// the world, friction constant or from a second OpenCRG file.
struct RoadSpec {
  std::string crg, friction_crg;  // paths (relative ones resolved against the model file)
  double friction = 1.0;
  double x = 0.0, y = 0.0, z = 0.0, phi = 0.0;
};

struct Model {
  std::string name;
  SolverSettings solver;
  std::vector<ElementSpec> elements;
  std::vector<std::string> plugins;  // model plugins loaded for this model (expression forces)
  RoadSpec road;                     // road.crg empty: no runtime road
  int nq() const;  // stored coordinates
  int nv() const;  // velocity dofs
  int nc() const;
};

Model load_model(const std::string& path);
const fd_element_type& find_element_type(const std::string& name);

}  // namespace fd
