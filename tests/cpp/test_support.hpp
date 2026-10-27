#pragma once
// Shared helpers of the acceptance tests: result reporting, models built in code or loaded from
// the shipped examples, and integration loops.

#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "freedyn2/hht.hpp"
#include "freedyn2/model.hpp"
#include "freedyn2/system.hpp"

using Vec = std::vector<double>;
using Tests = std::map<std::string, std::function<void()>>;

namespace fdtest {

// Print "RESULT <test> <metric> <value> tol <tol> ok|FAIL"; a failure fails the test.
void report(const char* test, const std::string& metric, double value, double tol);
// Print a value without a tolerance.
void info(const char* test, const std::string& metric, double value);
bool all_ok();

double norm_inf(const Vec& v);
// Shabana 2005 Eq. 2.13, written independently of the generator.
void rotation(const double* t, double A[3][3]);

// Rigid body element: Lie group (rigid_body: 7 coordinates, 6 velocity dofs) or Euler
// parameters (rigid_body_euler: 7 coordinates = dofs, normalization constraint).
fd::ElementSpec rigid_spec(const std::string& type, const std::string& name, const Vec& p, const Vec& q, const Vec& u);
const char* element_for(const std::string& form);
// One rigid body with body-frame initial angular velocity w_body and orientation theta.
fd::Model body_model(const Vec& J, const Vec& theta, const Vec& w_body, double alpha, double h,
                     double end_time, const std::string& form = "arnold_bruls");
// The same model with Euler-parameter bodies (for the negrut form, the legacy profile).
fd::Model with_element(fd::Model m, const std::string& type);
// A shipped example (examples/<name>.yaml, generated into FD_GEN_DIR).
fd::Model example(const std::string& name);

// Integrate a model; call f(t, y, sys) with the output channels at t = 0 and after every step.
void simulate(const fd::Model& m, const std::function<void(double, const Vec&, fd::System&)>& f,
              const std::string& ls = "sparselu");
// Dense row-major copy of a sparse generated output.
Vec dense(const fd::Sparsity& sp, const double* nz);

// Output channels of a whole run of an example, by name.
struct Run {
  std::vector<double> t;
  std::map<std::string, std::vector<double>> y;
};
Run run(const std::string& name, double h = 1e-3);

}  // namespace fdtest

// Test groups, registered by name.
void add_rigid_body_tests(Tests& tests);
void add_joint_tests(Tests& tests);
void add_force_tests(Tests& tests);
