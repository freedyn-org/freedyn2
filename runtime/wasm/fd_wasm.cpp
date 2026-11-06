// freedyn2 in WebAssembly: a C interface for JavaScript (fd_wasm.mjs).
//
// One simulation at a time: fd_run takes the text of a .fdm model file, simulates it as
// freedyn2_run does (step size control or fixed step, the model's solver settings) and keeps
// the result in memory: fd_result_names (t and the output columns, comma separated),
// fd_result_rows, fd_result_cols, fd_result_data (row major). Library elements only: models
// with a model plugin (expression forces, drivers) need the native runtime.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

#include "freedyn2/hht.hpp"
#include "freedyn2/linear_solver.hpp"
#include "freedyn2/model.hpp"
#include "freedyn2/system.hpp"

#define FD_EXPORT(name) extern "C" __attribute__((export_name(#name)))

namespace {
std::string g_error;
std::string g_names;
std::vector<double> g_data;
int g_cols = 0;
long g_steps = 0;
long g_rejected = 0;
}  // namespace

FD_EXPORT(fd_alloc) void* fd_alloc(size_t n) { return std::malloc(n); }
FD_EXPORT(fd_free) void fd_free(void* p) { std::free(p); }

// Returns 0 on success, 1 on an error (fd_error). end_time > 0 overrides the model's.
FD_EXPORT(fd_run) int fd_run(const char* fdm_text, double end_time) {
  g_error.clear();
  g_names.clear();
  g_data.clear();
  g_cols = 0;
  g_steps = g_rejected = 0;
  try {
    fd::Model model = fd::parse_model(fdm_text, "model.fdm");
    if (end_time > 0) model.solver.end_time = end_time;
    const fd::SolverSettings& s = model.solver;
    fd::System sys(model);
    fd::HhtIntegrator hht(sys, s, fd::default_linear_solver());

    g_names = "t";
    for (const auto& n : sys.output_names()) g_names += "," + n;
    g_cols = 1 + static_cast<int>(sys.output_names().size());
    std::vector<double> y;
    auto write_output = [&](double t) {
      sys.outputs(y);
      g_data.push_back(t);
      g_data.insert(g_data.end(), y.begin(), y.end());
    };
    if (s.adaptive) {
      fd::LegacyTimeStepper stepper(hht, s);
      g_steps = stepper.run(write_output);
      g_rejected = stepper.rejected_steps();
    } else {
      g_steps = std::lround(s.end_time / s.step);
      const long out_every = std::max(1L, std::lround(s.output_step / s.step));
      hht.initialize();
      write_output(0.0);
      for (long n = 1; n <= g_steps; ++n) {
        hht.step();
        if (n % out_every == 0 || n == g_steps) write_output(hht.time());
      }
    }
    return 0;
  } catch (const std::exception& e) {
    g_error = e.what();
    return 1;
  }
}

FD_EXPORT(fd_error) const char* fd_error() { return g_error.c_str(); }
FD_EXPORT(fd_result_names) const char* fd_result_names() { return g_names.c_str(); }
FD_EXPORT(fd_result_cols) int fd_result_cols() { return g_cols; }
FD_EXPORT(fd_result_rows) int fd_result_rows() { return g_cols ? static_cast<int>(g_data.size() / g_cols) : 0; }
FD_EXPORT(fd_result_data) const double* fd_result_data() { return g_data.data(); }
FD_EXPORT(fd_steps) long fd_steps() { return g_steps; }
FD_EXPORT(fd_rejected_steps) long fd_rejected_steps() { return g_rejected; }
