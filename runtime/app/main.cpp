// freedyn2_run: simulate a generated model and write a CSV result.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#include <time.h>
#define FD_THREAD_CPU_TIME 1
#elif defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#define FD_THREAD_CPU_TIME 1
#endif

#include "freedyn2/hht.hpp"
#include "freedyn2/linear_solver.hpp"
#include "freedyn2/model.hpp"
#include "freedyn2/system.hpp"

namespace {

void usage() {
  std::cerr << "usage: freedyn2_run model.fdm [-o result.csv] [--alpha A] [--end-time T]\n"
               "                    [--step H] [--tolerance E] [--output-step H] [--linear-solver klu|sparselu|dense]\n"
               "                    [--step-log FILE]   (t, h, status, Newton iterations of every step attempt)\n"
               "                    [--fixed-step]      (constant step size --step, no step size control)\n"
               "                    [--newton-fixed N]  (exactly N Newton corrections per step, fixed pivots: real-time)\n"
               "                    [--timing]          (fixed step: distribution of the wall-clock time per step)\n"
               "                    [--timing-log FILE] (fixed step: time, Newton iterations, new pivot orders per step)\n";
}

void write_row(std::ofstream& out, double t, const std::vector<double>& y) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.10g", t);
  out << buf;
  for (double v : y) {
    std::snprintf(buf, sizeof buf, ",%.17g", v);
    out << buf;
  }
  out << '\n';
}

// CPU time of the calling thread [us]: without the time the thread is preempted, so that the
// work of a step can be told apart from operating system jitter (wall-clock time).
double thread_cpu_us() {
#if defined(FD_THREAD_CPU_TIME) && defined(_WIN32)
  FILETIME created, exited, kernel, user;  // 100 ns units; resolution of the scheduler tick
  GetThreadTimes(GetCurrentThread(), &created, &exited, &kernel, &user);
  auto ticks = [](const FILETIME& f) { return (double(f.dwHighDateTime) * 4294967296.0 + f.dwLowDateTime) * 0.1; };
  return ticks(kernel) + ticks(user);
#elif defined(FD_THREAD_CPU_TIME)
  timespec ts;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return ts.tv_sec * 1e6 + ts.tv_nsec * 1e-3;
#else
  return 0.0;
#endif
}

void print_distribution(const char* what, const std::vector<double>& us) {
  std::vector<double> sorted = us;
  std::sort(sorted.begin(), sorted.end());
  auto pct = [&](double p) { return sorted[std::min(sorted.size() - 1, size_t(p * (sorted.size() - 1)))]; };
  std::printf("%s [us]: min %.1f  median %.1f  p99 %.1f  p99.9 %.1f  max %.1f\n", what, sorted.front(), pct(0.5),
              pct(0.99), pct(0.999), sorted.back());
}

// Distribution of the time per step (fixed step size): what a real-time budget must cover.
void print_timing(const std::vector<double>& us, const std::vector<double>& cpu_us, const std::vector<int>& newton) {
  print_distribution("time per step", us);
#ifdef FD_THREAD_CPU_TIME
  print_distribution("CPU time per step (thread)", cpu_us);
#endif
  std::map<int, long> hist;
  for (int k : newton) ++hist[k];
  std::printf("Newton iterations per step:");
  for (const auto& [k, c] : hist) std::printf("  %d: %ld", k, c);
  std::printf("\n");
  const size_t worst = std::max_element(us.begin(), us.end()) - us.begin();
  std::printf("slowest step: %zu (%d Newton iterations)\n", worst + 1, newton[worst]);
}

// One step attempt: kept in memory during the run so that logging does not distort the timing.
struct Attempt {
  double t, h;
  fd::StepResult r;
};

void write_step_log(const std::string& path, const std::vector<Attempt>& step_log) {
  std::ofstream log(path);
  char buf[96];
  log << "t h accepted iterations\n";
  for (const auto& a : step_log) {
    std::snprintf(buf, sizeof buf, "%.17g %.17g %d %d\n", a.t, a.h, a.r.status == fd::StepStatus::ok,
                  a.r.iterations);
    log << buf;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    usage();
    return 2;
  }
  try {
    fd::Model model = fd::load_model(argv[1]);
    std::string out_path = model.name + ".csv", linear_solver = fd::default_linear_solver(), step_log_path,
                timing_log_path;
    bool timing = false;
    for (int i = 2; i < argc; ++i) {
      auto next = [&]() -> const char* {
        if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + argv[i]);
        return argv[++i];
      };
      if (!std::strcmp(argv[i], "-o")) out_path = next();
      else if (!std::strcmp(argv[i], "--alpha")) model.solver.alpha = std::stod(next());
      else if (!std::strcmp(argv[i], "--end-time")) model.solver.end_time = std::stod(next());
      else if (!std::strcmp(argv[i], "--step")) model.solver.step = std::stod(next());
      else if (!std::strcmp(argv[i], "--tolerance")) model.solver.eps = std::stod(next());
      else if (!std::strcmp(argv[i], "--output-step")) model.solver.output_step = std::stod(next());
      else if (!std::strcmp(argv[i], "--linear-solver")) linear_solver = next();
      else if (!std::strcmp(argv[i], "--step-log")) step_log_path = next();
      else if (!std::strcmp(argv[i], "--fixed-step")) model.solver.adaptive = false;
      else if (!std::strcmp(argv[i], "--newton-fixed")) {
        model.solver.newton = "fixed";
        model.solver.newton_iterations = std::stoi(next());
        model.solver.adaptive = false;
      }
      else if (!std::strcmp(argv[i], "--timing")) timing = true;
      else if (!std::strcmp(argv[i], "--timing-log")) timing = true, timing_log_path = next();
      else {
        usage();
        return 2;
      }
    }
    const fd::SolverSettings& s = model.solver;
    fd::System sys(model);
    fd::HhtIntegrator hht(sys, s, linear_solver);

    std::ofstream out(out_path);
    if (!out) throw std::runtime_error("cannot write " + out_path);
    out << "t";
    for (const auto& n : sys.output_names()) out << ',' << n;
    out << '\n';
    std::vector<double> y;
    double output_seconds = 0.0;
    auto last_flush = std::chrono::steady_clock::now();
    auto write_output = [&](double t) {
      const auto t0 = std::chrono::steady_clock::now();
      sys.outputs(y);
      write_row(out, t, y);
      if (t0 - last_flush > std::chrono::milliseconds(250)) {  // readers (the viewer) see the run grow
        out.flush();
        last_flush = t0;
      }
      output_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    };

    long n_steps = 0, rejected = 0;
    const auto t_start = std::chrono::steady_clock::now();
    std::vector<Attempt> step_log;
    if (s.adaptive) {
      fd::LegacyTimeStepper stepper(hht, s);
      std::function<void(double, double, const fd::StepResult&)> log_attempt;
      if (!step_log_path.empty())
        log_attempt = [&](double t, double h, const fd::StepResult& r) { step_log.push_back({t, h, r}); };
      n_steps = stepper.run(write_output, log_attempt);
      rejected = stepper.rejected_steps();
    } else {
      n_steps = std::lround(s.end_time / s.step);
      const long out_every = std::max(1L, std::lround(s.output_step / s.step));
      std::vector<double> step_us(timing ? n_steps : 0), step_cpu_us(timing ? n_steps : 0);
      std::vector<int> step_newton(timing ? n_steps : 0), step_pivots(timing ? n_steps : 0);
      hht.initialize();
      write_output(0.0);
      double max_correction = 0.0;  // newton fixed: how far from converged the worst step ended
      for (long n = 1; n <= n_steps; ++n) {
        const long pivots0 = hht.full_factorizations();
        const double cpu0 = timing ? thread_cpu_us() : 0.0;
        const auto t0 = std::chrono::steady_clock::now();
        max_correction = std::max(max_correction, hht.step().correction);
        if (timing) {
          step_cpu_us[n - 1] = thread_cpu_us() - cpu0;
          step_pivots[n - 1] = static_cast<int>(hht.full_factorizations() - pivots0);
          step_us[n - 1] = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
          step_newton[n - 1] = hht.last_newton_iterations();
        }
        if (!step_log_path.empty())
          step_log.push_back({hht.time(), s.step, {fd::StepStatus::ok, hht.last_newton_iterations(), 0.0}});
        if (n % out_every == 0 || n == n_steps) write_output(hht.time());
      }
      if (timing) print_timing(step_us, step_cpu_us, step_newton);
      if (s.newton == "fixed")
        std::printf("largest velocity change of a last Newton correction: %.2e (relative to 1 + |qd|)\n",
                    max_correction);
      if (!timing_log_path.empty()) {
        std::ofstream log(timing_log_path);
        log << "step us cpu_us newton new_pivots\n";
        for (long k = 0; k < n_steps; ++k)
          log << k + 1 << ' ' << step_us[k] << ' ' << step_cpu_us[k] << ' ' << step_newton[k] << ' ' << step_pivots[k]
              << '\n';
      }
    }
    const double step_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count() - output_seconds;
    if (!step_log_path.empty()) write_step_log(step_log_path, step_log);
    std::printf("model %s: %ld steps (%ld rejected), h%s = %g, alpha = %g, Newton %s, linear solver %s\n",
                model.name.c_str(), n_steps, rejected, s.adaptive ? "_max" : "", s.step, s.alpha,
                s.newton.c_str(), linear_solver.c_str());
    std::printf("Newton iterations per step: %.3f\n", double(hht.newton_iterations()) / n_steps);
    std::printf("time per step: %.3f us (total %.3f s)\n", 1e6 * step_seconds / n_steps, step_seconds);
    std::printf("wrote %s\n", out_path.c_str());
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
