#pragma once
// A freedyn2 model as a co-simulation slave: the model file (.fdm, with its plugin and data
// files) from the resources of the FMU, its runtime inputs as inputs and its output channels as
// outputs. Value references: the inputs 0 .. ni-1, then the outputs ni .. ni+no-1, in the order
// freedyn2_run --channels lists them (fdgen.fmu writes the modelDescription.xml in that order).
// The model's time starts at the start time of the co-simulation (t_model = t - t_start).
// A communication step integrates with the model's step (settings.step) to its end
// (HhtIntegrator::advance_to); inputs are constant during it.

#include <memory>
#include <string>
#include <vector>

#include "freedyn2/hht.hpp"
#include "freedyn2/model.hpp"
#include "freedyn2/system.hpp"

namespace fd::fmu {

// file:///C:/a%20b/resources -> C:/a b/resources (FMI 2.0: a resource location URI)
std::string path_from_uri(const std::string& uri);
// FMI 3.0: the resource path is a native path, with a trailing separator
std::string native_dir(const std::string& path);

class Slave {
 public:
  explicit Slave(const std::string& model_file);

  int n_inputs() const { return sys_->n_inputs(); }
  int n_variables() const { return sys_->n_inputs() + static_cast<int>(sys_->output_names().size()); }
  void setup(double t_start) { t_start_ = t_start; }
  // Consistent initial state at t_start, with the inputs set so far (exit initialization mode).
  void initialize();
  // Integrate from t to t + h (t: the time the slave is at).
  void do_step(double t, double h);
  double time() const { return t_start_ + (hht_ ? hht_->time() : 0.0); }
  bool initialized() const { return initialized_; }

  // false: no such value reference (or an output set)
  bool get(unsigned vr, double& value) const;
  bool set(unsigned vr, double value);

 private:
  Model model_;
  std::unique_ptr<System> sys_;
  std::unique_ptr<HhtIntegrator> hht_;
  std::vector<double> y_;  // outputs at time()
  double t_start_ = 0.0;
  bool initialized_ = false;
};

}  // namespace fd::fmu
