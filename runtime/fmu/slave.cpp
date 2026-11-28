#include "slave.hpp"

#include <cctype>
#include <cmath>
#include <stdexcept>

namespace fd::fmu {

std::string path_from_uri(const std::string& uri) {
  std::string p = uri;
  if (p.rfind("file://", 0) == 0) p = p.substr(7);       // file:///x or file://host/x
  else if (p.rfind("file:", 0) == 0) p = p.substr(5);    // file:/x
  // Windows: /C:/... -> C:/...
  if (p.size() > 2 && p[0] == '/' && std::isalpha(static_cast<unsigned char>(p[1])) && p[2] == ':') p = p.substr(1);
  std::string out;  // percent decoding
  for (size_t i = 0; i < p.size(); ++i) {
    if (p[i] == '%' && i + 2 < p.size() && std::isxdigit(static_cast<unsigned char>(p[i + 1])) &&
        std::isxdigit(static_cast<unsigned char>(p[i + 2]))) {
      out += static_cast<char>(std::stoi(p.substr(i + 1, 2), nullptr, 16));
      i += 2;
    } else {
      out += p[i];
    }
  }
  while (out.size() > 1 && (out.back() == '/' || out.back() == '\\')) out.pop_back();
  return out;
}

std::string native_dir(const std::string& path) {
  std::string p = path;
  while (p.size() > 1 && (p.back() == '/' || p.back() == '\\')) p.pop_back();
  return p;
}

Slave::Slave(const std::string& model_file) : model_(load_model(model_file)) {
  model_.solver.adaptive = false;  // co-simulation: the model's step to the communication points
  sys_ = std::make_unique<System>(model_);
  // Eigen's SparseLU: the FMU carries no shared libraries (KLU is linked dynamically elsewhere)
  hht_ = std::make_unique<HhtIntegrator>(*sys_, model_.solver, "sparselu");
}

void Slave::initialize() {
  hht_->initialize();
  sys_->outputs(y_);
  initialized_ = true;
}

void Slave::do_step(double t, double h) {
  if (!initialized_) throw std::runtime_error("doStep before the initialization");
  if (std::abs(t - time()) > 1e-9 * std::max(1.0, std::abs(t)))
    throw std::runtime_error("doStep from t = " + std::to_string(t) + ", the slave is at t = " + std::to_string(time()) +
                             " (no rollback)");
  if (!(h > 0.0)) throw std::runtime_error("doStep with a step size of " + std::to_string(h));
  hht_->advance_to(t + h - t_start_);
  sys_->outputs(y_);
}

bool Slave::get(unsigned vr, double& value) const {
  const int ni = n_inputs();
  if (static_cast<int>(vr) < ni) {
    value = sys_->input(static_cast<int>(vr));
    return true;
  }
  const size_t k = vr - static_cast<unsigned>(ni);
  if (k >= sys_->output_names().size()) return false;
  value = initialized_ ? y_[k] : 0.0;
  return true;
}

bool Slave::set(unsigned vr, double value) {
  if (static_cast<int>(vr) >= n_inputs()) return false;
  sys_->set_input(static_cast<int>(vr), value);
  return true;
}

}  // namespace fd::fmu
