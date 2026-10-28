#include "freedyn2/model.hpp"

#include "freedyn2/plugin.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fd {

namespace {
// Number of global entries the elements index with the member list (largest index + 1).
int global_size(const std::vector<ElementSpec>& elements, std::vector<int> ElementSpec::*indices) {
  int n = 0;
  for (const auto& e : elements)
    for (int i : e.*indices) n = std::max(n, i + 1);
  return n;
}
}  // namespace

int Model::nq() const { return global_size(elements, &ElementSpec::coords); }
int Model::nv() const { return global_size(elements, &ElementSpec::dofs); }
int Model::nc() const { return global_size(elements, &ElementSpec::cons); }

const fd_element_type& find_element_type(const std::string& name) {
  for (int i = 0; i < fd_n_element_types; ++i)
    if (name == fd_element_types[i].name) return fd_element_types[i];
  if (const fd_element_type* t = find_plugin_element_type(name)) return *t;
  throw std::runtime_error("unknown element type '" + name + "'");
}

namespace {
template <class T>
std::vector<T> read_all(std::istringstream& ss) {
  std::vector<T> v;
  T x;
  while (ss >> x) v.push_back(x);
  return v;
}
}  // namespace

Model load_model(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open model file " + path);
  Model m;
  ElementSpec* cur = nullptr;
  std::string line;
  int lineno = 0;
  auto fail = [&](const std::string& msg) {
    throw std::runtime_error(path + ":" + std::to_string(lineno) + ": " + msg);
  };
  auto relative_to = [&](const std::string& file) {  // relative to the model file
    const auto slash = path.find_last_of("/\\");
    if (!(file.size() > 0 && (file[0] == '/' || file.find(':') != std::string::npos)) && slash != std::string::npos)
      return path.substr(0, slash + 1) + file;
    return file;
  };
  // A path is the rest of its line: paths may contain spaces (Windows user directories).
  auto rest_of_line = [](std::istringstream& ss) {
    std::string r;
    std::getline(ss >> std::ws, r);
    while (!r.empty() && std::isspace(static_cast<unsigned char>(r.back()))) r.pop_back();
    return r;
  };
  while (std::getline(in, line)) {
    ++lineno;
    std::istringstream ss(line);
    std::string key;
    if (!(ss >> key) || key[0] == '#') continue;
    if (key == "freedyn2_model") {
      int v = 0;
      ss >> v;
      if (v != 1) fail("unsupported model file version");
    } else if (key == "name") {
      ss >> m.name;
    } else if (key == "plugin") {
      // Model plugin (expression forces), relative to the model file.
      std::string lib = relative_to(rest_of_line(ss));
      load_plugin(lib);
      m.plugins.push_back(lib);
    } else if (key == "input") {
      InputSpec in;
      if (!(ss >> in.name >> in.value)) fail("input needs a name and a default value");
      for (const auto& other : m.inputs)
        if (other.name == in.name) fail("input '" + in.name + "' declared twice");
      m.inputs.push_back(in);
    } else if (key == "road_crg") {
      m.road.crg = relative_to(rest_of_line(ss));
    } else if (key == "road_friction_crg") {
      m.road.friction_crg = relative_to(rest_of_line(ss));
    } else if (key == "road_friction") {
      ss >> m.road.friction;
    } else if (key == "road_place") {
      ss >> m.road.x >> m.road.y >> m.road.z >> m.road.phi;
    } else if (key == "solver") {
      std::string k;
      while (ss >> k) {
        SolverSettings& s = m.solver;
        if (k == "alpha") ss >> s.alpha;
        else if (k == "end_time") ss >> s.end_time;
        else if (k == "step") ss >> s.step;
        else if (k == "output_step") ss >> s.output_step;
        else if (k == "newton_tol") ss >> s.newton_tol;
        else if (k == "max_newton") ss >> s.max_newton;
        else if (k == "newton") ss >> s.newton;
        else if (k == "newton_iterations") ss >> s.newton_iterations;
        else if (k == "hht_form") ss >> s.hht_form;
        else if (k == "error_norm") ss >> s.error_norm;
        else if (k == "adaptive") ss >> s.adaptive;
        else if (k == "h_min") ss >> s.h_min;
        else if (k == "eps") ss >> s.eps;
        else if (k == "exact_output") ss >> s.exact_output;
        else if (k == "output_begin") ss >> s.output_begin;
        else if (k == "vproj_tol") ss >> s.vproj_tol;
        else fail("unknown solver key '" + k + "'");
      }
    } else if (key == "element") {
      m.elements.emplace_back();
      cur = &m.elements.back();
      ss >> cur->type >> cur->name;
    } else if (key == "end") {
      if (cur && cur->coords.empty()) cur->coords = cur->dofs;
      cur = nullptr;
    } else if (cur && key == "dofs") {
      cur->dofs = read_all<int>(ss);
    } else if (cur && key == "coords") {
      cur->coords = read_all<int>(ss);
    } else if (cur && key == "newmark") {
      ss >> cur->newmark;
    } else if (cur && key == "cons") {
      cur->cons = read_all<int>(ss);
    } else if (cur && key == "p") {
      cur->p = read_all<double>(ss);
    } else if (cur && key == "q") {
      cur->q = read_all<double>(ss);
    } else if (cur && key == "u") {
      cur->u = read_all<double>(ss);
    } else if (cur && key == "road") {
      ss >> cur->road;
    } else if (cur && key == "inputs") {  // name:offset ...
      std::string slot;
      while (ss >> slot) {
        const auto colon = slot.rfind(':');
        if (colon == std::string::npos) fail("inputs: expected name:offset, got '" + slot + "'");
        const std::string name = slot.substr(0, colon);
        int index = -1;
        for (size_t i = 0; i < m.inputs.size(); ++i)
          if (m.inputs[i].name == name) index = static_cast<int>(i);
        if (index < 0) fail("element " + cur->name + ": input '" + name + "' not declared");
        cur->inputs.emplace_back(index, std::stoi(slot.substr(colon + 1)));
      }
    } else if (cur && key == "jacobian") {
      ss >> cur->jacobian;
      if (cur->jacobian != "full" && cur->jacobian != "no_qvv") fail("jacobian must be full or no_qvv");
    } else {
      fail("unexpected '" + key + "'");
    }
  }
  if (m.elements.empty()) throw std::runtime_error(path + ": no elements");
  if (m.solver.newton != "velocity" && m.solver.newton != "legacy" && m.solver.newton != "fixed")
    throw std::runtime_error(path + ": solver newton must be 'velocity', 'legacy' or 'fixed'");
  if (m.solver.newton_iterations < 1) throw std::runtime_error(path + ": solver newton_iterations must be >= 1");
  if (m.solver.error_norm != "mixed" && m.solver.error_norm != "legacy")
    throw std::runtime_error(path + ": solver error_norm must be 'mixed' or 'legacy'");
  return m;
}

}  // namespace fd
