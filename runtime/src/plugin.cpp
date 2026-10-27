#include "freedyn2/plugin.hpp"

#include <map>
#include <stdexcept>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace fd {

namespace {
struct Plugin {
  const fd_element_type* types;
  int n;
};
std::map<std::string, Plugin>& plugins() {
  static std::map<std::string, Plugin> p;  // by path
  return p;
}

void* open_library(const std::string& path, std::string& err) {
#ifdef _WIN32
  void* h = reinterpret_cast<void*>(LoadLibraryA(path.c_str()));
  if (!h) err = "LoadLibrary failed, error " + std::to_string(GetLastError());
  return h;
#else
  void* h = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!h) err = dlerror();
  return h;
#endif
}

void* symbol(void* h, const char* name) {
#ifdef _WIN32
  return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(h), name));
#else
  return dlsym(h, name);
#endif
}
}  // namespace

void load_plugin(const std::string& path) {
  if (plugins().count(path)) return;
  std::string err;
  void* h = open_library(path, err);
  if (!h) throw std::runtime_error("cannot load model plugin " + path + ": " + err);
  auto abi = reinterpret_cast<const char* (*)()>(symbol(h, "fd_plugin_abi"));
  auto types = static_cast<const fd_element_type*>(symbol(h, "fd_plugin_element_types"));
  auto n = static_cast<const int*>(symbol(h, "fd_plugin_n_element_types"));
  if (!abi || !types || !n) throw std::runtime_error(path + " is not a FreeDyn 2.0 model plugin");
  if (std::string(abi()) != FD_REGISTRY_ABI)
    throw std::runtime_error(path + ": generated for registry layout '" + abi() + "', this runtime has '" +
                             FD_REGISTRY_ABI + "'; regenerate the model");
  plugins()[path] = {types, *n};
}

const fd_element_type* find_plugin_element_type(const std::string& name) {
  for (const auto& [path, p] : plugins())
    for (int i = 0; i < p.n; ++i)
      if (name == p.types[i].name) return &p.types[i];
  return nullptr;
}

}  // namespace fd
