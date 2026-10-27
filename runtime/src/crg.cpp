#include "freedyn2/crg.hpp"

#include <cmath>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

extern "C" {
#include "crgBaseLib.h"
}

namespace fd {

namespace {

const double kGradientStep = 1e-4;  // [m], central difference of the height

// Whether the header of an OpenCRG file has a modifiers section ($ROAD_CRG_MODS).
bool has_modifiers_section(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind("$$$$", 0) == 0) break;  // the data follow
    std::string up;
    for (char c : line) up += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (up.rfind("$ROAD_CRG_MODS", 0) == 0) return true;
  }
  return false;
}

int load(const std::string& path) {
  crgMsgSetLevel(dCrgMsgLevelWarn);
  const int id = crgLoaderReadFile(path.c_str());
  if (id <= 0) throw std::runtime_error("cannot read OpenCRG file " + path);
  // As the OpenCRG tools: a failed check (e.g. a reference line curvature above the file's
  // warning limits) is reported; the data are used as read.
  if (!crgCheck(id)) std::fprintf(stderr, "warning: OpenCRG file %s: consistency check failed\n", path.c_str());
  // Modifiers (OpenCRG 6.4.6): a file with a modifiers section gets exactly those. Otherwise
  // the application's defaults apply; the library's would move the reference point to the
  // origin, FreeDyn's keep the coordinates of the file (placement: road.position / heading)
  // and only fill NaN grid values with the last valid value of their cross section.
  if (!has_modifiers_section(path)) {
    crgDataSetModifierRemoveAll(id);
    crgDataSetModifierSetInt(id, dCrgModGridNaNMode, dCrgGridNaNKeepLast);
  }
  crgDataSetModifiersApply(id);
  return id;
}

}  // namespace

Road::Road(const std::string& height_crg, const Placement& placement, double friction,
           const std::string& friction_crg)
    : place_(placement), mu_(friction), c_(std::cos(placement.phi)), s_(std::sin(placement.phi)) {
  height_set_ = load(height_crg);
  if (!friction_crg.empty()) friction_set_ = load(friction_crg);
  crgDataSetGetURange(height_set_, &u_min_, &u_max_);
  crgDataSetGetVRange(height_set_, &v_min_, &v_max_);
}

Road::~Road() {
  if (friction_set_ > 0) crgDataSetRelease(friction_set_);
  if (height_set_ > 0) crgDataSetRelease(height_set_);
}

int Road::add_point() {
  const int h = crgContactPointCreate(height_set_);
  if (h < 0) throw std::runtime_error("OpenCRG: cannot create a contact point");
  height_cp_.push_back(h);
  if (friction_set_ > 0) {
    const int f = crgContactPointCreate(friction_set_);
    if (f < 0) throw std::runtime_error("OpenCRG: cannot create a contact point");
    friction_cp_.push_back(f);
  }
  return static_cast<int>(height_cp_.size()) - 1;
}

double Road::height(int cp, double xl, double yl) const {
  double z = 0.0;
  if (!crgEvalxy2z(cp, xl, yl, &z))
    throw std::runtime_error("OpenCRG: no height at x = " + std::to_string(xl) + ", y = " + std::to_string(yl));
  return z;
}

RoadSample Road::eval(int point, double x, double y) const {
  // world -> CRG frame: rotate by -phi about the placement point
  const double ex = x - place_.x, ey = y - place_.y;
  const double xl = c_ * ex + s_ * ey, yl = -s_ * ex + c_ * ey;
  const int cp = height_cp_[point];
  const double h = kGradientStep;
  RoadSample r;
  r.z = height(cp, xl, yl) + place_.z;
  const double gx = (height(cp, xl + h, yl) - height(cp, xl - h, yl)) / (2 * h);
  const double gy = (height(cp, xl, yl + h) - height(cp, xl, yl - h)) / (2 * h);
  r.dzdx = c_ * gx - s_ * gy;  // gradient back to the world frame
  r.dzdy = s_ * gx + c_ * gy;
  if (friction_set_ > 0) {
    if (!crgEvalxy2z(friction_cp_[point], xl, yl, &r.mu)) throw std::runtime_error("OpenCRG: no friction value");
  } else {
    r.mu = mu_;
  }
  return r;
}

bool Road::eval_uv(int point, double u, double v, double* x, double* y, double* z) const {
  const int cp = height_cp_[point];
  return crgEvaluv2xy(cp, u, v, x, y) && crgEvaluv2z(cp, u, v, z);
}

}  // namespace fd
