#pragma once
// Road surface from ASAM OpenCRG files, evaluated with the official OpenCRG C library
// (runtime/third_party/opencrg): height z(x, y), its gradient and the friction coefficient,
// for tire elements that take the road as a local plane (*_local element types; System updates
// their road parameters at the wheel position).
//
// The files are read and evaluated as the standard defines (all data formats, reference line,
// slope, banking, options and modifiers of the file, border modes). Modifiers: those of the
// file if it has a modifiers section; else none but filling NaN grid values (keep last), so the
// road keeps the coordinates of the file. The gradient is the central difference of the
// library's height over +-1e-4 m.

#include <string>
#include <vector>

namespace fd {

struct RoadSample {
  double z = 0.0, dzdx = 0.0, dzdy = 0.0, mu = 1.0;
};

// Height CRG placed in the world (rotation phi about z, then translation), friction constant
// or from a second CRG on the same placement (its grid values are mu).
class Road {
 public:
  struct Placement {
    double x = 0.0, y = 0.0, z = 0.0, phi = 0.0;
  };
  Road(const std::string& height_crg, const Placement& placement, double friction,
       const std::string& friction_crg = "");
  ~Road();
  Road(const Road&) = delete;
  Road& operator=(const Road&) = delete;

  // A query point (a wheel): OpenCRG contact points keep the search history of one point.
  int add_point();
  RoadSample eval(int point, double x, double y) const;
  // OpenCRG road coordinates (no placement): (u, v) -> (x, y), z; for checks.
  bool eval_uv(int point, double u, double v, double* x, double* y, double* z) const;

  double u_min() const { return u_min_; }
  double u_max() const { return u_max_; }
  double v_min() const { return v_min_; }
  double v_max() const { return v_max_; }

 private:
  double height(int cp, double xl, double yl) const;

  int height_set_ = 0, friction_set_ = 0;  // OpenCRG data set ids (0: none)
  std::vector<int> height_cp_, friction_cp_;
  Placement place_;
  double mu_ = 1.0, c_ = 1.0, s_ = 0.0;
  double u_min_ = 0.0, u_max_ = 0.0, v_min_ = 0.0, v_max_ = 0.0;
};

}  // namespace fd
