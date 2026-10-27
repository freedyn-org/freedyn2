// freedyn2_road: evaluate an OpenCRG road at points, to check a road file and its placement.
//
//   freedyn2_road road.crg [--place x y z phi] [--friction-crg mu.crg] [--uv | --grid cell n] < points
//
// Reads "x y" per line from stdin, writes "x y z dz/dx dz/dy mu" per line (world frame).
// --uv: reads "u v" (road coordinates of the file, no placement) and writes "u v x y z".
// --grid cell n: no input; a grid over the whole road (u, v lines about cell apart, at most n
// points), for drawing it: "rows cols", then "x y z mu" (world frame) per point, row-major in u.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include "freedyn2/crg.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: freedyn2_road road.crg [--place x y z phi] [--friction-crg mu.crg] [--uv | --grid cell n] < points\n";
    return 2;
  }
  try {
    fd::Road::Placement place;
    std::string friction_crg;
    bool uv = false;
    double grid_cell = 0.0, grid_points = 0.0;
    for (int i = 2; i < argc; ++i) {
      if (!std::strcmp(argv[i], "--place") && i + 4 < argc) {
        place.x = std::stod(argv[++i]), place.y = std::stod(argv[++i]);
        place.z = std::stod(argv[++i]), place.phi = std::stod(argv[++i]);
      } else if (!std::strcmp(argv[i], "--friction-crg") && i + 1 < argc) {
        friction_crg = argv[++i];
      } else if (!std::strcmp(argv[i], "--uv")) {
        uv = true;
      } else if (!std::strcmp(argv[i], "--grid") && i + 2 < argc) {
        grid_cell = std::stod(argv[++i]), grid_points = std::stod(argv[++i]);
      } else {
        std::cerr << "unknown option " << argv[i] << '\n';
        return 2;
      }
    }
    fd::Road road(argv[1], place, 1.0, friction_crg);
    std::fprintf(stderr, "%s: u %g .. %g, v %g .. %g\n", argv[1], road.u_min(), road.u_max(), road.v_min(), road.v_max());
    const int point = road.add_point();
    if (grid_cell > 0) {
      const double lu = road.u_max() - road.u_min(), lv = road.v_max() - road.v_min();
      int nu = std::max(2, int(lu / grid_cell) + 1), nv = std::max(2, int(lv / grid_cell) + 1);
      if (double(nu) * nv > grid_points) {
        const double f = std::sqrt(double(nu) * nv / grid_points);
        nu = std::max(2, int(nu / f)), nv = std::max(2, int(nv / f));
      }
      const double c = std::cos(place.phi), s = std::sin(place.phi);
      std::printf("%d %d\n", nu, nv);
      for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nv; ++j) {
          double x, y, z;
          const double u = road.u_min() + lu * i / (nu - 1), v = road.v_min() + lv * j / (nv - 1);
          if (!road.eval_uv(point, u, v, &x, &y, &z)) throw std::runtime_error("no road at u, v");
          const double xw = place.x + c * x - s * y, yw = place.y + s * x + c * y;
          const fd::RoadSample r = road.eval(point, xw, yw);
          std::printf("%.9g %.9g %.7g %.5g\n", xw, yw, r.z, r.mu);
        }
      }
      return 0;
    }
    double a, b;
    while (std::cin >> a >> b) {
      if (uv) {
        double x, y, z;
        if (!road.eval_uv(point, a, b, &x, &y, &z)) throw std::runtime_error("no road at u, v");
        std::printf("%.17g %.17g %.17g %.17g %.17g\n", a, b, x, y, z);
      } else {
        const fd::RoadSample s = road.eval(point, a, b);
        std::printf("%.10g %.10g %.17g %.17g %.17g %.17g\n", a, b, s.z, s.dzdx, s.dzdy, s.mu);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
