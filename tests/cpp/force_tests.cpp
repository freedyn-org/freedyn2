// Acceptance tests: forces (library, model plugin) and drivers, on the shipped examples.
// Each test prints "RESULT <test> <metric> <value> tol <tol>" (fdtest::report).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "test_support.hpp"

using namespace fdtest;

namespace {

// 9. Forces. Channels by name; each model is a shipped example (examples/*.yaml).
// Damped oscillator m z'' + d z' + k z = -m g, z(0) = z'(0) = 0, relative to the static offset.
double oscillator_error(double h) {
  const Run r = run("oscillator", h);
  const double m = 2, k = 200, d = 4, g = 9.81;
  const double wn = std::sqrt(k / m), z = d / (2 * std::sqrt(k * m)), wd = wn * std::sqrt(1 - z * z);
  const double x0 = -m * g / k, A = -x0, B = z * wn * A / wd;
  double e = 0;
  for (size_t i = 0; i < r.t.size(); ++i) {
    const double t = r.t[i];
    const double ref = x0 + std::exp(-z * wn * t) * (A * std::cos(wd * t) + B * std::sin(wd * t));
    e = std::max(e, std::abs(r.y.at("mass.R_z")[i] - ref) / std::abs(x0));
  }
  return e;
}

void test_forces() {
  const char* T = "forces";
  const double e1 = oscillator_error(1e-3), e2 = oscillator_error(5e-4);
  report(T, "oscillator vs analytic (rel)", e1, 1e-4);
  report(T, "oscillator: |2 - order|", std::abs(2 - std::log2(e1 / e2)), 0.2);

  {  // Expression force from the model plugin: m z'' = -k z - d z' + F0 sin(w t), z(0) = z'(0) = 0.
    const Run r = run("forced_oscillator");
    const double m = 2, k = 200, d = 4, F0 = 10, w = 7;
    const double det = (k - m * w * w) * (k - m * w * w) + d * w * d * w;
    const double a = F0 * (k - m * w * w) / det, b = -F0 * d * w / det;  // z_p = a sin + b cos
    const double s = d / (2 * m), wd = std::sqrt(k / m - s * s), C = -b, D = (s * C - a * w) / wd;
    double e = 0, zmax = 0;
    for (size_t i = 0; i < r.t.size(); ++i) {
      const double t = r.t[i];
      const double ref = a * std::sin(w * t) + b * std::cos(w * t) + std::exp(-s * t) * (C * std::cos(wd * t) + D * std::sin(wd * t));
      e = std::max(e, std::abs(r.y.at("mass.R_z")[i] - ref));
      zmax = std::max(zmax, std::abs(ref));
    }
    report(T, "expression force (plugin) vs analytic (rel)", e / zmax, 1e-4);
  }
  {  // Bushing at the center of mass: translation linear and decoupled; energy conserved.
    // The translation error is the phase error of the trapezoidal rule, (w h)^2 / 12 per radian,
    // dominated by the stiffest mode (w = 38.7 rad/s, 1e-2 after 2 s at h = 1e-3).
    const double m = 2, k[3] = {1000, 2000, 3000}, kr[3] = {10, 20, 30}, v0[3] = {0.01, -0.02, 0.03};
    auto bushing = [&](double h, double* dE) {
      const Run r = run("bushing_mount", h);
      double ex = 0, E0 = 0, xmax = 0;
      *dE = 0;
      for (size_t i = 0; i < r.t.size(); ++i) {
        double E = r.y.at("block.T")[i];
        for (int c = 0; c < 3; ++c) {
          const std::string s = std::string("_") + "xyz"[c];
          const double w = std::sqrt(k[c] / m), ref = v0[c] / w * std::sin(w * r.t[i]);
          ex = std::max(ex, std::abs(r.y.at("block.R" + s)[i] - ref));
          xmax = std::max(xmax, std::abs(ref));
          const double d = r.y.at("mount.displacement" + s)[i], psi = r.y.at("mount.rotation" + s)[i];
          E += 0.5 * k[c] * d * d + 0.5 * kr[c] * psi * psi;
        }
        if (i == 0) E0 = E;
        *dE = std::max(*dE, std::abs(E - E0) / E0);
      }
      return ex / xmax;
    };
    double dE1, dE2;
    const double e1 = bushing(1e-3, &dE1), e2 = bushing(5e-4, &dE2);
    const double w = std::sqrt(k[2] / m), phase = w * w * 1e-6 / 12 * w * 2.0;
    report(T, "bushing: translation error / phase error", std::abs(e1 / phase - 1), 0.1);
    report(T, "bushing: |2 - order|", std::abs(2 - std::log2(e1 / e2)), 0.1);
    report(T, "bushing: max |E - E0| / E0 (alpha = -0.01)", dE1, 5e-5);
  }
  {  // Continuous angle measure over several revolutions: J phi'' = -k phi, phi'(0) = 20.
    const Run r = run("winding_spring");
    const double w = std::sqrt(4.0 / 1.0), amp = 20.0 / w;
    double e = 0, tmax = 0;
    for (size_t i = 0; i < r.t.size(); ++i) {
      e = std::max(e, std::abs(-r.y.at("spring.value")[i] / 4.0 - amp * std::sin(w * r.t[i])));
      tmax = std::max(tmax, std::abs(r.y.at("spring.value")[i]));
    }
    report(T, "winding spring: angle vs analytic (rel)", e / amp, 1e-4);
    report(T, "winding spring: angle passes pi (max |phi| / pi > 3)", tmax / 4.0 / M_PI > 3 ? 0.0 : 1.0, 0.5);
  }
  {  // Spline force law (Akima on linear data, exactly linear): m z'' = -k z, z'(0) = 0.5.
    const Run r = run("spline_slider");
    const double w = std::sqrt(200.0 / 2.0), amp = 0.5 / w;
    double e = 0;
    for (size_t i = 0; i < r.t.size(); ++i) e = std::max(e, std::abs(r.y.at("mass.R_z")[i] - amp * std::sin(w * r.t[i])));
    // Phase error of the trapezoidal rule (w h)^2 / 12 per radian: 4.2e-4 after 5 s at w = 10.
    report(T, "spline slider vs analytic (rel)", e / amp, 1e-3);
  }
  {  // Driver: rotor angle w t (rate from the velocity projection with Phi_t), flywheel on a
     // torsion spring: psi = -(w / om) sin(om t), om^2 = k / J2; driver torque = k psi.
    const Run r = run("driven_rotor");
    const double w = 2.0, k = 10.0, om = std::sqrt(k / 0.4);
    double e = 0, ew = 0, el = 0;
    for (size_t i = 0; i < r.t.size(); ++i) {
      const double ref = -(w / om) * std::sin(om * r.t[i]), psi = r.y.at("coupling.angle")[i];
      e = std::max(e, std::abs(psi - ref));
      ew = std::max(ew, std::abs(r.y.at("rotor.omega_z")[i] - w));
      el = std::max(el, std::abs(r.y.at("motor.lambda")[i] - k * psi));
    }
    report(T, "driver: flywheel vs analytic (rel)", e / (w / om), 2e-4);
    report(T, "driver: rotor rate - w", ew, 1e-9);
    // The multiplier also carries J1 times the discrete (HHT) acceleration of the driven rotor.
    report(T, "driver: |torque - k psi| / (k psi_max)", el / (k * w / om), 1e-5);
  }
  {  // Torsion pendulum J phi'' + d phi' + k phi = 0, phi(0) = 0, phi'(0) = 1.
    const Run r = run("torsion_pendulum");
    const double J = 0.5, k = 8, d = 0.2;
    const double wn = std::sqrt(k / J), z = d / (2 * std::sqrt(k * J)), wd = wn * std::sqrt(1 - z * z);
    double e = 0;
    for (size_t i = 0; i < r.t.size(); ++i) {
      const double ref = std::exp(-z * wn * r.t[i]) * std::sin(wd * r.t[i]) / wd;
      e = std::max(e, std::abs(r.y.at("spring.angle")[i] - ref) * wd);
    }
    report(T, "torsion pendulum vs analytic (rel)", e, 5e-5);
  }
  {  // Elastic 3D pendulum without damping: energy (alpha = -0.01).
    const Run r = run("spring_pendulum");
    double dE = 0, T_max = 0;
    const auto E = [&](size_t i) {
      const double l = r.y.at("spring.length")[i] - 0.8;
      return r.y.at("bob.T")[i] + 9.81 * r.y.at("bob.R_z")[i] + 0.5 * 100 * l * l;
    };
    for (size_t i = 0; i < r.t.size(); ++i) {
      dE = std::max(dE, std::abs(E(i) - E(0)));
      T_max = std::max(T_max, r.y.at("bob.T")[i]);
    }
    report(T, "spring pendulum max |E - E0| / T_max", dE / T_max, 1e-5);
  }
  {  // Internal forces only: momentum exact, angular momentum to the integration error.
    const Run r = run("two_bodies");
    const double ma = 1, mb = 2;
    double dp = 0, dL = 0, L0n = 0;
    double L0[3], P0[3];
    for (size_t i = 0; i < r.t.size(); ++i) {
      double P[3], L[3];
      for (int c = 0; c < 3; ++c) {
        const std::string s = std::string("_") + "xyz"[c];
        P[c] = ma * r.y.at("a.Rdot" + s)[i] + mb * r.y.at("b.Rdot" + s)[i];
        L[c] = r.y.at("a.H" + s)[i] + r.y.at("b.H" + s)[i];
      }
      for (const auto& [b, mm] : {std::pair<const char*, double>{"a", ma}, {"b", mb}}) {
        const std::string B = b;
        const double R[3] = {r.y.at(B + ".R_x")[i], r.y.at(B + ".R_y")[i], r.y.at(B + ".R_z")[i]};
        const double V[3] = {r.y.at(B + ".Rdot_x")[i], r.y.at(B + ".Rdot_y")[i], r.y.at(B + ".Rdot_z")[i]};
        L[0] += mm * (R[1] * V[2] - R[2] * V[1]);
        L[1] += mm * (R[2] * V[0] - R[0] * V[2]);
        L[2] += mm * (R[0] * V[1] - R[1] * V[0]);
      }
      if (i == 0) {
        std::copy(L, L + 3, L0);
        std::copy(P, P + 3, P0);
        L0n = std::sqrt(L[0] * L[0] + L[1] * L[1] + L[2] * L[2]);
      }
      for (int c = 0; c < 3; ++c) {
        dp = std::max(dp, std::abs(P[c] - P0[c]));
        dL = std::max(dL, std::abs(L[c] - L0[c]) / L0n);
      }
    }
    report(T, "two bodies: max |p - p0| [kg m/s]", dp, 1e-13);
    report(T, "two bodies: max |L - L0| / |L0|", dL, 1e-5);
  }
  {  // Constant global force and torque on a free body.
    const Run r = run("constant_loads");
    const double F[3] = {1.0, -2.0, 0.5}, M[3] = {0.2, 0.0, -0.1}, m = 2;
    double eR = 0, eH = 0;
    const char* s[] = {"_x", "_y", "_z"};
    for (size_t i = 0; i < r.t.size(); ++i)
      for (int c = 0; c < 3; ++c) {
        const double t = r.t[i];
        eR = std::max(eR, std::abs(r.y.at(std::string("body.R") + s[c])[i] - 0.5 * F[c] / m * t * t));
        eH = std::max(eH, std::abs(r.y.at(std::string("body.H") + s[c])[i] - r.y.at(std::string("body.H") + s[c])[0] - M[c] * t));
      }
    report(T, "constant force: max |R - F t^2 / 2m| [m]", eR, 1e-11);
    report(T, "constant torque: max |H - H0 - M t|", eH, 1e-5);
  }
}

// Runtime inputs (examples/input_oscillator.yaml): changed between steps with set_input.
void test_inputs() {
  const char* T = "inputs";
  const fd::Model model = example("input_oscillator");
  {
    fd::System sys(model);
    report(T, "two inputs declared", std::abs(sys.n_inputs() - 2.0), 0.0);
    report(T, "unknown name: index -1", std::abs(sys.input_index("nope") + 1.0), 0.0);
    bool threw = false;
    try {
      sys.set_input(7, 1.0);
    } catch (const std::out_of_range&) {
      threw = true;
    }
    report(T, "index out of range throws", threw ? 0.0 : 1.0, 0.0);
  }
  // Step of the force F from 0 to F1 at t1 (set for the steps that end after t1), and the driven
  // cart following a ramp of z_set that starts at t2. Reference: the step response of
  // m z'' + d z' + k z = F1 from rest at t1. An input is constant over a step, so the step
  // response is off by O(h): halving h halves the error.
  const double m = 2, k = 200, d = 4, F1 = 10, t1 = 0.5, t2 = 1.0, rate = 0.2;
  const double wn = std::sqrt(k / m), zeta = d / (2 * std::sqrt(k * m)), wd = wn * std::sqrt(1 - zeta * zeta);
  const double zs = F1 / k;
  double z_before = 0, e_cart = 0;
  auto step_error = [&](double h) {
    fd::Model mh = model;
    mh.solver.step = h;
    double e_force = 0;
    int iF = -1, iZ = -1;
    simulate(mh, [&](double t, const Vec& y, fd::System& sys) {
      if (iF < 0) iF = sys.input_index("F"), iZ = sys.input_index("z_set");
      const int iz = sys.output_index("mass.R") + 2, izc = sys.output_index("cart.R") + 2;  // z components
      if (t <= t1 + 1e-12) {
        z_before = std::max(z_before, std::abs(y[iz]));
      } else {
        const double tau = t - t1;
        const double ref = zs * (1 - std::exp(-zeta * wn * tau) * (std::cos(wd * tau) + zeta * wn / wd * std::sin(wd * tau)));
        e_force = std::max(e_force, std::abs(y[iz] - ref) / zs);
      }
      // the driver holds the cart on z_set at every step end (the value set for that step)
      e_cart = std::max(e_cart, std::abs(y[izc] - sys.input(iZ)));
      if (t + h > t1 + 1e-12) sys.set_input(iF, F1);  // for the step that ends at t + h
      sys.set_input(iZ, t + h > t2 ? rate * (t + h - t2) : 0.0);
    });
    return e_force;
  };
  const double e1 = step_error(1e-3), e2 = step_error(5e-4);
  report(T, "no motion before the force step [m]", z_before, 1e-14);
  report(T, "force step: |z - step response| / (F1 / k), h = 1 ms", e1, 5e-3);
  report(T, "force step: error ratio h / (h/2) - 2 (first order)", std::abs(e1 / e2 - 2), 0.1);
  report(T, "driver follows z_set [m]", e_cart, 1e-9);
}

}  // namespace

void add_force_tests(Tests& tests) {
  tests["forces"] = test_forces;
  tests["inputs"] = test_inputs;
}
