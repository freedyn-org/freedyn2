// Acceptance tests of the FreeDyn 2.0 runtime on generated models.
// Usage: fd_tests <test>. Each test prints "RESULT <test> <metric> <value> tol <tol> ok|FAIL" and
// fails if any of its metrics exceeds its tolerance.

#include <cstdio>
#include <exception>

#include "test_support.hpp"

int main(int argc, char** argv) {
  Tests tests;
  add_rigid_body_tests(tests);
  add_joint_tests(tests);
  add_force_tests(tests);
  if (argc != 2 || !tests.count(argv[1])) {
    std::fprintf(stderr, "usage: fd_tests <test>; tests:");
    for (const auto& t : tests) std::fprintf(stderr, " %s", t.first.c_str());
    std::fprintf(stderr, "\n");
    return 2;
  }
  try {
    tests.at(argv[1])();
  } catch (const std::exception& e) {
    std::printf("exception: %s\n", e.what());
    return 1;
  }
  return fdtest::all_ok() ? 0 : 1;
}
