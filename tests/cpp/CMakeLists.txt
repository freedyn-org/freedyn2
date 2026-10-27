add_executable(fd_tests main.cpp test_support.cpp rigid_body_tests.cpp joint_tests.cpp force_tests.cpp)
target_link_libraries(fd_tests PRIVATE freedyn2_runtime)
fd_copy_runtime_dlls(fd_tests)
target_compile_definitions(fd_tests PRIVATE FD_GEN_DIR="${FD_GEN_DIR}")

foreach(t fd_element fd_global free_body_alpha0 free_body_alpha01 precession dzhanibekov linear_solvers fast_rotor)
  add_test(NAME rigid_body.${t} COMMAND fd_tests ${t})
endforeach()
foreach(t pendulum double_pendulum newton_fixed)
  add_test(NAME joints.${t} COMMAND fd_tests ${t})
endforeach()
add_test(NAME forces.all COMMAND fd_tests forces)

# End-to-end: the command line runner on the shipped example.
add_test(NAME runner.free_body
  COMMAND freedyn2_run ${FD_GEN_DIR}/free_body.fdm -o ${CMAKE_CURRENT_BINARY_DIR}/free_body.csv)
