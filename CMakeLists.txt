cmake_minimum_required(VERSION 3.16)
project(freedyn2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)  # M_PI; fopen etc. in C code
  add_compile_options(/utf-8)
endif()

find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
find_package(Eigen3 REQUIRED NO_MODULE)  # 3.3 or newer (vcpkg has Eigen 5)
if(Eigen3_VERSION VERSION_LESS 3.3)
  message(FATAL_ERROR "Eigen ${Eigen3_VERSION} found, 3.3 or newer needed")
endif()

# --- Code generation: element C code + registry + model files -------------------
set(FD_GEN_DIR ${PROJECT_BINARY_DIR}/generated)
set(FD_EXAMPLE_NAMES free_body dzhanibekov pendulum double_pendulum
  oscillator torsion_pendulum spring_pendulum two_bodies constant_loads forced_oscillator input_oscillator
  bushing_mount winding_spring spline_slider driven_rotor)
# Examples with expression forces also produce a model plugin (<name>_plugin.so, .dll on Windows).
set(FD_PLUGIN_EXAMPLES forced_oscillator winding_spring spline_slider driven_rotor input_oscillator)
# Included from another project (add_subdirectory, FetchContent): the libraries only, no example
# models and no tests, unless asked for. All paths are relative to this project, not to the top.
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(FD_TOP_LEVEL ON)
else()
  set(FD_TOP_LEVEL OFF)
endif()
# The example model files and plugins are for the tests; a wheel build (pip install) skips them.
option(FD_BUILD_EXAMPLES "generate the example model files and plugins (needed by the tests)" ${FD_TOP_LEVEL})
if(NOT FD_BUILD_EXAMPLES)
  set(FD_EXAMPLE_NAMES)
  set(FD_PLUGIN_EXAMPLES)
endif()
set(FD_EXAMPLES)
set(FD_MODEL_FILES)
foreach(n ${FD_EXAMPLE_NAMES})
  list(APPEND FD_EXAMPLES ${PROJECT_SOURCE_DIR}/examples/${n}.yaml)
  list(APPEND FD_MODEL_FILES ${FD_GEN_DIR}/${n}.fdm)
endforeach()
file(GLOB FD_GENERATOR_SOURCES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/generator/fdgen/*.py)
file(GLOB FD_EXAMPLE_DATA CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/examples/data/*)
# Must match fdgen.codegen.ELEMENT_BUILDERS (checked by tests/python/test_codegen.py).
set(FD_ELEMENT_TYPES
  rigid_body rigid_body_euler
  transfix_ground transfix_ground_euler rotfix_ground rotfix_ground_euler
  transfix_bb transfix_bb_euler rotfix_bb rotfix_bb_euler
  spring_damper_ground spring_damper_ground_euler spring_damper_bb spring_damper_bb_euler
  force_global force_global_euler force_body force_body_euler
  torque_global torque_global_euler torque_body torque_body_euler
  torsion_ground torsion_ground_euler torsion_bb torsion_bb_euler
  bushing_ground bushing_ground_euler bushing_bb bushing_bb_euler
  tire tire_euler tire_local tire_local_euler
  sensor sensor_euler)
set(FD_ELEMENT_SOURCES)
foreach(t ${FD_ELEMENT_TYPES})
  list(APPEND FD_ELEMENT_SOURCES ${FD_GEN_DIR}/${t}.c)
endforeach()
set(FD_GENERATED
  ${FD_ELEMENT_SOURCES}
  ${FD_GEN_DIR}/fd_registry.c ${FD_GEN_DIR}/fd_registry.h
  ${FD_MODEL_FILES})
foreach(n ${FD_PLUGIN_EXAMPLES})
  list(APPEND FD_GENERATED ${FD_GEN_DIR}/${n}_plugin${CMAKE_SHARED_LIBRARY_SUFFIX})
endforeach()
add_custom_command(
  OUTPUT ${FD_GENERATED}
  # run from generator/ (python -m puts it on sys.path): PYTHONPATH stays as it is, which a pip
  # build environment (wheel) uses to provide casadi
  COMMAND ${Python3_EXECUTABLE} -m fdgen ${FD_EXAMPLES} -o ${FD_GEN_DIR}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/generator
  DEPENDS ${FD_GENERATOR_SOURCES} ${FD_EXAMPLES} ${FD_EXAMPLE_DATA} ${PROJECT_SOURCE_DIR}/schema/model.schema.json
  COMMENT "Generating element code and model files with CasADi"
  VERBATIM)

# Generated C is compiled as is; it needs no CasADi library.
add_library(fd_generated STATIC ${FD_ELEMENT_SOURCES} ${FD_GEN_DIR}/fd_registry.c)
target_include_directories(fd_generated PUBLIC ${FD_GEN_DIR})
set_target_properties(fd_generated PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(fd_generated PRIVATE STATIC_LINKED)  # Windows: no dllexport from the executables

# --- ASAM OpenCRG C library (vendored, Apache-2.0): reads and evaluates road files --------
file(GLOB FD_OPENCRG_SOURCES ${PROJECT_SOURCE_DIR}/runtime/third_party/opencrg/src/*.c)
add_library(opencrg STATIC ${FD_OPENCRG_SOURCES})
target_include_directories(opencrg PUBLIC runtime/third_party/opencrg/inc)
set_target_properties(opencrg PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(NOT MSVC)
  target_compile_options(opencrg PRIVATE -w)  # third-party code, compiled as is
  target_link_libraries(opencrg PUBLIC m)
endif()

# --- Runtime ---------------------------------------------------------------------
add_library(freedyn2_runtime STATIC
  runtime/src/generated_function.cpp
  runtime/src/scatter.cpp
  runtime/src/model.cpp
  runtime/src/plugin.cpp
  runtime/src/system.cpp
  runtime/src/linear_solver.cpp
  runtime/src/crg.cpp
  runtime/src/hht.cpp)
target_include_directories(freedyn2_runtime PUBLIC runtime/include)
target_link_libraries(freedyn2_runtime PUBLIC fd_generated opencrg Eigen3::Eigen ${CMAKE_DL_LIBS})

# Optional sparse solver SuiteSparse KLU (LGPL-2.1, linked dynamically), the default if found:
# apt install libsuitesparse-dev, or its CMake package KLU (which brings AMD, COLAMD, BTF along),
# e.g. built from the SuiteSparse sources with only these parts and no BLAS (see the Windows job
# in .github/workflows/ci.yml) and found through CMAKE_PREFIX_PATH.
find_package(KLU CONFIG QUIET)
find_path(KLU_INCLUDE_DIR klu.h PATH_SUFFIXES suitesparse)
find_library(KLU_LIBRARY klu)
if(TARGET SuiteSparse::KLU OR TARGET SuiteSparse::KLU_static)
  message(STATUS "KLU: CMake package")
  target_compile_definitions(freedyn2_runtime PUBLIC FD_WITH_KLU)
  if(TARGET SuiteSparse::KLU)
    target_link_libraries(freedyn2_runtime PRIVATE SuiteSparse::KLU)
  else()
    target_link_libraries(freedyn2_runtime PRIVATE SuiteSparse::KLU_static)
  endif()
elseif(KLU_INCLUDE_DIR AND KLU_LIBRARY)
  message(STATUS "KLU: ${KLU_LIBRARY}")
  target_include_directories(freedyn2_runtime PRIVATE ${KLU_INCLUDE_DIR})
  target_compile_definitions(freedyn2_runtime PUBLIC FD_WITH_KLU)
  target_link_libraries(freedyn2_runtime PRIVATE ${KLU_LIBRARY})
else()
  message(STATUS "KLU not found: linear solver Eigen SparseLU")
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(freedyn2_runtime PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Windows: DLLs of imported libraries (KLU and its parts) next to the programs that need them.
function(fd_copy_runtime_dlls target)
  if(WIN32 AND NOT CMAKE_VERSION VERSION_LESS 3.21)
    add_custom_command(TARGET ${target} POST_BUILD
      COMMAND ${CMAKE_COMMAND} "-DDLLS=$<TARGET_RUNTIME_DLLS:${target}>" "-DDEST=$<TARGET_FILE_DIR:${target}>"
              -P ${PROJECT_SOURCE_DIR}/cmake/copy_dlls.cmake)
  endif()
endfunction()

add_executable(freedyn2_run runtime/app/main.cpp)
target_link_libraries(freedyn2_run PRIVATE freedyn2_runtime)
fd_copy_runtime_dlls(freedyn2_run)
add_executable(freedyn2_road runtime/app/road.cpp)
target_link_libraries(freedyn2_road PRIVATE freedyn2_runtime)
fd_copy_runtime_dlls(freedyn2_road)
# both copy the same DLLs into the same directory: one after the other, not at the same time
# (parallel builds; file(COPY) of two programs at once fails on Windows)
add_dependencies(freedyn2_road freedyn2_run)

# --- Tests -----------------------------------------------------------------------
if(FD_TOP_LEVEL)
  include(CTest)
endif()
if(FD_TOP_LEVEL AND BUILD_TESTING AND FD_BUILD_EXAMPLES)
  add_subdirectory(tests/cpp)
endif()

# --- Python package (pip install / wheel with scikit-build-core) ---------------------
# The runtime programs and their shared libraries go to fdgen/bin, the schema and the examples
# to fdgen/schema and fdgen/examples (fdgen.resources finds them there).
if(SKBUILD)
  if(UNIX AND NOT APPLE)
    set_target_properties(freedyn2_run freedyn2_road PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()
  install(TARGETS freedyn2_run freedyn2_road RUNTIME DESTINATION fdgen/bin)
  set(FD_SHARED_DEPS)
  foreach(t SuiteSparse::KLU SuiteSparse::BTF SuiteSparse::AMD SuiteSparse::COLAMD SuiteSparse::SuiteSparseConfig)
    if(TARGET ${t})
      get_target_property(type ${t} TYPE)
      if(type STREQUAL "SHARED_LIBRARY")
        list(APPEND FD_SHARED_DEPS ${t})
      endif()
    endif()
  endforeach()
  if(FD_SHARED_DEPS)  # KLU and its parts as shared libraries (LGPL: replaceable by the user)
    install(IMPORTED_RUNTIME_ARTIFACTS ${FD_SHARED_DEPS} RUNTIME DESTINATION fdgen/bin LIBRARY DESTINATION fdgen/bin)
  endif()
  install(DIRECTORY schema/ DESTINATION fdgen/schema)
  install(DIRECTORY examples/ DESTINATION fdgen/examples PATTERN "__pycache__" EXCLUDE)
  install(FILES LICENSE NOTICE DESTINATION fdgen)
  # licenses of what the wheel contains besides FreeDyn: OpenCRG (compiled in), SuiteSparse
  # (the KLU libraries, from tools/build_deps.py), three.js (with the viewer)
  install(FILES runtime/third_party/opencrg/LICENSE runtime/third_party/opencrg/NOTICE DESTINATION fdgen/licenses/opencrg)
  if(FD_SHARED_DEPS)
    foreach(dir IN LISTS CMAKE_PREFIX_PATH)
      if(EXISTS "${dir}/share/licenses/suitesparse")
        install(DIRECTORY "${dir}/share/licenses/suitesparse/" DESTINATION fdgen/licenses/suitesparse)
      endif()
    endforeach()
  endif()
endif()
