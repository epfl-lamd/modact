cmake_minimum_required(VERSION 3.9)
project(modact_nomad)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(USE_MPI "build with MPI support" ON)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(NOMAD_ROOT "$ENV{NOMAD_HOME}" CACHE FILEPATH "Path to nomad root directory")

find_package(pybind11 REQUIRED)  # or `add_subdirectory(pybind11)`
find_package(MPI)

find_path(
    NOMAD_INCLUDE_DIR NAMES nomad.hpp sgtelib.hpp
    PATHS "${NOMAD_ROOT}"
    PATH_SUFFIXES src ext/sgtelib/src
)
find_path(
    NOMAD_SGTELIB_INCLUDE_DIR NAMES sgtelib.hpp
    PATHS "${NOMAD_ROOT}"
    PATH_SUFFIXES ext/sgtelib/src
)
find_library(
    NOMAD_LIBRARY NAMES nomad
    PATHS "${NOMAD_ROOT}"
    PATH_SUFFIXES lib
)
mark_as_advanced(NOMAD_INCLUDE_DIR NOMAD_SGTELIB_INCLUDE_DIR NOMAD_LIBRARY)
find_package_handle_standard_args(
    NOMAD REQUIRED_VARS
    NOMAD_INCLUDE_DIR
    NOMAD_SGTELIB_INCLUDE_DIR
    NOMAD_LIBRARY
)
add_library(nomad SHARED IMPORTED)

set_property(TARGET nomad
    PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    "${NOMAD_INCLUDE_DIR}" "${NOMAD_SGTELIB_INCLUDE_DIR}"
)

set_target_properties(nomad PROPERTIES IMPORTED_LOCATION "${NOMAD_LIBRARY}")

find_path(
    MODACT_INCLUDE_DIR NAMES modact_problems.h
    PATHS "${modact_nomad_SOURCE_DIR}/.."
)

include_directories(${MODACT_INCLUDE_DIR} ${NOMAD_INCLUDE_DIR} ${NOMAD_SGTELIB_INCLUDE_DIR})
add_executable(modact_nomad modact_nomad.cpp)
target_link_libraries(modact_nomad PRIVATE pybind11::embed ${NOMAD_LIBRARY})

if (APPLE)
  add_custom_command(TARGET modact_nomad 
      POST_BUILD COMMAND 
      ${CMAKE_INSTALL_NAME_TOOL} -change "libnomad.so" "${NOMAD_LIBRARY}"
      $<TARGET_FILE:modact_nomad>)
endif()

if(MPI_FOUND AND USE_MPI)
  find_library(
      NOMAD_MPI_LIBRARY NAMES nomad.MPI
      PATHS "${NOMAD_ROOT}"
      PATH_SUFFIXES lib
  )
  add_library(nomad.MPI SHARED IMPORTED)
  set_property(TARGET nomad.MPI
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES
      "${NOMAD_INCLUDE_DIR}" "${NOMAD_SGTELIB_INCLUDE_DIR}"
  )
  set_target_properties(nomad.MPI PROPERTIES IMPORTED_LOCATION "${NOMAD_MPI_LIBRARY}")
  add_executable(modact_nomad_mpi modact_nomad.cpp)

  target_link_libraries(modact_nomad_mpi PRIVATE pybind11::embed ${NOMAD_MPI_LIBRARY} MPI::MPI_C)
  target_compile_options(modact_nomad_mpi PRIVATE -DUSE_MPI)

  add_custom_command(TARGET modact_nomad_mpi 
    POST_BUILD COMMAND 
    ${CMAKE_INSTALL_NAME_TOOL} -change "libnomad.MPI.so" "${NOMAD_MPI_LIBRARY}"
    $<TARGET_FILE:modact_nomad_mpi>)
endif()
