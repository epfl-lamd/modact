cmake_minimum_required (VERSION 3.1.0)
project (modact_borg)


SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

find_path(
    MODACT_INCLUDE_DIR NAMES modact_problems.h
    PATHS "${PROJECT_SOURCE_DIR}/.."
)

add_subdirectory(borg_src)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -O3")

find_package (MPI REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(${MODACT_INCLUDE_DIR} "${PROJECT_SOURCE_DIR}/borg" ${MPI_CXX_INCLUDE_PATH} )
add_executable(modact_borg modact_borg.cpp)
target_link_libraries(modact_borg PRIVATE pybind11::embed BorgMS ${MPI_CXX_LIBRARIES})

#target_include_directories(modact_borg PRIVATE ${MPI_CXX_INCLUDE_PATH} ${PYTHON_INCLUDE_DIRS} ${PROJECT_BINARY_DIR})

if(MPI_CXX_COMPILE_FLAGS)
  set_target_properties(modact_borg PROPERTIES
    COMPILE_FLAGS "${MPI_CXX_COMPILE_FLAGS}")
endif()

if(MPI_CXX_LINK_FLAGS)
  set_target_properties(modact_borg PROPERTIES
    LINK_FLAGS "${MPI_CXX_LINK_FLAGS}")
endif()
