# add_library(Borg borg.c)
add_library(mt19937ar mt19937ar.c)

find_package (MPI REQUIRED)
#include_directories(${MPI_INCLUDE_DIRS})
add_library(BorgMS borgms.c)

if(MPI_COMPILE_FLAGS)
  set_target_properties(BorgMS PROPERTIES
    COMPILE_FLAGS "${MPI_COMPILE_FLAGS}")
endif()

if(MPI_LINK_FLAGS)
  set_target_properties(BorgMS PROPERTIES
    LINK_FLAGS "${MPI_LINK_FLAGS}")
endif()

target_include_directories(BorgMS PRIVATE ${MPI_C_INCLUDE_PATH})
target_link_libraries(BorgMS mt19937ar ${MPI_C_LIBRARIES})