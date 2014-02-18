

include_directories(${CMAKE_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${SUITESPARSE_INCLUDE_DIR} ${G2O_INCLUDE_DIR})

set(PROGRAMS 2D_optimizer_g2o 3D_from_disk_g2o)

foreach(program ${PROGRAMS})
  add_executable(${PROJECT_NAME}_${program} ${program}.cpp)
  target_link_libraries(${PROJECT_NAME}_${program} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})
endforeach(program)
