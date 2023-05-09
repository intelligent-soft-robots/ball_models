cmake_minimum_required(VERSION 3.12)
project(ball_models)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

################
# Dependencies #
################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)

ament_export_dependencies(
  pybind11)

# Include pybind11 and Eigen headers
include_directories(${PYBIND11_DIR}/include ${EIGEN3_INCLUDE_DIRS})

# prepare to export all needed targets
set(all_targets)
set(all_target_exports)

###########
# Library #
###########

add_library(${PROJECT_NAME} SHARED
  src/ball_trajectory.cpp
  src/racket_contact_model.cpp
  src/table_contact_model.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})

###################
# Python wrappers #
###################

add_library(${PROJECT_NAME}_wrp MODULE srcpy/wrappers.cpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME}_wrp PRIVATE pybind11::module)
target_link_libraries(${PROJECT_NAME}_wrp PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_wrp PRIVATE ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_wrp
                      PROPERTIES PREFIX "" SUFFIX "${PYTHON_MODULE_EXTENSION}")
target_include_directories(
  ${PROJECT_NAME}_wrp
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
install(TARGETS ${PROJECT_NAME}_wrp DESTINATION ${PYTHON_INSTALL_DIR})

######################
# Python Native code #
######################

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

######################
# Install and export #
######################

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${all_targets}
  EXPORT ${all_target_exports}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DEST-----
    
  INATION include)
ament_package()