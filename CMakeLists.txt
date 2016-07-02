cmake_minimum_required (VERSION 2.6)

project (path_planner)

if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RELEASE)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11 -O3 -DEIGEN_NO_DEBUG -w")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -DEIGEN_NO_DEBUG -w")
SET(BOOST_MIN_VERSION "1.54.0")
SET(PYTHON_MIN_VERSION "2.7.0")
include(FindPkgConfig)
include(GNUInstallDirs)

set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/include")
############################# LOAD PYTHON #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(PYTHON python-2.7)
    if(NOT PYTHON_FOUND)
       message(FATAL_ERROR "Python could not be found")
    endif()
    include_directories(${PYTHON_INCLUDE_DIRS})
    link_directories(${PYTHON_LIBRARY_DIRS})    
endif()

############################# LOAD BOOST #############################
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS python system thread timer)
if (NOT Boost_FOUND)
    set(Boost_INCLUDE_DIRS "${BOOST_INCLUDE_DIRS}")
    IF ( NOT Boost_INCLUDE_DIRS )
        message(FATAL_ERROR "Please point the environment variable BOOST_INCLUDE_DIRS to the include directory of Boost")
    ENDIF()
endif ()
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

############################# LOAD ROBOTS #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(ROBOTS robots)
    if(NOT ROBOTS_FOUND)
       message(FATAL_ERROR "ROBOTS could not be found")
    endif()
    include_directories(${ROBOTS_INCLUDE_DIRS})
    link_directories(${ROBOTS_LIBRARY_DIRS})     
endif()


############################# LOAD ROBOT_ENVIRONMENT #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(ROBOT_ENVIRONMENT robot_environment)
    if(NOT ROBOTS_FOUND)
       message(FATAL_ERROR "ROBOT_ENVIRONMENT could not be found")
    endif()
    include_directories(${ROBOT_ENVIRONMENT_INCLUDE_DIRS})
    link_directories(${ROBOT_ENVIRONMENT_LIBRARY_DIRS})     
endif()

############################# LOAD OMPL #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(OMPL ompl)
    if(NOT OMPL_FOUND)
       message(FATAL_ERROR "OMPL could not be found")
    endif()
    include_directories(${OMPL_INCLUDE_DIRS})
    link_directories(${OMPL_LIBRARY_DIRS})    
endif()

############################# LOAD EIGEN #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(EIGEN eigen3)
    if(NOT EIGEN_FOUND)
       message(FATAL_ERROR "EIGEN could not be found")
    endif()
    include_directories(${EIGEN_INCLUDE_DIRS})
    link_directories(${EIGEN_LIBRARY_DIRS})    
endif()
            
add_library(path_planner SHARED 
            ${CMAKE_CURRENT_SOURCE_DIR}/src/path_planner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamic_path_planner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/MotionValidator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/RobotGoalRegion.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/GoalRegion.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/state_propagator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/control_sampler.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/control_space.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/DirectedControlSampler.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/EST_control.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/PlanningSpaceInformation.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/rrt_control.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/python_binds.cpp)
            
target_link_libraries (path_planner
                       ${Boost_LIBRARIES}
                       ${PYTHON_LIBRARIES}
                       ${ROBOTS_LIBRARIES}
                       ${OMPL_LIBRARIES}
                       ${ROBOT_ENVIRONMENT_LIBRARIES})
                       
set(pkg_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/path_planner.pc.in")
set(pkg_conf_file_out "${CMAKE_CURRENT_BINARY_DIR}/path_planner.pc")
configure_file("${pkg_conf_file_in}" "${pkg_conf_file_out}" @ONLY)
install(FILES "${pkg_conf_file_out}" DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
install(TARGETS path_planner DESTINATION ${CMAKE_INSTALL_LIBDIR})


install(FILES ${INCLUDE_DIR}/control_sampler.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/control_space.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/DirectedControlSampler.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/dynamic_path_planner.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/EST_control.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/GoalRegion.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/RobotGoalRegion.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/MotionValidator.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/path_planner.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/PlanningSpaceInformation.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/rrt_control.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/state_propagator.hpp  DESTINATION include/path_planner)
install(FILES ${INCLUDE_DIR}/trajectory.hpp  DESTINATION include/path_planner)
#install(FILES "${pkg_conf_file_out}" DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
