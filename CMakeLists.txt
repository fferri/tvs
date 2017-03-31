cmake_minimum_required(VERSION 2.8.12)
project(tracked_motion)

get_filename_component(PARENT_DIR ${tracked_motion_SOURCE_DIR} DIRECTORY) # needs cmake 2.8.12

# to find SDL2
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${tracked_motion_SOURCE_DIR}/cmake")

add_definitions(-DDRAWSTUFF_TEXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/textures")
add_definitions(-DPOINTCLOUDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/pointclouds")

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree sample_consensus)
find_package(octomap REQUIRED)
find_package(SDL2 REQUIRED)

# we need ompl-1.0.0, other version break in different ways
# if ../ompl is there, we use that, otherwise we look for system's ompl
set(OMPL_DIR ${PARENT_DIR}/ompl)
if(EXISTS ${OMPL_DIR} AND IS_DIRECTORY ${OMPL_DIR}/src)
        set(OMPL_INCLUDE_DIRS ${OMPL_DIR}/src)
        file(GLOB OMPL_LIBRARIES "${OMPL_DIR}/build/Release/lib/*${CMAKE_SHARED_LIBRARY_SUFFIX}")
else()
        find_package(ompl REQUIRED)
endif()

set(ODE_DIR ${PARENT_DIR}/ode)
if(EXISTS ${ODE_DIR} AND IS_DIRECTORY ${ODE_DIR}/ode)
        # this code uses the drawstuff library which is normally private to ODE, so we need to hack it from the local build
        set(ODE_INCLUDE_DIRS ${ODE_DIR}/include)
        file(GLOB ODE_LIBRARIES "${ODE_DIR}/ode/src/.libs/*${CMAKE_SHARED_LIBRARY_SUFFIX}")
        file(GLOB DRAWSTUFF_LIBRARIES "${ODE_DIR}/drawstuff/src/.libs/*${CMAKE_STATIC_LIBRARY_SUFFIX}")
else()
        message(FATAL "this project needs to access the ODE src tree at ${ODE_DIR}")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-nullability-completeness -Wno-redeclared-class-member")

set(LIBS
        ${DRAWSTUFF_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
        ${SDL2_LIBRARY}
        ${OMPL_LIBRARIES}
        ${ODE_LIBRARIES}
)

include_directories(
        include
        ${OPENGL_INCLUDE_DIR}
        ${GLUT_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS}
        ${ODE_INCLUDE_DIRS}
        ../ode/ode/src/
)

add_library(tvs_common OBJECT
        src/Environment.cpp
        src/OcTreeBuilder.cpp
        src/TriMesh.cpp
        src/STLFile.cpp
        src/ODEUtils.cpp
        src/utils.cpp

        src/TrackKinematicModel.cpp
        src/PlanarJoint.cpp
        src/TrackBase.cpp
        src/Flipper.cpp
        src/Track.cpp
        src/TrackedVehicle.cpp
        src/TrackedVehicleEnvironment.cpp

        src/SkidSteeringVehicle.cpp
        src/SkidSteerVehicleEnvironment.cpp

        src/SimpleTrackBase.cpp
        src/SimpleFlipper.cpp
        src/SimpleTrack.cpp
        src/SimpleTrackedVehicle.cpp
        src/SimpleTrackedVehicleEnvironment.cpp
)

add_executable(simulator $<TARGET_OBJECTS:tvs_common> src/simulator.cpp)
target_link_libraries(simulator ${LIBS})

add_executable(planner $<TARGET_OBJECTS:tvs_common> src/OMPLTVSControlSpace.cpp src/OMPLTVSEnvironment.cpp src/OMPLTVSSimpleSetup.cpp src/OMPLTVSStatePropagator.cpp src/OMPLTVSStateSpace.cpp src/OMPLTVSStateValidityChecker.cpp src/planner.cpp)
target_link_libraries(planner ${LIBS})

add_executable(search-vis $<TARGET_OBJECTS:tvs_common> src/search_vis.cpp)
target_link_libraries(search-vis ${LIBS})

