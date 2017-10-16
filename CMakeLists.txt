cmake_minimum_required(VERSION 3.1)
project(tracked_motion)

set(CMAKE_CXX_STANDARD 11)

get_filename_component(PARENT_DIR ${tracked_motion_SOURCE_DIR} DIRECTORY) # needs cmake 2.8.12

# to find SDL2
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${tracked_motion_SOURCE_DIR}/cmake")

add_definitions(-DDRAWSTUFF_TEXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/textures")
add_definitions(-DPOINTCLOUDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/pointclouds")

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(PCL COMPONENTS common filters io kdtree sample_consensus)
find_package(octomap)
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
        ${SDL2_LIBRARY}
        ${OMPL_LIBRARIES}
        ${ODE_LIBRARIES}
)

include_directories(
    ${CMAKE_BUILD_DIR}
        include
        ${OPENGL_INCLUDE_DIR}
        ${GLUT_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS}
        ${ODE_INCLUDE_DIRS}
        ../ode/ode/src/
)

configure_file(${CMAKE_SOURCE_DIR}/config.h.in ${CMAKE_BINARY_DIR}/config.h)

set(SOURCES
        src/Environment.cpp
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

if(PCL_FOUND)
    set(LIBS ${LIBS} ${PCL_LIBRARIES})
    include_directories(${PCL_INCLUDE_DIRS})
endif()

if(OCTOMAP_FOUND)
    set(SOURCES ${SOURCES} src/OcTreeBuilder.cpp)
    set(LIBS ${LIBS} ${OCTOMAP_LIBRARIES})
    include_directories(${OCTOMAP_INCLUDE_DIRS})
endif()

add_library(tvs_common OBJECT ${SOURCES})

add_executable(simulator $<TARGET_OBJECTS:tvs_common> src/simulator.cpp)
target_link_libraries(simulator ${LIBS})

#add_executable(planner $<TARGET_OBJECTS:tvs_common> src/OMPLTVSControlSpace.cpp src/OMPLTVSEnvironment.cpp src/OMPLTVSSimpleSetup.cpp src/OMPLTVSStatePropagator.cpp src/OMPLTVSStateSpace.cpp src/OMPLTVSStateValidityChecker.cpp src/planner.cpp)
#target_link_libraries(planner ${LIBS})

add_executable(search-vis $<TARGET_OBJECTS:tvs_common> src/search_vis.cpp)
target_link_libraries(search-vis ${LIBS})

