project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

if (SHOW_TRAJECTORY)
    add_definitions(-DPLOT_TRAJECTORY=1)
endif(SHOW_TRAJECTORY)

if (VERBOSE_OUT)
    add_definitions(-DVERBOSE_OUT=1)
endif(VERBOSE_OUT)

if (TRACE_IPOPT)
    add_definitions(-DTRACE_IPOPT=1)
endif(TRACE_IPOPT)

if (EXIT_ON_IPOPT_FAILURE)
    add_definitions(-DEXIT_ON_IPOPT_FAILURE=1)
endif(EXIT_ON_IPOPT_FAILURE)

if (COLLECT_DATA)
    add_definitions(-DCOLLECT_DATA=1)
endif(COLLECT_DATA)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/control/MPC.cpp src/model/Vehicle.cpp src/model/RoadGeometry.cpp src/utils/utils.cpp src/utils/Config.cpp)
include_directories(libs)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_library(mpclib STATIC ${sources})

add_executable(mpc src/mpc_main.cpp)

target_link_libraries(mpc mpclib ipopt z ssl uv uWS)

add_executable(testmpc src/test.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
target_link_libraries(testmpc mpclib ipopt python2.7)
else()
target_link_libraries(testmpc mpclib ipopt python)
endif()
