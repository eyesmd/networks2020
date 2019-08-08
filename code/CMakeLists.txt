cmake_minimum_required(VERSION 2.8.4)
project(networks2019)

# Set compiler flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++14 -m64")
if ("${CMAKE_BUILD_TYPE}" MATCHES release OR "${CMAKE_BUILD_TYPE}" MATCHES Release)
    message("-- Compiling in Release Mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
elseif ("${CMAKE_BUILD_TYPE}" MATCHES debug OR "${CMAKE_BUILD_TYPE}" MATCHES Debug)
    message("-- Compiling in Debug Mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
endif()

# Add include directory.
include_directories("include/")

# Including GOC.
add_subdirectory(goc)
include_directories(goc/include)

# Create binaries.
add_executable(networks2019 src/main.cpp src/vrp_instance.cpp src/preprocess_capacity.cpp src/preprocess_time_windows.cpp src/preprocess_service_waiting.cpp src/preprocess_travel_times.cpp src/label.cpp src/monodirectional_labeling.cpp src/lazy_label.cpp src/pwl_domination_function.cpp src/bidirectional_labeling.cpp src/preprocess_triangle_depot.cpp)
target_link_libraries(networks2019 goc)
target_link_libraries(networks2019)