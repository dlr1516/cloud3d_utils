CMAKE_MINIMUM_REQUIRED(VERSION 3.9)
PROJECT(cloud3d_utils)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# #find packages
#find_package(OpenMP REQUIRED)

# rofl
find_package(rofl REQUIRED)
message(STATUS "rofl_FOUND ${rofl_FOUND}")
message(STATUS "rofl_INCLUDE_DIRS ${rofl_INCLUDE_DIRS}")
message(STATUS "rofl_LIBRARY_DIRS ${rofl_LIBRARY_DIRS}")
message(STATUS "rofl_LIBRARIES ${rofl_LIBRARIES}")

if(${rofl_FOUND})
  include_directories(${rofl_INCLUDE_DIRS})
  link_directories(${rofl_LIBRARY_DIRS})
endif()

# Eigen
find_package(Eigen3 REQUIRED)
message(STATUS "EIGEN3_INCLUDE_DIR " ${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

# PCL
find_package(PCL REQUIRED) # COMPONENTS common io base filters search kdtree registration visualization

if(${PCL_FOUND})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  add_definitions(-DENABLE_PCL_REGISTRATION)
endif()

# include directories
include_directories(include)

# Creates an ars3d_test library (many applications share these methods)
add_library(cloud3d_utils
  src/cloud_io.cpp
  src/cloud_transform.cpp
  src/cloud_metadata.cpp
)
target_compile_features(cloud3d_utils PRIVATE cxx_std_17)


add_executable(create_dataset
  src/create_dataset.cpp
  src/cloud_io.cpp
  src/cloud_transform.cpp
  src/cloud_metadata.cpp
)
target_link_libraries(create_dataset ${rofl_LIBRARIES} ${PCL_LIBRARIES} cloud3d_utils)


