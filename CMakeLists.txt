cmake_minimum_required( VERSION 2.8 )
project ( myslam )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Debug" )
#set( CMAKE_CXX_FLAGS "-std=c++11 "  )
#set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )
set( CMAKE_CXX_FLAGS "-std=c++11  -O3" )

#cmake模块路径(FindG2O.cmake和FindCSparse.cmake)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
# G2O
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

############### source and test ######################
add_subdirectory( src )
add_subdirectory( test )
