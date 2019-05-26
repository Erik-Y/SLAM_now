set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories( ${PROJECT_SOURCE_DIR}/include )
add_executable( run_vo run_vo.cpp )
target_link_libraries( run_vo myslam )
