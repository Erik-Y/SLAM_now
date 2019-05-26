set( THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)

include_directories( ${PROJECT_SOURCE_DIR}/include )

set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
add_library( myslam SHARED
    process_info.cpp
    config.cpp
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    g2o_edgetypes.cpp
    visual_odometry.cpp
    
)#compile cpp documents to shared library called myslam

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)
