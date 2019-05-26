#ifndef CAMERA_H
#define CAMERA_H

#include "myslam/common_include.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace myslam
{

// Pinhole RGBD camera model
class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr;
    float   fx_, fy_, cx_, cy_, depth_scale_;  // Camera intrinsics 
    cv::Mat DistCoef = (cv::Mat_<float>(5,1));//<5,1,cv::CV_32F>;
    cv::Mat K;
    Camera();
    Camera ( float fx, float fy, float cx, float cy, float depth_scale=0 ) :
        fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale )
    {}

    // coordinate transform: world, camera, pixel
   
    Vector3d pixel2camera( const Vector2d& p_p, double depth=1 ); 
    Vector2d camera2pixel( const Vector3d& point_3d  );

};

}
#endif 
