#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"
#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{
    
// forward declare 
class MapPoint;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    

    vector<double>                 depths_;

    SE3                            T_c_w_;
    
    
    
    unsigned long                  id_;         // id of this frame
    double                         time_stamp_; // when it is recorded
 
    Mat                            color_, depth_; // color and depth image 
    
    
    float depth_scale_ ;
    
public: // data members 
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Mat color=Mat(), Mat depth=Mat() );
    
    ~Frame();
    
    // factory function
    static Frame::Ptr createFrame(); 

    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp);


};

}

#endif // FRAME_H
