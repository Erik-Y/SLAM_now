#include "myslam/frame.h"
#include "myslam/config.h"


namespace myslam
{
Frame::Frame()
: id_(-1), time_stamp_(-1)
{
    depth_scale_ = Config::get<float>("camera.depth_scale");
}
Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Mat color, Mat depth )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), color_(color), depth_(depth)
{

    depth_scale_ = Config::get<float>("camera.depth_scale");
}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return  make_shared<myslam::Frame>(Frame(factory_id++)) ;
}


//return the depth of input keypoint, if the depth >0,
//return -1.0 if the depth = 0;
double Frame::findDepth ( const cv::KeyPoint& kp )//接受一个特征点，返回深度信息
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];
    if ( d!=0 )
    {
        return double(d)/depth_scale_;
    }
    else 
    {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/depth_scale_;
            }
           
        }
    }
    return -1.0;
}



}
