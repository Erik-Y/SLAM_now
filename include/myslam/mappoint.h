#ifndef MAPPOINT_H
#define MAPPOINT_H

namespace myslam
{
    
class Frame;
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long      id_; // ID
    vector<cv::Point3f>     kp_3d_;//3d points used for g2o
    vector<cv::KeyPoint>    kp_2d_;//2d keypoints used for 
    cv::Mat                 dpts_;//descriptors used for matching
    cv::Point3f    pos_;       // Position in world
    Vector3d    norm_;      // Normal of viewing direction 
    Mat         descriptor_; // Descriptor for matching 
    int         observed_times_;    // being observed by feature matching algo.
    int         matched_times_;     // being an inliner in pose estimation
    
    MapPoint();
    MapPoint( long id, cv::Point3f position, Mat descriptor );
    
    // factory function
    static MapPoint::Ptr createMapPoint(cv::Point3f position, Mat descriptor);
};
}

#endif // MAPPOINT_H
