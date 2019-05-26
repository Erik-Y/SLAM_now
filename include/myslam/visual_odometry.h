#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/mappoint.h"
#include "myslam/map.h"
#include <myslam/process_info.h>
#include <eigen3/Eigen/Core>

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;//VisualOdometry的数据成员，这个数据成员是一个指向VisualOdometry类对象的指针
    enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    ProcessInfo processinfo;
    VOState     state_;
    Frame::Ptr              frame_refe_, frame_curr_;
    Map::Ptr                localmap_;
    cv::Ptr<cv::ORB>        orb_;
    std::vector<cv::KeyPoint>    refe_kps_, curr_kps_,show_kps_;//keypoints
    cv::Mat                 refe_dpts_,curr_dpts_;//descriptors
    

    std::vector<cv::DMatch>      matches_;
    cv::FlannBasedMatcher        matcher_;
    cv::Mat                      inliers_kf_, inliers_map_;   
    std::vector<cv::Point3f>     kps_3d_;//3d points used for PnP
	std::vector<cv::Point3f>     refe_points_3d_, curr_points_3d_;
    Camera::Ptr             camera_;
    std::vector<double>          refe_depths_, curr_depths_;
    
    std::vector<cv::KeyPoint>     inliers_kps_2d_;
    
    SE3 T_c_r_estimated_;  // the estimated pose of current frame 
    
    int num_of_features_;   // number of features
    double scale_factor_;   // scale in image pyramid
    int level_pyramid_;     // number of pyramid levels
    
    int num_inliers_;        // number of inlier features in icp
    int max_pose6d_norm_;
    int num_lost_;           // number of lost times
    
    
    float match_ratio_;      // ratio for selecting  good matches
    int max_num_lost_;      // max number of continuous lost times
    int min_inliers_;       // minimum inliers
    
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames
    
public: // functions 
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame( Frame::Ptr frame );      // add a new frame 
    bool CheckEstimatePose();
    
protected:  
    // inner operation 
    
    void keypoints_detect();
    void descriptors_compute();
    void UndistortKeyPoints();
    void features_match();
    
    void poseEstimationPnP();
    
    void curr2refe();
    cv::Point3f get_3d_point( cv::KeyPoint& kp, double d);
    void get_3d_points();
    void addMappoints();
    void cullingMappoints();

    
};
}

#endif // VISUALODOMETRY_H
