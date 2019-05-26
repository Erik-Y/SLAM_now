#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <algorithm>
#include <boost/timer.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/block_solver.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
//#include<g2o/core/sparse_optimizer.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <sophus/se3.h>

#include "myslam/config.h"
#include "myslam/g2o_edgetypes.h"
#include "myslam/visual_odometry.h"
#include "myslam/mappoint.h"
#include "myslam/map.h"


namespace myslam
{

VisualOdometry::VisualOdometry(): 
     state_ ( INITIALIZING ), frame_refe_( nullptr ), frame_curr_( nullptr ), num_lost_ ( 0 ), num_inliers_ ( 0 ),camera_(nullptr), matcher_( new cv::flann::LshIndexParams(5,10,2)),localmap_(new Map())
{
    //localmap_ = new Map;
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    
    orb_=cv::ORB::create( num_of_features_, scale_factor_, level_pyramid_ );
   
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    max_pose6d_norm_    = Config::get<double> ( "max_pose6d_norm" );
    
}

VisualOdometry::~VisualOdometry()
{

}

bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case INITIALIZING:
    {
        //将当前帧和参考帧都置为刚输入进来的帧，把当前帧插入到关键帧容器中，将3d特征点以及对应的描述子放到参考pts_3d_ref_，descriptors_ref_
        state_ = OK;
        frame_curr_ = frame;
        
        keypoints_detect();//extract keypoints in current frame
        descriptors_compute();//compute descriptors of current keypoints
        show_kps_= curr_kps_;
        UndistortKeyPoints();
        curr2refe();//transform information of current frame to reference frame
        frame_refe_ = frame_curr_;
         
        break;
    }
    case OK:
    {
        static int flag = 0;
        
        frame_curr_ = frame;
        keypoints_detect();//extract keypoints in current frame
        descriptors_compute();//compute descriptors of current keypoints
        show_kps_= curr_kps_;
        UndistortKeyPoints();
       
        features_match();
        poseEstimationPnP();//计算SE3形式的变换矩阵，保存于T_c_r_estimated_，正确点个数保存于num_inliers
        addMappoints();
        if( CheckEstimatePose() )
        {
            
            curr2refe();//transform information of current frame to reference frame
            frame_curr_->T_c_w_ = T_c_r_estimated_ * frame_refe_->T_c_w_;  // T_c_w = T_c_r*T_r_w  
            frame_refe_ = frame_curr_;
            num_lost_ = 0;
        }
        else
        {
            ++num_lost_;
            if( num_lost_ >= max_num_lost_ )
                state_ = LOST;
            return false;
        }
        
        break;
    }


    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
}
void VisualOdometry::keypoints_detect()
{
    curr_kps_.clear();
    orb_->detect( frame_curr_->color_, curr_kps_);
}
void VisualOdometry::descriptors_compute()
{
    orb_->compute(frame_curr_->color_, curr_kps_,curr_dpts_);
}

void VisualOdometry::features_match()
{
    //use OpenCV's brute force match 
    vector<cv::DMatch> matches_raw;
   // cv::FlannBasedMatcher   matcher;
   // cv::BFMatcher matcher ( cv::NORM_HAMMING );
    matcher_.match ( refe_dpts_, curr_dpts_, matches_raw );
    
    // select the best matches
    float min_dis = std::min_element (
                        matches_raw.begin(), matches_raw.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;
    
    matches_.clear();
    for (auto m : matches_raw )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            processinfo.write("dist_of_match",frame_curr_->time_stamp_, m.distance );
            matches_.push_back(m);
        }
    }
    cout<<"good matches: "<<matches_.size()<<endl;
}



void VisualOdometry::poseEstimationPnP()//计算SE3形式的变换矩阵，保存于T_c_r_estimated_，正确点个数保存于num_inliers
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    vector<cv::KeyPoint> kps;
    
    for ( auto m:matches_ )
    {
        pts3d.push_back( kps_3d_[m.queryIdx] );
        pts2d.push_back( curr_kps_[m.trainIdx].pt );
        kps.push_back(  curr_kps_[m.trainIdx] );
        
    }
    
    Mat rvec, tvec;
    cv::Mat   inliers;   
    //随机采样一致性求解PNP，其中inliers是正确数据的容器，每一行包含一个正确点的信息，用于判断计算结果是否可取
   // cv::solvePnPRansac( pts3d, pts2d, camera_->K, camera_->DistCoef, rvec, tvec, false, 100, 4.0, 0.99, inliers_ );
    cv::solvePnPRansac( pts3d, pts2d, camera_->K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
    
    num_inliers_ = inliers.rows;
    processinfo.write("num_of_inliers", frame_curr_->time_stamp_, num_inliers_);
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_r_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
    //inliers.copyTo(inliers_kf_);
    inliers_kps_2d_.clear();
    for( int i = 0; i != inliers.rows; ++i )
    {
        inliers_kps_2d_.push_back(kps[inliers.at<int>(i,0)]);//可以和上面合并一下，能够减少时间。
    }
   
   //g2o优化
    typedef g2o::BlockSolverTraits<6,2>  Traits_6_2;
    typedef g2o::BlockSolver< Traits_6_2 > blocksolver_6_2;
  
    unique_ptr<blocksolver_6_2::LinearSolverType> linearsolver( new g2o::LinearSolverDense< blocksolver_6_2::PoseMatrixType>() );
    unique_ptr<blocksolver_6_2> blocksolver_6_2_ptr( new  blocksolver_6_2( move(linearsolver)));
    
    g2o::OptimizationAlgorithmLevenberg* optialgorithm_ptr( new g2o::OptimizationAlgorithmLevenberg(move( blocksolver_6_2_ptr ) ));
    
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( optialgorithm_ptr );
    
    //添加顶点
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap;
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(
        T_c_r_estimated_.rotation_matrix(),
        T_c_r_estimated_.translation()
                              ) );
    optimizer.addVertex(pose);
   
    for( int i = 0; i != inliers.rows; ++i )
    {
        int index = inliers.ptr<int>(i)[0];
        myslam::Edge_PoseOnly*  edge = new myslam:: Edge_PoseOnly() ;
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->camera_ = camera_;
        edge->point_ = Eigen::Vector3d(pts3d[index].x,pts3d[index].y,pts3d[index].z);
        edge->setMeasurement(Eigen::Vector2d(pts2d[index].x,pts2d[index].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge( edge );
    }

    optimizer.initializeOptimization();
    optimizer.optimize(20);
    T_c_r_estimated_ = Sophus::SE3(
        pose->estimate().rotation(),
        pose->estimate().translation()
                                    );
    
    
}

//eliminate the points without good depth in the curr_keypoints, 
//and save the rest keypoints to refe_kps_ and  refe_dpts_;
//save 3dpoints of new reference keypoints to kps_3d_;
void  VisualOdometry::curr2refe()
{
    if(!curr_kps_.empty())
    {   
          
        refe_kps_.clear();
        kps_3d_.clear();
        cv::Mat temp_dpts;
        
        for( unsigned i = 0; i != curr_kps_.size(); ++i )
        {
            double d = frame_curr_->findDepth(curr_kps_[i]);
            if( d > 0)
            {
                refe_kps_.push_back( curr_kps_[i] );
                temp_dpts.push_back( curr_dpts_.row(i));
                kps_3d_.push_back(get_3d_point( curr_kps_[i], d));
            }
        }
        temp_dpts.copyTo(refe_dpts_);
    }
}
                

cv::Point3f VisualOdometry::get_3d_point( cv::KeyPoint& kp, double d)
{
    Eigen::Vector3d p(camera_->pixel2camera( Eigen::Vector2d(kp.pt.x, kp.pt.y), d ));    
    return cv::Point3f( p(0,0), p(1,0), p(2,0) );
}
            

bool VisualOdometry::CheckEstimatePose()
{
    Sophus::Vector6d pose6d = T_c_r_estimated_.log();
   
    processinfo.write( "norm_pose6d",frame_curr_->time_stamp_ , pose6d.norm() );
    if( num_inliers_ < min_inliers_ )
    {
        cout <<"pose is rejected  because lacking of inliers !!!" <<endl;
        return false;
    }

    if( pose6d.norm() > max_pose6d_norm_ )
    {
        cout <<"pose is rejected  because motion is too large !!!" <<endl;
        return false;
    }
    return true;
}
void VisualOdometry::UndistortKeyPoints()
{
    if(camera_->DistCoef.at<float>(0)==0.0)
    {
        
        return;
    }

    // Fill matrix with points，N是keypoints的总数
    cv::Mat mat(curr_kps_.size(),2,CV_32F);//N行2列
    for(int i=0; i<curr_kps_.size(); i++)
    {
        mat.at<float>(i,0)=curr_kps_[i].pt.x;
        mat.at<float>(i,1)=curr_kps_[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);//变为N行1列2通道，reshape(int cn, int rows = 0);cn表示通道数，rows表示行数，=0表示与原来相同，不copy
    cv::undistortPoints(mat,mat,camera_->K,camera_->DistCoef,cv::Mat(),camera_->K);//矫正畸变
    mat=mat.reshape(1);//变为N行2列1通道


    for(int i =0; i != curr_kps_.size(); ++i )
    {
        curr_kps_[i].pt.x=mat.at<float>(i,0);
        curr_kps_[i].pt.y=mat.at<float>(i,1);
    }
}
void VisualOdometry::addMappoints()
{   
     if(localmap_->map_points_.size() <= 3000)
     { 
        cv::KeyPoint kp;
        cv::Mat descriptor;
        
        for( int i = 0; i != inliers_kf_.rows; ++i )
        {   
            int idx =matches_[inliers_kf_.at<int>(i,0)].trainIdx;
            curr_dpts_.row(idx).copyTo(descriptor);
            kp = curr_kps_[idx];
            double d = frame_curr_->findDepth(curr_kps_[idx]);
            
            MapPoint::Ptr mappoint = MapPoint::createMapPoint( get_3d_point(kp, d), descriptor );
            localmap_->insertMapPoint(mappoint);
       }
    }
    cout <<"mappoints_numbers: "<<localmap_->map_points_.size()<<endl;
}
void VisualOdometry::cullingMappoints()
{
}



}
