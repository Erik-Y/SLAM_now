// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 

#include <myslam/config.h>
#include "myslam/visual_odometry.h"
using namespace std;
using namespace myslam;
int main ( int argc, char** argv )
{
//     if ( argc != 2 )
//     {
//         cout<<"usage: run_vo parameter_file"<<endl;
//         return 1;
//     }
    
    //将default.yaml的路径及名称关联到config_->file_,这是一个cv::FileStorage类型
    myslam::Config::setParameterFile ("../config/default.yaml");
    int showTraj   = Config::get<int> ( "ShowTraj" );
    
    //将file_中dataset_dir对应的内容（数据集路径）取出
    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    
    //将associate.txt文件关联到ifstream对象fin
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }
    
    //将图片的路径/名称保存到vector中
    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        //atof将c风格字符串转化为double的函数，在stdlib.h中
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        //将图片的绝对路径/名称补充完整
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )//有错误产生时返回false,中断读取
            break;
    }

    
    
    //可视化
    //第一步：创建名为Visual Odometry的可视化窗口
    cv::viz::Viz3d vis("Visual Odometry");
    
    //第二步：创建坐标系部件，括号中为坐标系大小参数
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    //这里设置坐标系部件属性，然后添加到视图窗口上去
    //  首先利用setRenderingProperty()函数设置渲染属性，
    //  第一个参数是个枚举，对应要渲染的属性这里是线宽，后面是属性值
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    //用showWidget()函数将部件添加到窗口内
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor );
    
    
    //第三步、设置视角。这步是非必要步骤，进行设置有利于观察
    //  不设置也会有默认视角，就是可能比较别扭。而且开始后拖动鼠标，也可以改变观察视角。 
    //  构建三个3D点,这里其实就是构造makeCameraPose()函数需要的三个向量：
    //  相机位置坐标、相机焦点坐标、相机y轴朝向 
    //  蓝色-Z，红色-X，绿色-Y
    //  由这三个参数，用makeCameraPose()函数构造Affine3d类型的相机位姿，这里其实是视角位姿，
    //  也就是程序开始时你的观察视角
    cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose( cam_pose );
    
    //画面暂留，用于调试，等待q或e键
    vis.spin();
    
    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    
    //打开将被写入位姿数据的文件
    ofstream fo("./data.txt");
    if(!fo)
    {
        cout <<"failed to open file" <<endl;
        return -1;
    }
    fo.setf(ios::fixed,ios::floatfield);
    fo <<"# ground truth trajectory" <<endl;
    fo <<"# file: 'rgbd_dataset_freiburg1_xyz.bag'" <<endl;
    fo <<"# timestamp tx ty tz qx qy qz qw" <<endl;
    //定义指向Camera对象的指针，只需要一个
    myslam::Camera::Ptr camera (new myslam::Camera);
    
    //建立一个VisualOdometry对象的指针
    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);
    vo->camera_ = camera;
    vo->processinfo.create_file();
    
    //相机原始位置，用于绘制运动轨迹
    cv::Point3d line_begin = cv::Point3d( 0.0, 0.0, 0.0 );
    
    for ( int i=0; i<rgb_files.size(); i++ )

    {
        //imread参数默认为1，若为>0则读取3通道图片，若为=0，则读取灰度图片，若为<0则读取原图片
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            break;
        
        //创建指向Frame类对象的指针，由于每一个指针的ID要连续增长，故通过成员函数构建
        Frame::Ptr pFrame = myslam::Frame::createFrame();
        
        //对该Frame指针指向的对象赋值
        
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        // 声明一个计时器对象，并开始计时!
        boost::timer timer;
        
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed()<<endl <<endl;
        
        if ( vo->state_ == myslam::VisualOdometry::LOST )
        {   cout<<"vo stoped because LOST!!!"<<endl;
            break;
        }
        SE3 Tcw = vo->frame_refe_->T_c_w_.inverse();
        
        //将位姿数据写入文件
        fo <<pFrame->time_stamp_ <<" " <<Tcw.translation().transpose() <<" " <<Tcw.unit_quaternion().coeffs().transpose() <<endl;
        
      
        
        //可视化
        //**相机位姿
        cv::Affine3d M(
            cv::Affine3d::Mat3( 
                Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
                Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
                Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
            )
        );
        //**更新相机位姿
        vis.setWidgetPose( "Camera", M);
        
        if(showTraj == 1)
        {
        //**移动轨迹
            //****当前帧所处的空间位置
            cv::Point3d line_end = cv::Point3d( Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0) );
            //****构建两帧图片间的位移线段
            cv::viz::WLine line( line_begin, line_end, cv::viz::Color::green() );
            //****显示两帧图片间的相机位移
            string id = to_string(i);
            vis.showWidget(id, line);
            //****将当前帧所处的空间位置置为参考帧位置
            line_begin = line_end;
        
        }
       //**显示1毫秒
        vis.spinOnce(1, false);
        
        //显示对应的照片
        cv::Mat img_with_kp=color;
        vector<cv::KeyPoint> inlier_kp;
//         for(int i = 0; i != vo->inliers_.rows; ++i )
//         {
//             inlier_kp.push_back(vo->refe_kps_[vo->matches_[vo->inliers_.at<int>(i,0)].trainIdx]);
//         }
//         cv::drawKeypoints(color, inlier_kp, img_with_kp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
//         cv::imshow("image",img_with_kp );
       
//         for(int i = 0; i != vo->inliers_kf_.rows; ++i )
//         {
//             inlier_kp.push_back(vo->show_kps_[vo->matches_[vo->inliers_kf_.at<int>(i,0)].trainIdx]);
//         }
        for( auto kp : vo->inliers_kps_2d_ )
        {
            inlier_kp.push_back(kp);
        }
        cv::drawKeypoints(color, vo->inliers_kps_2d_, img_with_kp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
        cout<<"num of inlier_keypoints "<<vo->inliers_kps_2d_.size()<<endl;
        //cv::imshow("image",img_with_kp );
        cv::imshow("image",color );
        cv::waitKey(1);
        

    }
    cout <<"Work Done!" <<endl;

    return 0;
}
