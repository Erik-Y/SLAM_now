#ifndef G2O_EDGETYPES_H
#define G2O_EDGETYPES_H

#include "myslam/common_include.h"
#include "myslam/camera.h"
#include <eigen3/Eigen/Core>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

namespace myslam 
{
    
class  Edge_PoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    //构造函数
//     Edge_PoseOnly();
//     Edge_PoseOnly( Eigen::Vector3d point,Camera::Ptr camera );
    //析构函数
    //~Edge_PoseOnly(){}
    
    //误差计算函数
    virtual void computeError();

    
    //Jacobain矩阵计算函数
    virtual void linearizeOplus();

    
    //读取与写入函数
    virtual bool read( std::istream& in ){}
    virtual bool write(std::ostream& os) const {};
    
public:
    Eigen::Vector3d point_; 
    myslam::Camera::Ptr camera_;
};

}

#endif //G2O_EDGETYPES_H
    
