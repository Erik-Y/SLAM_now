#include "myslam/g2o_edgetypes.h"

namespace myslam
{
    
// Edge_PoseOnly::Edge_PoseOnly():g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>() {}
// 
// Edge_PoseOnly::Edge_PoseOnly( Eigen::Vector3d point,Camera::Ptr camera ): BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>(),_point(point), _camera(camera){}

//Edge_PoseOnly::~Edge_PoseOnly(){}

void Edge_PoseOnly::computeError()
{
    const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    _error = _measurement - camera_->camera2pixel( v->estimate().map(point_));
}

void Edge_PoseOnly::linearizeOplus()
{
    const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0] );
    Eigen::Vector3d xyz = v->estimate().map( point_);
    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];
    double z_2 = z*z;

    
    _jacobianOplusXi ( 0,0 ) =  x*y/z_2 *camera_->fx_;
    _jacobianOplusXi ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) *camera_->fx_;
    _jacobianOplusXi ( 0,2 ) = y/z * camera_->fx_;
    _jacobianOplusXi ( 0,3 ) = -1./z * camera_->fx_;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = x/z_2 * camera_->fx_;

    _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) *camera_->fy_;
    _jacobianOplusXi ( 1,1 ) = -x*y/z_2 *camera_->fy_;
    _jacobianOplusXi ( 1,2 ) = -x/z *camera_->fy_;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1./z *camera_->fy_;
    _jacobianOplusXi ( 1,5 ) = y/z_2 *camera_->fy_;
}



}
