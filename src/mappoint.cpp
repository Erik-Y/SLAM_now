#include "myslam/common_include.h"
#include "myslam/mappoint.h"

namespace myslam
{

MapPoint::MapPoint()
: id_(-1), pos_(cv::Point3f(0,0,0)), norm_(Vector3d(0,0,0)), observed_times_(0)
{

}

MapPoint::MapPoint ( long id, cv::Point3f position, Mat descriptor )
: id_(id), pos_(position), observed_times_(0)
{
    descriptor.copyTo(descriptor_);

}

MapPoint::Ptr MapPoint::createMapPoint( cv::Point3f position, Mat descriptor)
{
    static long factory_id = 0;
    return MapPoint::Ptr( 
        new MapPoint( factory_id++, position, descriptor)
    );
}

}
