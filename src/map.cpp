#include "myslam/map.h"

namespace myslam
{

void Map::insertKeyFrame ( Frame::Ptr frame )
{
    cout<<"Key frame size = "<<keyframes_.size()<<endl;
    //如果不含有这个帧，插入到末尾
    //**make_pair是pair的构造函数
    if ( keyframes_.find(frame->id_) == keyframes_.end() )
    {
        keyframes_.insert( make_pair(frame->id_, frame) );
        
    }
    else
    {
        keyframes_[ frame->id_ ] = frame;//如果含有这个帧，替换原来的
    }
}

void Map::insertMapPoint ( MapPoint::Ptr map_point )
{
    if ( map_points_.find(map_point->id_) == map_points_.end() )
    {
        map_points_.insert( make_pair(map_point->id_, map_point) );
    }
    else 
    {
        map_points_[map_point->id_] = map_point;
    }
}


}
