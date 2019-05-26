#include <myslam/process_info.h>
#include <vector>
#include <fstream>
#include "myslam/config.h"
namespace myslam
{
ProcessInfo::ProcessInfo()
{
    
     info_["dist_of_match"]  = Config::get<int>( "process_info_DistOfMatch" );
     info_["num_of_inliers"] = Config::get<int>( "process_info_NumOfInliers" );
     info_["norm_pose6d"] = Config::get<int>( "process_info_NormofPose6d" );

}
ProcessInfo::~ProcessInfo()
{
}
void ProcessInfo::create_file()
{
    for( auto file = info_.begin(); file != info_.end(); ++file )
    {
        if( file->second == 1 )
        {
            ofstream fo(path_+file->first);
            if(!fo)
            {
                cout <<"failed to creat "<< file->first <<endl;
            }
            fo.setf(ios::fixed,ios::floatfield);
            fo.close();
            file->second = 2;//enable write
        }
    }
}

void ProcessInfo::write( string key,const double content1 ,const float content2 )
{
    if( info_.find(key)->second == 2 ){
        ofstream fo;
        fo.open((path_+key),ofstream::out | ofstream::app);
        fo.setf(ios::fixed,ios::floatfield);
        fo <<content1 <<" " <<content2 <<endl;
        fo.close();
    }
}
    
}
