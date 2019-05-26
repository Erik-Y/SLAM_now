#ifndef PROCESS_INFO_H
#define PROCESS_INFO_H

#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include "myslam/common_include.h"

using namespace std;
namespace myslam
{
    
class ProcessInfo
{
public:
    map<string,int> info_;
    string path_="./";
public:
    ProcessInfo();
    ~ProcessInfo();
    //create a file and return the index;
    void create_file();
  
    void write( string key,const double content1 ,const float content2 );
    
};
}
#endif
