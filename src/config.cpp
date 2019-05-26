/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "myslam/config.h"

namespace myslam 
{
    
void Config::setParameterFile( const std::string& filename )
{
    //如果指向Config类的指针config_还没有被创建，则在此处创建，config_将指向一个新的且唯一的Config类的对象
    if( config_ == nullptr )
        config_ = make_shared<Config>(Config());
    
    //将file_关联到default.yaml文件，读取操作将在main函数进行
    config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
    if( config_->file_.isOpened() == false )
    {
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        config_->file_.release();
        return;
    }
}

Config::~Config()
{
    if( file_.isOpened() )
        file_.release();
}
//静态数据成员的类外定义+初始化
std::shared_ptr<Config> Config::config_ = nullptr;

}
