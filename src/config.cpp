#include "myslam/config.h"

namespace myslam 
{
    
void Config::setParameterFile( const std::string& filename )
{
    if ( config_ == nullptr ) // 如果config指向了一个空指针，则创建一个新的config
        config_ = shared_ptr<Config>(new Config);
    config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
    if ( config_->file_.isOpened() == false )
    {
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        config_->file_.release();
        return;
    }
}

// 析构函数
Config::~Config()
{
    if ( file_.isOpened() )
        file_.release();
}

shared_ptr<Config> Config::config_ = nullptr;

}