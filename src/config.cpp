#include"MYSLAM/config.h"

namespace MYSLAM{

// 打开文件
bool Config::SetParameterFile(std::string &filename){
    //没有config则构建
    if (config_ == nullptr ){
        config_=std::shared_ptr<Config>(new Config);
    }
    // config_->file_就定义为cv::FileStorage形式，变量是filename.c_str(),参数是cv::Filestorage::READ.是只读模式，不修改。
    config_->file_=cv::FileStorage(filename.c_str(),cv::FileStorage::READ);
    // 如果文档打不开，就用std::cerr输出错误信息
    if (config_->file_.isOpened()==false)
    {
        LOG(ERROR) << "parameter file " << filename << " does not exist.";
        config_->file_.release();//关闭文件并删除buff
        return false;
    }
    return true;//如果打开，返回ture
}

//析构函数：关闭文件，删除相关buff
Config:: ~Config(){
    if (file_.isOpened())
    {
       file_.release();
    }
}

std::shared_ptr<Config> Config::config_ = nullptr;//智能全局指针

}//namespace MYSLAM