#include <gflags/gflags.h>
#include"MYSLAM/visual_odometry.h"

//DEFINE_string参数1是一个变量；参数2是用户指定，给到参数1的；参数3，类似一个说明为这个变量的提示信息
//在程序中使用DEFINE_XXX函数定义的变量时，需要在每个变量前加上FLAGS_前缀。

DEFINE_string (config_file, "./config/default.yaml" ,"config file path");

int main (int argc , char **argv){
    google::ParseCommandLineFlags(&argc, &argv, true);

    MYSLAM::VisualOdometry::Ptr vo (new MYSLAM::VisualOdometry(FLAGS_config_file));

    // 初始化
    vo->Init();
    LOG(INFO) << "Found file finshed";
    // 开始运行
    vo->Run();

    return 0;
}