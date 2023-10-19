#include"MYSLAM/dataset.h"
#include "MYSLAM/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;
namespace MYSLAM{

//构造函数
Dataset::Dataset (const std::string &dataset_path) : dataset_path_(dataset_path){}

// 初始化，返回是否成功
bool Dataset::Init(){
    // 从calib.txt中获取相机内外参
    // 00序列calib参数文件中一共包含了4个相机矩阵：其中P0代表0号左边灰度相机、P1代表1号右边灰度相机、P2代表2号左边彩色相机、P3代表3号右边彩色相机.每个相机12个参数组成的内参矩阵
    std::ifstream fin(dataset_path_ + "/calib.txt");//读取calib.txt
    if (!fin)
    {
        LOG(ERROR)<<"cannot find " << dataset_path_ << "/calib.txt!";
        return false;
    }else{
        LOG(INFO) << "1.1";
    }
    // 遍历文件中的四个相机(P0,P1,P2,P3)
    for (int i = 0; i < 4; ++i)
    {
        //前三个字符是P0：所以这里定义了一个长度为3的字符数组，读完这三个字符后就遇到了第一个空格，fin将会跳过这个空格，读取参数
        char camera_name[3];
        for (int k = 0; k < 3; ++k)
        {
            fin >> camera_name[k];
        }

        double projection_data[12];
        for (int k = 0; k < 12; ++k)
        {
            fin >> projection_data[k];
        }
        //将相机后面对应的12个参数读入到projection_data[12]中
        Mat33  K;//内参矩阵,从相机坐标到像素坐标
        K << projection_data[0],projection_data[1],projection_data[2],
                  projection_data[4],projection_data[5],projection_data[6],
                  projection_data[8],projection_data[9],projection_data[10];
        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];

        t=K.inverse()*t;
        K=K*0.5;
        
        Camera::Ptr new_camera (new Camera (K(0,0),K(1,1),K(0,2),K(1,2), t.norm(),  SE3(SO3() , t)));
        cameras_.push_back(new_camera);
         LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
    }
    fin.close();
    current_image_index_=0;
    return true;
}

//创建并返回下一帧
Frame::Ptr Dataset::NextFrame(){
    boost::format fmt("%s/image_%d/%06d.png");//数据格式化
    // 1.读图
    cv::Mat image_left , image_right;
    image_left = cv::imread((fmt  % dataset_path_  % 0  % current_image_index_).str(), cv::IMREAD_GRAYSCALE);
    image_right = cv::imread((fmt  % dataset_path_  % 1  % current_image_index_).str(), cv::IMREAD_GRAYSCALE);

    // 如果左图或右图为空，则返回空指针
    if (image_left.data == nullptr || image_right.data == nullptr)
    {
        LOG(WARNING) << "cannot find images at index " << current_image_index_;       
        return nullptr;
    }
        
    // 2.改图大小
    cv::Mat image_left_resized , image_right_resized;
    cv::resize(image_left,image_left_resized, cv::Size(),0.5,0.5 ,cv::INTER_NEAREST);
    cv::resize(image_right,image_right_resized, cv::Size(),0.5,0.5 ,cv::INTER_NEAREST);

    // 工厂模式建立新帧
    auto new_frame =  Frame::CreateFrame();
    // 将改过尺寸的左右目图片赋给新帧
    new_frame ->left_img_ =image_left_resized;
    new_frame ->right_img_=image_right_resized;
    current_image_index_++;
    return new_frame;
}

}//namespace MYSLAM