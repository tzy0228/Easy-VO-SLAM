//  * 配置类，使用SetParameterFile确定配置文件
//  * 然后用Get得到对应值
//  * 单例模式
// Config 类负责参数文件的读取,并在程序任意地方都可随时提供参数的值。所以我们把 Config 写成单例模式(Singleton)。
// 它只有一个全局对象,当我们设置参数文件时,创建该对象并读取参数文件,随后就可以在任意地方访问参数值,最后在程序结束时自动销毁

#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include"MYSLAM/common_include.h"

namespace MYSLAM{

//配置文件类
class Config{

private:
static std::shared_ptr<Config>config_;//智能指针
cv::FileStorage file_;//文件
Config(){};//私有构造函数生成单例模式

public:
~Config();//析构函数：关闭文件，删除相关buff
//打开一个新的文件
static bool SetParameterFile(std::string &filename);

//函数模板， 模板是实现代码重用机制的一种工具，它可以实现类型参数化，即把类型定义为参数功能要求： 我们需要对int、char、string、double等类型的数据做交换操作，
//（ 有了模板功能，则只需要编写一个函数即可，编译器可以通过输入参数的类型，推断出形参的类型。）
template <typename T>

// 根据键值获取参数值
static T Get(const std::string &key) {
        return T(Config::config_->file_[key]);
    }
};

}//namespace MYSLAM

#endif  // MYSLAM_CONFIG_H