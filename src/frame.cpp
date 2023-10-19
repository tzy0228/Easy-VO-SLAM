#include "MYSLAM/frame.h"

namespace MYSLAM{

//Frame构造函数
Frame::Frame( long id , double time_stamp ,const SE3 &pose, const Mat &left,const Mat &right ):id_(id),time_stamp_(time_stamp), pose_(pose),left_img_(left),right_img_(right) {};

// 设置keyframe的函数
void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;//关键帧id=0
    is_keyframe_ = true; //是否为关键帧置于true
    keyframe_id_ = keyframe_factory_id++; //id++
}

//这里注意下，由factory_id++一个数去构造Frame对象时，调用的是默认构造函数，由于默认构造函数全都有默认值，所以就是按坑填，先填第一个id_，
//所以也就是相当于创建了一个只有ID号的空白帧。
Frame::Ptr  Frame::CreateFrame(){
    static long factory_id =0;
    Frame::Ptr new_frame(new Frame);
    new_frame->id_=factory_id++;
    return new_frame;
}
}//namespace MYSLAM

