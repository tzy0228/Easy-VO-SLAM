# Easy-VO-SLAM
《视觉SLAM十四讲第二版》-CH13工程代码注释版本

Ubuntu18.04 + Ros Melodic

一、安装Gteset 库

sudo apt-get install libgtest-dev
cd /usr/src/gtest
sudo mkdir build 
cd build
sudo cmake ..    //一定要以sudo的方式运行，否则没有写入权限
sudo make        //这个也一样要以sudo的方式
sudo cp libgtest*.a /usr/local/lib   //将生成的libgtest.a 和 libgtest_main.a 拷贝到系统的lib路径下

二、下载数据集
给大家分享一下百度网盘：[kitti](https://pan.baidu.com/share/init?surl=HjZKMyepnP7V7jFuJre5BA&pwd=2p1k)
提取码：2p1k
(整体文件很大，约22G，可以先下载一个00序列)

三、修改/config下的default.yaml中的数据集的路径
//下面是需要你根据自己的路径修改：
dataset_dir: /media/tzy/TZY-YP/VSLAM/Dataset/KITTI/data_odometry_gray/dataset/sequences/00

四、编译运行
cd ch13
mkdir build
cd build
cmake ..
make

测试:
cd bin
./test_triangulation


 运行：
注意不要在/bin 路径下执行，需要在/Easy-VO-SLAM下运行：
cd ..
./bin/run_kitti_stereo
