#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>

int main( int argc, char** argv )
{
    // 学习下vector的应用
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    // Type of the allocator object used to define the storage allocation model.
    // 什么是allocator?
    // 其实本质就是定义了一种class type，默认情况下是系统template
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿
    
    // 读取txt文件，fin是自定义对象名
    ifstream fin("./pose.txt");
    // 为啥不写做if(!fin.is_open())
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        // 有error时候return 1或者-1
        return 1;
    }
    
    // 分别处理5张图片
    for ( int i=0; i<5; i++ )
    {
        // 这是定义了读图片的格式~
        boost::format fmt( "./%s/%d.%s" ); //图像文件格式
        // push_back是什么意思？对于头尾可以插的
        // emplace_back据说功能一样，
        // 这个读rgb,depth图的方法很独特啊~
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        // 这个-1的读法是什么呢？
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
        
        // 这里才是正式读取txt文件的内容，以数组的形式读取！
        double data[7] = {0};
        // 这其实就是常用简写方法，省去了三段式for(i=1;i<10;i++)
        for ( auto& d:data )
            fin>>d;
        // 定义了旋转矩阵的四元数
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d T(q);
        // 这个还挺神奇的，pretranslate函数，应该是把最初的Pose转换成了世界坐标系
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        // 看来push_back是vector的一种常用操作，有点像堆栈
        poses.push_back( T );
    }
    
    // 计算点云并拼接
    // 相机内参 
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    // 这个depthScale其实挺奇怪的？
    double depthScale = 1000.0;
    
    cout<<"正在将图像转换为点云..."<<endl;
    
    // 定义点云使用的格式：这里用的是XYZRGB，有意思
    typedef pcl::PointXYZRGB PointT; 
    typedef pcl::PointCloud<PointT> PointCloud;
    
    // 新建一个点云，用到了new~
    PointCloud::Ptr pointCloud( new PointCloud ); 
    // 处理每对RGBD数据，并添加至点云中
    for ( int i=0; i<5; i++ )
    {
        cout<<"转换图像中: "<<i+1<<endl; 
        // 获取原始数据
        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];

        // 按照行列的方式对于每个像素进行操作
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                // depth是int型??
                // 又定义了指针类型。。
                // 这里直接index到横纵坐标值，这里是make sense的！
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                // 以下是将图像坐标系转换成相机坐标系！
                Eigen::Vector3d point; 
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                // 乘以T从而将相机坐标系转换成世界坐标系XYZ！
                // 还需要理解一下T的构造。。
                Eigen::Vector3d pointWorld = T*point;
                
                // 在点云中定义每一个像素点对应的三维空间点
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                // step是结构体中的一个变量？
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                // 最后一样是vector形式，通过push_back存储起来
                // 这里又出现了一个很重要的概念->，好好理解下。。
                // ->和*ptr.的效用是一样的！
                pointCloud->points.push_back( p );
            }
    }
    
    // 其实就是定义is_dense变量的值
    pointCloud->is_dense = false;
    // 这是一种什么关系呢？
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    // 最后将建好的点云存到本地
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
    return 0;
}
