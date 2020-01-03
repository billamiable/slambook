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
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    // 问题：什么是aligned_allocator?
    // STL compatible allocator to use with types requiring a non standrad alignment.
    // TO-DO: 对齐方式可以动态选择，主要原因是防止大小不同导致的内存错误
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses; // 相机位姿
    
    // 读取txt文件
    ifstream fin("./pose.txt");
    // 也可以写成if(!fin.is_open())
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        // 有error时候return 1或者-1
        return 1;
    }
    
    // 分别处理5张图片
    for ( int i=0; i<5; i++ )
    {
        // 这是定义了读图片的格式，allows formatted i/o
        // 这个读rgb,depth图的方法很独特，本质上和scanf差不多，学习下
        boost::format fmt( "./%s/%d.%s" ); //图像文件格式
        
        // TO-DO: emplace_back据说功能一样，但是效率更高，有待考察。。
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        // VIP: imread分为不同的flags，其中-1表示原图输入，0表示灰度图，1表示RGB三通道
        // -1 returns the loaded image as is (with alpha channel, otherwise it gets cropped)
        // ppm(Portable PixMap) can be one byte per pixel (up to 2 bytes), which stores RGB
        // pgm(Portable GreyMap) stores grayscale info, one value per pixel - up to 2 bytes
        // 这里可以用-1或者2，都是一个效果，2也可以把他们转换成2 bytes
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
        
        // 正式读取txt文件的内容，以数组的形式读取
        // 静态分配--数组初始化方法，默认都是0
        double data[7] = {0};
        
        
        // for(auto x:v) where, v is data, x is defined variable     
        // auto+&表示使用引用，因为后续要修改data!
        for ( auto& d:data ){
            fin>>d;
        }

        // 定义了旋转矩阵的四元数
        // pose文件的构成：前三个是XYZ，后四个表征四元数(3个虚部+1个实部)
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        // cout<<"quaternion = \n"<<q.coeffs() <<endl;
        Eigen::Isometry3d T(q);

        // pretranslate函数用于inital position setting
        // Applies on the right the translation matrix represented by the vector
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        
        // 通过push_back保存各个图像的初始位姿！
        poses.push_back( T );
    }
    
    // 计算点云并拼接
    // 相机内参 
    double cx = 325.5; // 这个值挺有意思，并不是640/2
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    // 说明一开始depth是以mm计的，后面要转换成meter
    double depthScale = 1000.0;
    
    cout<<"正在将图像转换为点云..."<<endl;
    
    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT; 
    typedef pcl::PointCloud<PointT> PointCloud;
    
    // 新建一个点云，用到了new
    // 这个构造函数的输入是一个指针，最后返回一个指针
    // PointCloud (PointCloud< PointT > &pc)
    PointCloud::Ptr pointCloud( new PointCloud ); 
    // 处理每对RGBD数据，并添加至点云中
    for ( int i=0; i<5; i++ )
    {
        cout<<"转换图像中: "<<i+1<<endl; 
        // 获取原始数据
        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        // 获取pose数据
        Eigen::Isometry3d T = poses[i];

        // 按照行列的方式对于每个像素进行操作
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                // depth是以mm为单位，unsigned int范围是0~2^32-1
                // 先用depth.ptr<>(v)取到指针，然后[u]取到值
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值

                if ( d==0 ) continue; // 为0表示没有测量到

                // 以下是将图像坐标系转换成相机坐标系
                Eigen::Vector3d point; 
                point[2] = double(d)/depthScale; // 由mm转换成m
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 

                // 将相机坐标系转换成世界坐标系
                Eigen::Vector3d pointWorld = T*point;
                
                // 在点云中定义每一个像素点对应的三维空间点
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];

                // data表征cv::Mat里Pointer to the user data
                // step表征cv::Mat里Number of bytes each matrix row occupies
                // RGB每个都是1byte，用color图对应像素点的值赋值罢了
                // VIP: opencv顺序是BGR！
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];

                // ->和*ptr.的效用是一样的
                // points是PointCloud的parameter，是PCL最基本的数据类型
                // 之后也可以通过pointCloud->points[i]取到第i个数据
                pointCloud->points.push_back( p );
            }
    }
    
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    // 最后调用save函数将建好的点云存到本地
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
    return 0;
}
