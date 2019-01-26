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
    // vector<type> variable_name;
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    // TO-DO: 这个应该是比较难的，可以先跳过，之后再研究。
    // Type of the allocator object used to define the storage allocation model.
    // 什么是allocator?
    // 其实本质就是定义了一种class type，默认情况下是系统template
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿
    
    // 读取txt文件，fin是自定义对象名
    ifstream fin("./pose.txt");
    // 为啥不写做if(!fin.is_open())？ it is essentially the same, at least here!
    // cout<<!fin<<endl;
    // cout<<!fin.is_open()<<endl;
    // exit(1);
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
        // it allows formatted i/o, similar to std::printf() and std::scanf()
        // 这个读rgb,depth图的方法很独特啊~哈哈~
        // 本质上就是和scanf一样样的，一行写成了两行。。还要转成str
        boost::format fmt( "./%s/%d.%s" ); //图像文件格式
        
        // push_back is the way to store variables
        // TO-DO: emplace_back据说功能一样，但是效率更高，有待考察。。
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        // VIP: imread分为不同的flags，其中-1表示原图输入，0表示灰度图，1表示RGB三通道
        // default is 1, which represents the color
        // -1 returns the loaded image as is (with alpha channel, otherwise it gets cropped)
        // ppm(Portable PixMap) can be one byte per pixel (up to 2 bytes), which stores RGB
        // pgm(Portable GreyMap) stores grayscale info, one value per pixel - up to 2 bytes
        // 这里可以用-1或者2，都是一个效果，2也可以把他们转换成2bytes
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
        // depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), 2 )); // 使用-1读取原始图像
        
        // 这里才是正式读取txt文件的内容，以数组的形式读取！
        // 静态分配--数组初始化方法，默认都是0
        double data[7] = {0};
        
        // it is new feature in c++11, auto: applys automatic type deduction
        // 简写方法，省去了三段式for (int i = 0; i < len; i++) + double d = data[i]
        // for(auto x:v) where, v is data, x is defined variable     
        // 在这里虽然data是首地址，但是实际运行时，还是令d=data[i]!
        // cout<<data<<endl;
        // exit(1);

        // VIP: c++ for loop needs {}!!!!!
        for ( auto& d:data ){
        // 这个是错的，因为这样就不能操作d后也对data操作了！
        // for ( auto d:data ){
            // 输入数据的类型即为double，按理说只要copy数据值即可
            fin>>d;
            // cout<<d<<endl;
            // exit(1);
        }
        // cout<<data[0]<<data[1]<<data[2]<<data[3]<<data[4]<<data[5]<<data[6]<<endl;
        // exit(1);

        // 定义了旋转矩阵的四元数
        // pose文件的构成：前三个是XYZ，后四个表征四元数(3个虚部+1个实部)
        // 可以有两种定义方式，一般选前者，因为简单
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        // Eigen::Quaterniond q = Eigen::Quaterniond ( data[6], data[3], data[4], data[5] );
        // cout<<"quaternion = \n"<<q.coeffs() <<endl;
        // exit(1);

        // 搞清楚了上面一行，这里就顺理成章了
        Eigen::Isometry3d T(q);

        // pretranslate函数用于inital position setting
        // Applies on the right the translation matrix represented by the vector
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        
        // 通过push_back保存各个图像的初始位姿！
        poses.push_back( T );
    }
    
    // 计算点云并拼接
    // 相机内参 
    double cx = 325.5; // 这个值也挺有意思，并不是640/2
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    // 说明一开始depth是以mm计的，后面要转换成meter
    double depthScale = 1000.0;
    
    cout<<"正在将图像转换为点云..."<<endl;
    
    // 定义点云使用的格式：这里用的是XYZRGB，有意思
    typedef pcl::PointXYZRGB PointT; 
    typedef pcl::PointCloud<PointT> PointCloud;
    
    // 新建一个点云，用到了new~
    // new表示对象所拥有的内存是动态分配的，直到你调用delete()方法对象才会被销毁，否则一直存在
    // new创建后应该是指针的形式
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
                // depth是以mm为单位，所以int型是合理的，而且都是正的，所以是unsigned
                // 这里直接index到横纵坐标值，这里是make sense的！
                // 这里可以理解成先用depth.ptr<>(v)取到指针，然后[u]取到值
                // 所以又是两步合成一步了！耶！现在能看懂了！
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                
                // 这个操作也是挺骚的哈哈！
                if ( d==0 ) continue; // 为0表示没有测量到
                // cout<<"d="<<d<<endl;
                // exit(1);

                // 以下是将图像坐标系转换成相机坐标系！
                Eigen::Vector3d point; 
                // 由mm转换成m
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 

                // 乘以T从而将相机坐标系转换成世界坐标系XYZ！
                // T本质是pose，是系统的输入，而且应该是预定义好了结构
                // 从而使得Eigen::Isometry3d可以直接与Eigen::Vector3d相乘
                // 得到的结果仍为Vector3d，对应世界坐标系下的坐标
                Eigen::Vector3d pointWorld = T*point;
                
                // 在点云中定义每一个像素点对应的三维空间点
                // typedef pcl::PointXYZRGB PointT; 
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];

                // data表征cv::Mat里Pointer to the user data
                // step表征cv::Mat里Number of bytes each matrix row occupies
                // 而由于RGB每个都是1byte，所以正好多少个byte就是隔着多少个地址！
                // 这里本质就是直接用color图对应像素点的值赋值罢了
                // cout<<"num is "<<v*color.step+u*color.channels()<<endl;
                // exit(1);
                // VIP:有意思的是，这里的顺序竟然是BGR！
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];

                // ->和*ptr.的效用是一样的！
                // points是PointCloud的parameter，是PCL最基本的数据类型
                // 最后一样是vector形式，通过push_back存储起来
                // 因此，之后也可以通过pointCloud->points[i]取到第i个数据
                pointCloud->points.push_back( p );
            }
    }
    
    // 定义另一个参数is_dense的值
    pointCloud->is_dense = false;
    // 参数size
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    // 最后将建好的点云存到本地，用的是值！
    // 后一个参数const pcl::PointCloud< PointT > &cloud
    // 这里&又不是取地址！本质就是输入的代名词~
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
    return 0;
}
