#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// 问题：为什么要使用char**？
// The name of an array is a pointer to the first element of the array
// Technically, the char* is not an array, but a pointer to a char.
// Similarly, char** is a pointer to a char*. Making it a pointer to a pointer to a char.
// 总结来说，当输入参数多于1个时，只用char*无法快速选到后面的参数，因此要用char**
int main ( int argc, char** argv )
{
    // 读取argv[1]指定的图像
    cv::Mat image;
    image = cv::imread ( argv[1] ); // cv::imread函数读取指定路径下的图像
    // 判断图像文件是否正确读取
    if ( image.data == nullptr ) // 数据不存在,可能是文件不存在
    {
        // Generally you use std::cout for normal output, 
        // std::cerr for errors, and std::clog for "logging"
        // The major difference is that std::cerr is not buffered like the other two.
        cerr<<"文件"<<argv[1]<<"不存在."<<endl;
        return 0;
    }
    
    // 文件顺利读取, 首先输出一些基本信息
    cout<<"图像宽为"<<image.cols<<",高为"<<image.rows<<",通道数为"<<image.channels()<<endl;
    cv::imshow ( "image", image );      // 用cv::imshow显示图像
    cv::waitKey ( 0 );                  // 暂停程序,等待一个按键输入
    // 判断image的类型
    // 可以使用type()函数判定类型
    if ( image.type() != CV_8UC1 && image.type() != CV_8UC3 )
    {
        // 图像类型不符合要求
        cout<<"请输入一张彩色图或灰度图."<<endl;
        return 0;
    }

    // 遍历图像, 请注意以下遍历方式亦可使用于随机像素访问，这个本质上就是每个都访问一遍的时间。。
    // 使用 std::chrono 来给算法计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    // 本质就是写了一个三重循环~
    // size_t is an unsigned integer data type
    // A signed integer can represent negative numbers; unsigned cannot.
    // 这里存储的方式其实很简单，就是在地址区内连续按行、通道、列的优先级顺序存储，每个占一个内存空间
    // 一个内存空间就是8 bit，以字节形式存储，就是一个字节，1 byte，可以存储2^8=256
    // 因此每个字节正好存储RGB中一个channel的数据量，即0~255
    for ( size_t y=0; y<image.rows; y++ )
    {
        // 用cv::Mat::ptr获得图像的行指针
        // Mat::ptr returns a pointer to the specified matrix row.
        unsigned char* row_ptr = image.ptr<unsigned char> ( y );  // row_ptr是第y行的头指针
        // printf("指针的值row_ptr[3]所指向的地址为 %p , 该地址上所保存的值为%d\n", &row_ptr[3], row_ptr[3]);

        for ( size_t x=0; x<image.cols; x++ )
        {
            // 访问位于 x,y 处的像素
            // 重新定义一个指针的原因是一个负责行，一个负责行下的列
            // 存储结构为BGR,BGR,...，因此data_ptr的值为row_ptr + x*image.channels()
            // 其中，操作符优先级: [] > &
            // unsigned char* data_ptr = row_ptr + x*image.channels() ;
            unsigned char* data_ptr = &row_ptr[ x*image.channels() ]; // data_ptr 指向待访问的像素数据
            // printf("指向指针row_ptr+x*image.channels()的指针所指向的地址为 %p , 或为 %p\n", row_ptr+x*image.channels(), &row_ptr[ x*image.channels() ]);
            
            // 输出该像素的每个通道,如果是灰度图就只有一个通道
            // for循环是先c=0，然后判断条件，再执行，然后c加1，再判断
            for ( int c = 0; c != image.channels(); c++ )
            {
                unsigned char data = data_ptr[c]; // data为I(x,y)第c个通道的值
                printf("指针data所指向的地址为 %p , 该地址上所保存的值为%d\n", &data, data);
            }
        }
    }
    // 计时方法
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"遍历图像用时："<<time_used.count()<<" 秒。"<<endl;

    // 关于 cv::Mat 的拷贝
    // 直接赋值并不会拷贝数据
    cv::Mat image_another = image;
    // 修改 image_another 会导致 image 发生变化
    // cv::Rect(x,y,width,height)
    // setTo is a OpenCV function
    image_another ( cv::Rect ( 0,0,100,100 ) ).setTo ( 0 ); // 将左上角100*100的块置零
    cv::imshow ( "image", image );
    cv::waitKey ( 0 );
    
    // 使用clone函数来拷贝数据
    cv::Mat image_clone = image.clone();
    image_clone ( cv::Rect ( 0,0,100,100 ) ).setTo ( 255 );
    cv::imshow ( "image", image );
    cv::imshow ( "image_clone", image_clone );
    cv::waitKey ( 0 );

    // 对于图像还有很多基本的操作,如剪切,旋转,缩放等,限于篇幅就不一一介绍了,请参看OpenCV官方文档查询每个函数的调用方法.
    // Destroys all of the HighGUI windows.
    // You can call cv::destroyWindow or cv::destroyAllWindows to close the window and de-allocate any associated memory usage. 
    // For a simple program, you do not really have to call these functions 
    // because all the resources and windows of the application are closed automatically by the operating system upon exit.
    // 所以结论是可以不管，除非项目很大
    cv::destroyAllWindows();
    return 0;
}
