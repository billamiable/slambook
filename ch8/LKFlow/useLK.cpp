#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
using namespace std; 

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

int main( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: useLK path_to_dataset"<<endl;
        return 1;
    }
    string path_to_dataset = argv[1];
    // 给了一个rgb和depth的对应关系，不过后面实际没用到depth
    string associate_file = path_to_dataset + "/associate.txt";
    
    ifstream fin( associate_file );
    if ( !fin ) 
    {
        cerr<<"I cann't find associate.txt!"<<endl;
        return 1;
    }
    
    string rgb_file, depth_file, time_rgb, time_depth;
    list< cv::Point2f > keypoints; // 因为要删除跟踪失败的点，所以使用list
    cv::Mat color, depth, last_color;
    
    // 对于每一个时刻都做一次track
    for ( int index=0; index<100; index++ )
    {
        fin>>time_rgb>>rgb_file>>time_depth>>depth_file; // 第一项是时间，后面没用到
        // rgb的FLAG默认为1，不用写也可以
        color = cv::imread( path_to_dataset+"/"+rgb_file );
        // 读原图
        depth = cv::imread( path_to_dataset+"/"+depth_file, -1 );
        if (index ==0 )
        {
            // 对第一帧提取FAST特征点
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect( color, kps );
            for ( auto kp:kps )
                keypoints.push_back( kp.pt );
            last_color = color; // 这里的目的是对于第一步没有前帧做的特殊处理赋值
            continue;
        }

        // 判断是否数据为空
        if ( color.data==nullptr || depth.data==nullptr )
            continue;
        // 对其他帧用LK跟踪特征点
        vector<cv::Point2f> next_keypoints; 
        vector<cv::Point2f> prev_keypoints;
        for ( auto kp:keypoints )
            prev_keypoints.push_back(kp);
        vector<unsigned char> status;
        vector<float> error; 
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        // 用了Opencv自带的optical flow函数
        // status表示是否跟丢, each element of the vector is set to 1 if the flow 
        // for the corresponding features has been found, otherwise, it is set to 0.
        // 真正的track是从第二帧开始的
        // next_keypoints contains the calculated new positions of input features in the second image
        cv::calcOpticalFlowPyrLK( last_color, color, prev_keypoints, next_keypoints, status, error );
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
        cout<<"LK Flow use time："<<time_used.count()<<" seconds."<<endl;
        // 把跟丢的点删掉
        int i=0; 
        for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++) // 这里有i++
        {   
            // 如果跟丢了，对于库进行操作
            if ( status[i] == 0 )
            {
                // return value: An iterator pointing to the new location of the element 
                // that followed the last element erased by the function call.
                // 即iter指向被erase的后一个数据
                iter = keypoints.erase(iter); // keypoints代表总的track住的特征库
                // 如果有跟丢的，直接跳出loop，检查下一个keypoint
                continue;
            }
            // 没有跟丢的情况
            // 将keypoint跟踪住的下一帧位置保存下来，更新keypoint库的位置
            *iter = next_keypoints[i];
            iter++;
        }
        cout<<"tracked keypoints: "<<keypoints.size()<<endl;
        if (keypoints.size() == 0)
        {
            cout<<"all keypoints are lost."<<endl;
            break; 
        }
        // 画出 keypoints
        cv::Mat img_show = color.clone(); // 深拷贝
        for ( auto kp:keypoints )
            // Point2f格式的特征点，可以直接画，OpenCV做了重载
            // 这里的第二个输入是point的center，要求就是opencv的Point格式，毕竟是自家人！哈哈！
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
        cv::imshow("corners", img_show);
        cv::waitKey(0);
        last_color = color; // 更新需要track的图片
    }
    return 0;
}