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
    // 就是给了一个rgb和depth的对应关系
    // 有意思，optical flow居然还用到了depth，后面没用
    string associate_file = path_to_dataset + "/associate.txt";
    
    ifstream fin( associate_file );
    if ( !fin ) 
    {
        cerr<<"I cann't find associate.txt!"<<endl;
        return 1;
    }
    
    string rgb_file, depth_file, time_rgb, time_depth;
    // 居然用了list
    list< cv::Point2f > keypoints;      // 因为要删除跟踪失败的点，使用list
    cv::Mat color, depth, last_color;
    
    for ( int index=0; index<100; index++ )
    {
        // 原来第一项是时间，后面都没用
        fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
        // rgb的FLAG默认为1，不用写也可以
        color = cv::imread( path_to_dataset+"/"+rgb_file );
        // 读原图
        depth = cv::imread( path_to_dataset+"/"+depth_file, -1 );
        if (index ==0 )
        {
            // 对第一帧提取FAST特征点，哈哈理解！
            vector<cv::KeyPoint> kps;
            // 提的FAST特征
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            // 基本的func
            detector->detect( color, kps );
            // 得到特征点的坐标值
            for ( auto kp:kps )
                keypoints.push_back( kp.pt );
            // 这一步是很重要的，这和下面的逻辑关系是什么呢？
            // 这里的目的只是对于第一步没有前帧做的特殊处理赋值罢了
            last_color = color;
            continue;
        }
        // 所以本质上depth是没有用到的！
        // 这是在干嘛？其实就是判断是否为空
        if ( color.data==nullptr || depth.data==nullptr )
            continue;
        // 对其他帧用LK跟踪特征点
        vector<cv::Point2f> next_keypoints; 
        vector<cv::Point2f> prev_keypoints;
        for ( auto kp:keypoints )
            // 不断地将keypoints保存到vector
            prev_keypoints.push_back(kp);
        // 这是干嘛的呢？噢噢，就是opencv函数参数的output
        vector<unsigned char> status;
        vector<float> error; 
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        // 用了Opencv自带的optical flow函数
        // status是否跟丢, each element of the vector is set to 1 if the flow 
        // for the corresponding features has been found, otherwise, it is set to 0.
        // 因此，我猜第一帧的时候输入的last_color和color是一样的
        // 真正的track是从第二帧开始的！对，就是这样！
        // TO-DO: 可是这样有点蠢啊。。能不能优化下
        // next_keypoints contains the calculated new positions of input features in the second image
        cv::calcOpticalFlowPyrLK( last_color, color, prev_keypoints, next_keypoints, status, error );
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
        cout<<"LK Flow use time："<<time_used.count()<<" seconds."<<endl;
        // 把跟丢的点删掉
        int i=0; 
        // vector还有begin和end函数，不是vector，是list!
        // An iterator to the beginning of the sequence container.
        for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
        {   
            // 如果跟丢了，对于库进行操作
            if ( status[i] == 0 )
            {
                // return value: An iterator pointing to the new location of the element 
                // that followed the last element erased by the function call.
                // 也就是被erase的后一个
                // 那为啥不在这里把nextpoint的对应的也删了？
                // 因为不需要处理next_keypoints，只要知道哪些track住了就好了，之后会传给库！
                // 注意：这里是总的keypoints库，也就是最开始的储存库，后面的keypoint是分别存在prev和next里的
                // 可以认为prev本质就是库的一个copy，所以库也是要不断删减的
                iter = keypoints.erase(iter);
                // 这个表示如果有跟丢的，直接跳出loop，其实这里让Iterator自动指向下一个了
                // 所以这里不用Iter++
                continue;
            }
            // 这是在干嘛？
            // 这是没有跟丢的情况下！让Iter指向next_keypoints[i]，好奇怪啊！
            // 不对，是把keypoints里的值改成next_point里的，其实就是赋值拷贝的过程
            // 相当于写了一个if循环，如果track，就拷贝，否则就删掉
            *iter = next_keypoints[i];
            // 这是正常情况下让iterator指向下一个
            iter++;
        }
        cout<<"tracked keypoints: "<<keypoints.size()<<endl;
        // 如果没有keypoint是跟住的
        if (keypoints.size() == 0)
        {
            cout<<"all keypoints are lost."<<endl;
            break; 
        }
        // 画出 keypoints
        // 这个应该是传说中的浅拷贝，我猜原因是涉及到时间序列
        // 其实这个是深拷贝，这个就会创建一个新的，原来的变化不会影响它
        // 直接赋值是浅拷贝，浅拷贝改原来的，拷贝的也会变
        cv::Mat img_show = color.clone();
        for ( auto kp:keypoints )
            // Point2f格式的特征点，可以直接画哎，厉害
            // 这里的第二个输入是point的center，要求就是opencv的Point格式，毕竟是自家人！哈哈！
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
        cv::imshow("corners", img_show);
        cv::waitKey(0);
        // 哈哈，又出现了，果然是有用的
        // 因为下一时刻又要读取一张图，所以在loop的最后要修改时间序列上的图
        last_color = color;
    }
    return 0;
}