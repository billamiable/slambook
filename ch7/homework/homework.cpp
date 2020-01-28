// 以下代码参考
// 特征提取耗时：https://www.cnblogs.com/cc111/p/9457319.html


#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>//SIFT
#include <chrono>
using namespace std;
using namespace cv;

int main( int argc, char** argv )
{

    //-------------------------------------------------------------------------------------
    // 特征提取耗时
    Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
    vector<KeyPoint> keypoints_1; //关键点
    Mat outImg; // imshow
    
    // ORB feature extraction
    chrono::steady_clock::time_point ORB_t1=chrono::steady_clock::now();
    Ptr<FeatureDetector> orb_detector = ORB::create(1000);
    orb_detector->detect ( img_1,keypoints_1 );
    chrono::steady_clock::time_point ORB_t2 = chrono::steady_clock::now();
    chrono::duration<double> ORB_time_used = chrono::duration_cast<chrono::duration<double> >( ORB_t2-ORB_t1 );
    cout<<"extract keypoints of ORB costs: "<<ORB_time_used.count()<<" seconds."<<endl;
    cout<<"ORB keypoint number = "<<keypoints_1.size()<<endl;
    drawKeypoints(img_1,keypoints_1,outImg,Scalar::all(-1),DrawMatchesFlags::DEFAULT); 
    imshow("1.png的ORB特征点",outImg);
    waitKey(0);

    // SIFT feature extraction
    chrono::steady_clock::time_point SIFT_t1=chrono::steady_clock::now();
    Ptr<xfeatures2d::SiftFeatureDetector> sift_detector = xfeatures2d::SiftFeatureDetector::create(1000);
    sift_detector->detect ( img_1,keypoints_1 );
    chrono::steady_clock::time_point SIFT_t2 = chrono::steady_clock::now();
    chrono::duration<double> SIFT_time_used = chrono::duration_cast<chrono::duration<double> >( SIFT_t2-SIFT_t1 );
    cout<<"extract keypoints of SIFT costs: "<<SIFT_time_used.count()<<" seconds."<<endl;
    cout<<"SIFT keypoint number = "<<keypoints_1.size()<<endl;
    drawKeypoints(img_1,keypoints_1,outImg,Scalar::all(-1),DrawMatchesFlags::DEFAULT); 
    imshow("1.png的SIFT特征点",outImg);
    waitKey(0);

    // SURF feature extraction
    chrono::steady_clock::time_point SURF_t1=chrono::steady_clock::now();
    Ptr<xfeatures2d::SurfFeatureDetector> surf_detector = xfeatures2d::SurfFeatureDetector::create(400);
    surf_detector->detect ( img_1,keypoints_1 );
    chrono::steady_clock::time_point SURF_t2 = chrono::steady_clock::now();
    chrono::duration<double> SURF_time_used = chrono::duration_cast<chrono::duration<double> >( SURF_t2-SURF_t1 );
    cout<<"extract keypoints of SURF costs: "<<SURF_time_used.count()<<" seconds."<<endl;
    cout<<"SURF keypoint number = "<<keypoints_1.size()<<endl;
    drawKeypoints(img_1,keypoints_1,outImg,Scalar::all(-1),DrawMatchesFlags::DEFAULT); 
    imshow("1.png的SURF特征点",outImg);
    waitKey(0);

    //-------------------------------------------------------------------------------------
    // 
    
    
    return 0;
}
