#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// #include "extra.h" // use this if in OpenCV2，这里自己写了几个函数
using namespace std;
using namespace cv;
// 测试时间用的
# include<ctime>
clock_t  Begin, End;
double duration;

/****************************************************
 * 本程序演示了如何使用2D-2D的特征匹配估计相机运动
 * **************************************************/

// 特征匹配封装成函数
// 这里的keypoints_1用了两种传参方式，上面的是reference传参，下面是copy传参
// const reference传参，保留前两者的优点，既不用创建copy，效率低；同时不会修改输入变量的值！
void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

// 这里改成const reference是不是会变快？
// 简单测试几乎没差别，都是17ms左右
// 但是实际上要测试快慢，最好到秒级别，才能看出差距
// 实际使用推荐使用const reference
void pose_estimation_2d2d (
    // const std::vector<KeyPoint>& keypoints_1,
    // const std::vector<KeyPoint>& keypoints_2,
    // const std::vector< DMatch >& matches,
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    std::vector< DMatch > matches,
    Mat& R, Mat& t );

// 像素坐标转相机归一化坐标，本质上是少乘了depth
Point2d pixel2cam ( const Point2d& p, const Mat& K );

int main ( int argc, char** argv )
{
    if ( argc != 3 )
    {
        cout<<"usage: pose_estimation_2d2d img1 img2"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    // 格式是cv::KeyPoint，里面包含了2D position, scale, orientation and other params
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    //-- 估计两张图像间运动
    Mat R,t;
    // 最后得到的是两张图像之间的R,T
    Begin = clock();//开始计时
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );
    // 跑1000个iter --- const引用12.55s ; copy传参13.65s.
    // for ( int i = 0; i < 1000; i++ )
    // {
    //     pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );
    // }
    End = clock();//结束计时
	  duration = double(End - Begin) / CLOCKS_PER_SEC*1000;
	  cout << "tick=" << double(End - Begin) << endl; // 点数,CLK_TCK是每秒所打点数
	  cout << "duration=" << duration <<"ms"<< endl;

    //-- 验证E=t^R*scale
    // 使用cv::Mat的at功能用于赋值 
    // 构造skew-symmetric matrix
    Mat t_x = ( Mat_<double> ( 3,3 ) <<
                0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
                t.at<double> ( 2,0 ),      0,                      -t.at<double> ( 0,0 ),
                -t.at<double> ( 1,0 ),     t.at<double> ( 0,0 ),      0 );
    // 注意尺度不确定性，最后差了-sqrt(2)倍！
    cout<<"t^R="<<endl<<t_x*R<<endl;

    //-- 验证对极约束
    // 针对特征点
    // 对于每一对match的特征点，计算x2^T*t^R*x1，理论上都应该接近于0
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 ); // intrinsic matrix
    // C++11针对container的for循环新功能
    // 使用方式：for (Type x:X)
    for ( DMatch m: matches )
    {
        // 需要归一化的原因：
        // 对于x2^T*E*x1时，要求x1,x2为与K无关的归一化坐标，而Point2d是图像坐标系的值
        Point2d pt1 = pixel2cam ( keypoints_1[ m.queryIdx ].pt, K );
        Mat y1 = ( Mat_<double> ( 3,1 ) << pt1.x, pt1.y, 1 );
        Point2d pt2 = pixel2cam ( keypoints_2[ m.trainIdx ].pt, K );
        Mat y2 = ( Mat_<double> ( 3,1 ) << pt2.x, pt2.y, 1 );
        // c++是用.t()实现转置, 格式为MatExpr cv::Mat::t	(		)	const
        // 和cv::transpose(a,a)一样，参数是Inputmat, outputmat
        Mat d = y2.t() * t_x * R * y1; // epipolar constraint，趋于0
        cout << "epipolar constraint = " << d << endl;
    }
    return 0;
}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3 
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2 
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误
    // 但有时候最小距离会非常小，因此设置一个经验值30作为下限
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

// 像素坐标转相机归一化坐标
// 可以认为是去除了K的影响
Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
                // (x-cx)/fx and (y-cy)/fy
                // 相当于是以f为1为基准，做了归一化
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}


void pose_estimation_2d2d ( 
                            // const std::vector<KeyPoint>& keypoints_1,
                            // const std::vector<KeyPoint>& keypoints_2,
                            // const std::vector< DMatch >& matches,
                            std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t )
{
    // 相机内参来源于TUM Freiburg2数据集
    // 如果自己用iphone拍照想要做三维重建，需要知道相机的内参！
    // 创建Mat对象的初始化方法：comma-separated initializer
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    //-- 把匹配点转换为vector<Point2f>的形式
    // 在findFundamentalMat函数中作为输入
    vector<Point2f> points1;
    vector<Point2f> points2;
    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        // queryIdx是cv::DMatch中的内置变量，用于query descriptor index
        // pt是KeyPoint的内置变量，表示coordinates of the keypoints
        // 总结一下，就是选取match的点，并提取他们的图像坐标系的坐标，存入cv::Point中
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵
    // 直接调用opencv3 calib3d中的函数，输入只需要匹配的特征点，本质是解线性方程组
    Mat fundamental_matrix;
    // method变量可以选择CV_FM_8POINT，表示8点法
    // 8点法指的是大于等于8对点的输入，7点法只有7对点
    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    Point2d principal_point ( 325.1, 249.7 );	//相机光心, TUM dataset标定值
    double focal_length = 521;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    // 隐含了对图像坐标系的值归一化
    // method变量默认为RANSAC算法
    // 求解方法并没有用fundmental matrix来求essential
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    // 3是ransacReprojThreshold
    // 这里有一个有意思的地方：因为homography只对平面有效，因此在这里会先用RANSAC找到最大的面
    // 然后根据这个面里的Inliers来计算homography matrix!
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    // 问题：为啥H_31,H_32≈0，这样不就变成affine transform了？
    // 测试了几个其他的，一样的结果，都为0
    // 用自己写的code再来算下homography_matrix，也是一样的结果
    // H_31,H_32的数值都非常小！一般都是10^-4级别的，但是不为0！
    // 问题：为何这么小呢？
    // 因为H_31*u_x+H_32*u_y+1=V_z，而V_z一般大概为1,u在0-2000，因此得证！
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    // 本质是四个解，只有一个depth为正
    // 输入是本质矩阵，匹配的特征点？，内参？（为啥要这些？），输出是R,T
    // 原因：E具有尺度不确定性，因此只能获得t的方向，无法得知大小
    // 要想获得具体的大小，必须输入真实世界的scale作为reference!
    // TO-DO: 有机会以下函数自己去写下，implement一下底层的写法！
    // 已经写过matlab版本，可以再去写一个c++的版本
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;

    // 相差一个sqrt(2)也是蛮有意思的
    cout<<"essential_matrix*-SQRT(2) is "<<endl<< essential_matrix*-sqrt(2.0)<<endl;
   
}
