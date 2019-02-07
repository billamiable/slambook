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
// 问题：这是啥意思？先在前面申明下，然后后面具体写实现？那为啥不直接写在这里？
// 回答：想法正确，也可以直接写在这里，在标准的C编译器中不合法，Xcode倒是可以
// 这里的keypoints_1用了两种方式，上面的是reference传参，下面是copy传参
// reference传参可以修改输入值影响函数外的结果，所以可以认为结果也是一种输出~
// copy传参则不能影响函数外，则认为是一种纯粹的输入~
// const reference传参呢，保留前两者的优点，既不用创建copy，效率低；同时不会修改输入变量的值！
void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

// 所以这里改成const reference是不是会变快？
// 几乎没差别~~都是17ms左右
void pose_estimation_2d2d (
    // const std::vector<KeyPoint>& keypoints_1,
    // const std::vector<KeyPoint>& keypoints_2,
    // const std::vector< DMatch >& matches,
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    std::vector< DMatch > matches,
    Mat& R, Mat& t );

// 像素坐标转相机归一化坐标
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

    // 这里和之前一样
    // 格式是cv::KeyPoint，里面包含了2D position, scale, orientation and other params
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    // 自己写的函数做到的
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
	  duration = double(End - Begin) / CLOCKS_PER_SEC*1000;//duration就是运行函数所打的
	  cout << "tick=" << double(End - Begin) << endl; //点数,CLK_TCK是每秒所打点数
	  cout << "duration=" << duration <<"ms"<< endl;

    //-- 验证E=t^R*scale
    // 这里用了Mat的at功能，给特定点赋值~ 
    // 总体这个赋值方式和下面的是一样的，只不过t不是定义为Mat_，所以就要用at
    // 这一步做的就是skew-symmetric matrix!
    Mat t_x = ( Mat_<double> ( 3,3 ) <<
                0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
                t.at<double> ( 2,0 ),      0,                      -t.at<double> ( 0,0 ),
                -t.at<double> ( 1,0 ),     t.at<double> ( 0,0 ),      0 );
    // 没理解，为啥不是essential?
    // 其实就是，但你忘了吗？尺度不确定性！！！
    // 最后就差了-sqrt(2)倍~~
    cout<<"t^R="<<endl<<t_x*R<<endl;

    //-- 验证对极约束
    // 所以这个验证是针对特征点的~
    // 对于每一对match的特征点，计算x2^T_hat*R*x1，理论上都应该接近于0
    // K就是intrinsic matrix
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    // 这个for循环有点神奇啊~
    // 我猜是和vector有关，的确！
    // C++11 provides a good facility to move through the containers.
    // for (Type x:X)
    for ( DMatch m: matches )
    {
        // 所以上面是算epipolar constraint里的每一项
        // 因此Point2d应该是图像坐标系的值
        // 之所以要归一化的原因：这里是essential matrix，所以输入是不带K的，要除去K的影响！
        Point2d pt1 = pixel2cam ( keypoints_1[ m.queryIdx ].pt, K );
        Mat y1 = ( Mat_<double> ( 3,1 ) << pt1.x, pt1.y, 1 );
        Point2d pt2 = pixel2cam ( keypoints_2[ m.trainIdx ].pt, K );
        Mat y2 = ( Mat_<double> ( 3,1 ) << pt2.x, pt2.y, 1 );
        // 这个就是epipolar constraint的形式，这里为0就是好~
        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }
    return 0;
}

// 在这里定义了函数的具体实现，本质上和之前的feature_extraction里差不多
// 这里的&是引用吧？为啥后面就没有呢？是说这里不能改吗？
// 知道了这里的keypoints,matches都要在里面修改，必然要用引用~
// 引用在内部存放的是一个对象的地址，它是该对象的别名
// const引用其实很好理解，就是你不想输入变，而同时保留引用的功能~
// 其实就是比直接
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

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

// 归一化图像坐标系的值
Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
                // (x-cx)/fx and (y-cy)/fy
                // 所以相当于是以f为1为基准，做了归一化~
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}


// 这个是此code的精华部分~
// 这里的&只作用于R,T上，因为只有这两个是output~
// 这里没有用const引用是因为名字就一样，没必要用引用~
// 这里用的都是没有归一化的图像坐标系值
void pose_estimation_2d2d ( 
                            // const std::vector<KeyPoint>& keypoints_1,
                            // const std::vector<KeyPoint>& keypoints_2,
                            // const std::vector< DMatch >& matches,
                            std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t )
{
    // 相机内参,TUM Freiburg2，所以图片不是乱找的，是从数据集采下来的
    // 所以说我当时用iphone拍的要想重建3D，还是需要知道相机的内参的~
    // 哇，这种输入数据的方式好高级
    // Mat_<type>本质和Mat一样，可以理解为长得更像matlab，就省去了.at的部分
    // 这种initialization的方法是OK的！叫comma-separated initializer
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    //-- 把匹配点转换为vector<Point2f>的形式，cv::Point的形式，这里只有x,y坐标值
    // 为甚要这么做捏？可能是findFundamentalMat的数据格式要求~
    // 对！只需要坐标值~
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        // 这里的取样方式很特别，可以研究下~
        // queryIdx是cv::DMatch中的内置变量，用于query descriptor index
        // pt是KeyPoint的内置变量，表示coordinates of the keypoints
        // 总结一下，就是选取match的点，并提取他们的图像坐标系的坐标，存入cv::Point中
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵
    // 直接调用opencv3的函数，输入只需要匹配的特征点！因为就是解线性方程组~
    // 这里不用内参，因为不是归一化的结果~
    Mat fundamental_matrix;
    // method变量！CV_FM_8POINT for an 8-point algorithm，可以的~
    // 8点法指的是大于等于8对点的输入；7点法只有7对点！哈哈！
    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    // 同样直接调用函数
    Point2d principal_point ( 325.1, 249.7 );	//相机光心, TUM dataset标定值
    double focal_length = 521;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    // 输入匹配的特征点，还需要内参的信息，也就是事先标定好的情况
    // 所以在这里面其实隐含了第一步要对图像坐标系的值归一化！
    // 这里倒是没有specify用的什么算法，也有method，默认是RANSAC
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    // 这里不需要内参，说明要求不高，其实就是图像坐标系的点之间的变换关系~
    // 所以这个在HW3才会大量用到，因为我们是将一张图转成投影变换后的另一张图~
    Mat homography_matrix;
    // 3是ransacReprojThreshold
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    // TO-DO: 原理理解了，但是这里为啥H_31,H_32≈0，这样不就变成affine了？？？
    // 测试了几个其他的，一样的结果，都为0，看来还没完全理解！
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    // 恩恩，果然这个也是直接调函数，输入是本质矩阵，匹配的特征点？，内参？（为啥要这些？），输出是R,T
    // 原因：E具有尺度不确定性，因此只能获得t的方向，无法得知大小
    // 要想获得具体的大小，必须输入真实世界的scale作为reference!
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;

    // to compare with outloop result
    // TO-DO: 相差一个sqrt(2)也是蛮有意思的哈哈，其实大概能猜到是怎么一个过程~
    cout<<"essential_matrix*-SQRT(2) is "<<endl<< essential_matrix*-sqrt(2.0)<<endl;
   
}
