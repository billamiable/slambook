#include <iostream>
// 引入opencv的core,feature2d,highgui的模块
// 注释掉的是opencv2的写法
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
// 咦，这里怎么用了两个namespace? 可能是cv和std没啥重合的名字吧？
using namespace cv;

int main ( int argc, char** argv )
{   
    // 对输入的要求
    if ( argc != 3 )
    {
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }
    //-- 读取图像，所以在命令行要给予输入，这里的CV_LOAD_IMAGE_COLOR也可以改成1
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    //-- 初始化，以vector的形式存储keypoints，类型是cv::KeyPoint
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    // 分别创建FAST角点，ORB的BRIEF描述子，以及匹配的对象，都是以指针的形式
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // Ptr<FeatureDetector> detector = FeatureDetector::create(detector_name);
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(descriptor_name);
    // 由于用的特征是ORB，descriptor是二进制的，所以使用汉明距离，而且这里直接就是暴力穷举
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    // 这些都和matlab里的功能很像！
    // 用->表示*detector.，因为detector是指针哈哈~我学会啦~
    // detech和compute都是结构体里的函数
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    // 这里是opencv的自带visualization函数
    Mat outimg1;
    drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("ORB Feature Points",outimg1);
    waitKey(0);
    Mat outimg2;
    drawKeypoints( img_2, keypoints_2, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("ORB Feature Points",outimg2);
    waitKey(0);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    // DMatch也是cv里定义的class，是private的~
    vector<DMatch> matches;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );

    //-- 第四步:匹配点对筛选
    // 这一步据说很重要，相当于加一个threshold~
    double min_dist=10000, max_dist=0;

    // 找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    // 在doc里写了，descriptors是按行排列的~
    // row=500, col=32，也就是找到了500个匹配，每个描述子的维度是32维？
    cout<<"row is "<<descriptors_1.rows<<endl;
    // 下面这个不work，因为matches没有rows这个member
    // cout<<"row is "<<matches.rows<<endl;
    cout<<"col is "<<descriptors_1.cols<<endl;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        // 将descriptor之间的距离放到了matches中的distance值中
        double dist = matches[i].distance;
        // 寻找最小和最大值
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    // 仅供娱乐的写法，这个是上面写法的简略版~可能是c++11的特性，
    // TO-DO: []()这个好像很高级的样子
    // 对于vector求min_element，可以用下面的快捷方法，用begin和end，相当于matches[i]
    // 后面这坨是啥？给了compare的方法！！returns ​true if a is less than b
    // 这里要用const的原因是，在比较过程中，不允许修改内容
    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    // 这个还挺有意思的~
    // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.
    // 但有时候最小距离会非常小,设置一个经验值30作为下限.
    // 所以本质上还是找最小距离，以此为标准，最后threshold是max(2*min_dist, 30.0)
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        // 注意这里是30.0，如果是30就会报错，因为max要求输入double型
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            // 保存好的结果，且只保存距离信息，内含index信息
            good_matches.push_back ( matches[i] );
        }
    }

    //-- 第五步:绘制匹配结果
    Mat img_match;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    imshow ( "所有匹配点对", img_match );
    waitKey(0);
    // 这个改进还是很明显的~这里keypoints都画了！是的！
    Mat img_goodmatch;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    imshow ( "优化后匹配点对", img_goodmatch );
    waitKey(0);

    return 0;
}
