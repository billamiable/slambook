#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

using namespace std;
using namespace cv;

void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

// 像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Mat& K );

Point2d cam2pixel ( const Mat& p, const Mat& K );

void bundleAdjustment (
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat& K,
    Mat& R, Mat& t
);

int main ( int argc, char** argv )
{
    if ( argc != 5 )
    {
        cout<<"usage: pose_estimation_3d2d img1 img2 depth1 depth2"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    // 所以feat只是从color map里找
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    // 建立3D点，这是世界坐标系的坐标
    // 有意思，不需要用到argv[4]？？
    // 理解了，因为一般都assume first image是世界坐标系，因此对于3d2d只需要之后的2d坐标即可！
    Mat d1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED );       // 深度图为16位无符号数，单通道图像
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    // 用于存储世界坐标系的3D点
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    // 这里本质是做好solve_pnp的输入
    for ( DMatch m:matches )
    {
        // m的定义在for里，关键是为啥前面是一个值，后面是list，这个就是vector的用法
        // 这里d1是深度图，Mat属性，这就很有意思了
        // 其实这个for loop取值也是挺好玩的~
        // 这里也不难，就是前面的东西，先找到keypoints是哪个match的，然后用了C++11的特性
        // ()[]--Lambda expressions，还不是这个，[]()反了
        // 理解了，其实就是基本的mat操作！
        // 首先定义了一个指针d1.ptr<..> (int i)，表示第i行
        // 然后根据首地址，取其中的元素，直接调取第j个元素即可！最终获得了深度值~
        ushort d = d1.ptr<unsigned short> (int ( keypoints_1[m.queryIdx].pt.y )) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        // 这个其实挺重要的，在之后的dirty训练里也要注意！
        if ( d == 0 )   // bad depth
            continue;
        // 这里为啥又是5000，不是之前的1000？
        // TUM定义：The depth images are scaled by a factor of 5000
        // 哈哈，其实就是不同的dataset的设定不同，所以之后自己做也要当心
        // 我记得最早也因为这件事困惑过~
        float dd = d/5000.0;
        // 转化成相机坐标系的归一化坐标
        Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
        // 这样就变成了真正的相机坐标系坐标！
        pts_3d.push_back ( Point3f ( p1.x*dd, p1.y*dd, dd ) );
        // trainIdx和queryIdx的区别？
        // queryIdx refers to keypoints1 and trainIdx refers to keypoints2
        // 搜嘎！
        pts_2d.push_back ( keypoints_2[m.trainIdx].pt );
    }
    // 输出一下匹配的参数
    cout<<"3d-2d pairs: "<<pts_3d.size() <<endl;

    Mat r, t;
    // 居然直接用函数！这个可以好好了解下~
    // 回头自己也重新写下底层的
    // 输入需要intrinsic，用于投影变换
    // 这个好奇怪，r,t不是以reference传参的形式，那凭啥修改啊？？
    // 注意！reference传参不是根据指针或值确定的，而是之前有没有引用
    // 因此，对于引用来说，指针或值都是可以引用的！！！
    // 这里的输出是空！
    // cout<<"R: "<<r<<" T: "<<t<<endl;
    // 只有在申明的地方可以看到这个，但是我找不到啊，是OPENCV不会告诉我们吗？
    // 找到了，OutputArray在opencv里是一个template，_OutputArray (Mat &m)
    // 而对应的InputArray则是_InputArray (const Mat &m)，加了const保证不动
    // 当然，这两个都是最基本的定义方式，具体的定义还是看各个应用场景！
    // Mat()表达的是distortion matrix
    // 默认方法：SOLVEPNP_EPNP，这个的确不好解，结果不稳定！
    solvePnP ( pts_3d, pts_2d, K, Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    // 还真的变了，这就奇怪了！
    // 输出的r是一个向量
    // cout<<"R: "<<r<<" T: "<<t<<endl;
    Mat R;
    // 还有这种公式！哈哈哈！
    // 罗德里格斯公式本身是计算旋转后的向量
    // 但这里只是纯粹地将旋转向量转成旋转矩阵
    cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    // R=
    // [0.9979193252225623, -0.05138618904650231, 0.03894200717249974;
    //  0.05033852907726567, 0.9983556574294724, 0.02742286945057129;
    //  -0.04028712992610883, -0.02540552801740052, 0.998865109165634]
    // t=
    // [-0.1255867099726712;
    //  -0.007363525262960201;
    //  0.06099926588593754]
    cout<<"R="<<endl<<R<<endl;
    cout<<"t="<<endl<<t<<endl;

    cout<<"calling bundle adjustment"<<endl;
    // 下面定义了bundle adjustment函数
    // 问题: PNP问题不就是用ba解的吗？为何重复两次操作？
    // 这里是进一步优化吗？应该是的
    // 我猜是两种方法！不对！需要solvepnp的结果作为下面的输入！但也只是程序的设定罢了~
    // 的确是可以作为两种方法存在的！
    // 问题是如果我comma掉上面的，直接用BA会报segment fault:11，是内存用尽
    // Mat R, t;
    bundleAdjustment ( pts_3d, pts_2d, K, R, t );
    // 最后结果差得不多，solvepnp的结果已经不错了！这是给了Init的情况！
    // 而且有意思的是，每次优化的结果是一样的，这个就厉害了！
    // T=
    //    0.997841  -0.0518393   0.0403291   -0.127516
    //   0.0507013      0.9983    0.028745 -0.00947167
    //  -0.0417507  -0.0266382    0.998773   0.0595037
    //           0           0           0           1
    
    // 如果不给initialization，可以得到下面的结果！
    // T=
    //    0.998278  -0.0553435   0.0194709  -0.0894187
    //   0.0548131    0.998137   0.0267963 -0.00589702
    //  -0.0209177  -0.0256829    0.999451   0.0471714
    //           0           0           0           1

    // 这里其实最好加一个correction_correct环节
    // 衡量下reprojection error的量级，最后就是Pixel级
    float avg_err;
    float error = 0;
    for ( int i=0; i<pts_3d.size(); i++ )
    {
        // cout<<"p2 = "<<pts_2d[i]<<endl;
        // cout<<"p2 = "<<pts_2d[i]<<endl;
        // cout << "type 1"<<typeid(pts1[i]).name() << endl;
        // 这个应该和p1尽量接近，结果的确是！
        Mat_<double> p1_2_3d = R * (Mat_<double>(3,1)<<pts_3d[i].x, pts_3d[i].y, pts_3d[i].z) + t;
        Point2d p1_2_2d = cam2pixel(p1_2_3d, K);
        // cout<<"transformed p2 = "<<
        //     p1_2_2d
        //     <<endl;
        // cout<<endl;
        // cout << "type 2"<<typeid(p2_1).name() << endl;
        // calculate error
        float e = abs(p1_2_2d.x - pts_2d[i].x) + 
                  abs(p1_2_2d.y - pts_2d[i].y);
        // cout<<"error is "<<e/3.0<<endl;
        // error.push_back(float (e/3.0));
        error += e;
    }
    // 量级大概是横竖各差1.5个pixel
    avg_err = error / pts_3d.size();
    cout<<"average error is "<<avg_err<<endl;
}

// 这部分应该和之前都一样
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
    // BFMatcher matcher ( NORM_HAMMING );
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

// 这个也一样，像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

// change to uv coordinate
Point2d cam2pixel ( const Mat& p, const Mat& K )
{
    return Point2d
           (
               p.at<double> ( 0,0 ) * K.at<double> ( 0,0 ) / p.at<double> ( 2,0 ) + K.at<double> ( 0,2 ),
               p.at<double> ( 1,0 ) * K.at<double> ( 0,0 ) / p.at<double> ( 2,0 ) + K.at<double> ( 1,2 )
           );
}

// 这个是本份code的精华所在
// 来了！
// 用到了g2o求解器，感觉还是得回过头去再理解下vertex和edge如何生成
// 问题：为何不直接用优化方法解？
// 因为直接解误差大，两步优化更好
void bundleAdjustment (
    const vector< Point3f > points_3d,
    const vector< Point2f > points_2d,
    const Mat& K,
    Mat& R, Mat& t )
{
    // 初始化g2o
    // pose:XYZ,3个角度;landmark:XYZ
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
    // 用的LM算法哈哈，可以试一下GN~~
    // 这里的GN也是一样的，那就真的奇怪了，凭啥3D2D需要Init，3D3D就不需要？
    // TO-DO: 从结果上来看，3D2D这个问题比3D3D要难，这也挺奇怪的？？
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex，首先设置顶点
    // 用李代数表示的相机位姿
    // g2o自带的，相机位姿顶点类VertexSE3Expmap，因为是位姿，所以是SE3
    // SE3 Vertex parameterized internally with a transformation matrix
    // and externally with its exponential map
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    // 也就是说需要R才可以这么做！所以不是两种方法！一会儿再研究下！
    // 这个就是拷贝一下rotation matrix，为啥不直接用呢？可能是怕被修改了。
    R_mat <<
               R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    // 首先设置Id
    pose->setId ( 0 );
    // 这个不理解
    // 其实就是位姿的一种表示，包含了四元数和translation matrix
    // Quat就是quaternion
    // sets the initial estimate from an array of double
    // 所以真的是需要外界的初始化
    // 在这里，有意思的是初始化是有区别的！
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            // translation vector
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                            // Eigen::Matrix3d::Identity(),
                            // Eigen::Vector3d( 0,0,0 )
                        ) );
    optimizer.addVertex ( pose );

    // 加完了pose，接下来加3D坐标点
    int index = 1;
    for ( const Point3f p:points_3d )   // landmarks
    {
        // 3D路标点类VertexSBAPointXYZ
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        // 一样先设置Id
        point->setId ( index++ );
        // 设置初始值
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        // TO-DO: 这个可以等之后看
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        // 最后加到optimizer里去
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        // f + cx,cy 默认fx=fy
        K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges，然后设置边
    index = 1;
    for ( const Point2f p:points_2d )
    {
        // 重投影误差边类EdgeProjectXYZ2UV
        // 把XYZ投影到UV平面上，做成了一个误差
        // 在这里有一个computeError函数，可以看到reprojection error的定义！最后xy如何融合没说。。
        // TO-DO: 还可以找到雅克比矩阵的计算，再看一遍原理后，再来理解下！
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        // 设置Id
        edge->setId ( index );
        // 每个边的两侧分别是pose和Landmark
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        // TO-DO:这里的具体还是要再看一下，总体的意思已经清楚了
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    // 最后求解即可~
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    // 这个不理解
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;
    cout<<endl<<"after optimization:"<<endl;
    // 总体的结果还是很准的~
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
    // assign value to R,T
    Eigen::Matrix4d output = Eigen::Isometry3d( pose->estimate() ).matrix();
    Eigen::Matrix3d R_out = output.block(0,0,3,3);
    Eigen::Vector3d T_out = output.block(0,3,3,4);
    cout<<"R_out="<<endl<<R_out<<endl;
    cout<<"T_out="<<endl<<T_out<<endl;
    eigen2cv(R_out, R);
    eigen2cv(T_out, t);
    cout<<"R="<<endl<<R<<endl;
    cout<<"T="<<endl<<t<<endl;
}
