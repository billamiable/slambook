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
    // 匹配对只是从RGB图里找，深度图作为3D坐标值来源
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    // 建立3D点，这是世界坐标系的坐标
    // 问题：不需要第2个view的depth图？
    // 原因：3D2D求解，一般assume first image是世界坐标系，因此只需要second image的2d坐标即可！
    Mat d1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED ); // 深度图为16位无符号数，单通道图像
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    // 用于存储世界坐标系的3D点
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    // 准备solve_pnp的输入，分别是2d和3d的匹配对，其中3d即为第1个view的世界坐标系下的坐标值
    for ( DMatch m:matches )
    {
        // ()[]是基本的mat操作，取数据值的方法
        // []()是Lambda expression
        // 首先定义了一个指针d1.ptr<..> (int i)，表示第i行
        // 然后根据首地址，取其中的元素，直接调取第j个元素，从而获得了深度值
        ushort d = d1.ptr<unsigned short> (int ( keypoints_1[m.queryIdx].pt.y )) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        // VIP：depth异常值处理，直接跳过
        if ( d == 0 )   // bad depth
            continue;
        // TUM定义：The depth images are scaled by a factor of 5000
        // 不同的dataset的设定不同，之后自己实现要注意细节
        float dd = d/5000.0;
        // 图像坐标系转化成相机坐标系的归一化坐标
        Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
        // 再转成相机坐标系的坐标
        pts_3d.push_back ( Point3f ( p1.x*dd, p1.y*dd, dd ) );
        // 问题：trainIdx和queryIdx的区别？
        // queryIdx refers to keypoints1 and trainIdx refers to keypoints2
        pts_2d.push_back ( keypoints_2[m.trainIdx].pt );
    }
    cout<<"3d-2d pairs: "<<pts_3d.size() <<endl;
    
    
    Mat r, t;
    // OutputArray在opencv里是一个template，_OutputArray (Mat &m)
    // InputArray是_InputArray (const Mat &m)，加了const保证不能修改输入
    // 输入需要intrinsic matrix，用于投影变换
    // 其中，Mat()对应distortion matrix
    // 默认方法：SOLVEPNP_EPNP，这个不好解，结果不稳定！
    // TO-DO: 回头自己也重新写下底层的实现
    solvePnP ( pts_3d, pts_2d, K, Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    // 输出的r是一个向量
    cout<<"r: "<<r<<" t: "<<t<<endl;
    Mat R;
    // opencv实现了罗德里格斯公式，将旋转向量转换为旋转矩阵
    cv::Rodrigues ( r, R ); // r为旋转向量形式
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
    // 问题: PNP问题不就是用ba解的吗？为何重复两次操作？
    // 一般是拿普通解法（如P3P或EPNP）得到一个好的初值，用BA做进一步优化
    // 如果直接用BA会报segment fault:11，是内存用尽
    // 说明初值给的不好，无法收敛
    // 有意思的是，每次优化的结果是一样的，
    // 而且BA后结果差得不多，说明solvepnp的结果已经不错了！
    bundleAdjustment ( pts_3d, pts_2d, K, R, t );
    
    // reprojection error的单位为Pixel
    float avg_err;
    float error = 0;
    for ( int i=0; i<pts_3d.size(); i++ )
    {
        // 将p1_3d->第2个view，得到p1_2_3d
        Mat_<double> p1_2_3d = R * (Mat_<double>(3,1)<<pts_3d[i].x, pts_3d[i].y, pts_3d[i].z) + t;
        // 将p1_2_3d转变成2d图像坐标系，应该与p2接近
        Point2d p1_2_2d = cam2pixel(p1_2_3d, K);
        // calculate error
        float e = abs(p1_2_2d.x - pts_2d[i].x) + 
                  abs(p1_2_2d.y - pts_2d[i].y);
        error += e;
    }
    // 误差量级大概是横竖各差1.5个pixel
    // 那比2D2D要多啊，之前是0.5个pixel
    avg_err = error / pts_3d.size();
    cout<<"average error is "<<avg_err<<endl;
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

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

// 3d相机坐标系转换成2d图像坐标系
Point2d cam2pixel ( const Mat& p, const Mat& K )
{
    return Point2d
           (
               p.at<double> ( 0,0 ) * K.at<double> ( 0,0 ) / p.at<double> ( 2,0 ) + K.at<double> ( 0,2 ),
               p.at<double> ( 1,0 ) * K.at<double> ( 0,0 ) / p.at<double> ( 2,0 ) + K.at<double> ( 1,2 )
           );
}

// 用到了g2o求解器
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
    // 从结果上来看，3D2D比3D3D要难，3D2D必须要给定好的初值，而3D3D要求很宽松
    // 从原理上，是因为ICP的极小值就是全局最优值，因此可以任意选择初值
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex，首先设置顶点
    // se3经过指数映射得到SE3
    // se3-李代数，SE3-李群
    // 最终希望用李代数，即se3表示的相机位姿
    // 使用g2o自带的相机位姿顶点类VertexSE3Expmap
    // SE3 Vertex parameterized internally with a transformation matrix
    // and externally with its exponential map
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    // 拷贝一下rotation matrix，不直接用可能是怕被修改了
    R_mat <<
               R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    pose->setId ( 0 ); // 首先设置Id
    // Quat就是quaternion，四元数
    // sets the initial estimate from an array of double
    // 此处需要外界的初始化，因此必须先用solvepnp
    // 当然也可以把这里的初值人为设定了
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            // translation vector
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                            // 以下用于测试，即给定一个固定的初值
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
        // 设置Id
        point->setId ( index++ );
        // 设置初始值，将landmark坐标赋值
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        // TO-DO: 等之后研究，即为marginalization
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        // 最后加到optimizer里去
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        // f和(cx,cy)，默认fx=fy
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
        // 在这里有一个computeError函数，可以看到reprojection error的定义！
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        // 设置Id
        edge->setId ( index );
        // 每个边的两侧分别是pose和Landmark
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    // 最后求解
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true ); // 打开调试输出
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;
    cout<<endl<<"after optimization:"<<endl;
    // 这里的T表示transformation matrix，是4*4的矩阵
    cout<<"Transformation matrix="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
    // assign value to R,T
    Eigen::Matrix4d output = Eigen::Isometry3d( pose->estimate() ).matrix();
    Eigen::Matrix3d R_out = output.block(0,0,3,3); // 利用Eigen的block函数获取子矩阵
    Eigen::Vector3d T_out = output.block(0,3,3,4);
    cout<<"R_out="<<endl<<R_out<<endl;
    cout<<"T_out="<<endl<<T_out<<endl;
    // 将eigen的matrix和vector形式转换成Mat格式
    eigen2cv(R_out, R);
    eigen2cv(T_out, t);
    cout<<"R="<<endl<<R<<endl;
    cout<<"T="<<endl<<t<<endl;
}
