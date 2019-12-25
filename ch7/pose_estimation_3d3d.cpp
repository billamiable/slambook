#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// 下面两个的顺序不能错！先Eigen,才能opencv-eigen
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>
#include <typeinfo>

using namespace std;
using namespace cv;

void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

// 像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Mat& K );

void pose_estimation_3d3d (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t
);

void bundleAdjustment(
    const vector<Point3f>& points_3d,
    const vector<Point3f>& points_2d,
    Mat& R, Mat& t
);

// 这里还专门自定义了g2o的一种边
// g2o edge
// TO-DO: 据说这是一种单元边,unary edge，只需要连接一个vertex
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly( const Eigen::Vector3d& point ) : _point(point) {}

    // 这个还是好理解的
    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // measurement is p, point is p'
        // 就是pose*p'-p，所以pose->estimate().map()实现的是相乘
        // 其实就是先取最近的迭代结果，然后R*P+T
        _error = _measurement - pose->estimate().map( _point );
    }

    // 这里主要是定义求导运算
    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        // estimate表示return the current estimate of the vertex
        // 因为是一个迭代的过程！这里的T表示Pose，而不仅仅表示translation matrix
        g2o::SE3Quat T(pose->estimate());
        // map就是R*P+T
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        // Oplus类似卷积符号，这里表示的是扰动！！！搜嘎！
        // TO-DO: 这个以后学习下吧，和Jacobian有关，3D2D里也留下了
        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -z;
        _jacobianOplusXi(0,2) = y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;

        _jacobianOplusXi(1,0) = z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = -x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;

        _jacobianOplusXi(2,0) = -y;
        _jacobianOplusXi(2,1) = x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }

    bool read ( istream& in ) {}
    bool write ( ostream& out ) const {}

// TO-DO: 为何这样定义也可以以后再搞清楚一点！
protected:
    Eigen::Vector3d _point;
};

// 这里其实已经是简化版的ICP，因为所有的点都是匹配好的
// 但在真实的应用场景中，很多是没有匹配的，而且有的甚至都没有深度值！
// 这个时候就需要结合PNP一起求解
// 这也是为啥叫ICP，而不是直接某个点，需要closest point
int main ( int argc, char** argv )
{
    if ( argc != 5 )
    {
        cout<<"usage: pose_estimation_3d3d img1 img2 depth1 depth2"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    // 建立3D点
    // 这次两个深度图都要读取了，因为是3D3D嘛！
    Mat depth1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED );       // 深度图为16位无符号数，单通道图像
    Mat depth2 = imread ( argv[4], CV_LOAD_IMAGE_UNCHANGED );       // 深度图为16位无符号数，单通道图像
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f> pts1, pts2;
    // same
    for ( DMatch m:matches )
    {
        // 这个就是前面的读取方式
        ushort d1 = depth1.ptr<unsigned short> ( int ( keypoints_1[m.queryIdx].pt.y ) ) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        ushort d2 = depth2.ptr<unsigned short> ( int ( keypoints_2[m.trainIdx].pt.y ) ) [ int ( keypoints_2[m.trainIdx].pt.x ) ];
        if ( d1==0 || d2==0 )   // bad depth
            continue;
        // same
        Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
        Point2d p2 = pixel2cam ( keypoints_2[m.trainIdx].pt, K );
        float dd1 = float ( d1 ) /5000.0;
        float dd2 = float ( d2 ) /5000.0;
        pts1.push_back ( Point3f ( p1.x*dd1, p1.y*dd1, dd1 ) );
        pts2.push_back ( Point3f ( p2.x*dd2, p2.y*dd2, dd2 ) );
    }

    // 本质上这里定义了求解3d3d的方法，第一是SVD，第二是BA
    cout<<"3d-3d pairs: "<<pts1.size() <<endl;
    Mat R, t;
    // 基本理解了！
    pose_estimation_3d3d ( pts1, pts2, R, t );
    cout<<"ICP via SVD results: "<<endl;
    cout<<"R = "<<R<<endl;
    cout<<"t = "<<t<<endl;
    // 这个为啥是Inverse？这个应该是transpose！
    // 对于Orthogonal matrix，transpose就是inverse
    // 这里的原因是之前PNP是第一帧到第二帧的变换，但这里是第二帧到第一帧！
    cout<<"R_inv = "<<R.t() <<endl;
    // 这个有点奇怪，噢噢，最后乘以了t?
    // 其实就是求了一个逆
    cout<<"t_inv = "<<-R.t() *t<<endl;

    cout<<"calling bundle adjustment"<<endl;
    // 优化方法，又是和前面一样，先给了一个initialization。
    // 不对！这里后面根本没用！
    // 我一样可以试下给个Initialization
    // 最好是最后计算一个指标，error的量级，这样就可以看出initialization有没有作用
    // init没有用？？hhh，根本里面就没有更改R,T
    // 明确下ba里是算1->2还是2->1
    // 2->1，和这个code前面的定义一致
    bundleAdjustment( pts1, pts2, R, t );
    // 的确修改了
    cout<<"Here, R="<<endl<<R<<endl;
    cout<<"Here, T="<<endl<<t<<endl;
    // 这个挺好的，就是correct_check环节
    // verify p1 = R*p2 + t
    float avg_err;
    float error = 0;
    // vector<float> error;
    // for ( int i=0; i<5; i++ )
    for ( int i=0; i<pts1.size(); i++ )
    {
        // cout<<"p1 = "<<pts1[i]<<endl;
        // cout<<"p2 = "<<pts2[i]<<endl;
        // cout << "type 1"<<typeid(pts1[i]).name() << endl;
        // 这个应该和p1尽量接近，结果的确是！
        Mat_<double> p2_1 = R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t;
        // cout<<"(R*p2+t) = "<<
        //     // R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t
        //     p2_1
        //     <<endl;
        // cout<<endl;
        // cout << "type 2"<<typeid(p2_1).name() << endl;
        // calculate error
        float e = abs(p2_1.at<double> ( 0,0 ) - pts1[i].x) + 
                  abs(p2_1.at<double> ( 1,0 ) - pts1[i].y) +
                  abs(p2_1.at<double> ( 2,0 ) - pts1[i].z);
        // cout<<"error is "<<e/3.0<<endl;
        // error.push_back(float (e/3.0));
        error += e;
    }
    avg_err = error / pts1.size();
    cout<<"average error is "<<avg_err<<endl;
}

// same
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
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
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

// same
Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

// 这个是本code的精华！但是要先自己把优化公式写下来~
// 写下来了，开始吧！
// 第一种方法：解析方法！
void pose_estimation_3d3d (
    // 输入和输出变量跃然纸上
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t
)
{
    Point3f p1, p2;     // center of mass，定义质心
    int N = pts1.size();
    // loop
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    // 获得质心！
    p1 = Point3f( Vec3f(p1) /  N);
    p2 = Point3f( Vec3f(p2) / N);
    // 定义去质心坐标
    vector<Point3f>     q1 ( N ), q2 ( N ); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        // 这个就是最基本的q_i*q_i^T，定义成W
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    cout<<"W="<<W<<endl;

    // SVD on W
    // 还有这种骚操作，直接用SVD来解
    // TO-DO: 之后学习下SVD，为何可以求解这种问题~
    // 这个都是我之后要学习的！自己学会怎么写！
    // eigen里面有内置的svd函数
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    // 获得U,V，这样就得到了R
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    
    // 行列式
    // TO-DO: 这个是在干嘛？
    // 感觉是对U进行特殊值处理
    if (U.determinant() * V.determinant() < 0)
	  {
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
	  }
    // 获得了最后的U,V
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    // 从而得到了R
    Eigen::Matrix3d R_ = U* ( V.transpose() );
    // 这个就是优化的第二项，直接得到T
    Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );

    // convert to cv::Mat from Eigen matrix
    // what is difference between Mat_ and Mat?
    // Mat_ is convenient if we use a lot of element access operations
    // It is a thin template on top of Mat
    R = ( Mat_<double> ( 3,3 ) <<
          R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
          R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
          R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
        );
    t = ( Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );
}

// 这个总体上应该和之前的差不多~
// 第二种方法，优化方法！
void bundleAdjustment (
    const vector< Point3f >& pts1,
    const vector< Point3f >& pts2,
    Mat& R, Mat& t )
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
    // 哟，这里居然敢用GN？？哈哈哈！测试下就好了
    // 这里还可以衡量error
    // 有意思的是，这里好解！GN和LB的结果一样，3D2D里GN直接不收敛。。
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    // vertex
    Eigen::Matrix3d R_mat;
    R_mat <<
               R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    // 必须用双引号！
    // cout<<"R in BA: "<<R.at<double> ( 0,0 )<<" "<<R.at<double> ( 0,1 )<<" "<<R.at<double> ( 0,2 )<<endl;
    // cout<<"T in BA: "<<t.at<double> ( 0,0 )<<" "<<t.at<double> ( 1,0 )<<" "<<t.at<double> ( 2,0 )<<endl;
    // same definition
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(
        R_mat,
        Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
        // 这里不知道，所以就直接
        // Eigen::Matrix3d::Identity(),
        // Eigen::Vector3d( 0,0,0 )
    ) );
    optimizer.addVertex( pose );

    // edges
    int index = 1;
    // 这个就是我们新定义的！之后需要看一下~
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    // size_t好久不见啊~
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(
            Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z) );
        // 为啥不用i呢哈哈~
        edge->setId( index );
        // TO-DO: 只设置一个vertex，表示单元边
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
        edge->setMeasurement( Eigen::Vector3d(
            pts1[i].x, pts1[i].y, pts1[i].z) );
        edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    // 正式开始求解~
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;
    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;
    
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
