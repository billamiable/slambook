#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>
using namespace std; 

// 从g2o的BaseVertex父类中派生出子类
// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    // If you define a structure having members of fixed-size vectorizable Eigen types, 
    // you must overload its "operator new" so that it generates 16-bytes-aligned pointers. 
    // Fortunately, Eigen provides you with a macro EIGEN_MAKE_ALIGNED_OPERATOR_NEW that does that for you.
    // 结构体包含eigen成员，必须进行宏定义 EIGEN_MAKE_ALIGNED_OPERATOR_NEW, 保证内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // 利用虚函数重载函数
    {   
        // 初始值设置
        // 发现：初值直接设置成真值，用GN也发散了。。
        // 结论：g2o的GN实现的不太好，通常用g2o都是LM或dogleg优化
        _estimate << 0,0,0;
    }
    
    // 定义增量方程里的加法，这里很简单，直接向量相加
    // 但是有的时候需要自定义加法，比如位姿相加
    virtual void oplusImpl( const double* update ) // 更新
    {
        _estimate += Eigen::Vector3d(update);
    }
    // 存盘和读盘：留空
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
};

// 由于在曲线拟合中，待估计的参数只有abc，用数组表示，只用到了一个node，因此是一元边（UnaryEdge）
// 从g2o的BaseUnaryEdge父类中派生出子类
// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 内存对齐
    CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {}
    // 计算曲线模型误差
    void computeError()
    {
        // static_cast类型转换，因为用到了继承
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate(); // Matrix<double,3,1>，所以有两个维度
        // 这里是计算误差项，y值为_measurement，表示ground truth
        _error(0,0) = _measurement - std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) ) ;
    }
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
public:
    double _x;  // x 值
};

int main( int argc, char** argv )
{
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N=100;                          // 数据点
    double w_sigma=1.0;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double abc[3] = {0,0,0};            // abc参数的估计值

    vector<double> x_data, y_data;      // 数据
    
    cout<<"generating data: "<<endl;
    for ( int i=0; i<N; i++ )
    {
        double x = i/100.0;
        x_data.push_back ( x );
        y_data.push_back (
            exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
        );
        cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }
    
    // 构建图优化，先设定g2o
    // 矩阵块：每个误差优化变量维度为4，误差值维度为1
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;
    // 线性方程求解器：稠密的增量方程
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
    // 梯度下降方法，从GN, LM, DogLeg 中选
    // 发现：LM可以收敛，GN不收敛，DogLeg收敛且快！
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm( solver );   // 设置求解器
    optimizer.setVerbose( true );       // 打开调试输出
    
    // 往图中增加顶点
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate( Eigen::Vector3d(0,0,0) );
    // 设置id，应该是当系统大了的时候用作区分
    v->setId(0);
    optimizer.addVertex( v );
    
    // 往图中增加边
    // 对于边，与ceres一样可以加入核函数，只需要设置一下
    for ( int i=0; i<N; i++ )
    {
        CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );
        edge->setId(i);
        // 由于这里本身只有一个node，因此设置一个0 node
        edge->setVertex( 0, v );                // 设置连接的顶点，这个很重要，就是残差相减的两项
        edge->setMeasurement( y_data[i] );      // 观测数值
        // 信息矩阵：协方差矩阵之逆，按照公式来赋值即可
        edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) );
        optimizer.addEdge( edge );
    }
    
    // 执行优化
    cout<<"start optimization"<<endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;
    
    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout<<"estimated model: "<<abc_estimate.transpose()<<endl;
    
    return 0;
}