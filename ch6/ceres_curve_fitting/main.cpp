#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// 结构体+模板元编程
// Difference between a struct and class: 
// access memember variables - In a struct they are public; in a class they are private
// 代价函数的计算模型
struct CURVE_FITTING_COST
{
    // 构造函数的initilization list，_x是结构体成员变量名，x是输入变量名
    CURVE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
    // 残差的计算
    // 模板元本质上是复杂了一点的template
    // Declare template: 
    // template <class T>
    // T someFunction(T arg)
    // {
    //   ... .. ...
    // }
    // 后面就可以直接用someFunction
    
    // 和template <class T>基本是一样的
    // 把T看成一个type，比如double，那T(-y)就是double(-y)
    template <typename T>
    bool operator() (
        // const T* abc，表示值固定；T* const abc，表示指针固定（因为abc是指针，const直接作用）
        // 以下表示指针指向固定，值固定
        const T* const abc,     // 模型参数，有3维
        T* residual ) const     // 残差
    {
        // residual和abc都是指针，因此都要[0]才能表征值
        residual[0] = T ( _y ) - ceres::exp ( abc[0]*T ( _x ) *T ( _x ) + abc[1]*T ( _x ) + abc[2] ); // y-exp(ax^2+bx+c)
        return true;
    }
    const double _x, _y;    // x,y数据
};

int main ( int argc, char** argv )
{
    // y=exp(ax^2+bx+c)+w
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N=100;                          // 数据点
    double w_sigma=1.0;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double abc[3] = {0,0,0};            // abc参数的估计值

    vector<double> x_data, y_data;      // 数据

    // 人工产生一些数据，然后拟合
    cout<<"generating data: "<<endl;
    for ( int i=0; i<N; i++ )
    {
        // 如果改成i/100，x就变成0了
        double x = i/100.0;
        x_data.push_back ( x );
        y_data.push_back (
            // rng是随机数生成器，产生高斯噪声，std=w_sigma=1.0，varaince=1.0
            exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
        );
        cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }

    // 构建最小二乘问题，ceres都是构造Problem来求解的
    ceres::Problem problem;
    // i++和++i在for循环中一样，类中用++i更高效，传回的是引用
    for ( int i=0; i<N; i++ )
    {
        // feed给problem的是残差，即y-exp(ax^2+bx+c)
        problem.AddResidualBlock (     // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            // 输入一维的x，输出三维的abc
            // 1 means number of residuals, 3 means Number of parameters in block 0
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3> ( 
                new CURVE_FITTING_COST ( x_data[i], y_data[i] ) // new新建对象
            ),
            // kernel function is what is applied on each data instance to 
            // map the original non-linear observations into a higher-dimensional space 
            // in which they become separable
            // 二范数随离群点的错误程度增大而增长很快，因此就会影响优化结果
            // 当有明显的离群点时，可以在这里加入penalty方法，比如new CachyLoss(0.5)或者Huber loss
            nullptr,            // 核函数，这里不使用，为空，可以理解为分段函数
            abc                 // 待估计参数，要求输入指针形式！
        );
    }

    // 配置求解器，options里有很多可以设置，多数都有默认值
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解，选择QR分解求解Ax=b
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve ( options, &problem, &summary );  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    // 输出结果
    cout<<summary.BriefReport() <<endl;
    cout<<"estimated a,b,c = ";
    for ( auto a:abc ) cout<<a<<" ";
    cout<<endl;

    return 0;
}

