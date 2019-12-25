#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// 可以的，有新的类型加入！结构体~
// 不仅仅是结构体，以下是模板元编程，作用是把本该发生在运行阶段的一些工作挪到编译期，从而获得更高性能
// Difference between a struct and class: 
// access memember variables - In a struct they are public; in a class they are private
// 代价函数的计算模型
struct CURVE_FITTING_COST
{
    // 搞清楚_x的效用~ 本质就是一个变量定义，与外层同样意义的变量区分开
    // TO-DO: 这个还没理解，以后再说吧~
    CURVE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
    // 残差的计算
    // 模板元本质上是复杂了一点的template
    // Declare template: 
    // template <class T>
    // T someFunction(T arg)
    // {
    //   ... .. ...
    // }
    // 后面就可以直接用someFunction~
    
    // 这个和template <class T>基本是一样的，除了这里typename不仅仅局限于class~
    // 以下就是ceres的基本定义形式，模板编程思路！
    // 把T看成一个type，比如double，那T(-y)就是double(-y)~
    template <typename T>
    // 输入，返回Bool型
    bool operator() (
        // 这里表示指针指向固定，值固定，若T是double，就是const double* const abc~
        // TO-DO: const T* abc/ T const* abc，表示值固定；T* const abc，表示指针固定（有点反直觉？）
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
        // 这里必须要用100.0，否则x就都变成0了
        // double x = i/100;
        double x = i/100.0;
        // 利用push_back存储到vector中
        x_data.push_back ( x );
        y_data.push_back (
            // rng是随机数生成器，产生高斯噪声，std=w_sigma=1.0，varaince=1.0
            exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
        );
        // 输出产生的数据
        cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }

    // 哈哈，都是解最小二乘哈哈~
    // 构建最小二乘问题，ceres都是构造Problem来求解的
    ceres::Problem problem;
    // i++和++i在for循环中一样，但是i=1;j=++i/i++，前者j=2，后者j=1，i都等于2
    for ( int i=0; i<N; i++ )
    {
        // feed给problem的是残差，即y-exp(ax^2+bx+c)
        problem.AddResidualBlock (     // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            // 输入一维的x，输出三维的abc
            // 1 means number of residuals, 3 means Number of parameters in block 0
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3> ( 
                // 这个CURVE_FITTING_COST是结构体CURVE_FITTING_COST里的函数！
                new CURVE_FITTING_COST ( x_data[i], y_data[i] )
            ),
            // kernel function is what is applied on each data instance to 
            // map the original non-linear observations into a higher-dimensional space 
            // in which they become separable
            // 二范数随离群点的错误程度增大而增长很快，因此就会影响优化结果
            // 当有明显的离群点时，可以在这里加入penalty方法，比如new CachyLoss(0.5)
            nullptr,            // 核函数，这里不使用，为空，可以理解为分段函数，比如Huber loss!原来是它啊！
            abc                 // 待估计参数，要求输入指针形式！
        );
    }

    // 这个可以看下官方文档！Google出品，资料还是挺齐全的~
    // 配置求解器，options里有很多可以设置，多数都有默认值
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解，选择QR分解，线性代数的~
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();  // 加入了时间戳
    ceres::Solve ( options, &problem, &summary );  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();  // 加入了时间戳
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;  // 输出优化时间

    // 输出结果
    cout<<summary.BriefReport() <<endl; // 输出report
    cout<<"estimated a,b,c = ";  // a,b,c是什么意思？
    // 哈哈，for loop with auto又见面了~
    for ( auto a:abc ) cout<<a<<" ";
    cout<<endl;

    return 0;
}

