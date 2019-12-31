// 以下代码参考
// Eigen提取block：https://blog.csdn.net/jiahao62/article/details/80655542
// Eigen提取block:https://www.cnblogs.com/newneul/p/8306430.html
// 求解Ax=b：https://www.cnblogs.com/newneul/p/8306442.html

#include <iostream>
using namespace std;
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

#define EQUATION_NUM 3
#define VARIABLE_NUM 3

int main( int argc, char** argv )
{
    // Eigen提取block
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE );
    cout << "before assign \n" << matrix_NN.block(0, 0, 3, 3) << endl;
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Identity();
    matrix_NN.block(0, 0, 3, 3) = matrix_33;
    cout << "after assign \n"  << matrix_NN.block(0, 0, 3, 3) << endl;

    //-------------------------------------------------------------------------------------
    // 求解Ax=b
    Eigen::Matrix<double, EQUATION_NUM, VARIABLE_NUM> A = Eigen::MatrixXd::Random(EQUATION_NUM, VARIABLE_NUM);
    Eigen::Matrix<double, EQUATION_NUM, 1> b = Eigen::MatrixXd::Random(EQUATION_NUM, 1);
    // 测试用例
    A << 10,3,1,2,-10,3,1,3,10;
    b << 14,-5,14;
    // 设置解变量
    Eigen::Matrix<double, VARIABLE_NUM, 1> x;

    clock_t time_stt = clock(); // 计时
    
    // 方法一：直接求逆，适用条件：方阵
    x = A.inverse()*b;
    cout <<"time use in normal inverse is "   << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms"<< endl;
    
	  // 方法二：QR分解，适用条件：方阵和非方阵
    // 当方程组有解时的出的是真解，若方程组无解得出的是近似解
    time_stt = clock();
    x = A.colPivHouseholderQr().solve(b);
    cout << "x^T = " << x.transpose() <<endl;
    cout <<"time use in Qr decomposition is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;

    // 方法三：最小二乘法，适用条件：方阵和非方阵
    // 方程组有解时得出真解，否则是最小二乘解(在求解过程中可以用QR分解 分解最小二成的系数矩阵)
    time_stt = clock();
    x = (A.transpose() * A ).inverse() * (A.transpose() * b);
    cout << "x^T = " << x.transpose() <<endl;
    cout <<"time use in least square is "     << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;

    // 方法四：LU分解法，适用条件：方阵
    // 满足分解的条件才行
    time_stt = clock();
    x = A.lu().solve(b);
    cout << "x^T = " << x.transpose() <<endl;
    cout <<"time use in LU decomposition is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;

    // 方法五：Cholesky分解法，适用条件：方阵
    // 结果与其他的方法差好多，如此简单的也错了。。
    time_stt = clock();
    x = A.llt().solve(b);
    cout << "x^T = " << x.transpose() <<endl;
    cout <<"time use in Cholesky decomposition is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;

    // 方法六：Jacobi迭代法，挖坑待补充。。


    //-------------------------------------------------------------------------------------
    // 坐标变换
    Eigen::Quaterniond q1(0.35, 0.2,  0.3, 0.1);
    cout << "quaternion 1 = \n" << q1.coeffs() << endl;
    Eigen::Quaterniond q2(-0.5, 0.4, -0.1, 0.2);
    cout << "quaternion 2 = \n" << q2.coeffs() << endl;
    q1.normalize(); // remember to normalize
    q2.normalize();
    cout << "normalized quaternion 1 = \n" << q1.coeffs() << endl;
    cout << "normalized quaternion 2 = \n" << q2.coeffs() << endl;
    Eigen::Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
    
    // 注意下标，T1w表示world to 1！
    Eigen::Isometry3d T1w(q1); // 初始化方法一
    // Eigen::Isometry3d T1w(q1.toRotationMatrix()); // 初始化方法二
    // Eigen::Isometry3d T1w = Eigen::Isometry3d::Identity(); 
    // T1w.rotate(q1.toRotationMatrix()); // 初始化方法三
    T1w.pretranslate(t1);
    cout << "Transform matrix 1 = \n" << T1w.matrix() <<endl;
    // world to 2
    Eigen::Isometry3d T2w(q2);
    T2w.pretranslate(t2);
    cout << "Transform matrix 2 = \n" << T2w.matrix() <<endl;

    Eigen::Vector3d p1(0.5, 0, 0.2);
    // first from 1 to world, then world to 2
    Eigen::Vector3d p2 = T2w * T1w.inverse() * p1; 
    cout << "coordinate in camera 2 is \n" << p2.transpose() << endl;

    return 0;
}
