#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>

/****************************
* 本程序演示了 Eigen 几何模块的使用方法
****************************/

// TO-DO: 搞清楚四元数的顺序问题，明天去填下。

int main ( int argc, char** argv )
{
    // Eigen/Geometry 模块提供了各种旋转和平移的表示
    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    // 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
    // 构造函数初始化
    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) );     //沿 Z 轴旋转 45 度
    // 保留三位小数！
    cout .precision(3);
    // 将旋转向量用矩阵的形式表征出来
    cout<<"rotation matrix =\n"<<rotation_vector.matrix() <<endl;                //用matrix()转换成矩阵
    // 将旋转向量形式转换成旋转矩阵形式
    cout<<"rotation matrix =\n"<<rotation_vector.toRotationMatrix() <<endl;
    rotation_matrix = rotation_vector.toRotationMatrix();
    
    // 定义了一个待旋转的向量
    Eigen::Vector3d v ( 1,0,0 );
    // 这里应该是重载<<的时候存在顺序的问题，因此需要加上transpose来正确显示想要展示的内容
    cout<<"(1,0,0) before rotation = "<<v.transpose()<<endl;
    // 用 AngleAxis 可以进行坐标变换
    // 里面应该是重载了*运算符
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;
    // 或者用旋转矩阵，效果一样，同理重载
    v_rotated = rotation_matrix * v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

    // 欧拉角: 可以将旋转矩阵直接转换成欧拉角
    // 这里的2,1,0分别代指ZYX，表示先后以这三个为轴旋转
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles ( 2,1,0 ); // ZYX顺序，即roll pitch yaw顺序
    cout<<"yaw pitch roll = "<<euler_angles.transpose()<<endl;

    // 欧式变换，包括了旋转与平移
    // 欧氏变换矩阵使用 Eigen::Isometry（等距）
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();    // 虽然称为3d，实质上是4＊4的矩阵
    // 通过调用成员函数的方式来赋值
    T.rotate ( rotation_vector );                         // 按照rotation_vector进行旋转
    // pretranslate函数用于inital position setting
    // Applies on the right the translation matrix represented by the vector
    T.pretranslate ( Eigen::Vector3d ( 1,3,4 ) );         // 把平移向量设成(1,3,4)
    cout << "Transform matrix = \n" << T.matrix() <<endl;

    // 用变换矩阵进行坐标变换，v是（1，0，0）
    Eigen::Vector3d v_transformed = T*v;                              // 相当于R*v+t，理解！
    cout<<"v tranformed = "<<v_transformed.transpose()<<endl;         // 结果正确

    // 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略

    // 四元数
    // 可以直接把AngleAxis赋值给四元数，反之亦然，也就是用角轴转成四元数
    Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部，顺序不一样
    // 也可以把旋转矩阵赋给它，结果一样！
    q = Eigen::Quaterniond ( rotation_matrix );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;
    // 使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated = q*v; // 注意数学上是qvq^{-1}
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl; //结果正确

    return 0;
}
