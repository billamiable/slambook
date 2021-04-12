#include <iostream>
#include <cmath>

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <chrono>
using namespace std::chrono;
using namespace std;



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

    Eigen::Matrix3d rotation_T = T.rotation();
    cout<<"---- T ---- rotation matrix  =\n"<<rotation_T <<endl;
    
    Eigen::Quaterniond q_T = Eigen::Quaterniond ( rotation_T );
    cout<<"---- T ---- quaternion = \n"<<q_T.coeffs() <<endl;

    Eigen::Vector3d translation_T = T.translation();
    cout<<"---- T ---- translation = \n"<<translation_T.transpose() <<endl;
    

    // 用变换矩阵进行坐标变换，v是（1，0，0）
    Eigen::Vector3d v_transformed = T*v;                              // 相当于R*v+t，理解！
    cout<<"v tranformed = "<<v_transformed.transpose()<<endl;         // 结果正确

    // 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略

    // 四元数
    // 可以直接把AngleAxis赋值给四元数，反之亦然，也就是用角轴转成四元数
    Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部，顺序不一样

    Eigen::Quaterniond q1(0.924, 0, 0, 0.383); // 赋值的时候是w,x,y,z
    cout<<"quaternion1 = \n"<<q1.coeffs() <<endl;

    // 也可以把旋转矩阵赋给它，结果一样！
    Eigen::Quaterniond q2 = Eigen::Quaterniond ( rotation_matrix );
    cout<<"quaternion = \n"<<q2.coeffs() <<endl;

    cout<<"--- rotation matrix =\n"<<q2.toRotationMatrix() <<endl;

    // 使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated = q*v; // 注意数学上是qvq^{-1}
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl; //结果正确

    auto t0 = std::chrono::high_resolution_clock::now();
    Eigen::Quaterniond quat = Eigen::Quaterniond ( T.rotation() );
    Eigen::Vector3d pos = T.translation();
    std::cout <<"time is " << std::chrono::duration_cast<std::chrono::nanoseconds> (std::chrono::high_resolution_clock::now() - t0).count()<< '\n';


    Eigen::Matrix4d T_m = Eigen::Matrix4d::Identity(); 
    T_m.block<3, 3>(0, 0) = rotation_matrix;
    T_m.block<3, 1>(0, 3) = Eigen::Vector3d ( 1,3,4 );
    cout << "Transform matrix = \n" << T_m <<endl;
    cout << "Rotation matrix= \n" << T_m.block<3, 3>(0, 0) << endl;
    cout << "Translation matrix= \n" << T_m.block<3, 1>(0, 3) << endl;

    // Test if Quaterniond only takes normalized rotation matrix
    cout << "Rotation matrix= \n" << rotation_matrix << endl;
    rotation_matrix *= 2.0;
    cout << "Rotation matrix= \n" << rotation_matrix << endl;
    double scale = std::sqrt((rotation_matrix.transpose() * rotation_matrix)(0, 0));
    Eigen::Matrix3d ret = (1. / scale) * rotation_matrix;
    Eigen::Matrix3d rotation_matrix_quat = Eigen::Quaterniond ( ret ).normalized().toRotationMatrix();;
    cout << "Rotation matrix= \n" << rotation_matrix_quat << endl;
    

    return 0;
}
