#ifndef PROJECTION_H
#define PROJECTION_H

#include "tools/rotation.h"

// camera : 9 dims array with 
// [0-2] : angle-axis rotation 
// [3-5] : translateion
// [6-8] : camera parameter, [6] focal length, [7-8] second and forth order radial distortion
// point : 3D location. 
// predictions : 2D predictions with center of the image plane. 

template<typename T>
// 表征了投影模型
// 1、世界坐标系转换到相机坐标系
// 2、投影至归一化相机平面
// 3、对归一化坐标去畸变
// 4、根据内参模型，计算像素坐标
inline bool CamProjectionWithDistortion(const T* camera, const T* point, T* predictions){
    // 执行第一步
    // Rodrigues' formula
    T p[3];
    AngleAxisRotatePoint(camera, point, p);
    // camera[3,4,5] are the translation
    p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5];

    // 执行第二步
    // Compute the center fo distortion
    // 负号的原因应该是translation的方向问题
    T xp = -p[0]/p[2];
    T yp = -p[1]/p[2];

    // 执行第三步和第四步
    // Apply second and fourth order radial distortion
    const T& l1 = camera[7];
    const T& l2 = camera[8];

    T r2 = xp*xp + yp*yp;
    T distortion = T(1.0) + r2 * (l1 + l2 * r2); // 畸变的系数

    const T& focal = camera[6];
    // TO-DO：下面默认cx=cy=0，合理吗？
    // 可能是为了兼顾不同分辨率的图像，因此使用的坐标是以图像中心为原点的
    predictions[0] = focal * distortion * xp;
    predictions[1] = focal * distortion * yp;

    return true;
}



#endif // projection.h