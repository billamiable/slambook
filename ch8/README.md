# 习题解答

**1、除了LK光流之外，还有哪些光流方法？它们各有什么特点？**

参考资料：https://blog.csdn.net/qq_38906523/article/details/80781242

光流方法主要分为以下几类：

- 基于梯度：利用时变图像灰度（或其滤波形式）的时空微分（即时空梯度函数）来计算像素的速度矢量。
- 基于匹配：包括基于特征和区域两种，基于特征的方法不断地对目标主要特征进行定位和跟踪，基于区域的方法先对类似的区域进行定位，然后通过相似区域的位移计算光流。
- 基于能量：要对输入图像序列进行时空滤波处理，这是一种时间和空间整合。
- 基于相位：速度是根据带通滤波器输出的相位特性确定。
- 基于神经动力学：源于大脑的运动感知模型，可以提供运动边缘的法向速度估计（待研究）。

以上内容基于较早的文献，最新的方法分类可以参考光流的综述文章“Optical flow modeling and computation: A survey”，发表时间为2015年。

**2、在本节的程序的求图像梯度过程中，我们简单地求了 u + 1 和 u − 1 的灰度之差除2，作为 u 方向上的梯度值。这种做法有什么缺点？提示：对于距离较近的特征，变化应该较快；而距离较远的特征在图像中变化较慢，求梯度时能否利用此信息?**

简单地对于所求点两侧的像素灰度值作差并除2，忽略了特征与相机的距离带来的差异性，具体表现为离相机距离较近的特征相比于距离较远的特征所占像素范围更大，同样的2个像素的跨度距离较近的可能仍为同一特征，而距离较远的则可能为不同特征。因此更好的方法为根据特征的深度值来动态控制像素跨度，即深度值越大，像素跨度应越小。


**3、在稀疏直接法中，假设单个像素周围小块的光度也不变，是否可以提高算法鲁棒性？请编程实现此事。**

参考资料：https://blog.csdn.net/qq_17032807/article/details/85265620

由于由单个像素变成了像素块，且满足局部平滑的先验，因此可以辅助求解，用于提高算法鲁棒性。实现方面只需要将edge定义下computeError函数中的以下语句：

```cpp
_error ( 0,0 ) = getPixelValue ( x,y ) - _measurement;
```

改为

```cpp
float sumValue = 0.0;
for (int i=x-2; i<=x+2; ++i)
    for(int j=y-2; j<=y+2; ++j)
        sumValue += getPixelValue(i,j);
sumValue /= 25;
_error(0,0) = sumValue - _measurement;
```

就可以实现像素到像素块的转变。


**4、使用Ceres实现RGB-D上的稀疏直接法和半稠密直接法。**

参考资料：https://blog.csdn.net/qq_17032807/article/details/85265620

与g2o相比，Ceres同样需要定义雅克比矩阵和误差，但是省去了节点与边的定义，具体的实现请见[博客](https://blog.csdn.net/qq_17032807/article/details/85265620)。（待研究）


**5、相比于RGB-D的直接法，单目直接法往往更加复杂。除了匹配未知之外，像素的距离也是待估计的。我们需要在优化时把像素深度也作为优化变量。阅读[59, 57]，你能理解它的原理吗？如果不能，请在13讲之后再回来阅读。**

参考文献57为LSD-SLAM，是一个large-scale direct monocular slam系统，由TUM的Daniel Cremers教授团队提出，参考文献59同样出自这个团队。具体原理待研究。


**6、由于图像的非凸性，直接法目前还只能用于短距离，非自动曝光的相机。你能否提出增强直接法鲁棒性的方案？阅读[58, 60]可能会给你一些灵感。**

参考资料：https://blog.csdn.net/qq_17032807/article/details/85265620

参考文献58为DSO，即Direct sparse odometry，是较为成熟的直接法单目SLAM算法。参考文献60为基于IMU和视觉紧耦合的双目SLAM系统。以上两个文章都出自TUM的Daniel Cremers团队。

这两篇文章试图对图像的非凸性进行改良，其中文献60利用IMU的短期位姿测量较为准确的特点，加上视觉信号在长期测量时的纠正，从而耦合取得不错的效果；文献58则定义了三种雅克比矩阵：图像雅克比、几何雅克比和光度雅克比。后两者作者认为相对于像素坐标量而言是光滑的，所以只计算一次，但图像雅克比不够光滑，因此引入了First Estimate Jacobian的优化方法，这是一个非常重要的概念。（待研究）


--


# 参考文献

- 光流：https://blog.csdn.net/qq_38906523/article/details/80781242
- 光流：Fortun, Denis, Patrick Bouthemy, and Charles Kervrann. "Optical flow modeling and computation: a survey." Computer Vision and Image Understanding 134 (2015): 1-21.
- 基于像素块的稀疏直接法：https://blog.csdn.net/qq_17032807/article/details/85265620
- Ceres实现RGBD稀疏和半稀疏直接法：https://blog.csdn.net/qq_17032807/article/details/85265620
- 直接法解决图像非凸性：https://blog.csdn.net/qq_17032807/article/details/85265620