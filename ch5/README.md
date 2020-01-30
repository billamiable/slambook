# 习题解答

**1、寻找一个相机(你手机或笔记本的摄像头即可)，标定它的内参。你可能会用到标定板，或者自己打印一张标定用的棋盘格。**

先打印棋盘格，平整贴在木板等固定物上，然后用手机或者相机从不同角度拍摄棋盘格获得至少10组照片，最后用OpenCV/ Matlab/ ROS的标定工具得到相机的内参。

- OpenCV的标定工具：https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
- Matlab的标定工具：https://www.mathworks.com/help/vision/ug/single-camera-calibrator-app.html
- ROS的标定工具：http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration


**2、叙述相机内参的物理意义。如果一个相机的分辨率变成两倍而其他地方不变，它的内参如何变化？**

内参的物理意义：fx, fy表示x,y方向上的焦距，单位为像素；cx, cy表示像素坐标系与归一化相机坐标系的圆点之间的平移。通过内参可以实现3D点在归一化相机坐标系和像素坐标系之间的转换。

内参的变化：因为分辨率变为两倍，表示像素坐标系上的u,v值都变为两倍，因此为了等式成立，fx, fy, cx, cy都要变为两倍。


**3、搜索特殊的相机(鱼眼或全景)相机的标定方法。它们与普通的针孔模型有何不同？**

- 鱼眼相机的标定

标定方法来源：https://blog.csdn.net/u010784534/article/details/50474371

标定方法：与普通镜头的成像模型没有区别，区别主要体现在畸变系数，鱼眼相机标定时增加了高阶系数。

- 全景相机的标定

标定方法来源：https://blog.csdn.net/kyjl888/article/details/72859601

标定方法：与普通镜头的成像模型不同，全景相机使用球面来表征像素，具体标定方法见上文。



**4、调研全局快门相机(global shutter)和卷帘快门相机(rolling shutter)的异同。它们在SLAM中有何优缺点？**

参考资料：

- https://blog.csdn.net/jucilan3330/article/details/81984887
- https://blog.csdn.net/qq_17032807/article/details/84971560

全局快门相机(global shutter)和卷帘快门相机(rolling shutter)的异同：

- Global shutter：CMOS中的所有像素全部在同时进行曝光。优点：提高了拍照效率，适用于高速移动物体；缺点：容易有噪点，相机价格高；
- Rolling shutter：CMOS中的像素逐行曝光。优点：相机价格便宜，大多数CMOS传感器采用该方法；缺点：在拍照物体高速移动时会出现blur。


**5、RGBD相机是如何标定的？以Kinect为例，需要标定哪些参数？(参照https://github.com/code-iai/iai_kinect2)**

参考资料：

- https://blog.csdn.net/aichipmunk/article/details/9264703
- https://blog.csdn.net/Siyuada/article/details/78981555

RGBD相机的标定：Kinect 1代的深度值是通过红外结构光的方式获得的，因此相比于普通相机多了红外相机的参数。流程上，一般首先标定RGB和红外相机的内参，其中红外相机的照片可以通过拍摄只是红外光源照射的标定板获得；接下来通过同一时刻记录的RGB和红外相机拍到的照片组进行外参的标定。具体请见上述链接。


**6、除了示例程序演示的遍历图像的方式，你还能举出哪些遍历图像的方法？**

参考资料：

- https://blog.csdn.net/qq_17032807/article/details/84971560
- https://blog.csdn.net/coolbare/article/details/78130486

遍历图像的方法：

- 使用at函数进行逐点读取：image.at<>(i,j)；
- 指针遍历：image.Ptr<>()[]，具体可见本章代码；
- 迭代器遍历：Mat_<Vec3b>::iterator it= image.begin<Vec3b>()；
- 指针算术方法：根据channel和每个单channel像素点所占内存空间来计算对应像素点的内存中的位置；

以上几种是比较常用的方法，还有别的方法请见上述链接。


**7、阅读OpenCV官方教程，学习它的基本用法。**

见https://docs.opencv.org/master/d9/df8/tutorial_root.html。


--

# 参考文献

- OpenCV的标定工具：https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
- Matlab的标定工具：https://www.mathworks.com/help/vision/ug/single-camera-calibrator-app.html
- ROS的标定工具：http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
- 鱼眼相机的标定：https://blog.csdn.net/u010784534/article/details/50474371
- 全景相机的标定：https://blog.csdn.net/kyjl888/article/details/72859601
- Global vs rolling shutter：https://blog.csdn.net/jucilan3330/article/details/81984887
- Global vs rolling shutter：https://blog.csdn.net/qq_17032807/article/details/84971560
- Kinect标定：https://blog.csdn.net/aichipmunk/article/details/9264703
- Kinect标定：https://blog.csdn.net/Siyuada/article/details/78981555
- 遍历图像：https://blog.csdn.net/qq_17032807/article/details/84971560
- 遍历图像：https://blog.csdn.net/coolbare/article/details/78130486
- OpenCV官方教程：https://docs.opencv.org/master/d9/df8/tutorial_root.html