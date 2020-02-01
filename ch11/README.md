# 习题解答

**1、如果将位姿图中的误差定义为：∆ξij = ξi ◦ ξ−1，推导按照此定义下的左乘扰动雅可比矩阵。**

目前还没完全理解右乘扰动雅克比矩阵的推导过程，主要困在公式(11.7)的最后两步，待理解右乘后来补充左乘的推导。


**2、参照g2o的程序，在Ceres中实现对“球”位姿图的优化。**

参考资料：https://blog.csdn.net/u012348774/article/details/84144084

代码思路：主要涉及到了设置新的Local Parameterization和重载SizedCostFunction函数来实现雅克比矩阵，其中使用Local Paramterization的原因请见以下[google forum](https://groups.google.com/forum/#!msg/ceres-solver/7HfF6DnCv7o/h38kAxYKAwAJ)。具体代码实现请见[博客](https://blog.csdn.net/u012348774/article/details/84144084)。


**3、对“球”中的信息按照时间排序，分别喂给g2o和gtsam优化，比较它们的性能差异。**

待补充。


**4、阅读isam相关论文，理解它是如何实现增量式优化的。**

参考资料：https://www.zhihu.com/question/279027808

增量式优化的理解：增量式优化的本质是根据当前测量来判断影响了哪些历史状态，从而有针对性地进行更新。可以这么认为，其上限为卡尔曼滤波，即只保留当前状态，抛弃所有的历史信息；其下限为批量式凸优化，即考虑所有的状态来优化。具体实现的细节请见[知乎回答](https://www.zhihu.com/question/279027808)和论文。



# 参考文献

- Ceres优化球位子图：https://blog.csdn.net/u012348774/article/details/84144084
- isam增量式优化：https://www.zhihu.com/question/279027808