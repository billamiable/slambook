# 绘制PR曲线代码
# 代码参考：https://blog.csdn.net/u013044310/article/details/89890626

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

def PR_curve(y, prob):
    # TP+FN
    positive = np.sum(y == 1)
    
    # 对预测概率进行排序，从而方便计算不同阈值时的数据
    prob_sort = np.sort(prob)[::-1]
    index = np.argsort(prob)[::-1]
    y_sort = y[index]

    precision, recall = [], []
    for i, _ in enumerate(prob_sort):
        # 给定初值，同时防止i=0作为分母
        if i==0:
            precision.append(1)
            recall.append(0)
        else:
            # :i表示0~i-1，因此认为是一个累加的过程
            precision.append(np.sum(y_sort[:i] == 1) / i)     # 所有回环中确实是真实回环的概率
            recall.append(np.sum(y_sort[:i] == 1) / positive) # 所有真实回环中被正确检测出来的概率

    plt.plot(recall, precision)
    plt.title('Precision-Recall Curve')
    plt.xlim([-0.01, 1.01]) # 设置显示范围
    plt.ylim([-0.01, 1.01])
    plt.ylabel('Precision')
    plt.xlabel('Recall')
    plt.show()

data_size = 50
y = np.random.randint(0, 2, size=data_size) # 随机产生0，1两个类别
prob = np.random.choice(np.arange(0.1, 1, 0.001), data_size) # 随机给定概率
PR_curve(y, prob)