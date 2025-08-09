# Random Sample Consensus(随机采样一致性)
随机采样一致（RANSAC）是一种迭代方法，用于从一组包含异常值的观测数据中估计数学模型的参数，
当异常值对估计值没有影响时。因此，它也可以解释为一种异常值检测方法。
它是一种非确定性算法，因为它只有在一定的概率下才会产生合理的结果，并且随着允许更多的迭代，这种概率会增加。

优点：
- 鲁棒性强，能够在大量异常点干扰下准确估计模型
- 简单易实现，应用广泛

不足：
- 迭代次数依赖于异常点比例，计算量可能较大
- 对阈值选择敏感，阈值过大或过小都会影响效果
- 不保证全局最优，只能保证概率意义上的最优

## RANSAC算法流程
### 1. 随机抽样
随机从数据集中随机选取一个最小样本集（最少数据点数目足以确定模型参数

### 2. 模型拟合
根据这组最小样本计算模型参数

### 3. 内点检测
计算所有数据点到该模型的误差，将误差小于设定阈值的点判定为内点

### 4. 判断模型优劣
若当前模型的内点数比历史最好模型多，则更新模型参数

### 5. 迭代
重复以上过程固定次数（或者直到达到满意内点比例），最后选择内点最多的模型作为结果

## RANSAC伪代码
```
Given:
    data – A set of observations.
    model – A model to explain the observed data points.
    n – The minimum number of data points required to estimate the model parameters.
    k – The maximum number of iterations allowed in the algorithm.
    t – A threshold value to determine data points that are fit well by the model (inlier).
    d – The number of close data points (inliers) required to assert that the model fits well to the data.

Return:
    bestFit – The model parameters which may best fit the data (or null if no good model is found).


iterations = 0
bestFit = null
bestErr = something really large // This parameter is used to sharpen the model parameters to the best data fitting as iterations go on.

while iterations < k do
    maybeInliers := n randomly selected values from data
    maybeModel := model parameters fitted to maybeInliers
    confirmedInliers := empty set
    for every point in data do
        if point fits maybeModel with an error smaller than t then
            add point to confirmedInliers
        end if
    end for
    if the number of elements in confirmedInliers is > d then
        // This implies that we may have found a good model.
        // Now test how good it is.
        betterModel := model parameters fitted to all the points in confirmedInliers
        thisErr := a measure of how well betterModel fits these points
        if thisErr < bestErr then
            bestFit := betterModel
            bestErr := thisErr
        end if
    end if
    increment iterations
end while

return bestFit
```

## 开发和改进
https://en.wikipedia.org/wiki/Random_sample_consensus#Parameters
- MSAC（M-estimator SAmple and Consensus）
- MLESAC（Maximum Likelihood Estimation SAmple and Consensus）