#include "kdtree_flann.h"


int main(int argc, char** argv){
    constexpr int Dim = 2;
    constexpr size_t N = 1000;
    // Step 1: 生成随机点
    auto points = generateRandomPoints1<float, Dim, true>(N, -2.0f, 2.0f);

    // Step 2: 转换为 flann::Matrix
    std::vector<float> raw_data;
    flann::Matrix<float> dataset = eigenVecToFlannMatrix<float, Dim>(points, raw_data);

    // Step 3: 构建 KD 树索引
    flann::Index<flann::L2<float>> index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();

    // Step 4: 查询一个点
    std::vector<float> query_point = {0.0f, 0.0f};
    flann::Matrix<float> query(query_point.data(), 1, Dim);


}