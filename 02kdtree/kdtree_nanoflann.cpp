#include <iostream>

#include "kdtree_nanoflann.h"

int main(int argc, char** argv){

//#define POINT2D
#ifdef POINT2D
    // 生成1000个随机2D数
    std::vector<Eigen::Vector2d> points2d;
    points2d = generateRandomPoints1<double, 2, true>(1000, -2, 2);
    // 构建KDtree
    Point2D point2d1_adaptor(points2d);
    KDTree2D kdtree2d1(2, point2d1_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kdtree2d1.buildIndex();
    // 随机设置一个2d点
    Eigen::Vector2d point(0,0);
    // 最近林查找
    size_t index;
    double sqrt_dist;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&index, &sqrt_dist);
    kdtree2d1.findNeighbors(resultSet, point.data(), nanoflann::SearchParameters());
    std::cout << "the closest point's idx: " << index << std::endl;
    std::cout << "the closest point: " << points2d[index].transpose() << std::endl;
    std::cout << "the closest point's dist: " << sqrt_dist << std::endl;

    // 半径查找
    const double search_radius = 0.1;  // 半径长度
    // nanoflann 进行半径搜索：
    nanoflann::SearchParameters params;
    std::vector<nanoflann::ResultItem<unsigned int, double>> indices_dists;
    size_t nMatches = kdtree2d1.radiusSearch(point.data(), search_radius * search_radius, indices_dists, params);
    std::cout << "找到 " << nMatches << " 个邻居点，半径为 " << search_radius << std::endl;
    for (size_t i = 0; i < nMatches; ++i)
    {
        auto& idx = indices_dists[i].first;
        auto& point2d = points2d[idx];
        auto& dist_sqr = indices_dists[i].second;
        std::cout << "idx: " << idx
                  << "point: (" << point2d.x()  << ", " << point2d.y()  << ")"
                  << "dist: " << dist_sqr << std::endl;
    }
    return 0;
#endif

#define POINT3D
#ifdef POINT3D
    auto points3d = generateRandomPoints2<double, 3, true>(1000, -2, 2);
    using KDTreeMatrix3D = KDTreeEigenMatrixAdaptor<double, 3>;
    KDTreeMatrix3D kdtree3d2(3, std::cref(points3d), 10);
    kdtree3d2.index_->buildIndex();
    Eigen::Vector3d point(0,0,0);
    size_t index;
    double sqrt_dist;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&index, &sqrt_dist);
    kdtree3d2.index_->findNeighbors(resultSet, point.data(), nanoflann::SearchParameters());
    std::cout << "the closest point's idx: " << index << std::endl;
    std::cout << "the closest point: " <<  points3d.row(index) << std::endl;
    std::cout << "the closest point's dist: " << sqrt_dist << std::endl;

    const double search_radius = 0.1;
    std::vector<nanoflann::ResultItem<long long, double>> ret_matches;
    nanoflann::SearchParameters params;
    size_t nMatches = kdtree3d2.index_->radiusSearch(
            point.data(),
            search_radius * search_radius,
            ret_matches,
            params);
    std::cout << "找到 " << nMatches << " 个邻居点，半径为 " << search_radius << std::endl;
    for (size_t i = 0; i < nMatches; ++i)
    {
        auto& idx = ret_matches[i].first;
        auto& dist_sqr = ret_matches[i].second;
        std::cout << "idx: " << idx
                  << "point: (" << points3d.row(idx) << ")"
                  << "dist: " << dist_sqr << std::endl;
    }
    return 0;
#endif



}