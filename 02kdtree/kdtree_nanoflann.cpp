#include <iostream>

#include "kdtree_nanoflann.h"

int main(int argc, char** argv){

#ifdef POINT2D
    // 生成1000个随机2D数
    std::vector<Eigen::Vector2d> points2d;
    points2d = generateRandomPoints1<double, 2, true>(1000, -2, 2);
    // 构建KDtree
    Point2D point2d1_adaptor(points2d);
    KDTree2D kdtree2d1(2, point2d1_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kdtree2d.buildIndex();
    // 随机设置一个2d点
    Eigen::Vector2d point(0,0);
    size_t index;
    double sqrt_dist;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&index, &sqrt_dist);
    kdtree2d1.findNeighbors(resultSet, point.data(), nanoflann::SearchParameters());
    std::cout << "the closest point's idx: " << index << std::endl;
    std::cout << "the closest point: " << points2d[index].transpose() << std::endl;
    std::cout << "the closest point's dist: " << sqrt_dist << std::endl;
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
    std::cout << "the closest point: " <<  points3d.row(index)
    << std::endl;
    std::cout << "the closest point's dist: " << sqrt_dist << std::endl;
    return 0;
#endif



}