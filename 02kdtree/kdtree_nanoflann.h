#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <vector>
#include <random>

struct Point2D {
    const std::vector<Eigen::Vector2d>& pts2d;
    Point2D(const std::vector<Eigen::Vector2d>& points2d) : pts2d(points2d){}
    inline size_t kdtree_get_point_count() const { return pts2d.size(); }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pts2d[idx](dim);
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};
using KDTree2D = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, Point2D>,
        Point2D,
        2
>;

struct Point3D {
    const std::vector<Eigen::Vector3d>& pts3d;
    Point3D(const std::vector<Eigen::Vector3d>& points3d) : pts3d(points3d){}
    inline size_t kdtree_get_point_count() const { return pts3d.size(); }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pts3d[idx](dim);
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};
using KDTree3D = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, Point3D>,
        Point3D,
        3
>;

template<typename Scalar, int Dim, bool fixed_seed>
std::vector<Eigen::Matrix<Scalar, Dim, 1>> generateRandomPoints1(
        size_t N,
        Scalar min_val = 0,
        Scalar max_val = 1){
    std::vector<Eigen::Matrix<Scalar, Dim, 1>> points;
    points.reserve(N);
    std::mt19937 gen(fixed_seed ? 42 : std::random_device{}());
    std::uniform_real_distribution<Scalar> dis(min_val, max_val);
    for (size_t i = 0; i < N; ++i) {
        Eigen::Matrix<Scalar, Dim, 1> p;
        for (int d = 0; d < Dim; ++d) {
            p[d] = dis(gen);
        }
        points.push_back(p);
    }
    return points;
}

//// 用nanoflann中的Matrix表达数据接口
//using KDTreeMatrix2D = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<double, Eigen::Dynamic, 2>, 2>;
//using KDTreeMatrix3D = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<double, Eigen::Dynamic, 3>, 3>;

// 写成一个通用矩阵模板
template <typename Scalar, int Dim>
using KDTreeEigenMatrixAdaptor = nanoflann::KDTreeEigenMatrixAdaptor<
        Eigen::Matrix<Scalar, Eigen::Dynamic, Dim>, Dim>;

// 随机生成数据
template<typename Scalar, int Dim, bool FixedSeed = false>
Eigen::Matrix<Scalar, Eigen::Dynamic, Dim> generateRandomPoints2(
        size_t N,
        Scalar min_val = -1,
        Scalar max_val = 1) {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Dim> points(N, Dim);
    std::mt19937 gen(FixedSeed ? 42 : std::random_device{}());
    std::uniform_real_distribution<Scalar> dis(min_val, max_val);
    for (size_t i = 0; i < N; ++i) {
        for (int d = 0; d < Dim; ++d) {
            points(i, d) = dis(gen);
        }
    }
    return points;
}
