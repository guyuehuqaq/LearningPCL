#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <vector>


template<typename Scalar, int Dim, bool fixed_seed>
std::vector<Eigen::Matrix<Scalar, Dim, 1>> generateRandomPoints1(
        size_t N,
        Scalar min_val = 0,
        Scalar max_val = 1) {
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

template <typename Scalar, int Dim>
flann::Matrix<Scalar> eigenVecToFlannMatrix(
        const std::vector<Eigen::Matrix<Scalar, Dim, 1>>& points,
        std::vector<Scalar>& storage) {

    size_t N = points.size();
    storage.resize(N * Dim);

    for (size_t i = 0; i < N; ++i)
        for (int d = 0; d < Dim; ++d)
            storage[i * Dim + d] = points[i][d];

    return flann::Matrix<Scalar>(storage.data(), N, Dim);
}

