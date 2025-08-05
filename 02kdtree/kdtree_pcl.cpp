#include <pcl/point_cloud.h>  // 点云类型
#include <pcl/kdtree/kdtree_flann.h>  // pcl中的kdtree类型

#include <iostream>
#include <vector>
#include <random>
#include <ctime>

// 生成随机点云数据
pcl::PointCloud<pcl::PointXYZ>::Ptr generateRandomData(
        size_t N,
        float min_val = -2.0f,
        float max_val = 2.0f,
        bool fixed_seed = true){
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->width = static_cast<int32_t>(N);
    cloud->height = 1; // 说明点是无组织的
    cloud->is_dense = true; // 点云中的点是否全部有效,有没有Nan,Inf
    cloud->points.resize(N);
    std::mt19937 gen;
    gen.seed(fixed_seed ? 42 : std::random_device{}());
    std::uniform_real_distribution<float> dis(min_val, max_val);
    for (size_t i = 0; i < N; ++i)
    {
        cloud->points[i].x = dis(gen);
        cloud->points[i].y = dis(gen);
        cloud->points[i].z = dis(gen);
    }
    return cloud;
}

int main(int argc, char** argv){

    // 生成数据，智能指针
    auto cloud = generateRandomData(1000, -2.0, 2.0);
    // 构建kdtree_pcl
    auto kdtree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
    kdtree->setInputCloud(cloud);
    // 设置搜索的点
    pcl::PointXYZ search_point(0.0,0.0,0.0);
    // KNN最近邻搜索
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( kdtree->nearestKSearch (search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )  //执行K近邻搜索
    {
        //打印所有近邻坐标
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
                      << " " << cloud->points[ pointIdxNKNSearch[i] ].y
                      << " " << cloud->points[ pointIdxNKNSearch[i] ].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }else{
        std::cout << "not find neighbor!" << std::endl;
    }

    // 半径搜索
    float radius = 0.1f;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (kdtree->radiusSearch(search_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
            std::cout << "idx: " << pointIdxRadiusSearch[i]
                      << "dis: " << pointRadiusSquaredDistance[i]
                      << "points: (" << cloud->points[pointIdxRadiusSearch[i]].x << ", "
                      << cloud->points[pointIdxRadiusSearch[i]].y << ", "
                      << cloud->points[pointIdxRadiusSearch[i]].z << ")"
                      << std::endl;
        }
    }
    else
    {
        std::cout << "radius not find point!" << std::endl;
    }

}