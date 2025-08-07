/*
 * @Description: 点云搜索  https://pcl.readthedocs.io/projects/tutorials/en/latest/octree.html#octree-search
 * @Author: HJW
 */

#include <pcl/point_cloud.h>   //点云头文件
#include <pcl/octree/octree.h> //八叉树头文件

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


int main(int argc, char **argv){
    // 如果用这个，每次执行都不一样
    //    srand((unsigned int)time(NULL)); //用系统时间初始化随机种子与 srand (time (NULL))的区别

    // 生成数据
    auto cloud = generateRandomData(1000, -2.0, 2.0);

    float resolution = 0.1f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud(); //构建octree

    pcl::PointXYZ searchPoint(0.0f,0.0f,0.0f);  // 设置寻找点
    /*************************************************************************************
  一旦PointCloud和octree联系一起，就能进行搜索操作，这里使用的是“体素近邻搜索”，把查询点所在体素中
   其他点的索引作为查询结果返回，结果以点索引向量的形式保存，因此搜索点和搜索结果之间的距离取决于octree的分辨率参数
*****************************************************************************************/

    std::vector<int> pointIdxVec; //存储体素近邻搜索结果向量
    if (octree.voxelSearch(searchPoint, pointIdxVec)) //执行搜索，返回结果到pointIdxVec
    {
        std::cout << "Neighbors within voxel search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z << ")"
                  << std::endl;

        for (size_t i = 0; i < pointIdxVec.size(); ++i) //打印结果点坐标
            std::cout << "    " << cloud->points[pointIdxVec[i]].x
                      << " " << cloud->points[pointIdxVec[i]].y
                      << " " << cloud->points[pointIdxVec[i]].z << std::endl;
    }

    /**********************************************************************************
K 被设置为10 ，K近邻搜索  方法把搜索结果写到两个分开的向量，第一个pointIdxNKNSearch包含搜索结果
（结果点的索引的向量）  第二个向量pointNKNSquaredDistance存储搜索点与近邻之间的距离的平方。
*************************************************************************************/

    //K 近邻搜索
    int K = 10;

    std::vector<int> pointIdxNKNSearch;         //结果点的索引的向量
    std::vector<float> pointNKNSquaredDistance; //搜索点与近邻之间的距离的平方

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;

    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
                      << " " << cloud->points[pointIdxNKNSearch[i]].y
                      << " " << cloud->points[pointIdxNKNSearch[i]].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    // 半径内近邻搜索

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 0.5;
    std::cout << "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;
    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
                      << " " << cloud->points[pointIdxRadiusSearch[i]].y
                      << " " << cloud->points[pointIdxRadiusSearch[i]].z
                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

    return 0;
}
