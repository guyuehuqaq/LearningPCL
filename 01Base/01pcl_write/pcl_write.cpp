#include <iostream>
#include <random>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    std::default_random_engine eng;
    std::uniform_real_distribution<float> dist(0.0f, 1024.0f);
    for (auto &point : cloud) {
        point.x = dist(eng);
        point.y = dist(eng);
        point.z = dist(eng);
    }

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    std::cout << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;

    for (const auto &point : cloud)
        std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    return (0);
}