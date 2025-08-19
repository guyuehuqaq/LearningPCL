/*
 * Description: 读取pcd文件
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  auto& point_cloud = *point_cloud_ptr;
  if (pcl::io::loadPCDFile("../test_pcd.pcd", point_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return -1;
  }
  std::cout << "Loaded "
            << point_cloud.width * point_cloud.height // 宽*高
            << " data points from test_pcd.pcd with the following fields! "
            << std::endl;
  return 0;
}