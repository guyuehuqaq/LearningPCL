/*
 * Description: 写pcd
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
  //实例化的模板类PointCloud  每一个点的类型都设置为pcl::PointXYZ
  /*************************************************
点PointXYZ类型对应的数据结构
  Structure PointXYZ{
   float x;
   float y;
   float z;
  };
**************************************************/

  // 创建点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_ptr;

  point_cloud.width = 100;
  point_cloud.height = 1;
  point_cloud.is_dense = false;
  point_cloud.points.resize(point_cloud.width*point_cloud.height);
  // 随机数生成点云数据
  for (size_t i = 0; i < point_cloud.points.size(); ++i)
  {
    point_cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    point_cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    point_cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }
  //把PointCloud对象数据存储在 test_pcd.pcd文件中
  pcl::io::savePCDFileASCII("../test_pcd.pcd", point_cloud);
  //打印输出存储的点云数据
  std::cerr << "Saved " << point_cloud.points.size() << " data points to test_pcd.pcd!" << std::endl;
//
//  for (size_t i = 0; i < point_cloud.points.size(); ++i)
//    std::cerr << "    " << point_cloud.points[i].x << " " << point_cloud.points[i].y << " " << point_cloud.points[i].z << std::endl;

  return 0;
}