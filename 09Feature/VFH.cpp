/*
 * Description: VFH,全局特征描述子，主要用于物体识别
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

int main(int argc, char **argv){

  // 1️⃣ 读取点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read("../ism_test_cat.pcd", *cloud);

  // 2️⃣ 创建法线估计向量
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);
  // 输出点云 带有法线描述
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.03);
  // 计算法线
  ne.compute(*cloud_normals);

  // 3️⃣ 创建VFH
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud(cloud);
  vfh.setInputNormals(cloud_normals);
  vfh.setSearchMethod(tree);
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308>());
  vfh.compute (*vfhs);
  std::cout << "vfhs.size: " << vfhs->size() << std::endl;
  return 0;
}