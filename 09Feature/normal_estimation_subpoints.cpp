/*
 * Description: 给定一个点云数据集，从中筛选子集，直接计算子集点的法线
 * 过程：1. 通过邻域搜索
 *      2. 局部拟合平面
 *      3. PCA特征提取，方差小的特征向量为法线
 * 局限性：如果通过邻域搜索得到的局部拟合平面不是近似平面，PCA方法计算的法线会存在较大误差
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

int main(int argc, char **argv){

  // 1️⃣ 读取点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile("../table_scene_lms400.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file\n");
    return -1;
  }
  std::cout << "Loaded point cloud with " << cloud->size() << " points.\n";

  // 2️⃣ 取点云前 10% 点作为子集
  size_t subset_size = cloud->size() / 10;
  std::vector<int> indices(subset_size);
  for (size_t i = 0; i < subset_size; ++i) indices[i] = i;

  pcl::IndicesPtr indices_ptr(new std::vector<int>(indices));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, *indices_ptr, *cloud_subset);

  // 3️⃣ 法线估计
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setIndices(indices_ptr);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);

  ne.setRadiusSearch(0.03); // 3cm 半径

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*cloud_normals);
  std::cout << "Computed normals for " << cloud_normals->size() << " points.\n";

  // 5️⃣ 可视化
  pcl::visualization::PCLVisualizer viewer("PointCloud + Normals");
  viewer.setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(cloud, "z");
  viewer.addPointCloud<pcl::PointXYZ>(cloud, color_handler, "cloud");

//  // 6️⃣ 手动显示法线箭头
//  for (size_t i = 0; i < cloud_subset->size(); ++i)
//  {
//    const auto& pt = cloud_subset->points[i];
//    const auto& n = cloud_normals->points[i];
//
//    Eigen::Vector3f start(pt.x, pt.y, pt.z);
//    Eigen::Vector3f end(pt.x + n.normal_x * 0.03f,
//                        pt.y + n.normal_y * 0.03f,
//                        pt.z + n.normal_z * 0.03f);
//
//    std::string arrow_id = "arrow_" + std::to_string(i);
//    viewer.addArrow(pcl::PointXYZ(end.x(), end.y(), end.z()),
//                    pcl::PointXYZ(start.x(), start.y(), start.z()),
//                    1.0, 0.0, 0.0,   // 红色箭头
//                    false,
//                    arrow_id);
//  }

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
  }

  return 0;
}