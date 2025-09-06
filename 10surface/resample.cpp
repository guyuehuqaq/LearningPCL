/*
 * Description: 使用MLS方法，重采样点云数据，得到更光滑的点云数据。
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>

int main(int argc, char **argv){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;
  reader.read("../ism_test_cat.pcd", *cloud);

  // 创建kdtree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

  // 创建存储MLS计计算后得到的法线数据
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  // 是否计算法线
  mls.setComputeNormals(true);
  // 设置参数
  mls.setInputCloud(cloud);
  mls.setPolynomialOrder(1); // 设置多项式拟合阶数
  mls.setSearchMethod(kdtree);
  mls.setSearchRadius(1);

//  mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
//  mls.setUpsamplingRadius(0.03);
//  mls.setUpsamplingStepSize(0.01);

  mls.process(*mls_points);
  pcl::PCDWriter writer;
//  writer.write("../bun0_mls.pcd", *mls_points);

  std::cout << "Original Cloud size: " << cloud->size() << std::endl;
  std::cout << "MLS Smoothed Cloud size: " << mls_points->size() << std::endl;

  // 可视化器
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("MLS Comparison"));
  viewer->initCameraParameters();

  // 左视口（原始点云）
  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("Original Cloud", 10, 10, "v1_text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_orig(cloud, 255, 255, 255);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, color_orig, "cloud_orig", v1);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_orig");

  // 右视口（MLS点云）
  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
  viewer->addText("MLS Smoothed Cloud", 10, 10, "v2_text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> color_mls(mls_points, 0, 255, 0);
  viewer->addPointCloud<pcl::PointNormal>(mls_points, color_mls, "cloud_mls", v2);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_mls");
  // 循环显示
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }

  return 0;
}
