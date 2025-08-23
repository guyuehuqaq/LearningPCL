/*
 * Description: 给定一个点云数据集， 直接计算云中每个点的表面法线
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
  // 打开点云代码
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("../table_scene_lms400.pcd", *cloud);

  // 创建法线估计估计向量
//  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // 多线程版本
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne; // OMP 版本
  ne.setInputCloud(cloud);
  // 创建一个空的KdTree对象，并把它传递给法线估计向量
  // 基于给出的输入数据集，KdTree将被建立
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod(tree);
  // 存储输出数据
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // 使用半径在查询点周围3厘米范围内的所有临近元素
  ne.setRadiusSearch(0.03);
  // 计算特征值
  ne.compute(*cloud_normals);
// cloud_normals->points.size ()应该与input cloud_downsampled->points.size ()有相同的尺寸

  // 存储特征值为点云
  pcl::PCDWriter writer;
  writer.write<pcl::Normal> ( "../table_cloud_normals.pcd" , *cloud_normals, false); // 保存文件
  // 可视化
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(cloud, "z");
  viewer.addPointCloud<pcl::PointXYZ>(cloud, color_handler, "cloud");
//  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals); // 将估计的点云表面法线添加到屏幕。

//  // 每隔 10 个点显示一个法线，长度 0.03
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 1, 0.03, "normals");

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce(100);
  }
  return 0;
}