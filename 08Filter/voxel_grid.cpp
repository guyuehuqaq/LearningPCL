/*
 * Description: 根据体素网格对点云数据进行下采样
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv){

  // pcl::PCLPointCloud2 通用点云模板
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  pcl::PCDReader reader;
  reader.read("../table_scene_lms400.pcd", *cloud);

  /******************************************************************************
创建一个voxel叶大小为1cm的pcl::VoxelGrid滤波器，
**********************************************************************************/
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //创建滤波对象
  sor.setInputCloud (cloud);            //设置需要过滤的点云给滤波对象
  sor.setLeafSize (0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
  sor.filter (*cloud_filtered);           //执行滤波处理，存储输出

  pcl::PCDWriter writer;
  writer.write("../table_scene_lms400_dawnsample.pcd", *cloud_filtered);

  /******************************************************************************
   可视化滤波前后的点云结果
  *******************************************************************************/

  // 数据格式转化
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_filter(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_xyz_filter);

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  int v1(0), v2(1);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); // 左半边
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2); // 右半边
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->setBackgroundColor(0, 0, 0, v2);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_xyz, 0, 255, 0); // 绿色
  viewer->addPointCloud(cloud_xyz, color, "sample cloud", v1);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

  // 滤波点云 → 红色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_filter(cloud_xyz_filter, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud_xyz_filter, color_filter, "cloud_filtered", v2);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_filtered");


  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce ();
    pcl_sleep(0.01);
  }

return 0;
}