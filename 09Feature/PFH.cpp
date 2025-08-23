/*
 * Description: 点特征直方图（PFH）描述子
 */

#include<iostream>
#include<vector>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/io/pcd_io.h>//点云文件pcd 读写
#include <pcl/features/normal_3d_omp.h>//法线特征
#include <pcl/visualization/pcl_plotter.h>// 直方图的可视化 方法2
#include <pcl/visualization/pcl_visualizer.h>

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

  // 3️⃣ 创建PFH估计对象pfh，并将输入点云数据集cloud和法线normals传递给它
  pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh; // phf特征估计其器
  pfh.setInputCloud(cloud);
  pfh.setInputNormals(cloud_normals);
  //创建一个空的kd树表示法，并把它传递给PFH估计对象。
  //基于已给的输入数据集，建立kdtree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
  //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); //-- older call for PCL 1.5-
  pfh.setSearchMethod(tree2);//设置近邻搜索算法
  //输出数据集
  pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_fe_ptr(new pcl::PointCloud<pcl::PFHSignature125>());//phf特征
  //使用半径在5厘米范围内的所有邻元素。
  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
  pfh.setRadiusSearch(0.05);
  //计算pfh特征值
  pfh.compute(*pfh_fe_ptr);
  cout << "phf feature size : " << pfh_fe_ptr->points.size() << endl;

  // ================点云可视化=====================
  pcl::visualization::PCLVisualizer viewer("PointCloud Viewer");
  viewer.setBackgroundColor(0, 0, 0); // 黑色背景
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> cloud_color(cloud, "z");
  viewer.addPointCloud<pcl::PointXYZ>(cloud, cloud_color, "cloud");
  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
  }

  // ========直方图可视化=============================
  pcl::visualization::PCLPlotter plotter;
  plotter.addFeatureHistogram(*pfh_fe_ptr, 300); //设置的很坐标长度，该值越大，则显示的越细致
  plotter.plot();

  return 0;

}