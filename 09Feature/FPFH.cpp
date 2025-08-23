/*
 * Description: Fast Point Feature Histograms(FPFH)
 * FPFH（Fast Point Feature Histogram）是 PFH（Point Feature Histogram） 的快速近似版本，用于描述点云局部几何特征，但计算量更小，速度更快
 * 和PFH的区别: 计算简化 PFH（SPFH） → 加权聚合邻居的 SPFH，减少成对计算。
 * SPFH：PFH的简化版，对点 p，只计算 p 与每个邻居 q 的特征（而不是所有邻居对）。
 * //////////////////////////////////
 * // FPFH(p) = SPFH(p) + (1/k) * sum_{q in N(p)} (1 / w_pq) * SPFH(q)
// 解释：
// 1️⃣ SPFH(p) ：点 p 的简化PFH特征，只考虑点 p 与其邻居点 q 的角度关系
// 2️⃣ SPFH(q) ：邻居点 q 的简化PFH特征
// 3️⃣ w_pq ：点 p 与邻居 q 的权重，一般与距离成反比，距离越近，贡献越大
// 4️⃣ FPFH(p) ：点 p 的快速PFH特征
//    - 它不仅包含 p 自身的局部几何信息（SPFH(p)）
//    - 还融合了邻居点的几何信息（加权平均 SPFH(q)）
// 5️⃣ 优点：比 PFH 快很多（O(n*k)），仍保留邻域几何特征
///////////////////////////////////////////
 */


#include<iostream>
#include<vector>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
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
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh; // phf特征估计其器
  fpfh.setInputCloud(cloud);
  fpfh.setInputNormals(cloud_normals);
  //创建一个空的kd树表示法，并把它传递给PFH估计对象。
  //基于已给的输入数据集，建立kdtree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
  //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); //-- older call for PCL 1.5-
  fpfh.setSearchMethod(tree2);//设置近邻搜索算法
  //输出数据集
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_fe(new pcl::PointCloud<pcl::FPFHSignature33>());//phf特征
  //使用半径在5厘米范围内的所有邻元素。
  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
  fpfh.setRadiusSearch(0.05);
  //计算pfh特征值
  fpfh.compute(*fpfh_fe);
  cout << "fphf feature size : " << fpfh_fe->points.size() << endl;
}

