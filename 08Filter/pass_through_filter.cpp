/*
 * Description: pcl中低通滤波器
 */

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char **argv){

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // 生成随机点云数据
  point_cloud_ptr->width  = 5;
  point_cloud_ptr->height = 1;
  point_cloud_ptr->points.resize (point_cloud_ptr->width * point_cloud_ptr->height);

  for (size_t i = 0; i < point_cloud_ptr->points.size (); ++i)   //填充数据
  {
    point_cloud_ptr->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    point_cloud_ptr->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    point_cloud_ptr->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  /************************************************************************************
 创建直通滤波器的对象，设立参数，滤波字段名被设置为Z轴方向，可接受的范围为（0.0，1.0）
 即将点云中所有点的Z轴坐标不在该范围内的点过滤掉或保留，这里是过滤掉，由函数setFilterLimitsNegative设定
 ***********************************************************************************/

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point_filter_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // 设置滤波器对象
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(point_cloud_ptr);            //设置输入点云
  pass.setFilterFieldName("z");         //设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits(0.0, 1.0);        //设置在过滤字段的范围
  //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
  pass.filter(*cloud_point_filter_ptr);            //执行滤波，保存过滤结果在cloud_filtered

  return 0;
}
