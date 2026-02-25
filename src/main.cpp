#include <chrono>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

// PCL 相关头文件
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>  // 必须包含用于 PCD 读写
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>  // 使用更强大的 PCLVisualizer
#include <boost/thread/thread.hpp>

// 使用引用传递 Ptr &cloud，否则函数外的指针不会被更新
bool readTxt(const std::string &file_path,
             std::vector<std::vector<double>> &coor_vec_vec,
             pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {

  std::ifstream infile(file_path);
  if (!infile) {
    std::cerr << "Open file failed !" << std::endl;
    return false;
  }

  std::string line;
  std::regex re("\\s+");
  cloud->clear();  // 清空点云

  while (std::getline(infile, line)) {
    if (line.empty())
      continue;

    std::sregex_token_iterator it(line.begin(), line.end(), re, -1);
    std::sregex_token_iterator end;

    std::vector<double> row_data;
    while (it != end) {
      if (!it->str().empty()) {
        try {
          row_data.push_back(std::stod(*it));
        } catch (...) {}
      }
      ++it;
    }

    if (row_data.size() >= 3) {
      // 保存到原始 vector
      coor_vec_vec.push_back(row_data);
      // 保存到 PCL 点云对象
      pcl::PointXYZ pt;
      pt.x = static_cast<float>(row_data[0]);
      pt.y = static_cast<float>(row_data[1]);
      pt.z = static_cast<float>(row_data[2]);
      cloud->push_back(pt);
    }
  }

  std::cout << "Successfully read " << cloud->size() << " points." << std::endl;
  infile.close();
  return true;
}

void visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  if (cloud->empty())
    return;

  // 使用 PCLVisualizer 代替 CloudViewer，支持更精细的控制
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample_cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }
}

void printInfo(const std::vector<std::vector<double>> &coor_vec_vec,
               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  if (cloud->empty())
    return;

  pcl::PointXYZ min, max;
  pcl::getMinMax3D(*cloud, min, max);

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);

  std::cout << "\n---------- 解析数据 ----------" << std::endl;
  std::cout << "维度: " << (coor_vec_vec.empty() ? 0 : coor_vec_vec[0].size())
            << std::endl;
  std::cout << "高度最大(Z): " << max.z << std::endl;
  std::cout << "高度最小(Z): " << min.z << std::endl;
  std::cout << "中心点: (" << centroid(0) << ", " << centroid(1) << ", "
            << centroid(2) << ")" << std::endl;
  std::cout << "总点数: " << cloud->points.size() << std::endl;
  std::cout << "------------------------------\n" << std::endl;
}

void nearestKsearch(int K, pcl::PointXYZ searchPoint,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  std::vector<int> pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);

  if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch,
                            pointKNNSquaredDistance) > 0) {
    std::cout << "K近邻查询成功：" << std::endl;
    for (size_t i = 0; i < pointIdxKNNSearch.size(); ++i)
      std::cout << "    Point: " << (*cloud)[pointIdxKNNSearch[i]].x << " "
                << (*cloud)[pointIdxKNNSearch[i]].y << " "
                << (*cloud)[pointIdxKNNSearch[i]].z
                << " (Squared distance: " << pointKNNSquaredDistance[i] << ")"
                << std::endl;
  }
}

void radiuKsearch(pcl::PointXYZ searchPoint, float radius,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                          pointRadiusSquaredDistance) > 0) {
    std::cout << "半径查询成功，找到 " << pointIdxRadiusSearch.size()
              << " 个点。" << std::endl;
  }
}

int main(int argc, char **argv) {
  // 设置控制台为 UTF-8
  //SetConsoleOutputCP(CP_UTF8);
  //SetConsoleCP(CP_UTF8);

  while (true) {
    auto beforeTime = std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(
        new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::vector<double>> coor_vec_vec;

    std::cout << "请输入TXT文件路径 (输入 'exit' 退出): " << std::endl;
    std::string txt_path;
    std::cin >> txt_path;
    if (txt_path == "exit")
      break;

    if (!readTxt(txt_path, coor_vec_vec, cloud_in))
      continue;

    auto afterTime = std::chrono::steady_clock::now();
    double duration =
        std::chrono::duration<double, std::milli>(afterTime - beforeTime)
            .count();
    std::cout << "解析耗时: " << duration << " ms" << std::endl;

    printInfo(coor_vec_vec, cloud_in);

    std::cout << "选择操作：1-最近邻查找, 2-半径查找, 3-直接可视化, 其他-跳过"
              << std::endl;
    int choose;
    std::cin >> choose;

    pcl::PointXYZ searchPoint;
    if (choose == 1) {
      int K;
      std::cout << "输入K值: ";
      std::cin >> K;
      std::cout << "输入查询点X Y Z: ";
      std::cin >> searchPoint.x >> searchPoint.y >> searchPoint.z;
      nearestKsearch(K, searchPoint, cloud_in);
    } else if (choose == 2) {
      float radius;
      std::cout << "输入半径: ";
      std::cin >> radius;
      std::cout << "输入查询点X Y Z: ";
      std::cin >> searchPoint.x >> searchPoint.y >> searchPoint.z;
      radiuKsearch(searchPoint, radius, cloud_in);
    }

    visualization(cloud_in);
  }
  return 0;
}