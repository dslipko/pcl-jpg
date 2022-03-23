#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

pcl::visualization::PCLVisualizer::Ptr simpleVis(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return (viewer);
}

int main() {
  pcl::PointCloud<pcl::PointXYZ> points;
  std::fstream file_;
  file_.open("car.bin", std::ios::in | std::ios::binary);
  if (!file_.is_open()) {
    std::cout << "Can't open the input file.";
    exit(1);
  } else {
    // TODO: Move std::out string to log
    std::cout << "File "
              << " opened succesfully!";
  }

  std::size_t file_size_ = std::filesystem::file_size("car.bin");
  std::size_t cloud_size = file_size_ / 3;
  points.resize(cloud_size);

  for (size_t i = 0; i < cloud_size; i++) {
    file_.read(reinterpret_cast<char *>(&points[i].x), sizeof(float));
    file_.read(reinterpret_cast<char *>(&points[i].y), sizeof(float));
    file_.read(reinterpret_cast<char *>(&points[i].z), sizeof(float));
  }
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(points, centroid);

  std::cout << "centroid:" << centroid[0] << " " << centroid[1] << " "
            << centroid[2] << " " << centroid[3] << " \n";
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = simpleVis(points.makeShared());
  viewer->pcl::visualization::PCLVisualizer::saveScreenshot("filename.png");
  return (0);
}