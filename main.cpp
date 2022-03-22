#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

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

  return (0);
}