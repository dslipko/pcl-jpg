#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>  // put_time
#include <iostream>
#include <sstream>  // stringstream

void showHelp(char *program_name) {
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[bin]"
            << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *viewer) {
  if (event.getKeySym() == "s" && event.keyDown()) {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream datetime;
    datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    ((pcl::visualization::PCLVisualizer *)viewer)
        ->saveScreenshot("Image" + datetime.str() + ".jpg");
    std::cout << "Export view as Image" + datetime.str() + ".jpg" << std::endl;
  }
}

int main (int argc, char** argv)
 {

  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }

  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".bin");
  
  
  if (filenames.size () != 1) {
      showHelp (argv[0]);
      return -1;
    } 
  std::fstream file_;
  file_.open(argv[filenames[0]], std::ios::in | std::ios::binary);
  if (!file_.is_open()) {
    std::cout << "Can't open the input file." << std::endl;
    exit(1);
  } else {
    // TODO: Move std::out string to log
    std::cout << "File " << argv[filenames[0]] 
              << " opened succesfully!" << std::endl;
  }
  pcl::PointCloud<pcl::PointXYZ> points;
  std::size_t file_size_ = std::filesystem::file_size(argv[filenames[0]]);
  std::size_t cloud_size = file_size_ / 3;
  points.resize(cloud_size);

  for (size_t i = 0; i < cloud_size; i++) {
    file_.read(reinterpret_cast<char *>(&points[i].x), sizeof(float));
    file_.read(reinterpret_cast<char *>(&points[i].y), sizeof(float));
    file_.read(reinterpret_cast<char *>(&points[i].z), sizeof(float));
  }

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(points, centroid);
  std::cout << "Find centroid:" << centroid[0] << " " << centroid[1] << " "
            << centroid[2] << " " << std::endl
            << "Transform points  cloud..." << std::endl;

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -centroid[0], -centroid[1], -centroid[2];

  pcl::transformPointCloud(points, points, transform);
  pcl::compute3DCentroid(points, centroid);
  std::cout << "New centroid:" << centroid[0] << " " << centroid[1] << " "
            << centroid[2] << " " << std::endl;

  pcl::visualization::PCLVisualizer viewer("PCL cloud to image export");
  viewer.setBackgroundColor(0, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ>(points.makeShared(), "sample cloud");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer.initCameraParameters();
  viewer.setCameraPosition(-1.01337, 15.255, -19.4681, 0.0255838, 0.802148,
                           0.596578, 0, 0, 1);
  viewer.setCameraFieldOfView(0.8575);
  viewer.setCameraClipDistances(9.27686, 50.4547);
  viewer.setSize(800, 480);
  viewer.setShowFPS(false);
  std::vector<pcl::visualization::Camera> cam;

  std::cout << "Press `s` to export view." << std::endl;
  viewer.registerKeyboardCallback(keyboardEventOccurred, &viewer);

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
      }
  return (0);
}