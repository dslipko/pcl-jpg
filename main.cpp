#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>  
#include <iostream>
#include <sstream>  
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


// This function displays the help.
void showHelp(char *program_name) {
  std::cout << "PCL-JPG exports 3D view of [.bin] file in [.jpg] images." << std::endl;

  std::cout << "Usage: " << program_name << " cloud_filename.[bin]"
            << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

// This function calls when the key `s` pressed for export view in .jpg
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *viewer) {
  if (event.getKeySym() == "s" && event.keyDown()) {
    // Add timestamp to image filename.
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream datetime;
    datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");

    // Save current view as .jpg image.
    ((pcl::visualization::PCLVisualizer *)viewer)
        ->saveScreenshot("Image" + datetime.str() + ".jpg");
    std::cout << "Export view as Image" + datetime.str() + ".jpg" << std::endl;
  }
}

int main(int argc, char **argv) {
  // Show help if needed.
  if (pcl::console::find_switch(argc, argv, "-h") ||
      pcl::console::find_switch(argc, argv, "--help")) {
    showHelp(argv[0]);
    return 0;
  }

  // Parse file extension.
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".bin");

  // Check valid extension.
  if (filenames.size() != 1) {
    showHelp(argv[0]);
    return -1;
  }

  // Open file.
  std::fstream file_;
  file_.open(argv[filenames[0]], std::ios::in | std::ios::binary);
  if (!file_.is_open()) {
    std::cout << "Can't open the input file." << std::endl;
    exit(1);
  } else {
    std::cout << "File " << argv[filenames[0]] << " opened succesfully!"
              << std::endl;
  }
  // Import data to point cloud.
  pcl::PointCloud<pcl::PointXYZ> points;
  std::size_t file_size_ = std::filesystem::file_size(argv[filenames[0]]);

  // Calculate cloud size (x,y,z per point).
  std::size_t cloud_size = file_size_ / 3;
  points.resize(cloud_size);

  // Read x,y,z points coordinates from input file.
  for (size_t i = 0; i < cloud_size; i++) {
    file_.read(reinterpret_cast<char *>(&points[i].x), sizeof(float));
    file_.read(reinterpret_cast<char *>(&points[i].y), sizeof(float));
    file_.read(reinterpret_cast<char *>(&points[i].z), sizeof(float));
  }

  // Calculate a centroid of cloud.
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(points, centroid);
  std::cout << "Find centroid:" << centroid[0] << " " << centroid[1] << " "
            << centroid[2] << " " << std::endl
            << "Transform points  cloud..." << std::endl;

  // Transform points cloud to match the center of coordinates to the calculated
  // centroid.
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Move (translation) the cloud on value -x,-y,-z of the calculated centroid.
  transform.translation() << -centroid[0], -centroid[1], -centroid[2];
  pcl::transformPointCloud(points, points, transform);

  // Calcule a new centroid to check a transformation of the cloud.
  pcl::compute3DCentroid(points, centroid);
  std::cout << "New centroid:" << centroid[0] << " " << centroid[1] << " "
            << centroid[2] << " " << std::endl;

  // Run PCLVisualizer window.
  pcl::visualization::PCLVisualizer viewer("PCL cloud to image export");
  viewer.setBackgroundColor(0, 0, 0);

  // Add our cloud to the current view.
  viewer.addPointCloud<pcl::PointXYZ>(points.makeShared(), "sample cloud");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  // Set the camera parameters.
  viewer.initCameraParameters();
  viewer.setCameraPosition(-1.01337, 15.255, -19.4681, 0.0255838, 0.802148,
                           0.596578, 0, 0, 1);
  viewer.setCameraFieldOfView(0.8575);
  viewer.setCameraClipDistances(9.27686, 50.4547);
  viewer.setSize(800, 480);
  viewer.setShowFPS(false);

  std::cout << "Press `s` to export view." << std::endl;

  // Registering keyboard press behavior.
  viewer.registerKeyboardCallback(keyboardEventOccurred, &viewer);

  // Main PCL Viewer loop.
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }
  return (0);
}