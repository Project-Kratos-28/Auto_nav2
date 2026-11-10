// Headless GLIM dump -> PCD exporter.
//
// The installed ros-humble-glim-ros (1.2.2) offline_viewer does not have the
// --export_path flag that exists in newer/master GLIM, so there is no built-in
// non-interactive way to export a saved dump. This talks to the same GlobalMapping
// class offline_viewer uses internally (load a dump, export_points()) without any
// GUI, and writes straight to PCD instead of PLY, skipping pcl_ply2pcd entirely.
#include <iostream>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <glim/util/config.hpp>
#include <glim/mapping/global_mapping.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: glim_dump_export <dump_path> <output.pcd> <glim_config_path>" << std::endl;
    std::cerr << "  dump_path        directory saved by Ctrl+C on glim_rosnode (e.g. maps/<timestamp>/)" << std::endl;
    std::cerr << "  output.pcd       where to write the exported point cloud" << std::endl;
    std::cerr << "  glim_config_path the same glim_config/ directory passed to glim_rosnode" << std::endl;
    return 1;
  }

  const std::string dump_path = argv[1];
  const std::string output_path = argv[2];
  const std::string config_path = argv[3];

  auto logger = spdlog::stdout_color_mt("glim_dump_export");
  spdlog::set_default_logger(logger);

  glim::GlobalConfig::instance(config_path);

  // Mirror offline_viewer's export-mode settings: don't re-run optimization,
  // just use the dump's already-optimized (already loop-closed, if any fired
  // during the live run) poses as-is.
  glim::GlobalMappingParams params;
  params.isam2_relinearize_skip = 1;
  params.isam2_relinearize_thresh = 0.0;
  params.enable_optimization = false;

  glim::GlobalMapping mapping(params);
  if (!mapping.load(dump_path)) {
    std::cerr << "failed to load dump: " << dump_path << std::endl;
    return 1;
  }

  auto points = mapping.export_points();
  if (!points || !points->has_points()) {
    std::cerr << "no points exported from dump - is the path right?" << std::endl;
    return 1;
  }

  const bool has_intensity = points->has_intensities();
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(points->size());
  for (size_t i = 0; i < points->size(); ++i) {
    pcl::PointXYZI pt;
    pt.x = static_cast<float>(points->points[i].x());
    pt.y = static_cast<float>(points->points[i].y());
    pt.z = static_cast<float>(points->points[i].z());
    pt.intensity = has_intensity ? static_cast<float>(points->intensities[i]) : 0.0f;
    cloud.push_back(pt);
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  cloud.is_dense = false;

  if (pcl::io::savePCDFileBinary(output_path, cloud) != 0) {
    std::cerr << "failed to write " << output_path << std::endl;
    return 1;
  }

  std::cout << "wrote " << cloud.size() << " points to " << output_path << std::endl;
  return 0;
}
