// ROS
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
// ROS package
#include <pointcloud_to_grid/pointcloud_to_grid_core.hpp>
// c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream> // 用于保存地图

auto height_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
auto intensity_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
auto height_gridmap = std::make_shared<grid_map_msgs::msg::GridMap>();
auto intensity_gridmap = std::make_shared<grid_map_msgs::msg::GridMap>();

// 全局累加地图指针
auto acc_height_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
auto acc_intensity_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();

class PointCloudToGrid : public rclcpp::Node
{
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      if (param.get_name() == "mapi_topic_name") { grid_map.mapi_topic_name = param.as_string(); }
      if (param.get_name() == "maph_topic_name") { grid_map.maph_topic_name = param.as_string(); }
      if (param.get_name() == "cloud_in_topic")
      {
        cloud_in_topic = param.as_string();
        auto sensor_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in_topic, sensor_qos, 
            std::bind(&PointCloudToGrid::lidar_callback, this, std::placeholders::_1));
      }
      if (param.get_name() == "cell_size") { grid_map.cell_size = param.as_double(); }
      if (param.get_name() == "position_x") { grid_map.position_x = param.as_double(); }
      if (param.get_name() == "position_y") { grid_map.position_y = param.as_double(); }
      if (param.get_name() == "length_x") { grid_map.length_x = param.as_double(); }
      if (param.get_name() == "length_y") { grid_map.length_y = param.as_double(); }
      if (param.get_name() == "obstacle_min_z") { obstacle_min_z = param.as_double(); }
      if (param.get_name() == "obstacle_max_z") { obstacle_max_z = param.as_double(); }
      grid_map.paramRefresh();
    }
    return result;
  }

public:
  PointCloudToGrid() : Node("pointcloud_to_grid_node"), count_(0)
  {
    this->declare_parameter<std::string>("mapi_topic_name", "intensity_grid");
    this->declare_parameter<std::string>("maph_topic_name", "height_grid");
    this->declare_parameter<std::string>("mapi_gridmap_topic_name", "intensity_gridmap");
    this->declare_parameter<std::string>("maph_gridmap_topic_name", "height_gridmap");
    this->declare_parameter<std::string>("acc_mapi_topic_name", "acc_intensity_grid");
    this->declare_parameter<std::string>("acc_maph_topic_name", "acc_height_grid");
    this->declare_parameter<std::string>("cloud_in_topic", cloud_in_topic);
    
    this->declare_parameter<float>("cell_size", 0.5);
    this->declare_parameter<float>("position_x", 0.0);
    this->declare_parameter<float>("position_y", 0.0);
    this->declare_parameter<float>("length_x", 20.0);
    this->declare_parameter<float>("length_y", 30.0);
    
    this->declare_parameter<double>("obstacle_min_z", 0.15); // Z > 0.15m 为障碍物
    this->declare_parameter<double>("obstacle_max_z", 2.0);  // Z < 2.0m 为障碍物

    this->get_parameter("mapi_topic_name", grid_map.mapi_topic_name);
    this->get_parameter("maph_topic_name", grid_map.maph_topic_name);
    this->get_parameter("mapi_gridmap_topic_name", grid_map.mapi_gridmap_topic_name);
    this->get_parameter("maph_gridmap_topic_name", grid_map.maph_gridmap_topic_name);
    this->get_parameter("acc_mapi_topic_name", acc_mapi_topic_name_);
    this->get_parameter("acc_maph_topic_name", acc_maph_topic_name_);
    this->get_parameter("cloud_in_topic", cloud_in_topic);
    this->get_parameter("cell_size", grid_map.cell_size);
    this->get_parameter("position_x", grid_map.position_x);
    this->get_parameter("position_y", grid_map.position_y);
    this->get_parameter("length_x", grid_map.length_x);
    this->get_parameter("length_y", grid_map.length_y);
    this->get_parameter("obstacle_min_z", obstacle_min_z);
    this->get_parameter("obstacle_max_z", obstacle_max_z);

    grid_map.paramRefresh();

    pub_igrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map.mapi_topic_name, 10);
    pub_hgrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map.maph_topic_name, 10);
    pub_igridmap = this->create_publisher<grid_map_msgs::msg::GridMap>(grid_map.mapi_gridmap_topic_name, 10);
    pub_hgridmap = this->create_publisher<grid_map_msgs::msg::GridMap>(grid_map.maph_gridmap_topic_name, 10);
    
    pub_acc_igrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(acc_mapi_topic_name_, 10);
    pub_acc_hgrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(acc_maph_topic_name_, 10);

    auto sensor_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_in_topic, sensor_qos, 
        std::bind(&PointCloudToGrid::lidar_callback, this, std::placeholders::_1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PointCloudToGrid::parametersCallback, this, std::placeholders::_1));
  }

  // Ctrl+C 退出时保存二进制纯黑白地图
  ~PointCloudToGrid()
  {
    saveMap("accumulated_bw_map", acc_hpoints);
  }

private:
  void saveMap(const std::string& map_name, const std::vector<signed char>& data)
  {
    if (!is_acc_initialized) return;

    std::string pgm_file = map_name + ".pgm";
    std::string yaml_file = map_name + ".yaml";

    std::ofstream pgm(pgm_file, std::ios::out | std::ios::binary);
    if (!pgm) { return; }

    pgm << "P5\n# CREATOR: pointcloud_to_grid\n"
        << grid_map.cell_num_x << " " << grid_map.cell_num_y << "\n255\n";

    // 写入纯黑白数据 (修复：Y 轴必须倒序遍历，以匹配图像的左上角原点规范)
    for (int y = grid_map.cell_num_y - 1; y >= 0; --y) {
      for (int x = 0; x < grid_map.cell_num_x; ++x) {
        signed char val = data[y * grid_map.cell_num_x + x];
        unsigned char pixel_val;
        
        // 由于数组初始化全是 0（白色空闲），只有检测到障碍物才会是 100
        if (val == 100) {
          pixel_val = 0;         // 障碍物为绝对黑色
        } else {
          pixel_val = 254;       // 其它所有区域（未知、地面）均为白色
        }
        pgm.write(reinterpret_cast<char*>(&pixel_val), 1);
      }
    }
    pgm.close();

    std::ofstream yaml(yaml_file);
    yaml << "image: " << pgm_file << "\n";
    yaml << "resolution: " << grid_map.cell_size << "\n";
    double origin_x = grid_map.position_x - grid_map.length_x / 2.0;
    double origin_y = grid_map.position_y - grid_map.length_y / 2.0;
    yaml << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.196\n";
    yaml.close();

    RCLCPP_INFO(this->get_logger(), "Saved Pure Black/White Map to %s", pgm_file.c_str());
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *out_cloud);
    
    grid_map.initGrid(intensity_grid);
    grid_map.initGrid(height_grid);
    grid_map.initGridMap(intensity_gridmap, "intensity");
    grid_map.initGridMap(height_gridmap, "height");
    
    // 【修改核心】：使用 0（即ROS中的完全空闲/白色）初始化数组，而不是 -1（灰色/未知）
    if (!is_acc_initialized) {
      acc_hpoints.resize(grid_map.cell_num_x * grid_map.cell_num_y, 0);
      acc_ipoints.resize(grid_map.cell_num_x * grid_map.cell_num_y, 0);
      grid_map.initGrid(acc_intensity_grid);
      grid_map.initGrid(acc_height_grid);
      is_acc_initialized = true;
    }

    std::vector<signed char> hpoints(grid_map.cell_num_x * grid_map.cell_num_y, 0);
    std::vector<signed char> ipoints(grid_map.cell_num_x * grid_map.cell_num_y, 0);

    for (pcl::PointXYZI p : out_cloud->points)
    {
      if (p.x > grid_map.bottomright_x && p.x < grid_map.topleft_x)
      {
        if (p.y > grid_map.bottomright_y && p.y < grid_map.topleft_y)
        {
          PointXY cell = grid_map.getIndex(p.x, p.y);
          if (cell.x >= 0 && cell.x < grid_map.cell_num_x && cell.y >= 0 && cell.y < grid_map.cell_num_y)
          {
            int idx = cell.y * grid_map.cell_num_x + cell.x;
            
            // 只要在该格子里检测到一个属于障碍物高度的点，就永久判定为 100 (黑色/障碍物)
            // 地面点或顶棚点会因为不在区间内被忽略，从而保留为 0 (白色/空闲)
            if (p.z > obstacle_min_z && p.z < obstacle_max_z) {
              hpoints[idx] = 100;
              acc_hpoints[idx] = 100;
            }
          }
        }
      }
    }

    height_grid->header.stamp = this->now();
    height_grid->header.frame_id = input_msg->header.frame_id;
    height_grid->data = hpoints;

    acc_height_grid->header.stamp = this->now();
    acc_height_grid->header.frame_id = input_msg->header.frame_id;
    acc_height_grid->data = acc_hpoints;

    pub_hgrid->publish(*height_grid);
    pub_acc_hgrid->publish(*acc_height_grid);
  }

  std::string acc_mapi_topic_name_;
  std::string acc_maph_topic_name_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_igrid, pub_hgrid;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_acc_igrid, pub_acc_hgrid;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_igridmap, pub_hgridmap;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  
  std::string cloud_in_topic = "nonground";
  
  // 地面过滤参数，可防止地面倾斜干扰
  double obstacle_min_z = 0.15;
  double obstacle_max_z = 2.0;

  bool is_acc_initialized = false;
  std::vector<signed char> acc_hpoints;
  std::vector<signed char> acc_ipoints;

  GridMap grid_map;
  size_t count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToGrid>());
  rclcpp::shutdown();
  return 0;
}
