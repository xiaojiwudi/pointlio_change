#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>            // [FIX] 补全头文件
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>                     // [FIX] 补全 PCD 保存头文件
#include <memory>
#include <chrono>
#include <filesystem>                           // [FIX] 补全 filesystem 头文件

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class PointLIOLegoProcessor : public rclcpp::Node
{
public:
    PointLIOLegoProcessor();

private:
    // ------------------------------------------------------------------
    //  回调：里程计
    // ------------------------------------------------------------------
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ------------------------------------------------------------------
    //  回调：点云主处理流程
    // ------------------------------------------------------------------
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // ------------------------------------------------------------------
    //  RANSAC 地面分割
    // ------------------------------------------------------------------
    void segmentGround(CloudT::Ptr input, CloudT::Ptr ground, CloudT::Ptr non_ground);

    // ------------------------------------------------------------------
    //  体素下采样
    // ------------------------------------------------------------------
    void voxelDownsample(CloudT::Ptr input, CloudT::Ptr output, double leaf);
    // ------------------------------------------------------------------
    //  特征提取（边缘 + 平面）
    // ------------------------------------------------------------------
    void extractFeatures(CloudT::Ptr input, CloudT::Ptr edge, CloudT::Ptr planar);

    // ------------------------------------------------------------------
    //  地面边界提取
    // ------------------------------------------------------------------
    void extractGroundEdge(CloudT::Ptr ground, CloudT::Ptr edge);

    // ------------------------------------------------------------------
    //  累积全局地图（TF 变换到 map 系）
    // ------------------------------------------------------------------
    void accumulateMap(CloudT::Ptr cloud, rclcpp::Time stamp);

    // ------------------------------------------------------------------
    //  [FIX] publishOnce — 修正参数签名（4 个参数，去掉多余的 planar_features）
    // ------------------------------------------------------------------
    void publishOnce(CloudT::Ptr ground_down, CloudT::Ptr edge_features,
                     CloudT::Ptr ground_edge, rclcpp::Time stamp);

    // ------------------------------------------------------------------
    //  定时器：发布全局累积地图
    // ------------------------------------------------------------------
    void publishMapTimer();

    // ------------------------------------------------------------------
    //  [NEW] saveMapCallback — 响应 /save_map 服务，将全局地图保存为 PCD
    // ------------------------------------------------------------------
    void saveMapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>  /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response>        res);

    // ------------------------------------------------------------------
    //  成员变量
    // ------------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_edge_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // [FIX] 补全服务声明
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;

    std::unique_ptr<tf2_ros::Buffer>              tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;

    nav_msgs::msg::Odometry::SharedPtr last_odom_;

    CloudT::Ptr  global_map_ = std::make_shared<CloudT>();
    std::mutex   map_mutex_;                   // [NEW] 多线程保护全局地图

    double      voxel_leaf_, curv_th_, planar_th_, ransac_th_;
    std::string map_save_dir_, map_save_prefix_;
};