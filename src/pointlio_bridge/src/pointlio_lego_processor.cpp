#include "pointlio_bridge/pointlio_lego_processor.h"

PointLIOLegoProcessor::PointLIOLegoProcessor() : Node("pointlio_lego_processor")
{
    // ====================== LeGO-LOAM 完全一致的参数 ======================
    this->declare_parameter<double>("voxel_leaf_size_ground", 0.3);
    this->declare_parameter<double>("curvature_threshold", 0.1);
    this->declare_parameter<double>("planar_threshold", 0.05);
    this->declare_parameter<double>("ransac_distance_threshold", 0.06);
    this->declare_parameter<double>("map_publish_period", 1.0);

    // ====================== 地图保存参数 ======================
    this->declare_parameter<std::string>("map_save_dir", "/tmp/lego_maps");
    this->declare_parameter<std::string>("map_save_prefix", "lego_map_mid360");

    voxel_leaf_ = this->get_parameter("voxel_leaf_size_ground").as_double();
    curv_th_ = this->get_parameter("curvature_threshold").as_double();
    planar_th_ = this->get_parameter("planar_threshold").as_double();
    ransac_th_ = this->get_parameter("ransac_distance_threshold").as_double();
    map_save_dir_ = this->get_parameter("map_save_dir").as_string();
    map_save_prefix_ = this->get_parameter("map_save_prefix").as_string();

    // 创建保存目录
    std::filesystem::create_directories(map_save_dir_);

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅
    auto qos = rclcpp::QoS(10).best_effort();
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_body", qos,
        std::bind(&PointLIOLegoProcessor::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_corrected", qos,
        std::bind(&PointLIOLegoProcessor::odomCallback, this, std::placeholders::_1));

    // 发布话题（与 lego_loam 完全一致）
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lego_loam_map", 10);
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lego_loam_ground", 10);
    ground_edge_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lego_loam_ground_edge", 10);

    // [FIX] 保存地图服务（补全实现）
    save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "save_map",
        std::bind(&PointLIOLegoProcessor::saveMapCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 定时器：周期性发布全局累积地图
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(this->get_parameter("map_publish_period").as_double()),
        std::bind(&PointLIOLegoProcessor::publishMapTimer, this));

    RCLCPP_INFO(this->get_logger(),
                "PointLIO → LeGO-LOAM 处理器启动完成 (Mid-360 + 地图保存功能已开启)");
    RCLCPP_INFO(this->get_logger(),
                "保存服务：/save_map    保存路径：%s", map_save_dir_.c_str());
}

void PointLIOLegoProcessor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    last_odom_ = msg;
}

void PointLIOLegoProcessor::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!last_odom_)
        return;

    // ROS → PCL
    CloudT::Ptr input_cloud(new CloudT);
    pcl::fromROSMsg(*msg, *input_cloud);

    // 1. RANSAC 地面分割
    CloudT::Ptr ground(new CloudT), non_ground(new CloudT);
    segmentGround(input_cloud, ground, non_ground);

    // 2. 地面下采样（0.3m，和 LeGO-LOAM 完全一致）
    CloudT::Ptr ground_down(new CloudT);
    voxelDownsample(ground, ground_down, voxel_leaf_);

    // 3. 特征提取（边缘 + 平面）
    CloudT::Ptr edge_features(new CloudT), planar_features(new CloudT);
    extractFeatures(non_ground, edge_features, planar_features);

    // 4. 地面边界
    CloudT::Ptr ground_edge(new CloudT);
    extractGroundEdge(ground_down, ground_edge);

    // 5. 累积全局地图（变换到 map 系）
    accumulateMap(input_cloud, msg->header.stamp);

    // 6. [FIX] publishOnce 修正为 4 个参数（去掉多余的 planar_features）
    publishOnce(ground_down, edge_features, ground_edge, msg->header.stamp);
}

void PointLIOLegoProcessor::segmentGround(CloudT::Ptr input, CloudT::Ptr ground, CloudT::Ptr non_ground)
{
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_th_);
    seg.setInputCloud(input);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*ground);
    extract.setNegative(true);
    extract.filter(*non_ground);
}

void PointLIOLegoProcessor::voxelDownsample(CloudT::Ptr input, CloudT::Ptr output, double leaf)
{
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*output);
}

void PointLIOLegoProcessor::extractFeatures(CloudT::Ptr input, CloudT::Ptr edge, CloudT::Ptr planar)
{
    if (input->empty())
        return;

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(input);
    ne.setRadiusSearch(0.5);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    for (size_t i = 0; i < input->size(); ++i)
    {
        double curv = std::abs(normals->points[i].normal_z);
        if (curv < curv_th_)
            edge->push_back(input->points[i]);
        else if (curv > planar_th_)
            planar->push_back(input->points[i]);
    }
}

void PointLIOLegoProcessor::extractGroundEdge(CloudT::Ptr ground, CloudT::Ptr edge)
{
    if (ground->size() < 10)
        return;

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(ground);
    ne.setRadiusSearch(0.4);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    for (size_t i = 0; i < ground->size(); ++i)
    {
        if (std::abs(normals->points[i].normal_z) < 0.85)
            edge->push_back(ground->points[i]);
    }
}

void PointLIOLegoProcessor::accumulateMap(CloudT::Ptr cloud, rclcpp::Time stamp)
{
    geometry_msgs::msg::TransformStamped trans;
    try
    {
        trans = tf_buffer_->lookupTransform(
            "map", "base_link", stamp, rclcpp::Duration::from_seconds(0.1));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "TF lookup failed: %s", ex.what());
        return;
    }

    // 构造变换矩阵
    Eigen::Matrix4f tf_mat = Eigen::Matrix4f::Identity();
    tf_mat.block<3, 3>(0, 0) =
        Eigen::Quaternionf(
            trans.transform.rotation.w,
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z)
            .toRotationMatrix();
    tf_mat.block<3, 1>(0, 3) << trans.transform.translation.x,
        trans.transform.translation.y,
        trans.transform.translation.z;

    CloudT::Ptr transformed(new CloudT);
    pcl::transformPointCloud(*cloud, *transformed, tf_mat);

    std::lock_guard<std::mutex> lock(map_mutex_);
    *global_map_ += *transformed;

    // 防止内存爆炸：超过 80k 点时做一次 0.2m 下采样
    if (global_map_->size() > 80000)
    {
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(global_map_);
        vg.setLeafSize(0.2f, 0.2f, 0.2f);
        CloudT::Ptr tmp(new CloudT);
        vg.filter(*tmp);
        global_map_ = tmp;
    }
}

void PointLIOLegoProcessor::publishOnce(CloudT::Ptr ground_down, CloudT::Ptr edge_features,
                                        CloudT::Ptr ground_edge, rclcpp::Time stamp)
{
    auto publish = [&](auto &pub, CloudT::Ptr cloud)
    {
        if (cloud->empty())
            return;
        sensor_msgs::msg::PointCloud2 ros_msg;
        pcl::toROSMsg(*cloud, ros_msg);
        ros_msg.header.frame_id = "map";
        ros_msg.header.stamp = stamp;
        pub->publish(ros_msg);
    };

    publish(ground_pub_, ground_down);
    publish(ground_edge_pub_, ground_edge);
    publish(map_pub_, edge_features); // 实时特征，定时器里发全局累积地图
}

void PointLIOLegoProcessor::publishMapTimer()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (global_map_->empty())
        return;

    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(*global_map_, ros_msg);
    ros_msg.header.frame_id = "map";
    ros_msg.header.stamp = this->now();
    map_pub_->publish(ros_msg);
}

void PointLIOLegoProcessor::saveMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (global_map_->empty())
    {
        res->success = false;
        res->message = "全局地图为空，跳过保存。";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return; 
    }

    // 用当前时间戳生成唯一文件名
    auto now = std::chrono::system_clock::now();
    auto epoch_sec = std::chrono::duration_cast<std::chrono::seconds>(
                         now.time_since_epoch())
                         .count();
    std::string filename = map_save_dir_ + "/" +
                           map_save_prefix_ + "_" +
                           std::to_string(epoch_sec) + ".pcd";

    // 保存为二进制 PCD（压缩更小，读写更快）
    if (pcl::io::savePCDFileBinary(filename, *global_map_) == 0)
    {
        res->success = true;
        res->message = "地图已保存：" + filename +
                       "  点数：" + std::to_string(global_map_->size());
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
    }
    else
    {
        res->success = false;
        res->message = "PCD 写入失败，路径：" + filename;
        RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
    }
}