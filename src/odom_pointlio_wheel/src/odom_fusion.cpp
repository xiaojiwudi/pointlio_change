/**
 * odom_fusion_node.cpp  —  ESKF 版
 *
 * 融合 Point-LIO 激光里程计 × 轮式里程计
 * 方法：误差状态卡尔曼滤波（ESKF）
 *        姿态用四元数表示，无奇异，与 Point-LIO 内部风格一致
 *
 * ── 状态定义 ────────────────────────────────────────────────────
 *   名义状态  X  = { p(3), v(3), q(4) }   四元数 q = [w, x, y, z]
 *   误差状态 δx  = { δp(3), δv(3), δθ(3) }  共 9 维
 *              δθ 是旋转向量（李代数），更新后通过 q ⊗ exp(δθ/2) 注入
 *
 * ── 话题 ────────────────────────────────────────────────────────
 *   订阅  /Odometry         nav_msgs/Odometry   Point-LIO 输出
 *   订阅  /wheel_odom       nav_msgs/Odometry   轮式里程计
 *   发布  /odom_fused       nav_msgs/Odometry   融合里程计
 *   发布  /odom_fused_path  nav_msgs/Path       可视化轨迹
 *
 * ── 依赖 ────────────────────────────────────────────────────────
 *   ROS 2 Humble+，Eigen3，tf2
 */
#include "odom_pointlio_wheel/odom_fusion.h"


OdomFusionNode::OdomFusionNode() : Node("odom_fusion_node"), initialized_(false)
  {
    declare_params();
    load_params();
    init_filter();

    sub_lio_ = create_subscription<Odometry>(
      lio_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&OdomFusionNode::cb_lio, this, std::placeholders::_1));

    sub_wheel_ = create_subscription<Odometry>(
      wheel_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&OdomFusionNode::cb_wheel, this, std::placeholders::_1));

    pub_fused_ = create_publisher<Odometry>("/odom_fused", 10);
    pub_path_  = create_publisher<Path>("/odom_fused_path", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(),
      "[ESKF Fusion] 启动\n  LIO  : %s\n  Wheel: %s",
      lio_topic_.c_str(), wheel_topic_.c_str());
  }

// ── 参数声明 & 加载 ───────────────────────────────────────────
  void OdomFusionNode::declare_params()
  {
    declare_parameter("lio_topic",   "/Odometry");
    declare_parameter("wheel_topic", "/wheel_odom");
    declare_parameter("frame_id",    "odom");
    declare_parameter("child_frame", "base_link");

    declare_parameter("q_pos", 0.01);   // 位置过程噪声谱密度
    declare_parameter("q_vel", 0.10);   // 速度过程噪声谱密度
    declare_parameter("q_att", 0.005);  // 姿态过程噪声谱密度

    declare_parameter("r_lio_pos", 0.05);  // LIO 位置观测噪声 [m²]
    declare_parameter("r_lio_vel", 0.10);  // LIO 速度观测噪声 [(m/s)²]
    declare_parameter("r_lio_att", 0.02);  // LIO 姿态观测噪声 [rad²]

    declare_parameter("r_wheel_vx", 0.05); // 轮式前向速度噪声 [(m/s)²]
    declare_parameter("r_wheel_vy", 0.50); // 轮式侧向速度噪声（侧滑不可信）

    declare_parameter("wheel_offset_yaw", 0.0); // 轮式相对 LIO body 系的 yaw 偏转 [rad]
  }

  void OdomFusionNode::load_params()
  {
    lio_topic_   = get_parameter("lio_topic").as_string();
    wheel_topic_ = get_parameter("wheel_topic").as_string();
    frame_id_    = get_parameter("frame_id").as_string();
    child_frame_ = get_parameter("child_frame").as_string();

    q_pos_ = get_parameter("q_pos").as_double();
    q_vel_ = get_parameter("q_vel").as_double();
    q_att_ = get_parameter("q_att").as_double();

    r_lio_pos_ = get_parameter("r_lio_pos").as_double();
    r_lio_vel_ = get_parameter("r_lio_vel").as_double();
    r_lio_att_ = get_parameter("r_lio_att").as_double();

    r_wheel_vx_  = get_parameter("r_wheel_vx").as_double();
    r_wheel_vy_  = get_parameter("r_wheel_vy").as_double();
    wheel_dyaw_  = get_parameter("wheel_offset_yaw").as_double();
  }

  void OdomFusionNode::init_filter()
  {
    p_nom_ = Eigen::Vector3d::Zero();
    v_nom_ = Eigen::Vector3d::Zero();
    q_nom_ = Eigen::Quaterniond::Identity();

    P_ = Eigen::MatrixXd::Identity(NE, NE) * 1.0;

    Q_ = Eigen::MatrixXd::Zero(NE, NE);
    Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * q_pos_;
    Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * q_vel_;
    Q_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * q_att_;
  }

  void OdomFusionNode::eskf_predict(double dt)
  {
    if (dt <= 0.0 || dt > 1.0) return;

    // 名义状态
    p_nom_ += v_nom_ * dt;

    // 误差协方差
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(NE, NE);
    F.block<3,3>(0, 3) = Eigen::Matrix3d::Identity() * dt; // ∂δp/∂δv·dt

    P_ = F * P_ * F.transpose() + Q_ * dt;
  }

  void OdomFusionNode::eskf_update(const Eigen::VectorXd &innov,
                   const Eigen::MatrixXd &H,
                   const Eigen::MatrixXd &R)
  {
    // 卡尔曼增益
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // 最优误差状态估计
    Eigen::VectorXd dx = K * innov;

    // ── 将误差注入名义状态 ────────────────────────────────────
    p_nom_ += dx.segment<3>(0);
    v_nom_ += dx.segment<3>(3);

    // 姿态注入（李群上的加法）：q ← q ⊗ exp(δθ)
    // 注意：expmap 内部已处理 φ/2，此处直接传 δθ
    Eigen::Vector3d dtheta = dx.segment<3>(6);
    q_nom_ = (q_nom_ * expmap(dtheta)).normalized();

    // ── 协方差更新（Joseph 形式，数值稳定）────────────────────
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(NE, NE) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();

    // ── ESKF 重置（误差状态回零，协方差修正）────────────────
    // 对于小角度：重置 Jacobian G ≈ I，可省略
    // 严格实现时：P ← G·P·Gᵀ，G = I - [δθ/2]×  对于 δθ 子块
    // 这里省略（Point-LIO 原始代码也在小角度下省略）
    // 重置 Jacobian
    //Eigen::MatrixXd G = Eigen::MatrixXd::Identity(NE, NE);
    //Eigen::Vector3d dtheta = dx.segment<3>(6);
    //G.block<3,3>(6,6) = Eigen::Matrix3d::Identity() - 0.5 * skew(dtheta);

    // 修正协方差
    //P_ = G * P_ * G.transpose();
  }

  void OdomFusionNode::cb_lio(const Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    double t = rclcpp::Time(msg->header.stamp).seconds();

    const auto &qm = msg->pose.pose.orientation;
    Eigen::Quaterniond q_lio(qm.w, qm.x, qm.y, qm.z);
    q_lio.normalize();

    if (!initialized_) {
      p_nom_ = { msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 msg->pose.pose.position.z };
      v_nom_ = { msg->twist.twist.linear.x,
                 msg->twist.twist.linear.y,
                 msg->twist.twist.linear.z };
      q_nom_ = q_lio;
      last_t_ = t;
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "[ESKF] 已初始化，t=%.3f  p=(%.2f %.2f %.2f)",
        t, p_nom_.x(), p_nom_.y(), p_nom_.z());
      return;
    }

    double dt = t - last_t_;
    last_t_ = t;
    eskf_predict(dt);

    // ── 残差构造 ──────────────────────────────────────────────
    Eigen::Vector3d p_obs(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
    Eigen::Vector3d v_obs(msg->twist.twist.linear.x,
                          msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z);

    // 姿态残差在切空间：δθ = log( q_nom⁻¹ ⊗ q_lio )
    Eigen::Quaterniond dq = q_nom_.conjugate() * q_lio;
    dq.normalize();
    if (dq.w() < 0.0) dq.coeffs() = -dq.coeffs();  // 取短弧
    Eigen::Vector3d att_innov = logmap(dq);

    Eigen::VectorXd innov(9);
    innov << p_obs - p_nom_,
             v_obs - v_nom_,
             att_innov;

    // ── H (9×9)：各子块对误差状态是 Identity ─────────────────
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(9, NE);

    // ── R：优先使用 LIO 自报协方差 ───────────────────────────
    Eigen::MatrixXd R_lio = Eigen::MatrixXd::Zero(9, 9);
    double pp = msg->pose.covariance[0];
    double rp = (pp > 1e-9 && pp < 1e2) ? pp : r_lio_pos_;
    R_lio.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * rp;
    R_lio.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * r_lio_vel_;
    R_lio.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * r_lio_att_;

    eskf_update(innov, H, R_lio);
    publish(msg->header.stamp);
  }

  void OdomFusionNode::cb_wheel(const Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!initialized_) return;

    // 外参旋转：轮式 body → LIO body（绕 z 轴）
    double cy = std::cos(wheel_dyaw_), sy = std::sin(wheel_dyaw_);
    double vx_b =  cy * msg->twist.twist.linear.x
                 - sy * msg->twist.twist.linear.y;
    double vy_b =  sy * msg->twist.twist.linear.x
                 + cy * msg->twist.twist.linear.y;
    Eigen::Vector3d v_body(vx_b, vy_b, 0.0);

    // body → 世界系
    Eigen::Matrix3d Rw = q_nom_.normalized().toRotationMatrix();
    Eigen::Vector3d v_world = Rw * v_body;

    // 残差（仅 xy）
    Eigen::VectorXd innov(2);
    innov << v_world.x() - v_nom_.x(),
             v_world.y() - v_nom_.y();

    // H (2×9)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, NE);
    H(0, 3) = 1.0;  // ∂vx_world/∂δvx
    H(1, 4) = 1.0;  // ∂vy_world/∂δvy
    // 姿态耦合：∂(R·v)/∂δθ = -[R·v]×  取前两行
    Eigen::Matrix3d Jatt = -skew(Rw * v_body);
    H.block<2,3>(0, 6) = Jatt.topRows<2>();

    // R (2×2)
    Eigen::MatrixXd R_wh = Eigen::MatrixXd::Zero(2, 2);
    R_wh(0, 0) = r_wheel_vx_;
    R_wh(1, 1) = r_wheel_vy_;

    eskf_update(innov, H, R_wh);

    // 轮速帧到来时也广播 TF，提升频率至轮速频率（50~200 Hz）
    // 时间戳用 now()，因为轮式消息没有 LIO 的精确硬件时间戳
    publish(now());
  }

  void OdomFusionNode::publish(const rclcpp::Time &stamp)
  {
    Odometry out;
    out.header.stamp    = stamp;
    out.header.frame_id = frame_id_;
    out.child_frame_id  = child_frame_;

    out.pose.pose.position.x = p_nom_.x();
    out.pose.pose.position.y = p_nom_.y();
    out.pose.pose.position.z = p_nom_.z();

    // 四元数直接输出（无欧拉角转换，无奇异）
    Eigen::Quaterniond qn = q_nom_.normalized();
    out.pose.pose.orientation.w = qn.w();
    out.pose.pose.orientation.x = qn.x();
    out.pose.pose.orientation.y = qn.y();
    out.pose.pose.orientation.z = qn.z();

    out.twist.twist.linear.x = v_nom_.x();
    out.twist.twist.linear.y = v_nom_.y();
    out.twist.twist.linear.z = v_nom_.z();

    // 协方差（pose: px py pz rx ry rz；twist: vx vy vz）
    std::fill(out.pose.covariance.begin(),  out.pose.covariance.end(),  0.0);
    std::fill(out.twist.covariance.begin(), out.twist.covariance.end(), 0.0);
    out.pose.covariance[0]  = P_(0, 0);   // px
    out.pose.covariance[7]  = P_(1, 1);   // py
    out.pose.covariance[14] = P_(2, 2);   // pz
    out.pose.covariance[21] = P_(6, 6);   // roll
    out.pose.covariance[28] = P_(7, 7);   // pitch
    out.pose.covariance[35] = P_(8, 8);   // yaw
    out.twist.covariance[0] = P_(3, 3);   // vx
    out.twist.covariance[7] = P_(4, 4);   // vy
    out.twist.covariance[14]= P_(5, 5);   // vz

    pub_fused_->publish(out);

    // 路径（rviz 可视化）
    geometry_msgs::msg::PoseStamped ps;
    ps.header = out.header;
    ps.pose   = out.pose.pose;
    path_.header = out.header;
    path_.poses.push_back(ps);
    if (path_.poses.size() > 5000) path_.poses.erase(path_.poses.begin());
    pub_path_->publish(path_);

    // TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header         = out.header;
    tf_msg.child_frame_id = child_frame_;
    tf_msg.transform.translation.x = p_nom_.x();
    tf_msg.transform.translation.y = p_nom_.y();
    tf_msg.transform.translation.z = p_nom_.z();
    tf_msg.transform.rotation = out.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }
