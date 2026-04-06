#ifndef ODOM_POINTLIO_WHEEL_ODOM_FUSION__H__
#define ODOM_POINTLIO_WHEEL_ODOM_FUSION__H__

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <mutex>
#include <deque>
#include <cmath>

using Odometry = nav_msgs::msg::Odometry;
using Path     = nav_msgs::msg::Path;

// ════════════════════════════════════════════════════════════════
//  四元数 / SO(3) 工具函数
// ════════════════════════════════════════════════════════════════

// 旋转向量 φ → 四元数  δq = exp(φ/2)
inline Eigen::Quaterniond expmap(const Eigen::Vector3d &phi)
{
  double angle = phi.norm();
  if (angle < 1e-10)
    return Eigen::Quaterniond(1.0, 0.5*phi.x(), 0.5*phi.y(), 0.5*phi.z()).normalized();
  Eigen::Vector3d axis = phi / angle;
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

// 四元数 → 旋转向量  φ = log(q)*2
inline Eigen::Vector3d logmap(const Eigen::Quaterniond &q)
{
  Eigen::AngleAxisd aa(q.normalized());
  return aa.axis() * aa.angle();
}

// 向量的反对称矩阵（叉积矩阵）
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d S;
  S <<    0, -v.z(),  v.y(),
       v.z(),     0, -v.x(),
      -v.y(),  v.x(),     0;
  return S;
}

// ════════════════════════════════════════════════════════════════
//  ESKF 误差状态维度：NE = 9  ( δp δv δθ 各 3 维 )
// ════════════════════════════════════════════════════════════════
static constexpr int NE = 9;

class OdomFusionNode : public rclcpp::Node
{
public:
  OdomFusionNode();
private:

  // ── 参数声明 & 加载 ───────────────────────────────────────────
  void declare_params();

  void load_params();

  void init_filter();
  

  // ════════════════════════════════════════════════════════════════
  //  ESKF 预测步（匀速运动模型，LIO 帧驱动）
  //
  //  名义状态积分：
  //    p ← p + v·dt
  //    v ← v  （无外力输入）
  //    q ← q  （无角速度输入；下一 LIO 帧修正）
  //
  //  误差协方差传播：
  //    F = I + Fc·dt，Fc 中只有 ∂δp/∂δv = I 非零
  //    P ← F·P·Fᵀ + Q·dt
  // ════════════════════════════════════════════════════════════════
  void eskf_predict(double dt);
  

  // ════════════════════════════════════════════════════════════════
  //  ESKF 通用更新步
  //
  //  innov : 残差 z - h(X_nom)，已在误差空间线性化
  //  H     : 误差状态观测矩阵 ∂h/∂δx  (m × NE)
  //  R     : 观测噪声协方差           (m × m)
  //
  //  更新后将 δx 注入名义状态，协方差用 Joseph 形式保证正定
  // ════════════════════════════════════════════════════════════════
  void eskf_update(const Eigen::VectorXd &innov,
                   const Eigen::MatrixXd &H,
                   const Eigen::MatrixXd &R);
  

  // ════════════════════════════════════════════════════════════════
  //  Point-LIO 回调
  //
  //  观测：位置(3) + 速度(3) + 姿态残差 δθ(3) = 9 维
  //  姿态残差：δθ = log( q_nom⁻¹ ⊗ q_lio )，在切空间线性化
  // ════════════════════════════════════════════════════════════════
  void cb_lio(const Odometry::SharedPtr msg);

  // ════════════════════════════════════════════════════════════════
  //  轮式里程计回调
  //
  //  观测：世界系 vx、vy（2 维），z/roll/pitch 不参与
  //
  //  观测函数：h(X) = [I₂ | 0] · R(q) · R_ext · v_wheel_body
  //  线性化（对误差状态）：
  //    ∂h/∂δv  = I₂  （世界系速度分量）
  //    ∂h/∂δθ  = [-R·v_body]× 的前两行  （姿态-速度耦合）
  // ════════════════════════════════════════════════════════════════
  void cb_wheel(const Odometry::SharedPtr msg);
  

  // ════════════════════════════════════════════════════════════════
  //  发布融合结果
  // ════════════════════════════════════════════════════════════════
  void publish(const rclcpp::Time &stamp);

  // ── 成员变量 ─────────────────────────────────────────────────
  std::mutex mtx_;
  bool   initialized_;
  double last_t_{0.0};

  // 名义状态（流形上）
  Eigen::Vector3d    p_nom_;
  Eigen::Vector3d    v_nom_;
  Eigen::Quaterniond q_nom_;

  // 误差状态协方差  (9×9)
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;

  // 参数
  std::string lio_topic_, wheel_topic_, frame_id_, child_frame_;
  double q_pos_, q_vel_, q_att_;
  double r_lio_pos_, r_lio_vel_, r_lio_att_;
  double r_wheel_vx_, r_wheel_vy_;
  double wheel_dyaw_;

  // ROS 接口
  rclcpp::Subscription<Odometry>::SharedPtr sub_lio_, sub_wheel_;
  rclcpp::Publisher<Odometry>::SharedPtr    pub_fused_;
  rclcpp::Publisher<Path>::SharedPtr        pub_path_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  nav_msgs::msg::Path path_;
};















#endif