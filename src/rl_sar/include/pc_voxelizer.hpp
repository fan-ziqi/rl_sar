#pragma once
#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>
#include <torch/torch.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

// ------------------------------
// 公共：网格与参数（任何模式都需要）
// ------------------------------
struct VoxelGridSpec {
  float voxel = 0.10f;
  float sx = 3.2f, sy = 3.2f, sz = 3.2f;  // 默认 32*0.1 的对称盒
  float z_min = 0.0f;                     // 兼容字段，不再使用

  int Nx = 0, Ny = 0, Nz = 0;
  Eigen::Vector3f origin;

  void finalize() {
    Nx = static_cast<int>(std::ceil(sx / voxel));
    Ny = static_cast<int>(std::ceil(sy / voxel));
    Nz = static_cast<int>(std::ceil(sz / voxel));
    origin = Eigen::Vector3f(-sx * 0.5f, -sy * 0.5f, -sz * 0.5f);
  }
};

struct VoxelizerParams {
  // 话题与坐标系
  std::string front_topic = "front_points";
  std::string back_topic  = "back_points";
  std::string frame_id    = "base_link";
  std::string torso_frame = "torso_link";

  // 网格与裁切
  float voxel = 0.10f;
  float sx = 3.2f, sy = 3.2f, sz = 3.2f;
  float z_clip_min = -1e9f, z_clip_max = 1e9f;

  // 是否显式使用 python 的两台雷达位姿（不用 TF 中的传感器帧）
  bool  use_explicit_lidar_pose = false;
  Eigen::Vector3f lidar_front_pos = Eigen::Vector3f( 0.12f, 0.0f, 0.15f);
  Eigen::Vector3f lidar_back_pos  = Eigen::Vector3f(-0.10f, 0.0f, 0.15f);
  float lidar_front_yaw_deg = 0.0f;    // front 朝 +X
  float lidar_back_yaw_deg  = 180.0f;  // back  朝 -X
};

// ------------------------------
// 非 ROS2 分支（桩件，保证可编译）
// ------------------------------
#ifndef USE_ROS2

class PointcloudVoxelizer {
public:
  explicit PointcloudVoxelizer(const VoxelizerParams& p) {
    spec_.voxel = p.voxel;
    spec_.sx = p.sx; spec_.sy = p.sy; spec_.sz = p.sz; spec_.finalize();
  }
  // 对齐 python 输出维度: [1, 1, Nx, Nz, Ny]
  torch::Tensor fetchVoxelObservation() {
    if (!occ_.defined()) {
      occ_ = torch::zeros({spec_.Nx, spec_.Ny, spec_.Nz},
                          torch::TensorOptions().dtype(torch::kUInt8).device(torch::kCPU));
    } else {
      occ_.zero_();
    }
    std::cout << occ_.sizes() << std::endl;
    return occ_.to(torch::kFloat32).unsqueeze(0);//.unsqueeze(1).permute({0,1,2,4,3});
  }
  const VoxelGridSpec& spec() const { return spec_; }
private:
  VoxelGridSpec spec_;
  torch::Tensor occ_;
};

#else  // ------------------------ USE_ROS2 分支 ------------------------

#ifndef NO_TF2
#define NO_TF2 0
#endif

// 只有在 USE_ROS2 时，才包含 ROS2 头
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#if !NO_TF2
  #include <geometry_msgs/msg/transform_stamped.hpp>
  #include <tf2_ros/transform_listener.h>
  #include <tf2_ros/buffer.h>
#endif

class PointcloudVoxelizer : public rclcpp::Node {
public:
  using PC2 = sensor_msgs::msg::PointCloud2;

  explicit PointcloudVoxelizer(const VoxelizerParams& p)
  : rclcpp::Node("pointcloud_voxelizer"), params_(p) {
    spec_.voxel = params_.voxel;
    spec_.sx = params_.sx; spec_.sy = params_.sy; spec_.sz = params_.sz; spec_.finalize();

#if !NO_TF2
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
#endif

    sub_front_ = create_subscription<PC2>(
      params_.front_topic, rclcpp::SensorDataQoS(),
      std::bind(&PointcloudVoxelizer::pcCallbackFront, this, std::placeholders::_1));
    sub_back_  = create_subscription<PC2>(
      params_.back_topic,  rclcpp::SensorDataQoS(),
      std::bind(&PointcloudVoxelizer::pcCallbackBack,  this, std::placeholders::_1));

    std::atomic_store(&latest_pts_, std::make_shared<std::vector<Eigen::Vector3f>>());
  }

  // 输出 [1, 1, Nx, Nz, Ny]（uint8）
  torch::Tensor fetchVoxelObservation() {
    auto pts_ptr = std::atomic_load(&latest_pts_);
    const auto& pts = *pts_ptr;

    if (!occ_.defined()) {
      occ_ = torch::empty({spec_.Nx, spec_.Ny, spec_.Nz},
                          torch::TensorOptions().dtype(torch::kUInt8).device(torch::kCPU));
    }
    std::memset(occ_.data_ptr<uint8_t>(), 0, static_cast<size_t>(occ_.numel()));

    auto* buf = occ_.data_ptr<uint8_t>();
    const int Nx = spec_.Nx, Ny = spec_.Ny, Nz = spec_.Nz;
    const float vx = spec_.voxel;
    const float hx = spec_.sx * 0.5f, hy = spec_.sy * 0.5f, hz = spec_.sz * 0.5f;

    auto lin = [&](int ix, int iy, int iz)->size_t {
      return static_cast<size_t>(ix) * Ny * Nz + static_cast<size_t>(iy) * Nz + static_cast<size_t>(iz);
    };

    for (const auto& p : pts) {
      if (p.x() < -hx || p.x() >= hx) continue;
      if (p.y() < -hy || p.y() >= hy) continue;
      if (p.z() < -hz || p.z() >= hz) continue;
      if (p.z() < params_.z_clip_min || p.z() > params_.z_clip_max) continue;

      const float fx = (p.x() + hx) / vx;
      const float fy = (p.y() + hy) / vx;
      const float fz = (p.z() + hz) / vx;
      int ix = static_cast<int>(std::floor(fx));
      int iy = static_cast<int>(std::floor(fy));
      int iz = static_cast<int>(std::floor(fz));

      if ((unsigned)ix < (unsigned)Nx && (unsigned)iy < (unsigned)Ny && (unsigned)iz < (unsigned)Nz)
        buf[lin(ix,iy,iz)] = 1;
    }
    return occ_.to(torch::kFloat32).unsqueeze(0);
  }

  const VoxelGridSpec& spec() const { return spec_; }

private:
#if !NO_TF2
  static Eigen::Isometry3f toIso(const geometry_msgs::msg::TransformStamped& T) {
    Eigen::Isometry3f X = Eigen::Isometry3f::Identity();
    const auto& t = T.transform.translation;
    const auto& q = T.transform.rotation;
    X.translation() = Eigen::Vector3f(t.x, t.y, t.z);
    Eigen::Quaternionf Q(static_cast<float>(q.w), static_cast<float>(q.x),
                         static_cast<float>(q.y), static_cast<float>(q.z));
    Q.normalize();
    X.linear() = Q.toRotationMatrix();
    return X;
  }
#endif

  enum class SensorSide { Front, Back };

  static Eigen::Matrix3f yaw_deg_to_R(float yaw_deg) {
    constexpr float PI = 3.14159265358979323846f;
    const float r = yaw_deg * PI / 180.0f;
    const float c = std::cos(r), s = std::sin(r);
    Eigen::Matrix3f R; R << c,-s,0,  s,c,0,  0,0,1;
    return R;
  }

#if !NO_TF2
  bool lookup_torso_to_base(Eigen::Isometry3f& X_base_torso) {
    try {
      auto T = tf_buffer_->lookupTransform(params_.frame_id, params_.torso_frame, rclcpp::Time(0), std::chrono::milliseconds(50));
      X_base_torso = toIso(T); return true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "TF %s<- %s failed: %s",
        params_.frame_id.c_str(), params_.torso_frame.c_str(), e.what());
      return false;
    }
  }
#endif

  bool transform_with_explicit_pose(const PC2& msg, SensorSide side, std::vector<Eigen::Vector3f>& out) {
    Eigen::Isometry3f X_base_torso = Eigen::Isometry3f::Identity();
#if !NO_TF2
    if (!lookup_torso_to_base(X_base_torso)) return false;
#endif
    Eigen::Isometry3f X_torso_sensor = Eigen::Isometry3f::Identity();
    if (side == SensorSide::Front) {
      X_torso_sensor.linear()      = yaw_deg_to_R(params_.lidar_front_yaw_deg);
      X_torso_sensor.translation() = params_.lidar_front_pos;
    } else {
      X_torso_sensor.linear()      = yaw_deg_to_R(params_.lidar_back_yaw_deg);
      X_torso_sensor.translation() = params_.lidar_back_pos;
    }
    const Eigen::Isometry3f X_base_sensor = X_base_torso * X_torso_sensor;

    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
    out.reserve(out.size() + msg.width * msg.height);
    const size_t N = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    for (size_t i = 0; i < N; ++i, ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      out.emplace_back(X_base_sensor * Eigen::Vector3f(x, y, z));
    }
    return true;
  }

#if !NO_TF2
  bool transform_with_tf(const PC2& msg, std::vector<Eigen::Vector3f>& out) {
    const std::string src = msg.header.frame_id.empty() ? params_.frame_id : msg.header.frame_id;
    geometry_msgs::msg::TransformStamped T;
    try {
      T = tf_buffer_->lookupTransform(params_.frame_id, src, msg.header.stamp, std::chrono::milliseconds(50));
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "TF lookup (%s -> %s) failed: %s",
                  src.c_str(), params_.frame_id.c_str(), e.what());
      return false;
    }
    const Eigen::Isometry3f X = toIso(T);
    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
    out.reserve(out.size() + msg.width * msg.height);
    const size_t N = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    for (size_t i = 0; i < N; ++i, ++ix, ++iy, ++iz) {
      float x = *ix, y = *iy, z = *iz;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      out.emplace_back(X * Eigen::Vector3f(x, y, z));
    }
    return true;
  }
#endif

  void pcCallbackFront(const PC2::SharedPtr msg) {
    std::vector<Eigen::Vector3f> tmp;
#if !NO_TF2
    bool ok = params_.use_explicit_lidar_pose
                ? transform_with_explicit_pose(*msg, SensorSide::Front, tmp)
                : transform_with_tf(*msg, tmp);
#else
    bool ok = transform_with_explicit_pose(*msg, SensorSide::Front, tmp);
#endif
    if (!ok) return;
    { std::lock_guard<std::mutex> lk(mtx_); latest_front_.swap(tmp); }
    fuseLatestClouds();
  }

  void pcCallbackBack(const PC2::SharedPtr msg) {
    std::vector<Eigen::Vector3f> tmp;
#if !NO_TF2
    bool ok = params_.use_explicit_lidar_pose
                ? transform_with_explicit_pose(*msg, SensorSide::Back, tmp)
                : transform_with_tf(*msg, tmp);
#else
    bool ok = transform_with_explicit_pose(*msg, SensorSide::Back, tmp);
#endif
    if (!ok) return;
    { std::lock_guard<std::mutex> lk(mtx_); latest_back_.swap(tmp); }
    fuseLatestClouds();
  }

  void fuseLatestClouds() {
    auto out = std::make_shared<std::vector<Eigen::Vector3f>>();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      out->reserve(latest_front_.size() + latest_back_.size());
      out->insert(out->end(), latest_front_.begin(), latest_front_.end());
      out->insert(out->end(), latest_back_.begin(),  latest_back_.end());
    }
    std::atomic_store(&latest_pts_, out);
  }

private:
  VoxelizerParams params_;
  VoxelGridSpec   spec_;

  std::vector<Eigen::Vector3f> latest_front_, latest_back_;
  std::mutex mtx_;

  std::shared_ptr<std::vector<Eigen::Vector3f>> latest_pts_;
  torch::Tensor occ_;

  rclcpp::Subscription<PC2>::SharedPtr sub_front_, sub_back_;

#if !NO_TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
#endif
};

#endif // USE_ROS2