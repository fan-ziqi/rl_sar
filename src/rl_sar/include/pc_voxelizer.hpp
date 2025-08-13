#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>

#include <torch/torch.h>
#include <Eigen/Core>

#ifdef USE_ROS2
  #include <rclcpp/rclcpp.hpp>
  #include <sensor_msgs/msg/point_cloud2.hpp>
  #include <sensor_msgs/point_cloud2_iterator.hpp>
#endif

struct VoxelGridSpec {
  float voxel = 0.10f;
  float sx = 10.0f, sy = 10.0f, sz = 3.0f;
  float z_min = -0.20f;

  int Nx = 0, Ny = 0, Nz = 0;
  Eigen::Vector3f origin;

  void finalize() {
    Nx = static_cast<int>(std::ceil(sx / voxel));
    Ny = static_cast<int>(std::ceil(sy / voxel));
    Nz = static_cast<int>(std::ceil(sz / voxel));
    origin = Eigen::Vector3f(-sx*0.5f, -sy*0.5f, z_min);
  }
};

struct VoxelizerParams {
  std::string front_topic = "front_points";
  std::string back_topic  = "back_points";
  std::string frame_id    = "base_link";
  float voxel = 0.10f;
  float sx = 10.0f, sy = 10.0f, sz = 3.0f, z_min = -0.20f;
  float z_clip_min = -1e9f, z_clip_max = 1e9f;
};

#ifdef USE_ROS2

class PointcloudVoxelizer : public rclcpp::Node {
public:
  using PC2 = sensor_msgs::msg::PointCloud2;

  explicit PointcloudVoxelizer(const VoxelizerParams& p)
  : rclcpp::Node("pointcloud_voxelizer"), params_(p) {
    spec_.voxel = params_.voxel;
    spec_.sx = params_.sx; spec_.sy = params_.sy; spec_.sz = params_.sz; spec_.z_min = params_.z_min;
    spec_.finalize();

    sub_front_ = create_subscription<PC2>(
      params_.front_topic, rclcpp::SensorDataQoS(),
      std::bind(&PointcloudVoxelizer::pcCallbackFront, this, std::placeholders::_1));
    sub_back_  = create_subscription<PC2>(
      params_.back_topic,  rclcpp::SensorDataQoS(),
      std::bind(&PointcloudVoxelizer::pcCallbackBack,  this, std::placeholders::_1));

    // 初始空快照
    std::atomic_store(&latest_pts_, std::make_shared<std::vector<Eigen::Vector3f>>());
  }

  // 输出 [1, Nz, Ny, Nx]，uint8
  torch::Tensor fetchVoxelObservation() {
    // auto pts_ptr = std::atomic_load(&latest_pts_);
    // const auto& pts = *pts_ptr;

    // if (!occ_.defined()) {
    //   const std::vector<int64_t> sz = {
    //     static_cast<int64_t>(spec_.Nz),
    //     static_cast<int64_t>(spec_.Ny),
    //     static_cast<int64_t>(spec_.Nx)
    //   };
    //   occ_ = torch::empty(sz, torch::TensorOptions().dtype(torch::kUInt8).device(torch::kCPU));
    // }
    // std::memset(occ_.data_ptr<uint8_t>(), 0, static_cast<size_t>(occ_.numel()));

    // auto* buf = occ_.data_ptr<uint8_t>();
    // const int Nx = spec_.Nx, Ny = spec_.Ny, Nz = spec_.Nz;
    // auto lin = [&](int ix, int iy, int iz)->size_t { return (size_t)((iz * Ny + iy) * Nx + ix); };

    // const float vx = spec_.voxel;
    // const Eigen::Vector3f org = spec_.origin;

    // for (const auto& p : pts) {
    //   if (p.x() < org.x() || p.y() < org.y() || p.z() < org.z()) continue;
    //   if (p.x() >= org.x() + spec_.sx || p.y() >= org.y() + spec_.sy || p.z() >= org.z() + spec_.sz) continue;
    //   if (p.z() < params_.z_clip_min || p.z() > params_.z_clip_max) continue;

    //   const Eigen::Vector3f rel = p - org;
    //   int ix = (int)std::floor(rel.x() / vx);
    //   int iy = (int)std::floor(rel.y() / vx);
    //   int iz = (int)std::floor(rel.z() / vx);
    //   if ((unsigned)ix < (unsigned)Nx && (unsigned)iy < (unsigned)Ny && (unsigned)iz < (unsigned)Nz)
    //     buf[lin(ix,iy,iz)] = 1;
    // }
    // return occ_.unsqueeze(0);
    return torch::zeros({1, 32, 32, 32}, torch::TensorOptions().dtype(torch::kUInt8));
  }

  const VoxelGridSpec& spec() const { return spec_; }

private:
  static void parsePC2XYZ(const PC2& msg, std::vector<Eigen::Vector3f>& out) {
    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
    out.reserve(out.size() + msg.width * msg.height);
    for (size_t i = 0; i < msg.width * msg.height; ++i, ++ix, ++iy, ++iz) {
      float x = *ix, y = *iy, z = *iz;
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) out.emplace_back(x,y,z);
    }
  }

  void pcCallbackFront(const PC2::SharedPtr msg) {
    std::vector<Eigen::Vector3f> tmp; parsePC2XYZ(*msg, tmp);
    {
      std::lock_guard<std::mutex> lk(mtx_);
      latest_front_.swap(tmp);
    }
    fuseLatestClouds();
  }

  void pcCallbackBack(const PC2::SharedPtr msg) {
    std::vector<Eigen::Vector3f> tmp; parsePC2XYZ(*msg, tmp);
    {
      std::lock_guard<std::mutex> lk(mtx_);
      latest_back_.swap(tmp);
    }
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

  // 用 free functions 做原子 load/store
  std::shared_ptr<std::vector<Eigen::Vector3f>> latest_pts_;

  torch::Tensor occ_;
  rclcpp::Subscription<PC2>::SharedPtr sub_front_, sub_back_;
};

#else  // 非 ROS2：空桩，保证可编译

class PointcloudVoxelizer {
public:
  explicit PointcloudVoxelizer(const VoxelizerParams&) {}
  torch::Tensor fetchVoxelObservation() {
    return torch::zeros({1,1,1,1}, torch::TensorOptions().dtype(torch::kUInt8));
  }
  const VoxelGridSpec& spec() const { return spec_; }
private:
  VoxelGridSpec spec_;
};

#endif