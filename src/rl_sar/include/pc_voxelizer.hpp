#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <torch/torch.h>
#include <Eigen/Core>
#include <atomic>
#include <memory>
#include <vector>
#include <cmath>
#include <mutex>

struct VoxelGridSpec {
  float voxel = 0.10f;       // size for each grid
  float sx = 10.0f;          // x range
  float sy = 10.0f;          // y range
  float sz = 3.0f;           // z range
  float z_min = -0.20f;      // z lowest
  int Nx = 0, Ny = 0, Nz = 0;
  Eigen::Vector3f origin;

  void finalize() {
    Nx = static_cast<int>(std::ceil(sx / voxel));
    Ny = static_cast<int>(std::ceil(sy / voxel));
    Nz = static_cast<int>(std::ceil(sz / voxel));
    origin = Eigen::Vector3f(-sx*0.5f, -sy*0.5f, z_min);
  }
};

class PointcloudVoxelizer : public rclcpp::Node {
public:
  using PC2 = sensor_msgs::msg::PointCloud2;

  struct Params {
    std::string front_topic = "front_points";
    std::string back_topic  = "back_points";
    std::string frame_id    = "base_link";
    float voxel = 0.10f;
    float sx = 10.0f, sy = 10.0f, sz = 3.0f, z_min = -0.20f;
    float z_clip_min = -1e9f, z_clip_max = 1e9f;
  };

  explicit PointcloudVoxelizer(const Params& p)
  : Node("pointcloud_voxelizer"), params_(p)
  {
    // 规格
    spec_.voxel = params_.voxel;
    spec_.sx = params_.sx; spec_.sy = params_.sy; spec_.sz = params_.sz; spec_.z_min = params_.z_min;
    spec_.finalize();

    // 订阅
    sub_front_ = create_subscription<PC2>(params_.front_topic, rclcpp::SensorDataQoS(),
      std::bind(&PointcloudVoxelizer::pcCallbackFront, this, std::placeholders::_1));
    sub_back_  = create_subscription<PC2>(params_.back_topic,  rclcpp::SensorDataQoS(),
      std::bind(&PointcloudVoxelizer::pcCallbackBack,  this, std::placeholders::_1));

    latest_pts_.store(std::make_shared<std::vector<Eigen::Vector3f>>());

    RCLCPP_INFO(get_logger(),
      "Voxelizer ready: voxel=%.3f box=(%.1f,%.1f,%.1f) z_min=%.2f -> (Nx,Ny,Nz)=(%d,%d,%d)",
      spec_.voxel, spec_.sx, spec_.sy, spec_.sz, spec_.z_min, spec_.Nx, spec_.Ny, spec_.Nz);
  }

  torch::Tensor fetchVoxelObservation()
  {
    auto pts_ptr = latest_pts_.load();
    const auto& pts = *pts_ptr;

    if (!occ_.defined()) {
      occ_ = torch::empty({spec_.Nz, spec_.Ny, spec_.Nx}, torch::TensorOptions().dtype(torch::kUInt8).device(torch::kCPU));
    }
    std::memset(occ_.data_ptr<uint8_t>(), 0, occ_.numel() * sizeof(uint8_t));

    auto *buf = occ_.data_ptr<uint8_t>();
    const int Nx = spec_.Nx, Ny = spec_.Ny, Nz = spec_.Nz;
    auto idx = [&](int ix, int iy, int iz) -> size_t {
      return static_cast<size_t>((iz * Ny + iy) * Nx + ix);
    };

    const float vx = spec_.voxel;
    const Eigen::Vector3f org = spec_.origin;

    for (const auto& p : pts) {
      if (p.x() < org.x() || p.y() < org.y() || p.z() < org.z()) continue;
      if (p.x() >= org.x() + spec_.sx || p.y() >= org.y() + spec_.sy || p.z() >= org.z() + spec_.sz) continue;
      if (p.z() < params_.z_clip_min || p.z() > params_.z_clip_max) continue;

      const Eigen::Vector3f rel = p - org;
      int ix = static_cast<int>(std::floor(rel.x() / vx));
      int iy = static_cast<int>(std::floor(rel.y() / vx));
      int iz = static_cast<int>(std::floor(rel.z() / vx));

      if ((unsigned)ix < (unsigned)Nx && (unsigned)iy < (unsigned)Ny && (unsigned)iz < (unsigned)Nz) {
        buf[idx(ix,iy,iz)] = 1;
      }
    }

    return occ_.unsqueeze(0);
  }

  const VoxelGridSpec& spec() const { return spec_; }

  void fuseLatestClouds()
  {
    std::shared_ptr<std::vector<Eigen::Vector3f>> out = std::make_shared<std::vector<Eigen::Vector3f>>();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      out->reserve(latest_front_.size() + latest_back_.size());
      out->insert(out->end(), latest_front_.begin(), latest_front_.end());
      out->insert(out->end(), latest_back_.begin(),  latest_back_.end());
    }
    latest_pts_.store(out);
  }

private:
  static void parsePC2XYZ(const PC2& msg, std::vector<Eigen::Vector3f>& out)
  {
    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
    out.reserve(out.size() + msg.width * msg.height);

    for (size_t i = 0; i < msg.width * msg.height; ++i, ++ix, ++iy, ++iz) {
      float x = *ix, y = *iy, z = *iz;
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        out.emplace_back(x, y, z);
      }
    }
  }

  void pcCallbackFront(const PC2::SharedPtr msg)
  {
    std::vector<Eigen::Vector3f> tmp;
    parsePC2XYZ(*msg, tmp);
    {
      std::lock_guard<std::mutex> lk(mtx_);
      latest_front_.swap(tmp); // O(1) switch
    }
    fuseLatestClouds();
  }

  void pcCallbackBack(const PC2::SharedPtr msg)
  {
    std::vector<Eigen::Vector3f> tmp;
    parsePC2XYZ(*msg, tmp);
    {
      std::lock_guard<std::mutex> lk(mtx_);
      latest_back_.swap(tmp);
    }
    fuseLatestClouds();
  }

private:
  Params params_;
  VoxelGridSpec spec_;

  std::vector<Eigen::Vector3f> latest_front_, latest_back_;
  std::mutex mtx_;

  std::atomic<std::shared_ptr<std::vector<Eigen::Vector3f>>> latest_pts_;

  torch::Tensor occ_;

  rclcpp::Subscription<PC2>::SharedPtr sub_front_, sub_back_;
};