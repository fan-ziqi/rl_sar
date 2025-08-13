// OnnxPolicy.hpp
#pragma once
#include <onnxruntime_cxx_api.h>
#include <torch/torch.h>
#include <memory>
#include <array>
#include <vector>
#include <string>

class OnnxPolicy {
public:
    OnnxPolicy() : env_(ORT_LOGGING_LEVEL_WARNING, "rl") {}

    inline void load(const std::string& onnx_path, int cuda_device = 0) {
        OrtCUDAProviderOptions cuda{};
        cuda.device_id = cuda_device;
        cuda.arena_extend_strategy = 0;
        cuda.gpu_mem_limit = SIZE_MAX;
        cuda.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive;
        cuda.do_copy_in_default_stream = 1;

        so_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        so_.AppendExecutionProvider_CUDA(cuda);

        session_ = std::make_unique<Ort::Session>(env_, onnx_path.c_str(), so_);

        if (session_->GetInputCount() != 3) {
            throw std::runtime_error("Expect 3 inputs: policy, grid_map_, mask");
        }

        output_names_ = session_->GetOutputNames();
    }

    inline torch::Tensor run(const torch::Tensor& policy_f32_b1x223,
                         const torch::Tensor& grid_bool_b1x32x32x32,
                         const torch::Tensor& mask_bool_b1) const {
        TORCH_CHECK(session_, "OnnxPolicy not loaded. Call load() first.");

        // 1) 规范化输入：CPU + contiguous + 正确 dtype
        auto policy = policy_f32_b1x223.to(torch::kFloat32).contiguous().cpu();   // [1,223]
        auto grid   = grid_bool_b1x32x32x32.to(torch::kBool).contiguous().cpu(); // [1,32,32,32]
        auto mask   = mask_bool_b1.to(torch::kBool).contiguous().cpu();          // [1]

        std::vector<int64_t> sp(policy.sizes().begin(), policy.sizes().end());
        std::vector<int64_t> sg(grid.sizes().begin(),   grid.sizes().end());
        std::vector<int64_t> sm(mask.sizes().begin(),   mask.sizes().end());

        Ort::MemoryInfo cpu_mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);

        Ort::Value in_p = Ort::Value::CreateTensor<float>(
            cpu_mem, policy.data_ptr<float>(), static_cast<size_t>(policy.numel()), sp.data(), sp.size());
        Ort::Value in_g = Ort::Value::CreateTensor<bool>(
            cpu_mem, grid.data_ptr<bool>(),   static_cast<size_t>(grid.numel()),   sg.data(), sg.size());
        Ort::Value in_m = Ort::Value::CreateTensor<bool>(
            cpu_mem, mask.data_ptr<bool>(),   static_cast<size_t>(mask.numel()),   sm.data(), sm.size());

        const std::array<const char*,3> input_names = {"policy", "grid_map_", "mask"};
        Ort::Value inputs[] = {std::move(in_p), std::move(in_g), std::move(in_m)};

        // 2) 准备输出名（ORT 1.22 C++ API 需要显式给出至少一个）
        std::vector<const char*> out_names_c;
        if (output_names_.empty()) {
            // 兜底：极端情况下 load() 没填充时再取一次
            const_cast<OnnxPolicy*>(this)->output_names_ = session_->GetOutputNames();
        }
        out_names_c.reserve(output_names_.size());
        for (const auto& s : output_names_) out_names_c.push_back(s.c_str());
        if (out_names_c.empty()) {
            throw std::runtime_error("No output names available from the model.");
        }

        // 3) 运行推断（必须传输出名与数量）
        auto outputs = session_->Run(Ort::RunOptions{nullptr},
                                    input_names.data(), inputs, 3,
                                    out_names_c.data(), out_names_c.size());

        // for (size_t i = 0; i < outputs.size(); ++i) {
        //     auto info = outputs[i].GetTensorTypeAndShapeInfo();
        //     auto shp  = info.GetShape();
        //     std::cerr << "[ORT out] " << output_names_[i] << " shape=[";
        //     for (size_t k = 0; k < shp.size(); ++k) std::cerr << shp[k] << (k+1<shp.size()? ",":"");
        //     std::cerr << "]\n";
        // }

        // 4) 读取第0个输出（期望 float）
        auto& out0 = outputs.at(4);
        auto info = out0.GetTensorTypeAndShapeInfo();
        if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            throw std::runtime_error("Expect float output tensor for actions.");
        }
        const auto oshp = info.GetShape();
        float* ptr = out0.GetTensorMutableData<float>();

        std::vector<int64_t> tshape(oshp.begin(), oshp.end());
        // from_blob + clone 保证内存独立
        return torch::from_blob(ptr, tshape, torch::TensorOptions().dtype(torch::kFloat32)).clone();
    }

private:
    Ort::Env env_;
    Ort::SessionOptions so_;
    std::unique_ptr<Ort::Session> session_;
    mutable Ort::AllocatorWithDefaultOptions allocator_;
    std::vector<std::string> output_names_;
};