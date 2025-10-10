/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "inference_runtime.hpp"
#include <stdexcept>
#include <iostream>
#include <numeric>

#ifdef USE_TORCH
#include <ATen/Parallel.h>
#endif

namespace InferenceRuntime
{

// ============================================================================
// TorchModel Implementation
// ============================================================================

TorchModel::TorchModel()
{
#ifdef USE_TORCH
    // Disable gradient computation to improve inference performance
    torch::autograd::GradMode::set_enabled(false);
    std::cout << "Torch environment initialized" << std::endl;
#endif
}

TorchModel::~TorchModel()
{
}

void TorchModel::set_torch_threads(int num_threads)
{
#ifdef USE_TORCH
    torch::set_num_threads(num_threads);
    std::cout << "Torch threads set to: " << num_threads << std::endl;
#else
    std::cout << "Torch support not compiled, ignoring thread setting" << std::endl;
#endif
}

bool TorchModel::load(const std::string& model_path)
{
    try
    {
#ifdef USE_TORCH
        // Load TorchScript model
        model_ = torch::jit::load(model_path);
        model_.eval();
        torch::autograd::GradMode::set_enabled(false);
        model_path_ = model_path;
        loaded_ = true;
        std::cout << "Successfully loaded Torch model: " << model_path << std::endl;
        return true;
#else
        std::cout << "Torch support not compiled. Please define USE_TORCH." << std::endl;
        loaded_ = false;
        return false;
#endif
    }
    catch (const std::exception& e)
    {
        std::cout << "Failed to load Torch model: " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

std::vector<float> TorchModel::forward(const std::vector<std::vector<float>>& inputs)
{
    if (!loaded_)
    {
        throw std::runtime_error("Model not loaded");
    }

#ifdef USE_TORCH
    try
    {
        // Convert input vectors to Torch tensors
        std::vector<torch::jit::IValue> torch_inputs;
        for (const auto& input : inputs)
        {
            std::vector<int64_t> shape = {1, static_cast<int64_t>(input.size())};
            torch_inputs.push_back(vector_to_torch(input, shape));
        }

        // Execute forward inference
        auto output = model_.forward(torch_inputs).toTensor();

        // Convert output tensor to vector
        return torch_to_vector(output);
    }
    catch (const std::exception& e)
    {
        std::cout << "Torch inference error: " << e.what() << std::endl;
        throw;
    }
#else
    throw std::runtime_error("Torch support not compiled");
#endif
}

std::vector<float> TorchModel::forward(const std::vector<float>& input)
{
    // Wrap single input as multi-input format, then call multi-input version of forward
    return forward(std::vector<std::vector<float>>{input});
}

#ifdef USE_TORCH
torch::Tensor TorchModel::vector_to_torch(const std::vector<float>& data, const std::vector<int64_t>& shape)
{
    // Create Torch tensor from vector data
    auto tensor = torch::from_blob(const_cast<float*>(data.data()), shape, torch::kFloat32).clone();
    return tensor;
}

std::vector<float> TorchModel::torch_to_vector(const torch::Tensor& tensor)
{
    // Convert Torch tensor to contiguous tensor on CPU
    auto cpu_tensor = tensor.contiguous().to(torch::kCPU);

    // Get data pointer
    float* data_ptr = cpu_tensor.data_ptr<float>();

    // Get total number of elements
    int64_t num_elements = cpu_tensor.numel();

    // Copy data to vector
    std::vector<float> result(data_ptr, data_ptr + num_elements);

    return result;
}
#endif

// ============================================================================
// ONNXModel Implementation
// ============================================================================

ONNXModel::ONNXModel()
#ifdef USE_ONNX
    : memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
#endif
{
#ifdef USE_ONNX
    // Initialize ONNX Runtime environment
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ONNXModel");
#endif
}

ONNXModel::~ONNXModel()
{
#ifdef USE_ONNX
    session_.reset();
    env_.reset();
#endif
}

bool ONNXModel::load(const std::string& model_path)
{
    try
    {
#ifdef USE_ONNX
        // Configure session options
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        // Create inference session
        session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), session_options);

        // Setup input/output information
        setup_input_output_info();

        model_path_ = model_path;
        loaded_ = true;
        std::cout << "Successfully loaded ONNX model: " << model_path << std::endl;
        return true;
#else
        std::cout << "ONNX support not compiled. Please define USE_ONNX." << std::endl;
        loaded_ = false;
        return false;
#endif
    }
    catch (const std::exception& e)
    {
        std::cout << "Failed to load ONNX model: " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

std::vector<float> ONNXModel::forward(const std::vector<std::vector<float>>& inputs)
{
    if (!loaded_)
    {
        throw std::runtime_error("Model not loaded");
    }

#ifdef USE_ONNX
    try
    {
        // Create memory info
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        // Prepare input tensors
        std::vector<Ort::Value> ort_inputs;
        for (size_t i = 0; i < inputs.size() && i < input_shapes_.size(); ++i)
        {
            auto input_shape = session_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
            auto input_tensor = Ort::Value::CreateTensor<float>(
                memory_info,
                const_cast<float*>(inputs[i].data()),
                inputs[i].size(),
                input_shape.data(),
                input_shape.size()
            );
            ort_inputs.push_back(std::move(input_tensor));
        }

        // Prepare input/output name C string arrays
        std::vector<const char*> input_names_cstr;
        std::vector<const char*> output_names_cstr;

        for (const auto& name : input_node_names_)
        {
            input_names_cstr.push_back(name.c_str());
        }
        for (const auto& name : output_node_names_)
        {
            output_names_cstr.push_back(name.c_str());
        }

        // Execute inference
        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            input_names_cstr.data(),
            ort_inputs.data(),
            ort_inputs.size(),
            output_names_cstr.data(),
            output_names_cstr.size()
        );

        // Extract output data
        return extract_output_data(outputs);
    }
    catch (const std::exception& e)
    {
        std::cout << "ONNX inference error: " << e.what() << std::endl;
        throw;
    }
#else
    throw std::runtime_error("ONNX support not compiled");
#endif
}

std::vector<float> ONNXModel::forward(const std::vector<float>& input)
{
    // Wrap single input as multi-input format, then call multi-input version of forward
    return forward(std::vector<std::vector<float>>{input});
}

#ifdef USE_ONNX
void ONNXModel::setup_input_output_info()
{
    // Get input node information
    size_t num_input_nodes = session_->GetInputCount();
    input_node_names_.reserve(num_input_nodes);
    input_shapes_.reserve(num_input_nodes);

    for (size_t i = 0; i < num_input_nodes; ++i)
    {
        // Get input name
        auto input_name = session_->GetInputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
        input_node_names_.push_back(std::string(input_name.get()));

        // Get input shape
        Ort::TypeInfo input_type_info = session_->GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        auto input_dims = input_tensor_info.GetShape();

        std::vector<int64_t> shape;
        for (auto dim : input_dims)
        {
            // Handle dynamic dimensions
            if (dim == -1)
            {
                shape.push_back(1);
            }
            else
            {
                shape.push_back(dim);
            }
        }
        input_shapes_.push_back(shape);
    }

    // Get output node information
    size_t num_output_nodes = session_->GetOutputCount();
    output_node_names_.reserve(num_output_nodes);
    output_shapes_.reserve(num_output_nodes);

    for (size_t i = 0; i < num_output_nodes; ++i)
    {
        // Get output name
        auto output_name = session_->GetOutputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
        output_node_names_.push_back(std::string(output_name.get()));

        // Get output shape
        Ort::TypeInfo output_type_info = session_->GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        auto output_dims = output_tensor_info.GetShape();

        std::vector<int64_t> shape;
        for (auto dim : output_dims)
        {
            // Handle dynamic dimensions
            if (dim == -1)
            {
                shape.push_back(1);
            }
            else
            {
                shape.push_back(dim);
            }
        }
        output_shapes_.push_back(shape);
    }
}

std::vector<float> ONNXModel::extract_output_data(const std::vector<Ort::Value>& outputs)
{
    if (outputs.empty())
    {
        throw std::runtime_error("No outputs from ONNX model");
    }

    // Get first output tensor
    auto& output = outputs[0];
    float* output_data = const_cast<float*>(output.GetTensorData<float>());

    // Calculate total number of output elements
    auto output_shape = output.GetTensorTypeAndShapeInfo().GetShape();

    int64_t num_elements = 1;
    for (auto dim : output_shape)
    {
        if (dim > 0)
        {
            num_elements *= dim;
        }
    }

    // Copy output data to vector
    std::vector<float> result(output_data, output_data + num_elements);

    return result;
}
#endif

// ============================================================================
// ModelFactory Implementation
// ============================================================================

std::unique_ptr<Model> ModelFactory::create_model(ModelType type)
{
    switch (type)
    {
        case ModelType::TORCH:
        {
            auto model = std::make_unique<TorchModel>();
            // Set Torch thread count to 4 for performance optimization
            model->set_torch_threads(4);
            return model;
        }
        case ModelType::ONNX:
            return std::make_unique<ONNXModel>();
        default:
            return nullptr;
    }
}

ModelFactory::ModelType ModelFactory::detect_model_type(const std::string& model_path)
{
    // Extract file extension from path
    std::filesystem::path path(model_path);
    std::string extension = path.extension().string();

    // Convert to lowercase for case-insensitive comparison
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

    // Determine model type based on extension
    if (extension == ".pt" || extension == ".pth")
    {
        return ModelType::TORCH;
    }
    else if (extension == ".onnx")
    {
        return ModelType::ONNX;
    }
    else
    {
        throw std::runtime_error("Unknown model file extension: " + extension + ". Supported: .pt, .pth, .onnx");
    }
}

std::unique_ptr<Model> ModelFactory::load_model(const std::string& model_path, ModelType type)
{
    // If type is AUTO, automatically detect model type
    if (type == ModelType::AUTO)
    {
        type = detect_model_type(model_path);
    }

    // Create and load model
    auto model = create_model(type);
    if (model && model->load(model_path))
    {
        return model;
    }
    return nullptr;
}

} // namespace InferenceRuntime
