/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 *
 * Test program for model inference stability and performance
 * Tests both direct torch usage and abstraction layer
 */

#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <cmath>

#ifdef USE_TORCH
#include <torch/script.h>
#endif

#ifdef USE_ONNX
#include <onnxruntime_cxx_api.h>
#endif

#include "../library/core/inference_runtime/inference_runtime.hpp"

struct TestResult {
    std::string name;
    double avg_time_ms;
    double min_time_ms;
    double max_time_ms;
    double max_output_diff;
    bool is_stable;
    std::vector<float> first_output;
};

void print_header(const std::string& title) {
    std::cout << "\n========================================" << std::endl;
    std::cout << title << std::endl;
    std::cout << "========================================" << std::endl;
}

void print_output_sample(const std::vector<float>& output, int count = 5) {
    std::cout << std::fixed << std::setprecision(6);
    for (int i = 0; i < std::min(count, (int)output.size()); i++) {
        std::cout << std::setw(9) << output[i] << " ";
    }
}

void print_test_result(const TestResult& result) {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\n--- " << result.name << " ---" << std::endl;
    std::cout << "  Avg Time: " << result.avg_time_ms << " ms" << std::endl;
    std::cout << "  Min Time: " << result.min_time_ms << " ms" << std::endl;
    std::cout << "  Max Time: " << result.max_time_ms << " ms" << std::endl;
    std::cout << "  Range:    " << (result.max_time_ms - result.min_time_ms) << " ms" << std::endl;
    std::cout << "  Max Diff: " << std::scientific << result.max_output_diff << std::endl;
    std::cout << "  Status:   " << (result.is_stable ? "✓ STABLE" : "✗ UNSTABLE") << std::endl;
    std::cout << "  Output[0:5]: ";
    print_output_sample(result.first_output);
    std::cout << std::endl;
}

#ifdef USE_TORCH
TestResult test_direct_torch(const std::string& model_path, const std::vector<float>& input, int iterations) {
    TestResult result;
    result.name = "Direct Torch (No Abstraction)";

    print_header("Testing Direct Torch API");
    std::cout << "Loading model with torch::jit::load()..." << std::endl;

    // Load model directly with torch
    auto model = torch::jit::load(model_path);
    model.eval();

    std::cout << "Model loaded successfully" << std::endl;
    std::cout << "\nRunning " << iterations << " iterations..." << std::endl;

    std::vector<std::vector<float>> all_outputs;
    std::vector<double> times;

    for (int i = 0; i < iterations; i++) {
        // Convert input to tensor
        auto input_tensor = torch::tensor(input, torch::kFloat32).reshape({1, (int64_t)input.size()});

        auto start = std::chrono::high_resolution_clock::now();

        // Forward pass
        torch::autograd::GradMode::set_enabled(false);
        auto output_tensor = model.forward({input_tensor}).toTensor();

        // Convert back to vector
        auto cpu_tensor = output_tensor.contiguous().to(torch::kCPU);
        float* data_ptr = cpu_tensor.data_ptr<float>();
        int64_t num_elements = cpu_tensor.numel();
        std::vector<float> output(data_ptr, data_ptr + num_elements);

        auto end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

        times.push_back(elapsed_ms);
        all_outputs.push_back(output);

        if (i < 5 || i >= iterations - 2) {
            std::cout << "Iter " << std::setw(2) << i << " | Time: " << std::setw(7)
                      << std::fixed << std::setprecision(4) << elapsed_ms << " ms | Output[0:5]: ";
            print_output_sample(output);
            std::cout << std::endl;
        } else if (i == 5) {
            std::cout << "..." << std::endl;
        }
    }

    // Calculate statistics
    result.avg_time_ms = 0.0;
    result.min_time_ms = times[0];
    result.max_time_ms = times[0];

    for (double t : times) {
        result.avg_time_ms += t;
        if (t < result.min_time_ms) result.min_time_ms = t;
        if (t > result.max_time_ms) result.max_time_ms = t;
    }
    result.avg_time_ms /= times.size();

    // Check stability
    result.max_output_diff = 0.0;
    result.is_stable = true;
    result.first_output = all_outputs[0];

    for (size_t i = 0; i < all_outputs[0].size(); i++) {
        for (size_t j = 1; j < all_outputs.size(); j++) {
            double diff = std::abs(all_outputs[j][i] - all_outputs[0][i]);
            if (diff > result.max_output_diff) {
                result.max_output_diff = diff;
            }
            if (diff > 1e-5) {
                result.is_stable = false;
            }
        }
    }

    return result;
}

TestResult test_direct_torch_frozen(const std::string& model_path, const std::vector<float>& input, int iterations) {
    TestResult result;
    result.name = "Direct Torch API (No Abstraction)";

    print_header("Testing Direct Torch API (No Abstraction)");
    std::cout << "Loading model with torch::jit::load() + freeze()..." << std::endl;

    // Load and freeze model
    auto model = torch::jit::load(model_path);
    model.eval();
    model = torch::jit::freeze(model);

    std::cout << "Model loaded and frozen successfully" << std::endl;
    std::cout << "\nRunning " << iterations << " iterations..." << std::endl;

    std::vector<std::vector<float>> all_outputs;
    std::vector<double> times;

    for (int i = 0; i < iterations; i++) {
        auto input_tensor = torch::tensor(input, torch::kFloat32).reshape({1, (int64_t)input.size()});

        auto start = std::chrono::high_resolution_clock::now();

        torch::autograd::GradMode::set_enabled(false);
        auto output_tensor = model.forward({input_tensor}).toTensor();

        auto cpu_tensor = output_tensor.contiguous().to(torch::kCPU);
        float* data_ptr = cpu_tensor.data_ptr<float>();
        int64_t num_elements = cpu_tensor.numel();
        std::vector<float> output(data_ptr, data_ptr + num_elements);

        auto end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

        times.push_back(elapsed_ms);
        all_outputs.push_back(output);

        if (i < 5 || i >= iterations - 2) {
            std::cout << "Iter " << std::setw(2) << i << " | Time: " << std::setw(7)
                      << std::fixed << std::setprecision(4) << elapsed_ms << " ms | Output[0:5]: ";
            print_output_sample(output);
            std::cout << std::endl;
        } else if (i == 5) {
            std::cout << "..." << std::endl;
        }
    }

    // Calculate statistics
    result.avg_time_ms = 0.0;
    result.min_time_ms = times[0];
    result.max_time_ms = times[0];

    for (double t : times) {
        result.avg_time_ms += t;
        if (t < result.min_time_ms) result.min_time_ms = t;
        if (t > result.max_time_ms) result.max_time_ms = t;
    }
    result.avg_time_ms /= times.size();

    // Check stability
    result.max_output_diff = 0.0;
    result.is_stable = true;
    result.first_output = all_outputs[0];

    for (size_t i = 0; i < all_outputs[0].size(); i++) {
        for (size_t j = 1; j < all_outputs.size(); j++) {
            double diff = std::abs(all_outputs[j][i] - all_outputs[0][i]);
            if (diff > result.max_output_diff) {
                result.max_output_diff = diff;
            }
            if (diff > 1e-5) {
                result.is_stable = false;
            }
        }
    }

    return result;
}
#endif

#ifdef USE_ONNX
TestResult test_direct_onnx(const std::string& model_path, const std::vector<float>& input, int iterations) {
    TestResult result;
    result.name = "Direct ONNX Runtime (No Abstraction)";

    print_header("Testing Direct ONNX Runtime API");
    std::cout << "Loading model with Ort::Session()..." << std::endl;

    // Initialize ONNX Runtime environment
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "DirectONNXTest");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // Create session
    auto session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

    // Get input/output info
    auto input_name = session->GetInputNameAllocated(0, Ort::AllocatorWithDefaultOptions());
    auto output_name = session->GetOutputNameAllocated(0, Ort::AllocatorWithDefaultOptions());
    auto input_shape = session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();

    std::cout << "Model loaded successfully" << std::endl;
    std::cout << "Input shape: [";
    for (auto dim : input_shape) std::cout << dim << " ";
    std::cout << "]" << std::endl;
    std::cout << "\nRunning " << iterations << " iterations..." << std::endl;

    std::vector<std::vector<float>> all_outputs;
    std::vector<double> times;

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    for (int i = 0; i < iterations; i++) {
        // Prepare input tensor
        std::vector<float> input_copy = input;
        auto input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            input_copy.data(),
            input_copy.size(),
            input_shape.data(),
            input_shape.size()
        );

        const char* input_names[] = {input_name.get()};
        const char* output_names[] = {output_name.get()};

        auto start = std::chrono::high_resolution_clock::now();

        // Run inference
        auto output_tensors = session->Run(
            Ort::RunOptions{nullptr},
            input_names, &input_tensor, 1,
            output_names, 1
        );

        // Extract output
        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
        std::vector<float> output(output_data, output_data + output_size);

        auto end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

        times.push_back(elapsed_ms);
        all_outputs.push_back(output);

        if (i < 5 || i >= iterations - 2) {
            std::cout << "Iter " << std::setw(2) << i << " | Time: " << std::setw(7)
                      << std::fixed << std::setprecision(4) << elapsed_ms << " ms | Output[0:5]: ";
            print_output_sample(output);
            std::cout << std::endl;
        } else if (i == 5) {
            std::cout << "..." << std::endl;
        }
    }

    // Calculate statistics
    result.avg_time_ms = 0.0;
    result.min_time_ms = times[0];
    result.max_time_ms = times[0];

    for (double t : times) {
        result.avg_time_ms += t;
        if (t < result.min_time_ms) result.min_time_ms = t;
        if (t > result.max_time_ms) result.max_time_ms = t;
    }
    result.avg_time_ms /= times.size();

    // Check stability
    result.max_output_diff = 0.0;
    result.is_stable = true;
    result.first_output = all_outputs[0];

    for (size_t i = 0; i < all_outputs[0].size(); i++) {
        for (size_t j = 1; j < all_outputs.size(); j++) {
            double diff = std::abs(all_outputs[j][i] - all_outputs[0][i]);
            if (diff > result.max_output_diff) {
                result.max_output_diff = diff;
            }
            if (diff > 1e-5) {
                result.is_stable = false;
            }
        }
    }

    return result;
}
#endif

TestResult test_abstraction_layer(const std::string& model_path, const std::vector<float>& input, int iterations) {
    TestResult result;
    result.name = "Abstraction Layer (InferenceRuntime)";

    print_header("Testing Abstraction Layer (InferenceRuntime)");
    std::cout << "Loading model with ModelFactory::load_model()..." << std::endl;

    auto model = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!model) {
        std::cout << "ERROR: Failed to load model!" << std::endl;
        result.is_stable = false;
        return result;
    }

    std::cout << "Model loaded successfully (type: " << model->get_model_type() << ")" << std::endl;
    std::cout << "\nRunning " << iterations << " iterations..." << std::endl;

    std::vector<std::vector<float>> all_outputs;
    std::vector<double> times;

    for (int i = 0; i < iterations; i++) {
        auto start = std::chrono::high_resolution_clock::now();

        auto output = model->forward(input);

        auto end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

        times.push_back(elapsed_ms);
        all_outputs.push_back(output);

        if (i < 5 || i >= iterations - 2) {
            std::cout << "Iter " << std::setw(2) << i << " | Time: " << std::setw(7)
                      << std::fixed << std::setprecision(4) << elapsed_ms << " ms | Output[0:5]: ";
            print_output_sample(output);
            std::cout << std::endl;
        } else if (i == 5) {
            std::cout << "..." << std::endl;
        }
    }

    // Calculate statistics
    result.avg_time_ms = 0.0;
    result.min_time_ms = times[0];
    result.max_time_ms = times[0];

    for (double t : times) {
        result.avg_time_ms += t;
        if (t < result.min_time_ms) result.min_time_ms = t;
        if (t > result.max_time_ms) result.max_time_ms = t;
    }
    result.avg_time_ms /= times.size();

    // Check stability
    result.max_output_diff = 0.0;
    result.is_stable = true;
    result.first_output = all_outputs[0];

    for (size_t i = 0; i < all_outputs[0].size(); i++) {
        for (size_t j = 1; j < all_outputs.size(); j++) {
            double diff = std::abs(all_outputs[j][i] - all_outputs[0][i]);
            if (diff > result.max_output_diff) {
                result.max_output_diff = diff;
            }
            if (diff > 1e-5) {
                result.is_stable = false;
            }
        }
    }

    return result;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <model_path> <input_size> [iterations]" << std::endl;
        std::cout << "Example: " << argv[0] << " policy/go2/himloco/himloco.pt 270 30" << std::endl;
        return 1;
    }

    std::string model_path = argv[1];
    int input_size = std::stoi(argv[2]);
    int iterations = (argc >= 4) ? std::stoi(argv[3]) : 30;

    print_header("Model Inference Stability & Performance Test");
    std::cout << "Model:      " << model_path << std::endl;
    std::cout << "Input size: " << input_size << std::endl;
    std::cout << "Iterations: " << iterations << std::endl;
    std::cout << "Input:      all values = 0.1" << std::endl;

    // Create input vector filled with 0.1
    std::vector<float> input(input_size, 0.1f);

    std::vector<TestResult> results;

    bool is_torch_model = (model_path.find(".pt") != std::string::npos || model_path.find(".pth") != std::string::npos);
    bool is_onnx_model = (model_path.find(".onnx") != std::string::npos);

    // Test 1: Direct API (without abstraction layer)
#ifdef USE_TORCH
    if (is_torch_model) {
        try {
            // Use frozen version for best stability
            results.push_back(test_direct_torch_frozen(model_path, input, iterations));
        } catch (const std::exception& e) {
            std::cout << "ERROR in direct torch test: " << e.what() << std::endl;
        }
    }
#endif

#ifdef USE_ONNX
    if (is_onnx_model) {
        try {
            results.push_back(test_direct_onnx(model_path, input, iterations));
        } catch (const std::exception& e) {
            std::cout << "ERROR in direct ONNX test: " << e.what() << std::endl;
        }
    }
#endif

    // Test 2: With abstraction layer (InferenceRuntime)
    try {
        results.push_back(test_abstraction_layer(model_path, input, iterations));
    } catch (const std::exception& e) {
        std::cout << "ERROR in abstraction layer test: " << e.what() << std::endl;
    }

    // Print comparison
    print_header("Summary & Comparison");

    for (const auto& result : results) {
        print_test_result(result);
    }

    // Final verdict
    std::cout << "\n========================================" << std::endl;
    bool all_stable = true;
    for (const auto& result : results) {
        if (!result.is_stable) {
            all_stable = false;
            std::cout << "⚠ WARNING: " << result.name << " is UNSTABLE!" << std::endl;
        }
    }

    if (all_stable) {
        std::cout << "✓ All tests PASSED - outputs are stable" << std::endl;
    } else {
        std::cout << "\n✗ Some tests FAILED - outputs are unstable" << std::endl;
        std::cout << "This indicates BatchNorm or other stateful layers are updating during inference." << std::endl;
    }
    std::cout << "========================================" << std::endl;

    return all_stable ? 0 : 1;
}

