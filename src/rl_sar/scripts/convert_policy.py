#!/usr/bin/env python3
"""
Convert between PyTorch TorchScript (.pt) and ONNX (.onnx) formats
Automatically detects input format and converts to the other format
"""

import torch
import sys
import os
import numpy as np


def detect_input_shape_from_pt(pt_path):
    """
    Auto-detect input shape from PyTorch TorchScript model
    Reads from model graph or infers from first layer parameters

    Args:
        pt_path: Path to .pt file

    Returns:
        tuple: (input_size, output_size)
    """
    print("Reading input/output dimensions from PT model...")

    model = torch.jit.load(pt_path)
    model.eval()

    # Method 1: Try to read from graph type information
    try:
        graph = model.graph
        inputs = list(graph.inputs())

        if len(inputs) >= 2:
            input_node = inputs[1]  # First actual input (after self)

            if hasattr(input_node, 'type') and hasattr(input_node.type(), 'sizes'):
                sizes = input_node.type().sizes()
                if sizes is not None and len(sizes) >= 2:
                    input_size = sizes[-1]  # Last dimension

                    # Get output size by running a test
                    dummy_input = torch.randn(1, input_size)
                    with torch.no_grad():
                        output = model(dummy_input)
                    output_size = output.shape[-1]

                    print(f"✓ Detected from graph: input={input_size}, output={output_size}")
                    return int(input_size), int(output_size)
    except Exception as e:
        print(f"  Graph reading failed: {e}")

    # Method 2: Infer from first layer parameters
    print("  Inferring from model parameters...")
    try:
        params = dict(model.named_parameters())

        # Look for first linear layer weights
        for name, param in params.items():
            if 'weight' in name and len(param.shape) == 2:
                # Linear layer weight shape is [out_features, in_features]
                input_size = param.shape[1]

                # Verify by running a test
                try:
                    dummy_input = torch.randn(1, input_size)
                    with torch.no_grad():
                        output = model(dummy_input)
                    output_size = output.shape[-1]

                    print(f"✓ Detected from parameters: input={input_size}, output={output_size}")
                    return int(input_size), int(output_size)
                except:
                    continue
    except Exception as e:
        print(f"  Parameter inference failed: {e}")

    raise ValueError(
        "Could not auto-detect input dimensions from PT model. "
        "The model may have an unsupported structure."
    )


def detect_shape_from_onnx(onnx_path):
    """
    Auto-detect input/output shape from ONNX model

    Args:
        onnx_path: Path to .onnx file

    Returns:
        tuple: (input_size, output_size)
    """
    import onnx

    print("Reading input/output dimensions from ONNX model...")

    model = onnx.load(onnx_path)

    # Get input shape
    input_shape = model.graph.input[0].type.tensor_type.shape.dim
    input_size = int(input_shape[-1].dim_value)

    # Get output shape
    output_shape = model.graph.output[0].type.tensor_type.shape.dim
    output_size = int(output_shape[-1].dim_value)

    print(f"✓ Detected input size: {input_size}, output size: {output_size}")

    return input_size, output_size


def convert_pt_to_onnx(pt_path, onnx_path):
    """
    Convert TorchScript .pt model to ONNX format

    Args:
        pt_path: Path to input .pt file
        onnx_path: Path to output .onnx file
    """
    print(f"\nLoading PyTorch model: {pt_path}")

    # Auto-detect input/output size
    input_size, output_size = detect_input_shape_from_pt(pt_path)

    # Load TorchScript model
    model = torch.jit.load(pt_path)
    model.eval()

    # Create dummy input
    dummy_input = torch.randn(1, input_size)

    print(f"Input shape: {dummy_input.shape}")
    print(f"Converting to ONNX format...")

    # Export to ONNX with fixed batch size (no dynamic axes)
    torch.onnx.export(
        model,                      # model
        dummy_input,                # model input
        onnx_path,                  # output path
        export_params=True,         # store trained parameters
        opset_version=11,           # ONNX opset version
        do_constant_folding=True,   # constant folding optimization
        input_names=['input'],      # input names
        output_names=['output']     # output names
    )

    print(f"✓ Successfully converted to: {onnx_path}")

    # Verify the conversion
    print("\nVerifying conversion...")
    import onnxruntime as ort

    ort_session = ort.InferenceSession(onnx_path)

    # Test with the same dummy input
    ort_inputs = {ort_session.get_inputs()[0].name: dummy_input.numpy()}
    ort_outputs = ort_session.run(None, ort_inputs)

    # Compare outputs
    with torch.no_grad():
        pt_output = model(dummy_input).numpy()

    max_diff = abs(pt_output - ort_outputs[0]).max()
    print(f"Max difference between PyTorch and ONNX outputs: {max_diff}")

    if max_diff < 1e-5:
        print("✓ Conversion verified successfully!")
    else:
        print(f"⚠ Warning: Large difference detected ({max_diff})")

    return True


def convert_onnx_to_pt(onnx_path, pt_path):
    """
    Convert ONNX model to PyTorch TorchScript format
    Tries to preserve model structure as much as possible

    Args:
        onnx_path: Path to input .onnx file
        pt_path: Path to output .pt file
    """
    print(f"\nLoading ONNX model: {onnx_path}")

    try:
        import onnx
        from onnx2torch import convert
    except ImportError as e:
        print(f"\n✗ Error: Required library not found")
        print("Please install: pip install onnx2torch")
        raise ImportError("onnx2torch is required for ONNX to PT conversion")

    # Load ONNX model
    onnx_model = onnx.load(onnx_path)

    # Get input/output shapes
    input_size, output_size = detect_shape_from_onnx(onnx_path)

    # Create dummy input
    dummy_input = torch.randn(1, input_size)

    print("Converting to PyTorch format...")
    pytorch_model = convert(onnx_model)
    pytorch_model.eval()

    # Test the converted model
    print("Testing converted model...")
    with torch.no_grad():
        test_output = pytorch_model(dummy_input)

    # Try to use torch.jit.script first to preserve structure better
    print("Attempting to script model (preserves structure)...")
    try:
        scripted_model = torch.jit.script(pytorch_model)
        scripted_model.save(pt_path)
        print(f"✓ Successfully scripted and saved to: {pt_path}")

        # Verify
        verify_conversion(onnx_path, pt_path, dummy_input)

        print("\n⚠ Note: Model structure from ONNX conversion may differ from original")
        print("  Recommendation: Keep original .pt file if available")
        return True

    except Exception as e:
        print(f"  Scripting failed: {e}")
        print("  Falling back to tracing (will lose module structure)...")

    # Fallback to tracing
    print("Tracing model with TorchScript...")
    with torch.no_grad():
        traced_model = torch.jit.trace(
            pytorch_model,
            example_inputs=dummy_input,
            strict=False
        )

    # Save as TorchScript
    traced_model.save(pt_path)
    print(f"✓ Successfully converted to: {pt_path}")

    print("\n⚠ Warning: Model structure information is lost during conversion")
    print("  Original PT model had named modules (actor, normalizer, etc.)")
    print("  Converted model only contains computation graph")
    print("  Recommendation: Keep original .pt file if available")

    # Verify
    verify_conversion(onnx_path, pt_path, dummy_input)
    return True


def verify_conversion(onnx_path, pt_path, dummy_input):
    """
    Verify the conversion by comparing outputs
    """
    print("\nVerifying conversion...")
    import onnxruntime as ort

    # Load ONNX for comparison
    ort_session = ort.InferenceSession(onnx_path)
    ort_inputs = {ort_session.get_inputs()[0].name: dummy_input.numpy()}
    ort_outputs = ort_session.run(None, ort_inputs)

    # Load converted PyTorch model
    loaded_pt_model = torch.jit.load(pt_path)
    loaded_pt_model.eval()

    with torch.no_grad():
        pt_output = loaded_pt_model(dummy_input).numpy()

    max_diff = abs(pt_output - ort_outputs[0]).max()
    print(f"Max difference between ONNX and PyTorch outputs: {max_diff}")

    if max_diff < 1e-5:
        print("✓ Conversion verified successfully!")
    else:
        print(f"⚠ Warning: Large difference detected ({max_diff})")


def convert_model(input_path):
    """
    Auto-detect file format and convert to the other format

    Args:
        input_path: Path to input file (.pt or .onnx)
    """
    if not os.path.exists(input_path):
        print(f"Error: File not found: {input_path}")
        sys.exit(1)

    # Detect file type
    file_ext = os.path.splitext(input_path)[1].lower()

    if file_ext == '.pt':
        # Convert PT to ONNX
        output_path = input_path.replace('.pt', '.onnx')
        print("=" * 60)
        print("Mode: PyTorch (.pt) → ONNX (.onnx)")
        print("=" * 60)
        convert_pt_to_onnx(input_path, output_path)

    elif file_ext == '.onnx':
        # Convert ONNX to PT
        output_path = input_path.replace('.onnx', '.pt')
        print("=" * 60)
        print("Mode: ONNX (.onnx) → PyTorch (.pt)")
        print("=" * 60)
        print("\nNote: Requires onnx2torch library")
        print("If not installed, run: pip install onnx2torch")
        print()
        convert_onnx_to_pt(input_path, output_path)

    else:
        print(f"Error: Unsupported file format: {file_ext}")
        print("Supported formats: .pt, .onnx")
        sys.exit(1)

    print("\n" + "=" * 60)
    print(f"✓ Conversion completed!")
    print(f"Input file:  {input_path}")
    print(f"Output file: {output_path}")
    print("=" * 60)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python convert_policy.py <model_file_path>")
        print()
        print("Examples:")
        print("  python convert_policy.py policy/go2/himloco/himloco.pt")
        print()
        print("Supported formats:")
        print("  .pt   → auto convert to .onnx")
        print("  .onnx → auto convert to .pt")
        sys.exit(1)

    input_path = sys.argv[1]

    try:
        convert_model(input_path)
    except Exception as e:
        print(f"\nError: Exception occurred during conversion: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
