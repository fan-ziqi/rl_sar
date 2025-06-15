# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import re
import shutil
from pathlib import Path


def extract_stl_references(urdf_folder):
    """
    从URDF文件中提取所有引用的STL文件路径
    """
    stl_files = set()
    urdf_folder = Path(urdf_folder)

    # 遍历URDF文件夹中的所有.urdf文件
    for urdf_file in urdf_folder.glob('*.urdf'):
        with open(urdf_file, 'r', encoding='utf-8') as f:
            content = f.read()
            # 使用正则表达式查找所有mesh filename引用
            matches = re.findall(r'<mesh filename="package://(.*?\.stl)"', content, re.IGNORECASE)
            for match in matches:
                # 提取STL文件名（去掉前面的g1_description/meshes/等路径）
                stl_name = match.split('/')[-1]
                stl_files.add(stl_name)

    return stl_files


def copy_used_stl_files(stl_references, meshes_folder, output_folder):
    """
    将使用的STL文件复制到新文件夹
    """
    meshes_folder = Path(meshes_folder)
    output_folder = Path(output_folder)

    # 确保输出文件夹存在
    output_folder.mkdir(parents=True, exist_ok=True)

    copied_files = []
    missing_files = []

    for stl_name in stl_references:
        # 在meshes文件夹中查找STL文件
        found = False
        for stl_path in meshes_folder.rglob(stl_name):
            if stl_path.is_file():
                # 构建目标路径
                dest_path = output_folder / stl_name

                # 复制文件
                shutil.copy2(stl_path, dest_path)
                copied_files.append(str(dest_path))
                found = True
                break

        if not found:
            missing_files.append(stl_name)

    return copied_files, missing_files


def main():
    # 配置路径
    urdf_folder = 'urdf'  # URDF文件所在文件夹
    meshes_folder = 'meshes'  # 原始meshes文件夹
    output_folder = 'meshes_new'  # 输出文件夹

    print("正在扫描URDF文件中的STL引用...")
    stl_references = extract_stl_references(urdf_folder)

    if not stl_references:
        print("没有找到任何STL文件引用。")
        return

    print(f"找到 {len(stl_references)} 个STL文件引用:")
    for ref in sorted(stl_references):
        print(f"  - {ref}")

    print("\n正在复制使用的STL文件...")
    copied_files, missing_files = copy_used_stl_files(stl_references, meshes_folder, output_folder)

    print("\n复制完成:")
    print(f"成功复制 {len(copied_files)} 个文件到 {output_folder}")
    for copied in sorted(copied_files):
        print(f"  - {copied}")

    if missing_files:
        print("\n警告: 以下文件未找到:")
        for missing in sorted(missing_files):
            print(f"  - {missing}")

if __name__ == '__main__':
    main()