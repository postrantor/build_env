# ---
# date: 2025-01-11
# author: postrantor
# description: format code by clang-format
# ---

# format-target-folder-by-clang-format
#
# 功能描述：
#   格式化指定文件夹中的 C++ 源文件（`.h`, `.cpp`, `.hpp`），使用指定的 `.clang-format` 配置文件。
#
# 参数：
#   $1: folder - 需要格式化的目标文件夹路径。
#   $2: format_file - `.clang-format` 配置文件的路径。
#
# 使用示例：
#   format-target-folder-by-clang-format "src/quadruped/model-predictive-control/" ".clang-format"
#
# 注意事项：
#   1. 确保 `clang-format` 已安装并可用。
#   2. 确保 `.clang-format` 配置文件存在且格式正确。
#   3. 该函数会递归查找目标文件夹中的所有 `.h`, `.cpp`, `.hpp` 文件，并对其进行格式化。
#
# Bash script to automatically format LCM source code (currently C and C++).
# Requires `clang-format` utilily, which is part of the LLVM project. More
# information can be found here: https://clang.llvm.org/docs/ClangFormat.html
#
# To install `clang-format` on Ubuntu do:
#
#     $ sudo apt install clang-format
#
function format-target-folder-by-clang-format() {
  local folder=$1
  local format_file=$2
  local format_command="clang-format -i -style=file:$format_file"
  echo "running: $format_command"
  find "$folder" -type f \( -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) | xargs -I {} $format_command {}
}

# format-packages
#
# 功能描述：
#   在指定的搜索路径下查找所有 `package.xml` 文件，检查其内容是否包含指定的 `<name>${package_name}</name>` 标签。
#   如果找到匹配的标签，并且目标文件夹中不存在 `COLCON_IGNORE` 文件，则对该文件夹中的 C++ 源文件进行格式化。
#
# 参数：
#   $@: package_names - 需要查找的包名称列表（可变参数）。
#   search_path: 搜索 `package.xml` 文件的根路径，默认为 `${QUAD_WORKDIR}/src`。
#   format_file: `.clang-format` 配置文件的路径，默认为 `${QUAD_WORKDIR}/.clang-format`。
#
# 使用示例：
#   format-packages "package1" "package2"
#
# 注意事项：
#   1. 确保 `QUAD_WORKDIR` 环境变量已正确设置。
#   2. 确保 `.clang-format` 配置文件存在且格式正确。
#   3. 如果目标文件夹中存在 `COLCON_IGNORE` 文件，则跳过该文件夹的格式化操作。
#   4. 该函数会递归查找 `search_path` 下的所有 `package.xml` 文件。
#
export clang_format_path="${QUAD_WORKDIR}"/.clang-format
function format-packages() {
  local package_names=("$@")  # 从参数中获取 package_name 列表
  local search_path="${QUAD_WORKDIR}/src"  # 指定搜索路径
  local format_file=$clang_format_path

  # 查找所有 package.xml 文件
  find "$search_path" -name "package.xml" | while read -r xml_file; do
    for package_name in "${package_names[@]}"; do
      if [[ -n "$xml_file" ]]; then
        if grep -q "<name>${package_name}</name>" "$xml_file"; then
          echo "Found <name>${package_name}</name> in $xml_file"
          # 检查是否存在 COLCON_IGNORE 文件
          local package_dir=$(dirname "$xml_file")
          if [[ -f "$package_dir/COLCON_IGNORE" ]]; then
            echo "Skipping $package_dir due to COLCON_IGNORE"
          else
            format-target-folder-by-clang-format "$package_dir" "$format_file"
          fi
          break
        fi
      else
        echo "No ${package_name}.xml found in $search_path"
      fi
    done
  done
}
