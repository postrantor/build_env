# ---
# date: 2026-08-21
# description: 文本转换辅助 (pandoc)。
# ---

# pandoc_markdown
#
# 功能描述：
#   用 pandoc 把文档转成 markdown+simple_tables。支持单文件或按后缀批量转换。
#
# 参数：
#   $1: file - 单个源文件 (遗留写法, 输出为 `<file>.md`)。
#   --path <path> - 输入文件或目录。
#   --type <suffix> - 源后缀 (如 rst)。
#   --output <path> - 输出目录 (默认与 --path 相同)。
#   --no-keep - 转换成功后删除源文件。
#
# 使用示例：
#   pandoc_markdown README.rst
#   pandoc_markdown --path ./ --type rst --output ./
#   pandoc_markdown --path ./ --type rst --output ./ --no-keep
#
# 注意事项：
#   1. 需要已安装 pandoc。
#   2. 批量模式必须同时提供 --path 与 --type。
#
pandoc_markdown() {
    local file=""
    local input_path=""
    local input_type=""
    local output_path=""
    local keep_source=true

    # Parse CLI options while preserving the legacy single-file argument form.
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --path)
                input_path="$2"
                shift 2
                ;;
            --type)
                input_type="${2#.}"
                shift 2
                ;;
            --output)
                output_path="$2"
                shift 2
                ;;
            --no-keep)
                keep_source=false
                shift
                ;;
            -h|--help)
                cat <<'EOF'
Usage:
  pandoc_markdown [--no-keep] <file>
  pandoc_markdown --path <path> --type <suffix> [--output <path>] [--no-keep]

Examples:
  pandoc_markdown README.rst
  pandoc_markdown --path ./ --type rst --output ./
  pandoc_markdown --path ./ --type rst --output ./ --no-keep
EOF
                return 0
                ;;
            *)
                if [[ -z "$file" ]]; then
                    file="$1"
                    shift
                else
                    echo "Unknown argument: $1" >&2
                    return 1
                fi
                ;;
        esac
    done

    # Convert one explicit file and optionally remove it after a successful run.
    if [[ -n "$file" ]]; then
        if pandoc \
            -i "$file" \
            -t markdown+simple_tables \
            --wrap=none \
            -s \
            -o "${file}.md" \
            --extract-media=.; then
            if [[ "$keep_source" == false ]]; then
                rm -f "$file"
            fi
        fi
        return
    fi

    # Directory/file batch mode requires both an input path and source suffix.
    if [[ -z "$input_path" || -z "$input_type" ]]; then
        echo "Usage: pandoc_markdown --path <path> --type <suffix> [--output <path>]" >&2
        return 1
    fi

    if [[ ! -e "$input_path" ]]; then
        echo "Path does not exist: $input_path" >&2
        return 1
    fi

    # Normalize paths so relative output paths stay stable with or without trailing slashes.
    output_path="${output_path:-$input_path}"
    [[ "$input_path" != "/" ]] && input_path="${input_path%/}"
    [[ "$output_path" != "/" ]] && output_path="${output_path%/}"
    mkdir -p "$output_path"

    # Convert a single --path file into the requested output directory.
    if [[ -f "$input_path" ]]; then
        local output_file="$output_path/$(basename "${input_path%.*}").md"
        mkdir -p "$(dirname "$output_file")"
        if pandoc \
            -i "$input_path" \
            -t markdown+simple_tables \
            --wrap=none \
            -s \
            -o "$output_file" \
            --extract-media="$(dirname "$output_file")"; then
            if [[ "$keep_source" == false ]]; then
                rm -f "$input_path"
            fi
        fi
        return
    fi

    local source_file=""
    local relative_file=""
    local output_file=""

    # Convert all matching files under the input directory, preserving subdirectories.
    while IFS= read -r -d '' source_file; do
        relative_file="${source_file#"$input_path"/}"
        output_file="$output_path/${relative_file%.*}.md"
        mkdir -p "$(dirname "$output_file")"

        pandoc \
            -i "$source_file" \
            -t markdown+simple_tables \
            --wrap=none \
            -s \
            -o "$output_file" \
            --extract-media="$(dirname "$output_file")" &&
            if [[ "$keep_source" == false ]]; then
                rm -f "$source_file"
            fi
    done < <(find "$input_path" -type f -name "*.${input_type}" -print0)
}
