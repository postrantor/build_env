# ---
# date: 2026-08-21
# description: 媒体工具。media-compress 批量转码压缩 (ffmpeg); 调度入口 media / setup-media。
# ---

# media-compress
#
# 功能描述：
#   基于 ffmpeg 批量转码压缩视频。默认 H.265、CRF/CQ 28, 输出到源目录下 compressed/。
#
# 参数：
#   完整选项见 `media --help` 与 docs/usage/media.md。
#   $@: [选项] <文件或目录>...
#
# 使用示例：
#   media-compress --info .
#   media-compress -s 720p -q 30 --no-audio .
#
# 注意事项：
#   1. 需要 ffmpeg / ffprobe, bash >= 4.3。
#   2. 函数体在子 shell 中运行, 不会把 set -e 泄漏到当前 shell。
#
media-compress() {
  (
    set -euo pipefail

if ((BASH_VERSINFO[0] < 4 || (BASH_VERSINFO[0] == 4 && BASH_VERSINFO[1] < 3))); then
    echo "需要 bash 4.3 或更高版本" >&2
    exit 1
fi

SCRIPT_NAME=media

# ---------------------------------------------------------------- 默认参数 ---
CODEC=h265             # h264 | h265 | vp9 | av1
QUALITY=28             # CRF / CQ，越小质量越高
BITRATE=""             # 指定后改用码率模式，QUALITY 失效
MAXRATE=""
BUFSIZE=""
TWOPASS=0
RESOLUTION=""          # 720p / 1280x720 / -2:720 / 50%
FPS=""
SPEED=1                # 播放倍速，>1 加速，<1 减速
SPEED_LABEL=0          # 是否在右下角叠加倍率文字
LABEL_SIZE=""          # 文字像素高度，留空按画面高度自适应
PRESET=""              # 留空则按编码器取默认
HW=auto                # auto | on | off
PIPELINE=auto          # auto | gpu | dec | cpu，见 --pipeline
GPU_ID=""              # 指定使用第几块 NVIDIA 卡
AQ=1                   # NVENC 空间自适应量化，同画质下更省体积
AUDIO_MODE=encode      # encode | copy | none
AUDIO_BITRATE=128k
PIX_FMT=""
CONTAINER=mp4
OUTDIR=""
SUFFIX=""
RECURSIVE=0
JOBS=1
FORCE=0
DRYRUN=0
INFO=0
PREVIEW=""
START=""
KEEP_MTIME=1
DELETE_SOURCE=0
ASSUME_YES=0
EXTS="mp4,mkv,mov,avi,m4v,ts,mts,flv,wmv,webm,mpg,mpeg,3gp"

declare -a INPUTS=()
declare -a FILES=()
declare -a RELS=()

# die
#
# 功能描述：
#   向 stderr 打印错误并 `exit 1`。
#
# 参数：
#   $@: message - 错误内容。
#
die() {
    echo "$SCRIPT_NAME: $*" >&2
    exit 1
}

# warn
#
# 功能描述：
#   向 stderr 打印警告, 不退出。
#
# 参数：
#   $@: message - 警告内容。
#
warn() { echo "$SCRIPT_NAME: $*" >&2; }

# usage
#
# 功能描述：
#   打印 `media` 命令行帮助到 stdout。
#
# 参数：
#   (无)
#
usage() {
    cat <<EOF
用法: $SCRIPT_NAME [选项] <文件或目录>...

对指定路径下的视频批量转码压缩，默认输出到源目录下的 compressed/ 子目录。

画质与码率:
  -q, --quality N       质量值 CRF/CQ，越小越清晰、文件越大（默认 $QUALITY）
                        参考: h265 用 24-30，h264 用 20-26
  -b, --bitrate RATE    目标平均码率，如 4M / 2500k（指定后忽略 --quality）
      --maxrate RATE    码率上限（配合 --bufsize 做限流）
      --bufsize RATE    码率控制缓冲区大小（默认为 maxrate 的 2 倍）
      --two-pass        码率模式下做两遍编码，码率控制更准（更慢）

画面:
  -s, --resolution SPEC 输出分辨率，仅缩小不放大。支持:
                        720p / 1080p / 480p / 2160p ... 按高度缩放
                        1280x720   限制在该框内，保持宽高比
                        -2:720     直接传给 ffmpeg scale 的写法
                        50%        按百分比缩放
      --fps N           输出帧率，如 30（默认保持原帧率）
      --pix-fmt FMT     像素格式，如 yuv420p（默认跟随源）

变速:
  -x, --speed N         播放倍速，如 4 表示 4 倍速播放，0.5 表示放慢一半
                        注意: 加速后帧率同比变高，建议配合 --fps 30 降帧
      --speed-label     在画面右下角叠加倍率标记，如 x4、x0.5
      --label-size N    叠加文字像素高度（默认按画面高度的 1/20 自适应）

编码器:
  -c, --codec NAME      h264 | h265 | vp9 | av1（默认 $CODEC）
  -p, --preset NAME     编码预设，CPU: ultrafast..veryslow；NVENC: p1..p7
      --hw MODE         硬件编码 NVENC: auto | on | off（默认 $HW）

NVIDIA 硬件加速:
      --pipeline MODE   解码与滤镜跑在哪（默认 $PIPELINE）
                        auto  能全程走 GPU 就走，否则回退 CPU
                        gpu   NVDEC 解码 + scale_cuda 缩放 + NVENC 编码，最快
                        dec   NVDEC 解码，滤镜仍在 CPU
                        cpu   全部用 CPU 解码和滤镜
      --hwdec           等价于 --pipeline dec
      --gpu N           指定使用第几块 NVIDIA 卡（默认交给驱动选）
      --no-aq           关闭 NVENC 空间自适应量化（默认开，同画质更省体积）

音频:
  -n, --no-audio        去除音频轨
      --audio-copy      直接复制音频轨，不重新编码
      --audio-bitrate R 音频码率（默认 $AUDIO_BITRATE，重编码时生效）

输入输出:
  -o, --outdir DIR      输出目录（默认 <源目录>/compressed）
      --suffix STR      输出文件名后缀，如 _720p
      --container EXT   输出容器 mp4 | mkv | mov | webm（默认 $CONTAINER）
  -e, --ext LIST        参与处理的扩展名，逗号分隔（默认 $EXTS）
  -R, --recursive       递归子目录
  -f, --force           输出文件已存在时覆盖（默认跳过）
      --no-keep-mtime   不把源文件的修改时间复制到输出（默认复制）
      --delete-source   转码成功且输出更小时删除源文件（危险，需确认）

调试与执行:
      --info            只列出视频基本信息（时长、起止时间、规格），不转码
  -j, --jobs N          并行任务数（默认 $JOBS；NVENC 建议 2-4）
      --preview SEC     只取源片前 SEC 秒，用来快速试参数
      --start TIME      从指定时间点开始，如 00:01:30
      --dry-run         只打印将要执行的 ffmpeg 命令
  -y, --yes             对所有确认提示回答 yes
  -h, --help            显示本帮助

示例:
  # 先看看有哪些视频、多长、什么时候录的
  $SCRIPT_NAME --info .

  # 用默认参数（h265 CQ 28）压缩当前目录下所有视频
  $SCRIPT_NAME .

  # 缩到 720p、CRF 30、去掉音频，输出到 /data/out
  $SCRIPT_NAME -s 720p -q 30 --no-audio -o /data/out ./稳定性测试

  # 限定 3M 码率、上限 4M，两遍编码，纯 CPU x265
  $SCRIPT_NAME -b 3M --maxrate 4M --two-pass --hw off video.mp4

  # 先用 60 秒预览确认画质，再跑全量
  $SCRIPT_NAME --preview 60 -s 720p -q 30 VID20260729110047.mp4

  # 8 倍速快放长录像，降到 30fps，右下角标注 x8，去掉音频
  $SCRIPT_NAME -x 8 --fps 30 --speed-label -n -s 720p .
EOF
}

# hsize
#
# 功能描述：
#   把字节数转成可读单位 (`B` / `KiB` / `MiB` / `GiB` / `TiB`)。
#
# 参数：
#   $1: bytes - 字节数 (默认 0)。
#
hsize() {
    awk -v b="${1:-0}" 'BEGIN{
        split("B KiB MiB GiB TiB",u," "); i=1
        while (b >= 1024 && i < 5) { b /= 1024; i++ }
        if (i == 1) printf "%d %s", b, u[i]; else printf "%.2f %s", b, u[i]
    }'
}

# hms
#
# 功能描述：
#   把秒数格式化为 `1h02m03s`。
#
# 参数：
#   $1: seconds - 秒数 (默认 0)。
#
hms() {
    local t=${1:-0}
    printf '%dh%02dm%02ds' $((t / 3600)) $((t % 3600 / 60)) $((t % 60))
}

# filesize
#
# 功能描述：
#   返回文件大小 (字节)。`-L` 跟随符号链接, 拿到目标文件而非链接本身。
#
# 参数：
#   $1: path - 文件路径。
#
filesize() { stat -Lc %s -- "$1"; }

# has_encoder
#
# 功能描述：
#   检查 ffmpeg 是否提供指定编码器。
#
# 参数：
#   $1: name - 编码器名, 如 `libx265`。
#
# 注意事项：
#   1. 用 awk 读完整个列表再判定; `grep -q` 会提前退出导致上游 SIGPIPE, 在 pipefail 下误判。
#
has_encoder() {
    ffmpeg -hide_banner -loglevel quiet -encoders 2>/dev/null |
        awk -v n="$1" '$2 == n { found = 1 } END { exit !found }'
}

# has_filter
#
# 功能描述：
#   检查 ffmpeg 是否提供指定滤镜。
#
# 参数：
#   $1: name - 滤镜名, 如 `scale_cuda`。
#
has_filter() {
    ffmpeg -hide_banner -loglevel quiet -filters 2>/dev/null |
        awk -v n="$1" '$2 == n { found = 1 } END { exit !found }'
}

# has_hwaccel
#
# 功能描述：
#   检查 ffmpeg 是否提供指定硬件加速方式。
#
# 参数：
#   $1: name - 加速方式名, 如 `cuda`。
#
# 注意事项：
#   1. `ffmpeg -hwaccels` 首行是表头, 会跳过。
#
has_hwaccel() {
    ffmpeg -hide_banner -hwaccels 2>/dev/null |
        awk -v n="$1" 'NR > 1 && $1 == n { found = 1 } END { exit !found }'
}

# nvdec_supports
#
# 功能描述：
#   判断源视频编码是否有对应的 NVDEC (`*_cuvid`) 解码器。
#
# 参数：
#   $1: codec - ffprobe 的 `codec_name`, 如 `h264` / `hevc`。
#
nvdec_supports() {
    local dec
    case $1 in
    h264) dec=h264_cuvid ;;
    hevc) dec=hevc_cuvid ;;
    vp9) dec=vp9_cuvid ;;
    vp8) dec=vp8_cuvid ;;
    av1) dec=av1_cuvid ;;
    mpeg1video) dec=mpeg1_cuvid ;;
    mpeg2video) dec=mpeg2_cuvid ;;
    mpeg4) dec=mpeg4_cuvid ;;
    vc1) dec=vc1_cuvid ;;
    mjpeg) dec=mjpeg_cuvid ;;
    *) return 1 ;;
    esac
    ffmpeg -hide_banner -loglevel quiet -decoders 2>/dev/null |
        awk -v n="$dec" '$2 == n { found = 1 } END { exit !found }'
}

# video_has_rotation
#
# 功能描述：
#   判断画面是否带非 0 旋转元数据 (手机竖拍常见)。
#
# 参数：
#   $1: src - 视频文件路径。
#
# 注意事项：
#   1. ffmpeg 默认 autorotate 会插入 transpose/hflip, 它们只吃内存帧, 与 `-hwaccel_output_format cuda` 的全 GPU 管线不兼容。
#
video_has_rotation() {
    local src=$1 rot
    rot=$(ffprobe -v error -select_streams v:0 -show_entries stream_tags=rotate \
        -of default=nw=1:nk=1 -- "$src" 2>/dev/null || true)
    if [[ ! $rot =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
        # 部分容器只有 Display Matrix，没有 rotate 标签
        rot=$(ffprobe -v quiet -select_streams v:0 -show_streams -of compact=p=0 -- "$src" 2>/dev/null |
            grep -oE '(^|\|)rotation=-?[0-9.]+' | head -1 | cut -d= -f2 || true)
    fi
    [[ $rot =~ ^-?[0-9]+([.][0-9]+)?$ ]] || return 1
    awk -v r="$rot" 'BEGIN{ r = r % 360; if (r < 0) r += 360; exit !(r > 0.001 && r < 359.999) }'
}

# find_font
#
# 功能描述：
#   查找可供 `drawtext` 使用的字体文件。
#
# 参数：
#   (无)
#
# 注意事项：
#   1. 倍率标记只含 ASCII, 任意西文字体都够用。
#
find_font() {
    local f
    if command -v fc-match >/dev/null 2>&1; then
        f=$(fc-match -f '%{file}' 'sans-serif:bold' 2>/dev/null || true)
        [[ -n $f && -f $f ]] && { printf '%s' "$f"; return 0; }
    fi
    for f in /usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf \
        /usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf \
        /usr/share/fonts/truetype/freefont/FreeSansBold.ttf; do
        [[ -f $f ]] && { printf '%s' "$f"; return 0; }
    done
    return 1
}

# encoder_has_opt
#
# 功能描述：
#   检查编码器 help 中是否包含指定选项。
#
# 参数：
#   $1: encoder - 编码器名。
#   $2: option - 选项名 (不含前导 `-`)。
#
encoder_has_opt() {
    local help
    help=$(ffmpeg -hide_banner -loglevel quiet -h "encoder=$1" 2>/dev/null || true)
    [[ $help == *" -$2 "* ]]
}

# encoder_has_preset
#
# 功能描述：
#   检查编码器是否支持某个 preset 取值。
#
# 参数：
#   $1: encoder - 编码器名。
#   $2: preset - preset 名。
#
encoder_has_preset() {
    local help
    help=$(ffmpeg -hide_banner -loglevel quiet -h "encoder=$1" 2>/dev/null || true)
    grep -qE "^[[:space:]]+$2[[:space:]]" <<<"$help"
}

# nvenc_works
#
# 功能描述：
#   真机跑一帧确认 NVENC 可用 (有编译支持但驱动缺失时会失败)。
#
# 参数：
#   $1: encoder - NVENC 编码器名, 如 `hevc_nvenc`。
#
nvenc_works() {
    ffmpeg -hide_banner -loglevel error -nostdin \
        -f lavfi -i "testsrc2=size=320x180:rate=30" -frames:v 3 \
        -c:v "$1" -f null - >/dev/null 2>&1
}

# ---------------------------------------------------------------- 参数解析 ---
# 先把 --opt=value 拆成 --opt value
declare -a ARGV=()
for arg in "$@"; do
    if [[ $arg == --*=* ]]; then
        ARGV+=("${arg%%=*}" "${arg#*=}")
    else
        ARGV+=("$arg")
    fi
done
set -- ${ARGV[@]+"${ARGV[@]}"}

while (($#)); do
    case $1 in
    -q | --quality | --crf | --cq) QUALITY=$2; shift 2 ;;
    -b | --bitrate) BITRATE=$2; shift 2 ;;
    --maxrate) MAXRATE=$2; shift 2 ;;
    --bufsize) BUFSIZE=$2; shift 2 ;;
    --two-pass | --twopass) TWOPASS=1; shift ;;
    -s | --resolution | --size | --scale) RESOLUTION=$2; shift 2 ;;
    --fps | --framerate) FPS=$2; shift 2 ;;
    --pix-fmt) PIX_FMT=$2; shift 2 ;;
    -x | --speed) SPEED=${2%[xX]}; shift 2 ;;
    --speed-label | --label) SPEED_LABEL=1; shift ;;
    --label-size) LABEL_SIZE=$2; shift 2 ;;
    -c | --codec) CODEC=${2,,}; shift 2 ;;
    -p | --preset) PRESET=$2; shift 2 ;;
    --hw) HW=${2,,}; shift 2 ;;
    --pipeline) PIPELINE=${2,,}; shift 2 ;;
    --hwdec) PIPELINE=dec; shift ;;
    --gpu) GPU_ID=$2; shift 2 ;;
    --no-aq) AQ=0; shift ;;
    -n | --no-audio | --an) AUDIO_MODE=none; shift ;;
    --audio-copy) AUDIO_MODE=copy; shift ;;
    --audio-bitrate) AUDIO_BITRATE=$2; shift 2 ;;
    -o | --outdir | --output) OUTDIR=$2; shift 2 ;;
    --suffix) SUFFIX=$2; shift 2 ;;
    --container | --format) CONTAINER=${2#.}; shift 2 ;;
    -e | --ext) EXTS=$2; shift 2 ;;
    -R | -r | --recursive) RECURSIVE=1; shift ;;
    -j | --jobs) JOBS=$2; shift 2 ;;
    -f | --force | --overwrite) FORCE=1; shift ;;
    --no-keep-mtime) KEEP_MTIME=0; shift ;;
    --delete-source) DELETE_SOURCE=1; shift ;;
    --preview) PREVIEW=$2; shift 2 ;;
    --start | --ss) START=$2; shift 2 ;;
    --dry-run | -N) DRYRUN=1; shift ;;
    --info | --list) INFO=1; shift ;;
    -y | --yes) ASSUME_YES=1; shift ;;
    -h | --help) usage; exit 0 ;;
    --) shift; while (($#)); do INPUTS+=("$1"); shift; done ;;
    -*) die "未知选项: $1（用 --help 查看用法）" ;;
    *) INPUTS+=("$1"); shift ;;
    esac
done

command -v ffmpeg >/dev/null || die "未找到 ffmpeg，请先安装：sudo apt install ffmpeg"
command -v ffprobe >/dev/null || die "未找到 ffprobe，请先安装：sudo apt install ffmpeg"

((${#INPUTS[@]})) || die "请指定至少一个文件或目录（用 --help 查看用法）"
[[ $JOBS =~ ^[0-9]+$ ]] && ((JOBS >= 1)) || die "--jobs 需要是正整数: $JOBS"
[[ $HW == auto || $HW == on || $HW == off ]] || die "--hw 只能是 auto|on|off: $HW"
[[ $PIPELINE =~ ^(auto|gpu|dec|cpu)$ ]] || die "--pipeline 只能是 auto|gpu|dec|cpu: $PIPELINE"
[[ -z $GPU_ID || $GPU_ID =~ ^[0-9]+$ ]] || die "--gpu 需要是非负整数: $GPU_ID"
[[ -z $BITRATE || $QUALITY == 28 ]] || warn "已指定 --bitrate，--quality $QUALITY 将被忽略"
((TWOPASS == 0)) || [[ -n $BITRATE ]] || die "--two-pass 需要配合 --bitrate 使用"

[[ $SPEED =~ ^[0-9]+(\.[0-9]+)?$ ]] && awk -v s="$SPEED" 'BEGIN{exit !(s > 0)}' ||
    die "--speed 需要是大于 0 的数字: $SPEED"
[[ -z $LABEL_SIZE || $LABEL_SIZE =~ ^[0-9]+$ ]] || die "--label-size 需要是正整数: $LABEL_SIZE"

SPEED_ON=0
if awk -v s="$SPEED" 'BEGIN{exit (s == 1)}'; then SPEED_ON=1; fi

if ((SPEED_ON)) && [[ $AUDIO_MODE == copy ]]; then
    warn "变速时无法直接复制音频，已改为重新编码"
    AUDIO_MODE=encode
fi

# 预览产物单独命名，避免和后续全量输出撞名被当成"已存在"跳过
[[ -n $PREVIEW && -z $SUFFIX ]] && SUFFIX=_preview

# collect_files
#
# 功能描述：
#   按扩展名收集待处理视频, 填入全局 `FILES` / `RELS`; 默认跳过 `compressed/` 输出目录。
#
# 参数：
#   (无; 使用全局 `INPUTS` / `EXTS` / `RECURSIVE` / `OUTDIR`)
#
collect_files() {
    local input root f abs rel
    local ext_re
    ext_re=$(echo "$EXTS" | tr ',' '|' | tr -d ' ')
    [[ -n $ext_re ]] || die "--ext 不能为空"

    local outdir_abs=""
    [[ -n $OUTDIR ]] && outdir_abs=$(readlink -m -- "$OUTDIR")

    for input in "${INPUTS[@]}"; do
        if [[ -f $input ]]; then
            FILES+=("$input")
            RELS+=("$(basename -- "$input")")
        elif [[ -d $input ]]; then
            root=${input%/}
            [[ -z $root ]] && root=/
            # 显式指定 compressed 目录时说明就是想处理里面的文件，不再跳过
            local skip_compressed=1
            [[ $root == */compressed || $root == */compressed/* ]] && skip_compressed=0
            local depth_opt=()
            ((RECURSIVE)) || depth_opt=(-maxdepth 1)
            while IFS= read -r -d '' f; do
                # 跳过自己的输出目录，避免重复压缩
                abs=$(readlink -m -- "$f")
                [[ -n $outdir_abs && $abs == "$outdir_abs"/* ]] && continue
                ((skip_compressed)) && [[ -z $outdir_abs && $f == */compressed/* ]] && continue
                FILES+=("$f")
                rel=${f#"$root"/}
                RELS+=("$rel")
            done < <(find "$root" ${depth_opt[@]+"${depth_opt[@]}"} -type f \
                -regextype posix-extended -iregex ".*\.($ext_re)" -print0 | sort -z)
        else
            die "路径不存在: $input"
        fi
    done

    ((${#FILES[@]})) || die "在指定路径下没找到视频文件（扩展名: $EXTS）"
}

# parse_name_time
#
# 功能描述：
#   从文件名解析拍摄开始时间 (如 `VID20260729110047` / `VID_20260729_110047`), 输出 Unix 时间戳。
#
# 参数：
#   $1: path - 文件路径或文件名。
#
parse_name_time() {
    local name=${1##*/}
    [[ $name =~ (20[0-9][0-9])([01][0-9])([0-3][0-9])[_-]?([0-2][0-9])([0-5][0-9])([0-5][0-9]) ]] || return 1
    date -d "${BASH_REMATCH[1]}-${BASH_REMATCH[2]}-${BASH_REMATCH[3]} ${BASH_REMATCH[4]}:${BASH_REMATCH[5]}:${BASH_REMATCH[6]}" \
        +%s 2>/dev/null
}

# fmt_time
#
# 功能描述：
#   把 Unix 时间戳格式化为 `YYYY-MM-DD HH:MM:SS`。
#
# 参数：
#   $1: epoch - Unix 时间戳。
#
fmt_time() { date -d "@$1" '+%Y-%m-%d %H:%M:%S'; }

# show_info
#
# 功能描述：
#   输出每个文件的时长、起止时间、规格, 不做任何转码。
#
# 参数：
#   (无; 使用全局 `FILES`)
#
show_info() {
    local -a names=() durs=() starts=() ends=() sizes=() specs=() origins=()
    local i src dur size w h fps codec ct k v
    local namew=4 total_dur=0 total_size=0

    for i in "${!FILES[@]}"; do
        src=${FILES[i]}
        dur="" size="" w="" h="" fps="" codec="" ct=""
        while IFS='=' read -r k v; do
            case $k in
            duration) dur=$v ;;
            size) size=$v ;;
            width) w=$v ;;
            height) h=$v ;;
            r_frame_rate) fps=$v ;;
            codec_name) codec=$v ;;
            TAG:creation_time | creation_time) ct=$v ;;
            esac
        done < <(ffprobe -v error -select_streams v:0 \
            -show_entries 'format=duration,size:format_tags=creation_time:stream=width,height,r_frame_rate,codec_name' \
            -of default=nw=1 -- "$src" 2>/dev/null)

        local dsec=0
        [[ ${dur%%.*} =~ ^[0-9]+$ ]] && dsec=${dur%%.*}
        [[ -n $size ]] || size=$(filesize "$src")

        # 起止时间来源按可靠性排序：文件名 > 容器元数据 > 文件修改时间
        local start="" end="" origin="-" mt cte
        mt=$(stat -Lc %Y -- "$src")
        if start=$(parse_name_time "$src"); then
            end=$((start + dsec))
            origin=name
        elif [[ -n $ct ]] && cte=$(date -d "$ct" +%s 2>/dev/null); then
            # creation_time 有的设备写录制开始、有的写录制结束，靠文件修改时间判断是哪种
            if ((cte > mt - 120 && cte < mt + 120)); then
                end=$cte
                start=$((end - dsec))
            else
                start=$cte
                end=$((start + dsec))
            fi
            origin=meta
        else
            end=$mt
            start=$((end - dsec))
            origin=mtime
        fi

        names+=("${RELS[i]}")
        ((${#RELS[i]} > namew)) && namew=${#RELS[i]}
        durs+=("$(hms "$dsec")")
        starts+=("$(fmt_time "$start")")
        ends+=("$(fmt_time "$end")")
        sizes+=("$(hsize "$size")")
        origins+=("$origin")
        specs+=("${w:-?}x${h:-?} $(awk -v r="${fps:-0}" 'BEGIN{split(r,a,"/"); printf "%g", (a[2] ? a[1]/a[2] : 0)}')fps ${codec:-?}")
        total_dur=$((total_dur + dsec))
        total_size=$((total_size + size))
    done

    printf "%-${namew}s  %-9s  %-19s  %-19s  %10s  %-8s  %s\n" \
        FILE DURATION START END SIZE ORIGIN SPEC
    for i in "${!names[@]}"; do
        printf "%-${namew}s  %-9s  %-19s  %-19s  %10s  %-8s  %s\n" \
            "${names[i]}" "${durs[i]}" "${starts[i]}" "${ends[i]}" "${sizes[i]}" "${origins[i]}" "${specs[i]}"
    done
    printf '\n共 %d 个文件，总时长 %s，总大小 %s\n' \
        "${#names[@]}" "$(hms "$total_dur")" "$(hsize "$total_size")"
    printf '起止时间来源 ORIGIN: name=文件名时间戳  meta=容器 creation_time  mtime=文件修改时间\n'
}

if ((INFO)); then
    collect_files
    show_info
    exit 0
fi

# ---------------------------------------------------------------- 编码器选择 ---
SW_ENC=""
HW_ENC=""
case $CODEC in
h264 | avc | x264) CODEC=h264; SW_ENC=libx264; HW_ENC=h264_nvenc ;;
h265 | hevc | x265) CODEC=h265; SW_ENC=libx265; HW_ENC=hevc_nvenc ;;
vp9) SW_ENC=libvpx-vp9; HW_ENC="" ;;
av1) SW_ENC=$(has_encoder libsvtav1 && echo libsvtav1 || echo libaom-av1); HW_ENC=av1_nvenc ;;
*) die "不支持的编码格式: $CODEC（可选 h264|h265|vp9|av1）" ;;
esac

ENCODER=""
if [[ $HW != off && -n $HW_ENC ]] && has_encoder "$HW_ENC" && nvenc_works "$HW_ENC"; then
    ENCODER=$HW_ENC
elif [[ $HW == on ]]; then
    die "硬件编码不可用: ${HW_ENC:-无对应 NVENC 编码器}（改用 --hw off 走 CPU）"
else
    has_encoder "$SW_ENC" || die "ffmpeg 不支持编码器 $SW_ENC"
    ENCODER=$SW_ENC
fi

IS_NVENC=0
[[ $ENCODER == *_nvenc ]] && IS_NVENC=1

if [[ -z $PRESET ]]; then
    if ((IS_NVENC)); then
        if encoder_has_preset "$ENCODER" p5; then PRESET=p5; else PRESET=slow; fi
    else
        case $ENCODER in
        libx264 | libx265) PRESET=medium ;;
        libsvtav1) PRESET=8 ;;
        esac
    fi
fi

AQ_OK=0
((IS_NVENC && AQ)) && encoder_has_opt "$ENCODER" spatial-aq && AQ_OK=1

# ------------------------------------------------------------ 硬件管线选择 ---
CUDA_OK=0
has_hwaccel cuda && CUDA_OK=1

# 全 GPU 管线要求解码、缩放、编码三段都在显存里完成
GPU_PIPE_OK=0
((CUDA_OK && IS_NVENC)) && has_filter scale_cuda && GPU_PIPE_OK=1

case $PIPELINE in
auto) if ((GPU_PIPE_OK)); then PIPELINE=gpu; else PIPELINE=cpu; fi ;;
gpu) ((GPU_PIPE_OK)) ||
    die "全 GPU 管线不可用（需要 NVENC 编码器 + cuda 解码 + scale_cuda 滤镜），改用 --pipeline cpu" ;;
dec) ((CUDA_OK)) || die "ffmpeg 不支持 cuda 硬件解码，改用 --pipeline cpu" ;;
esac

# build_scale_filter
#
# 功能描述：
#   把分辨率规格转成 scale 滤镜参数部分; 只缩小不放大, 并保证宽高为偶数。
#
# 参数：
#   $1: spec - 如 `720p` / `1280x720` / `-2:720` / `50%`。
#
# 注意事项：
#   1. 输出不带滤镜名, 便于同一份参数拼给 CPU 的 `scale` 或 GPU 的 `scale_cuda`。
#
build_scale_filter() {
    local spec=${1,,} h w pct
    case $spec in
    "" | keep | none | source | src) return 0 ;;
    8k | 4320p) h=4320 ;;
    4k | 2160p) h=2160 ;;
    2k | 1440p) h=1440 ;;
    1080p | fhd) h=1080 ;;
    720p | hd) h=720 ;;
    576p) h=576 ;;
    480p | sd) h=480 ;;
    360p) h=360 ;;
    240p) h=240 ;;
    *%)
        pct=${spec%\%}
        [[ $pct =~ ^[0-9]+(\.[0-9]+)?$ ]] || die "分辨率百分比无效: $1"
        printf 'trunc(iw*%s/100/2)*2:trunc(ih*%s/100/2)*2' "$pct" "$pct"
        return 0
        ;;
    *x* | *:*)
        w=${spec%%[x:]*}
        h=${spec##*[x:]}
        [[ -n $w && -n $h ]] || die "分辨率格式无效: $1"
        if [[ $w == -* || $h == -* || $w == auto || $h == auto ]]; then
            [[ $w == auto ]] && w=-2
            [[ $h == auto ]] && h=-2
            printf '%s:%s' "$w" "$h"
        else
            printf '%s:%s:force_original_aspect_ratio=decrease:force_divisible_by=2' "$w" "$h"
        fi
        return 0
        ;;
    *[!0-9]*) die "分辨率格式无效: $1" ;;
    *) h=$spec ;;
    esac
    # 纯高度：等比缩放且不放大；-- 防止 printf 把 -2 当成自己的选项
    printf -- '-2:min(ih\\,%s)' "$h"
}

# build_atempo_chain
#
# 功能描述：
#   生成 `atempo` 滤镜链。单次只接受 0.5~2.0, 超出范围串成多级。
#
# 参数：
#   $1: speed - 播放倍速。
#
build_atempo_chain() {
    awk -v s="$1" 'BEGIN{
        out = ""
        while (s > 2.0) { out = out "atempo=2.0,"; s /= 2.0 }
        while (s < 0.5) { out = out "atempo=0.5,"; s *= 2.0 }
        printf "%s%s", out, sprintf("atempo=%.6g", s)
    }'
}

SCALE_ARGS=$(build_scale_filter "$RESOLUTION")

DRAWTEXT=""
LABEL_TEXT=""
if ((SPEED_LABEL)); then
    has_filter drawtext || die "ffmpeg 不支持 drawtext 滤镜，无法叠加倍率标记"
    LABEL_TEXT=$(awk -v s="$SPEED" 'BEGIN{printf "x%g", s}')
    LABEL_FONT=$(find_font) || die "未找到可用字体，先装一个: sudo apt install fonts-dejavu-core"
    DRAWTEXT="drawtext=fontfile='$LABEL_FONT':text='$LABEL_TEXT'\
:fontcolor=white:fontsize=${LABEL_SIZE:-h/20}\
:box=1:boxcolor=black@0.45:boxborderw=8\
:x=w-tw-(h/36):y=h-th-(h/36)"
fi

# build_vf
#
# 功能描述：
#   按管线生成视频滤镜链: gpu 走 CUDA 滤镜, dec/cpu 走 CPU 滤镜。
#
# 参数：
#   $1: mode - `gpu` / `dec` / `cpu`。
#
build_vf() {
    local mode=$1 vf=""
    if [[ $mode == gpu ]]; then
        # scale_cuda 必须真正处理帧（passthrough=0），否则后续滤镜拿不到 CUDA 帧上下文
        vf="scale_cuda=${SCALE_ARGS:-iw:ih}:passthrough=0"
    elif [[ -n $SCALE_ARGS ]]; then
        vf="scale=$SCALE_ARGS"
    fi
    # setpts 改时间戳做变速，放在 fps 之前，让 fps 按变速后的时间轴抽帧
    ((SPEED_ON)) && vf+="${vf:+,}setpts=PTS/$SPEED"
    [[ -n $FPS ]] && vf+="${vf:+,}fps=$FPS"
    if [[ -n $DRAWTEXT ]]; then
        # drawtext 只能处理内存帧，GPU 管线要先下载再传回显存
        [[ $mode == gpu ]] && vf+=",hwdownload,format=nv12|p010le"
        vf+="${vf:+,}$DRAWTEXT"
        [[ $mode == gpu ]] && vf+=",hwupload_cuda"
    fi
    printf '%s' "$vf"
}

VF_CPU=$(build_vf cpu)
VF_GPU=""
((GPU_PIPE_OK)) && VF_GPU=$(build_vf gpu)

AF=""
((SPEED_ON)) && AF=$(build_atempo_chain "$SPEED")

# build_ffmpeg_cmd
#
# 功能描述：
#   构造 ffmpeg 命令到全局数组 `FFCMD`。
#
# 参数：
#   $1: src - 源文件。
#   $2: dst - 输出文件。
#   $3: pass - `0` 单遍 / `1` 第一遍 / `2` 第二遍。
#   $4: passlog - 两遍编码日志前缀。
#   $5: mode - 实际使用的管线 (`gpu` / `dec` / `cpu`)。
#
build_ffmpeg_cmd() {
    local src=$1 dst=$2 pass=${3:-0} passlog=${4:-} mode=${5:-cpu} vf

    FFCMD=(ffmpeg -hide_banner -nostdin -y -loglevel warning)
    if ((JOBS > 1)); then FFCMD+=(-nostats); else FFCMD+=(-stats); fi
    case $mode in
    gpu)
        # 解码后的帧留在显存，直接喂给 scale_cuda 和 NVENC，省掉两次显存内存拷贝
        FFCMD+=(-hwaccel cuda -hwaccel_output_format cuda)
        vf=$VF_GPU
        ;;
    dec)
        FFCMD+=(-hwaccel cuda)
        vf=$VF_CPU
        ;;
    *) vf=$VF_CPU ;;
    esac
    [[ -n $GPU_ID && $mode != cpu ]] && FFCMD+=(-hwaccel_device "$GPU_ID")
    [[ -n $START ]] && FFCMD+=(-ss "$START")
    # -t 放在 -i 之前按源片计时，否则变速后截出来的是输出时长
    [[ -n $PREVIEW ]] && FFCMD+=(-t "$PREVIEW")
    FFCMD+=(-i "$src")

    FFCMD+=(-map 0:v:0)
    if [[ $AUDIO_MODE == none || $pass == 1 ]]; then
        FFCMD+=(-an)
    else
        FFCMD+=(-map "0:a?")
    fi
    FFCMD+=(-sn -dn -map_metadata 0)

    FFCMD+=(-c:v "$ENCODER")
    [[ -n $PRESET ]] && FFCMD+=(-preset "$PRESET")
    [[ -n $vf ]] && FFCMD+=(-vf "$vf")
    [[ -n $PIX_FMT ]] && FFCMD+=(-pix_fmt "$PIX_FMT")
    ((AQ_OK)) && FFCMD+=(-spatial-aq 1)
    [[ -n $GPU_ID ]] && ((IS_NVENC)) && FFCMD+=(-gpu "$GPU_ID")

    if [[ -n $BITRATE ]]; then
        FFCMD+=(-b:v "$BITRATE")
        ((IS_NVENC)) && FFCMD+=(-rc vbr)
        if [[ -n $MAXRATE ]]; then
            FFCMD+=(-maxrate "$MAXRATE" -bufsize "${BUFSIZE:-$(bitrate_x2 "$MAXRATE")}")
        elif [[ -n $BUFSIZE ]]; then
            FFCMD+=(-bufsize "$BUFSIZE")
        fi
        if ((TWOPASS)); then
            if ((IS_NVENC)); then
                encoder_has_opt "$ENCODER" multipass && FFCMD+=(-multipass fullres)
            else
                FFCMD+=(-pass "$pass" -passlogfile "$passlog")
            fi
        fi
    else
        case $ENCODER in
        libx264 | libx265 | libsvtav1 | libaom-av1) FFCMD+=(-crf "$QUALITY") ;;
        libvpx-vp9) FFCMD+=(-crf "$QUALITY" -b:v 0) ;;
        *_nvenc) FFCMD+=(-rc vbr -cq "$QUALITY" -b:v 0) ;;
        esac
    fi

    [[ $ENCODER == libx265 ]] && FFCMD+=(-x265-params log-level=error)
    [[ $ENCODER == libvpx-vp9 ]] && FFCMD+=(-row-mt 1 -deadline good -cpu-used 2)

    if [[ $pass == 1 ]]; then
        FFCMD+=(-f null -)
        return 0
    fi

    if [[ $AUDIO_MODE == copy ]]; then
        FFCMD+=(-c:a copy)
    elif [[ $AUDIO_MODE == encode ]]; then
        case $CONTAINER in
        webm) FFCMD+=(-c:a libopus -b:a "$AUDIO_BITRATE") ;;
        *) FFCMD+=(-c:a aac -b:a "$AUDIO_BITRATE") ;;
        esac
        [[ -n $AF ]] && FFCMD+=(-af "$AF")
    fi

    # hvc1 tag 让 macOS / iOS 能播放 mp4 里的 HEVC
    [[ $CODEC == h265 && ($CONTAINER == mp4 || $CONTAINER == mov) ]] && FFCMD+=(-tag:v hvc1)
    case $CONTAINER in
    mp4 | mov | m4v) FFCMD+=(-movflags +faststart) ;;
    esac

    FFCMD+=("$dst")
}

# bitrate_x2
#
# 功能描述：
#   把码率字符串翻倍, 用作 `bufsize` 默认值。
#
# 参数：
#   $1: rate - 码率, 如 `4M` / `2500k`。
#
bitrate_x2() {
    local r=${1,,} num unit
    num=${r%[km]}
    unit=${r#"$num"}
    awk -v n="$num" -v u="$unit" 'BEGIN{printf "%g%s", n*2, u}'
}

collect_files

# ---------------------------------------------------------------- 任务概览 ---
TOTAL=${#FILES[@]}
total_src_bytes=0
for f in "${FILES[@]}"; do
    total_src_bytes=$((total_src_bytes + $(filesize "$f")))
done

if ((IS_NVENC)); then quality_desc="质量 CQ=$QUALITY"; else quality_desc="质量 CRF=$QUALITY"; fi
if [[ -n $BITRATE ]]; then
    quality_desc="码率 $BITRATE${MAXRATE:+ (上限 $MAXRATE)}"
    ((TWOPASS)) && quality_desc+=" 两遍编码"
fi

case $AUDIO_MODE in
none) audio_desc="去除" ;;
copy) audio_desc="直接复制" ;;
*) audio_desc="aac $AUDIO_BITRATE" ;;
esac

speed_line=""
if ((SPEED_ON || SPEED_LABEL)); then
    speed_desc="${SPEED}x"
    ((SPEED_ON)) || speed_desc="原速"
    ((SPEED_LABEL)) && speed_desc+="  右下角标注: $LABEL_TEXT"
    speed_line="速度:   $speed_desc"$'\n'
fi

case $PIPELINE in
gpu) pipe_desc="全 GPU（NVDEC 解码 + CUDA 滤镜 + NVENC 编码）" ;;
dec) pipe_desc="NVDEC 解码 + CPU 滤镜" ;;
*) pipe_desc="CPU 解码与滤镜" ;;
esac
((AQ_OK)) && pipe_desc+="  |  spatial-aq"
[[ -n $GPU_ID ]] && pipe_desc+="  |  GPU $GPU_ID"

cat <<EOF
待处理: $TOTAL 个文件, 共 $(hsize "$total_src_bytes")
编码器: $ENCODER${PRESET:+ (preset $PRESET)}  |  $quality_desc
管线:   $pipe_desc
分辨率: ${RESOLUTION:-保持原样}${FPS:+  帧率: $FPS}
${speed_line}音频:   $audio_desc
输出:   ${OUTDIR:-<源目录>/compressed}/*.${CONTAINER}${SUFFIX:+  后缀: $SUFFIX}${PREVIEW:+  仅取源片前 ${PREVIEW}s}
并行:   $JOBS

EOF

if ((SPEED_ON)) && [[ -z $FPS ]] && awk -v s="$SPEED" 'BEGIN{exit !(s > 1)}'; then
    warn "提示: ${SPEED}x 加速会让输出帧率变为原来的 ${SPEED} 倍，加 --fps 30 降帧才能明显减小体积"
fi

if ((DELETE_SOURCE)) && ((!ASSUME_YES)) && ((!DRYRUN)); then
    read -r -p "确认转码成功后删除源文件？输入 yes 继续: " ans
    [[ $ans == yes ]] || die "已取消"
fi

RUNDIR=$(mktemp -d -t media-compress.XXXXXX)
trap 'rm -rf "$RUNDIR"' EXIT

# encode_file
#
# 功能描述：
#   按指定管线编码一个文件 (含两遍编码流程), 返回 ffmpeg 的退出码。
#
# 参数：
#   $1: src - 源文件。
#   $2: dst - 输出文件。
#   $3: log - ffmpeg 日志路径。
#   $4: idx - 任务序号 (用于 passlog 文件名)。
#   $5: mode - 管线 (`gpu` / `dec` / `cpu`)。
#
encode_file() {
    local src=$1 dst=$2 log=$3 idx=$4 mode=$5 rc=0
    if ((TWOPASS)) && [[ -n $BITRATE ]] && ((!IS_NVENC)); then
        build_ffmpeg_cmd "$src" "$dst" 1 "$RUNDIR/pass-$idx" "$mode"
        run_ffmpeg "$log" || rc=$?
        if ((rc == 0)); then
            build_ffmpeg_cmd "$src" "$dst" 2 "$RUNDIR/pass-$idx" "$mode"
            run_ffmpeg "$log" || rc=$?
        fi
    else
        build_ffmpeg_cmd "$src" "$dst" 0 "" "$mode"
        run_ffmpeg "$log" || rc=$?
    fi
    return "$rc"
}

# media_compress_one
#
# 功能描述：
#   处理单个视频: 跳过已存在输出、选择管线、转码、可选删除源文件, 并把结果写入 `$RUNDIR/$idx`。
#
# 参数：
#   $1: idx - 任务序号 (从 1 起)。
#   $2: src - 源文件路径。
#   $3: rel - 相对显示名。
#
# 注意事项：
#   1. 并行时本函数运行在子 shell 中, 需摘掉继承来的 EXIT trap, 否则会提前删掉 `RUNDIR`。
#
media_compress_one() {
    # 并行时本函数运行在子 shell 中，需摘掉继承来的 EXIT trap，否则会提前删掉 RUNDIR
    trap - EXIT
    local idx=$1 src=$2 rel=$3
    local name=${rel##*/} subdir=${rel%/*}
    local base=${name%.*}
    [[ $subdir == "$rel" ]] && subdir=""

    local outdir
    if [[ -n $OUTDIR ]]; then
        outdir=$OUTDIR${subdir:+/$subdir}
    else
        outdir=$(dirname -- "$src")/compressed
    fi
    local dst="$outdir/${base}${SUFFIX}.${CONTAINER}"

    local src_bytes
    src_bytes=$(filesize "$src")

    if [[ -e $dst ]] && ((!FORCE)); then
        printf '[%d/%d] 跳过 %s（输出已存在，用 -f 覆盖）\n' "$idx" "$TOTAL" "$rel"
        printf 'skip\t%s\t%s\t0\t0\n' "$rel" "$src_bytes" >"$RUNDIR/$idx"
        return 0
    fi

    # NVDEC 认不出的源格式只让这一个文件退回 CPU，不影响整批
    local mode=$PIPELINE vcodec
    vcodec=$(ffprobe -v error -select_streams v:0 -show_entries stream=codec_name \
        -of csv=p=0 -- "$src" 2>/dev/null || echo "")
    if [[ $mode != cpu ]] && ! nvdec_supports "$vcodec"; then
        warn "$rel: ${vcodec:-未知格式} 不支持 NVDEC 硬解，该文件改走 CPU 管线"
        mode=cpu
    fi
    # 旋转元数据会触发 CPU 侧 transpose，全 GPU 帧无法喂进去；改走 dec（硬解+内存滤镜）
    if [[ $mode == gpu ]] && video_has_rotation "$src"; then
        warn "$rel: 含旋转元数据，全 GPU 管线不兼容 autorotate，该文件改走 dec 管线"
        mode=dec
    fi

    if ((DRYRUN)); then
        build_ffmpeg_cmd "$src" "$dst" 0 "" "$mode"
        printf '[%d/%d] %s\n' "$idx" "$TOTAL" "$rel"
        printf '        %s\n' "$(printf '%q ' "${FFCMD[@]}")"
        printf 'dry\t%s\t%s\t0\t0\n' "$rel" "$src_bytes" >"$RUNDIR/$idx"
        return 0
    fi

    mkdir -p -- "$outdir"
    local log="$RUNDIR/$idx.log"
    local dur dur_desc=""
    dur=$(ffprobe -v error -show_entries format=duration -of csv=p=0 -- "$src" 2>/dev/null || echo "")
    [[ ${dur%.*} =~ ^[0-9]+$ ]] && dur_desc=", $(hms "${dur%.*}")"
    printf '[%d/%d] %s  (%s%s)\n' "$idx" "$TOTAL" "$rel" "$(hsize "$src_bytes")" "$dur_desc"

    local t0=$SECONDS rc=0
    encode_file "$src" "$dst" "$log" "$idx" "$mode" || rc=$?
    # 硬件管线的坑比较多（驱动、显存、冷门码流），失败了用 CPU 兜底而不是判这个文件死刑
    if { ((rc != 0)) || [[ ! -s $dst ]]; } && [[ $mode != cpu ]]; then
        ((rc != 0)) || rc=1
        warn "[$idx/$TOTAL] $rel: $mode 管线失败（退出码 $rc），改用 CPU 管线重试"
        rm -f -- "$dst"
        mode=cpu
        rc=0
        encode_file "$src" "$dst" "$log" "$idx" cpu || rc=$?
    fi
    local elapsed=$((SECONDS - t0))

    if ((rc != 0)) || [[ ! -s $dst ]]; then
        rm -f -- "$dst"
        printf '[%d/%d] 失败 %s (ffmpeg 退出码 %d)\n' "$idx" "$TOTAL" "$rel" "${rc:-1}" >&2
        tail -n 15 -- "$log" >&2 || true
        printf 'fail\t%s\t%s\t0\t%s\n' "$rel" "$src_bytes" "$elapsed" >"$RUNDIR/$idx"
        return 0
    fi

    local dst_bytes
    dst_bytes=$(filesize "$dst")
    ((KEEP_MTIME)) && touch -r "$src" -- "$dst"

    printf '[%d/%d] 完成 %s: %s -> %s (%s) 用时 %s\n' "$idx" "$TOTAL" "$rel" \
        "$(hsize "$src_bytes")" "$(hsize "$dst_bytes")" \
        "$(awk -v a="$src_bytes" -v b="$dst_bytes" 'BEGIN{printf "%+.1f%%", (b-a)*100/a}')" \
        "$(hms "$elapsed")"

    if ((DELETE_SOURCE)); then
        if ((dst_bytes > 0 && dst_bytes < src_bytes && !SPEED_ON)) && [[ -z $PREVIEW && -z $START ]]; then
            rm -f -- "$src"
        else
            warn "保留源文件 $rel（输出未变小，或处于预览/裁剪/变速模式）"
        fi
    fi

    printf 'ok\t%s\t%s\t%s\t%s\n' "$rel" "$src_bytes" "$dst_bytes" "$elapsed" >"$RUNDIR/$idx"
}

# run_ffmpeg
#
# 功能描述：
#   执行全局 `FFCMD`。单任务把进度显示到终端, 并行时只写日志。
#
# 参数：
#   $1: log - 日志文件路径。
#
# 注意事项：
#   1. 单任务用 `tee` 时不能写 `pipeline || true`, 否则会冲掉 `PIPESTATUS`, 失败被当成成功。
#
run_ffmpeg() {
    local log=$1 rc=0
    if ((JOBS > 1)); then
        "${FFCMD[@]}" >>"$log" 2>&1 || rc=$?
    else
        # 不能写 pipeline || true：true 会冲掉 PIPESTATUS，失败会被当成成功，
        # 进而跳过 CPU 兜底，留下 0 字节的空输出文件。
        set +e
        "${FFCMD[@]}" 2>&1 | tee -a "$log"
        rc=${PIPESTATUS[0]}
        set -e
    fi
    return "$rc"
}

# ------------------------------------------------------------------ 主循环 ---
START_TS=$SECONDS
idx=0
for i in "${!FILES[@]}"; do
    idx=$((idx + 1))
    if ((JOBS > 1)); then
        while (($(jobs -rp | wc -l) >= JOBS)); do wait -n || true; done
        media_compress_one "$idx" "${FILES[i]}" "${RELS[i]}" &
    else
        media_compress_one "$idx" "${FILES[i]}" "${RELS[i]}"
    fi
done
wait

# -------------------------------------------------------------------- 汇总 ---
ok=0 failed=0 skipped=0 sum_src=0 sum_dst=0
declare -a fail_list=()
for i in $(seq 1 "$TOTAL"); do
    [[ -f "$RUNDIR/$i" ]] || continue
    IFS=$'\t' read -r status rel sb db _el <"$RUNDIR/$i"
    case $status in
    ok) ok=$((ok + 1)); sum_src=$((sum_src + sb)); sum_dst=$((sum_dst + db)) ;;
    fail) failed=$((failed + 1)); fail_list+=("$rel") ;;
    skip | dry) skipped=$((skipped + 1)) ;;
    esac
done

total_elapsed=$((SECONDS - START_TS))
echo
if ((ok > 0)); then
    printf '成功 %d 个: %s -> %s，节省 %s (%s)\n' "$ok" \
        "$(hsize "$sum_src")" "$(hsize "$sum_dst")" "$(hsize $((sum_src - sum_dst)))" \
        "$(awk -v a="$sum_src" -v b="$sum_dst" 'BEGIN{printf "%.1f%%", (a-b)*100/a}')"
    if [[ -n $PREVIEW || -n $START ]]; then
        echo '注意: 处于预览/裁剪模式，只编码了片段，上面的体积对比不代表全量压缩结果'
    fi
fi
((skipped > 0)) && printf '跳过 %d 个\n' "$skipped"
if ((failed > 0)); then
    printf '失败 %d 个:\n' "$failed"
    printf '  %s\n' "${fail_list[@]}"
fi
printf '总耗时 %s\n' "$(hms "$total_elapsed")"

((failed == 0)) || exit 1
  )
}

# setup-media
#
# 功能描述：
#   媒体工具调度器。当前目标: compress (视频转码)。后续可在此追加 media-* 子函数。
#
# 参数：
#   $1: compress|video - 显式指定视频压缩; 省略则默认走 media-compress。
#   $@: 透传给 media-compress 的选项与路径。
#
# 使用示例：
#   setup-media --info .
#   setup-media compress -s 720p .
#
setup-media() {
  case "${1-}" in
  compress | video)
    shift
    media-compress "$@"
    ;;
  -h | --help | help)
    media-compress --help
    ;;
  *)
    media-compress "$@"
    ;;
  esac
}

# media
#
# 功能描述：
#   setup-media 的短名。
#
# 参数：
#   $@: 透传给 setup-media。
#
# 使用示例：
#   media --info .
#   media -s 720p -q 30 --no-audio .
#
media() {
  setup-media "$@"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  setup-media "$@"
fi
