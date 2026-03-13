#!/bin/bash

# 脚本名称: setup_stewart_env.sh
# 功能: 初始化 stewart 开发环境

ENV_NAME="stewart"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "🚀 正在配置 Stewart 机器人开发环境..."

# 1. 检查 conda 是否可用
if ! command -v conda &> /dev/null; then
    echo "❌ 错误: 未找到 conda 命令，请先安装 Miniconda 或 Anaconda。"
    exit 1
fi

# 2. 激活环境
echo "📦 激活 conda 环境: ${ENV_NAME} ..."
source $(conda info --base)/etc/profile.d/conda.sh
conda activate ${ENV_NAME}

if [ "$CONDA_DEFAULT_ENV" != "${ENV_NAME}" ]; then
    echo "❌ 错误: 无法激活 ${ENV_NAME} 环境，请先运行 'conda create -n ${ENV_NAME} python=3.10'。"
    exit 1
fi

# 3. 加载 ROS2 Humble 环境
# 假设 ROS2 安装在默认路径，如果不是请修改此处
ROS2_SETUP_FILE="/opt/ros/humble/setup.bash"
if [ -f "$ROS2_SETUP_FILE" ]; then
    echo "🤖  sourcing ROS2 Humble 环境..."
    source $ROS2_SETUP_FILE
else
    echo "⚠️  警告: 未在默认路径找到 ROS2 Humble ($ROS2_SETUP_FILE)，请手动 source 正确的 setup.bash。"
fi

# 4. 配置自包含的动态库路径 (关键步骤)
# 假设 controlcan.so 位于项目根目录的 lib 文件夹下，请根据实际情况调整路径
LIB_DIR="${PROJECT_ROOT}/lib" 
if [ -d "$LIB_DIR" ]; then
    echo "🔗 添加自定义动态库路径到 LD_LIBRARY_PATH: $LIB_DIR"
    export LD_LIBRARY_PATH="$LIB_DIR:$LD_LIBRARY_PATH"
else
    echo "⚠️  提示: 未找到预定义的 lib 目录 ($LIB_DIR)，请确认 controlcan.so 所在位置并手动导出。"
fi

# 5. 安装 Python 依赖
echo "📥 安装 Python 依赖项..."
pip install --upgrade pip
pip install -r "${PROJECT_ROOT}/requirements.txt"

# 6. 验证环境
echo "✅ 环境配置完成！"
echo "-----------------------------------------"
echo "环境名: ${CONDA_DEFAULT_ENV}"
echo "Python版本: $(python --version)"
echo "ROS2版本: ${ROS_DISTRO:-未检测到}"
echo "-----------------------------------------"
echo "💡 接下来您可以运行:"
echo "   pytest tests/              # 运行单元测试"
echo "   ros2 run stewart_bot main  # 启动主节点 (示例)"
echo "-----------------------------------------"