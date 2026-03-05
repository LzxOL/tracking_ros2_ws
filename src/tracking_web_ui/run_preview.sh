#!/bin/bash
# 预览模式：无需 ROS2，直接启动界面
cd "$(dirname "$0")"
pip install -q fastapi uvicorn opencv-python-headless numpy 2>/dev/null
exec python app.py --preview
