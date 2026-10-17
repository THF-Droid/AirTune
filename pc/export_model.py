"""驱动 AirTune.export_model_h，把训练好的 int8 模型导出成 F411 用的 C 头文件。

用法（在本目录 pc/ 下）：
    python export_model.py              # 默认导出到 ../firmware/Core/Inc/airtune_model.h
    python export_model.py <输出路径>    # 自定义输出
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from AirTune import export_model_h

DEFAULT_OUT = os.path.join(_HERE, "..", "firmware", "Core", "Inc", "airtune_model.h")

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUT
    export_model_h(out)
