"""按操作系统自动选择视觉实现。

Windows 加载 v1.0 原版（vision_windows），macOS 加载暗光优化版（vision_mac）；
其余平台（Linux 等）同样使用优化版。
上层程序 `from vision import vision` 的用法保持不变。
"""
import sys

if sys.platform == "darwin":
    from . import vision_mac as vision
else:
    from . import vision_windows as vision
