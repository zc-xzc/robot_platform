# -*- coding: utf-8 -*-
"""
siheng_servo SDK 打包配置。
安装: pip install .
"""

from setuptools import setup, find_packages

setup(
    name="siheng_servo",
    version="1.0.0",
    description="上海四横 D-AIS48025A 伺服驱动器 Python SDK (Modbus RTU)",
    author="siheng_servo",
    packages=find_packages(),
    python_requires=">=3.7",
    install_requires=[
        "pyserial>=3.4",
        "pyModbusTCP>=0.1.8",
    ],
    extras_require={
        "gui": ["PyQt5>=5.15"],
        "dev": ["pytest>=6.0", "wheel", "twine"],
    },
    entry_points={
        "console_scripts": [
            "siheng-gui=siheng_servo.gui:launch",
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Topic :: System :: Hardware :: Hardware Drivers",
    ],
)
