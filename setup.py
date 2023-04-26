#!/usr/bin/env python3
"""
Setup script for Object Tracker package
"""

from setuptools import setup, find_packages

# Read the README file
with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

# Read requirements
with open("requirements.txt", "r", encoding="utf-8") as fh:
    requirements = [line.strip() for line in fh if line.strip() and not line.startswith("#")]

setup(
    name="ros_visual_tracking_system",
    version="1.0.0",
    author="ROS Visual Tracking System Developer",
    author_email="developer@example.com",
    description="A professional ROS package for real-time visual object tracking using computer vision",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/ros-visual-tracking-system",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Image Processing",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    python_requires=">=3.6",
    install_requires=requirements,
    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-cov>=2.0",
            "flake8>=3.8",
            "black>=21.0",
            "mypy>=0.800",
        ],
    },
    entry_points={
        "console_scripts": [
                    "hsv_tracker=ros_visual_tracking_system.nodes.hsv_tracker_node:main",
        "apriltag_tracker=ros_visual_tracking_system.nodes.apriltag_tracker_node:main",
        ],
    },
    include_package_data=True,
    package_data={
        "ros_visual_tracking_system": [
            "launch/*.launch",
            "config/*.yaml",
            "config/*.rviz",
        ],
    },
    zip_safe=False,
)
