"""
Setup script for AI Robot Navigation System
"""
from setuptools import setup, find_packages
import os

# Read the contents of README file
this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

# Read requirements
with open(os.path.join(this_directory, 'requirements.txt'), encoding='utf-8') as f:
    requirements = [line.strip() for line in f if line.strip() and not line.startswith('#')]

setup(
    name="ai-robot-navigation",
    version="1.0.0",
    author="AI Navigation Team",
    author_email="navigation@ai-robot.com",
    description="Autonomous 2D robot navigation system using A* pathfinding with collision avoidance",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ai-team/robot-navigation",
    project_urls={
        "Bug Tracker": "https://github.com/ai-team/robot-navigation/issues",
        "Documentation": "https://github.com/ai-team/robot-navigation/wiki",
        "Source Code": "https://github.com/ai-team/robot-navigation",
    },
    packages=find_packages(exclude=["tests*", "environment*"]),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Robotics",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    python_requires=">=3.7",
    install_requires=requirements,
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=4.0.0",
            "black>=22.0.0",
            "flake8>=5.0.0",
            "mypy>=0.991",
            "pre-commit>=2.20.0",
        ],
        "docs": [
            "sphinx>=5.0.0",
            "sphinx-rtd-theme>=1.0.0",
            "sphinxcontrib-napoleon>=0.7",
        ],
        "performance": [
            "numpy>=1.21.0",
            "scipy>=1.9.0",
        ],
        "visualization": [
            "matplotlib>=3.5.0",
            "plotly>=5.0.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "robot-navigate=scripts.run_navigation:main",
            "robot-test=scripts.test_pathfinding:main",
            "robot-debug=scripts.debug_navigation:main",
            "robot-benchmark=scripts.benchmark:main",
        ],
    },
    include_package_data=True,
    package_data={
        "ai_navigator": [
            "*.py",
        ],
        "scripts": [
            "*.py",
        ],
        "data": [
            "obstacle_maps/*.json",
            "performance_results/*.csv",
        ],
    },
    zip_safe=False,
    keywords=[
        "robotics",
        "navigation", 
        "pathfinding",
        "artificial-intelligence",
        "a-star",
        "collision-detection",
        "autonomous-systems",
        "robot-simulation",
    ],
    platforms=["any"],
    license="MIT",
    test_suite="tests",
    tests_require=[
        "pytest>=7.0.0",
        "pytest-cov>=4.0.0",
    ],
)
