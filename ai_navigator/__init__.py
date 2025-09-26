"""
AI Robot Navigation Package
Autonomous navigation system for 2D robot simulation
"""

__version__ = "1.0.0"
__author__ = "AI Navigation Team"

from .robot_ai import RobotNavigationAI
from .pathfinding import AStarPathfinder, RefinedPathfinder
from .collision_detector import CollisionDetector, Obstacle
from .utils import Point, Node

__all__ = [
    "RobotNavigationAI",
    "AStarPathfinder", 
    "RefinedPathfinder",
    "CollisionDetector",
    "Obstacle",
    "Point",
    "Node"
]
