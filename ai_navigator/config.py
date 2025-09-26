"""
Configuration settings for Robot Navigation AI
"""

# Environment Settings - Match your simulation server exactly
API_BASE_URL = "http://localhost:5000"
WEBSOCKET_URL = "ws://localhost:5000"  # Changed to match Flask-SocketIO

# Canvas Settings - Match your sim-11 canvas size
CANVAS_WIDTH = 650
CANVAS_HEIGHT = 600

# Robot Settings
ROBOT_RADIUS = 18
ROBOT_SPEED = 2
SAFETY_MARGIN = 10

# Navigation Settings
GRID_SIZE = 10  # Grid resolution for A* algorithm
MAX_ITERATIONS = 1000
WAYPOINT_DISTANCE = 50  # Distance between waypoints

# Collision Settings
COLLISION_THRESHOLD = 25  # Distance threshold for collision detection
MAX_COLLISION_COUNT = 3
ADAPTIVE_MARGIN_INCREMENT = 5

# Pathfinding Settings
HEURISTIC_WEIGHT = 1.0  # Weight for heuristic in A*
DIAGONAL_COST = 1.414  # sqrt(2) for diagonal movement
STRAIGHT_COST = 1.0

# Timeout Settings
MOVE_TIMEOUT = 30  # seconds
WEBSOCKET_TIMEOUT = 5  # seconds
API_TIMEOUT = 10  # seconds
