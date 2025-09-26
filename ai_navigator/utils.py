"""
Utility functions for Robot Navigation AI
"""
import math
from typing import List, Tuple, Dict, Any

class Point:
    """2D Point class for coordinates"""
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    
    def __eq__(self, other):
        if not isinstance(other, Point):
            return False
        return abs(self.x - other.x) < 0.001 and abs(self.y - other.y) < 0.001
    
    def __hash__(self):
        return hash((round(self.x, 3), round(self.y, 3)))
    
    def __str__(self):
        return f"Point({self.x:.1f}, {self.y:.1f})"
    
    def __repr__(self):
        return self.__str__()
    
    def distance_to(self, other) -> float:
        """Calculate Euclidean distance to another point"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def manhattan_distance_to(self, other) -> float:
        """Calculate Manhattan distance to another point"""
        return abs(self.x - other.x) + abs(self.y - other.y)
    
    def copy(self):
        """Create a copy of this point"""
        return Point(self.x, self.y)

class Node:
    """Node class for A* pathfinding"""
    def __init__(self, point: Point, g_cost: float = 0, h_cost: float = 0, parent=None):
        self.point = point
        self.g_cost = g_cost  # Distance from start
        self.h_cost = h_cost  # Heuristic distance to goal
        self.f_cost = g_cost + h_cost  # Total cost
        self.parent = parent
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        if not isinstance(other, Node):
            return False
        return self.point == other.point
    
    def __hash__(self):
        return hash(self.point)
    
    def __str__(self):
        return f"Node({self.point}, f={self.f_cost:.1f})"

def euclidean_distance(p1: Point, p2: Point) -> float:
    """Calculate Euclidean distance between two points"""
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def manhattan_distance(p1: Point, p2: Point) -> float:
    """Calculate Manhattan distance between two points"""
    return abs(p1.x - p2.x) + abs(p1.y - p2.y)

def diagonal_distance(p1: Point, p2: Point) -> float:
    """Calculate diagonal distance (Chebyshev distance)"""
    dx = abs(p1.x - p2.x)
    dy = abs(p1.y - p2.y)
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

def is_point_in_bounds(point: Point, width: int, height: int, margin: int = 0) -> bool:
    """Check if point is within canvas bounds with optional margin"""
    return (margin <= point.x < width - margin and 
            margin <= point.y < height - margin)

def clamp_point_to_bounds(point: Point, width: int, height: int, margin: int = 0) -> Point:
    """Clamp point to stay within bounds with margin"""
    x = max(margin, min(width - margin, point.x))
    y = max(margin, min(height - margin, point.y))
    return Point(x, y)

def get_neighbors(point: Point, grid_size: int = 10) -> List[Point]:
    """Get 8-directional neighbors for a grid point"""
    neighbors = []
    directions = [
        (-1, -1), (-1, 0), (-1, 1),  # Top row
        (0, -1),           (0, 1),   # Middle row (skip center)
        (1, -1),  (1, 0),  (1, 1)    # Bottom row
    ]
    
    for dx, dy in directions:
        new_x = point.x + dx * grid_size
        new_y = point.y + dy * grid_size
        neighbors.append(Point(new_x, new_y))
    
    return neighbors

def interpolate_points(start: Point, end: Point, num_points: int) -> List[Point]:
    """Generate intermediate points between start and end"""
    if num_points <= 0:
        return []
    
    points = []
    for i in range(1, num_points + 1):
        t = i / (num_points + 1)
        x = start.x + t * (end.x - start.x)
        y = start.y + t * (end.y - start.y)
        points.append(Point(x, y))
    
    return points

def simplify_path(path: List[Point], tolerance: float = 5.0) -> List[Point]:
    """Simplify path by removing unnecessary intermediate points using Douglas-Peucker algorithm"""
    if len(path) <= 2:
        return path
    
    simplified = [path[0]]  # Always keep start point
    
    for i in range(1, len(path) - 1):
        # Check if we can skip this point
        prev_point = simplified[-1]
        curr_point = path[i]
        next_point = path[i + 1]
        
        # Calculate distance from current point to line between prev and next
        line_dist = point_to_line_distance(curr_point, prev_point, next_point)
        
        if line_dist > tolerance:
            simplified.append(curr_point)
    
    simplified.append(path[-1])  # Always keep end point
    return simplified

def point_to_line_distance(point: Point, line_start: Point, line_end: Point) -> float:
    """Calculate perpendicular distance from point to line segment"""
    # Vector from line_start to line_end
    line_vec_x = line_end.x - line_start.x
    line_vec_y = line_end.y - line_start.y
    
    # Vector from line_start to point
    point_vec_x = point.x - line_start.x
    point_vec_y = point.y - line_start.y
    
    # Length of line segment
    line_length = math.sqrt(line_vec_x**2 + line_vec_y**2)
    
    if line_length == 0:
        return euclidean_distance(point, line_start)
    
    # Project point onto line segment
    dot_product = (point_vec_x * line_vec_x + point_vec_y * line_vec_y) / (line_length**2)
    
    # Clamp to line segment
    dot_product = max(0, min(1, dot_product))
    
    # Calculate closest point on line segment
    closest_x = line_start.x + dot_product * line_vec_x
    closest_y = line_start.y + dot_product * line_vec_y
    
    # Return distance to closest point
    return euclidean_distance(point, Point(closest_x, closest_y))

def calculate_path_length(path: List[Point]) -> float:
    """Calculate total length of a path"""
    if len(path) < 2:
        return 0.0
    
    total_length = 0.0
    for i in range(len(path) - 1):
        total_length += euclidean_distance(path[i], path[i + 1])
    
    return total_length

def format_coordinates(point: Point) -> Dict[str, float]:
    """Convert Point to dictionary format for API calls"""
    return {"x": point.x, "y": point.y}

def parse_coordinates(coord_dict: Dict[str, Any]) -> Point:
    """Convert dictionary coordinates to Point object"""
    return Point(float(coord_dict["x"]), float(coord_dict["y"]))

def angle_between_points(p1: Point, p2: Point) -> float:
    """Calculate angle in radians from p1 to p2"""
    return math.atan2(p2.y - p1.y, p2.x - p1.x)

def rotate_point(point: Point, center: Point, angle: float) -> Point:
    """Rotate point around center by angle (in radians)"""
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    
    # Translate to origin
    x = point.x - center.x
    y = point.y - center.y
    
    # Rotate
    new_x = x * cos_angle - y * sin_angle
    new_y = x * sin_angle + y * cos_angle
    
    # Translate back
    return Point(new_x + center.x, new_y + center.y)

# Utility functions for debugging and logging
def format_point_list(points: List[Point], max_points: int = 5) -> str:
    """Format list of points for logging"""
    if not points:
        return "[]"
    
    if len(points) <= max_points:
        return "[" + ", ".join(str(p) for p in points) + "]"
    else:
        shown = points[:max_points]
        return "[" + ", ".join(str(p) for p in shown) + f", ... and {len(points) - max_points} more]"

def validate_point(point: Point, width: int, height: int) -> bool:
    """Validate that a point has valid coordinates"""
    return (isinstance(point.x, (int, float)) and 
            isinstance(point.y, (int, float)) and
            not math.isnan(point.x) and 
            not math.isnan(point.y) and
            not math.isinf(point.x) and 
            not math.isinf(point.y))
