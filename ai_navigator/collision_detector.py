"""
Collision Detection Module for Robot Navigation AI
"""
import math
from typing import List, Dict, Any
from .utils import Point, euclidean_distance

class Obstacle:
    """Obstacle representation"""
    def __init__(self, x: float, y: float, size: float):
        self.center = Point(x, y)
        self.size = size
        self.radius = size / 2  # Treat square obstacles as circles for collision
    
    def __str__(self):
        return f"Obstacle(center={self.center}, size={self.size})"
    
    def __repr__(self):
        return self.__str__()

class CollisionDetector:
    """Handles all collision detection logic"""
    
    def __init__(self, robot_radius: float, safety_margin: float = 10):
        self.robot_radius = robot_radius
        self.base_safety_margin = safety_margin
        self.current_safety_margin = safety_margin
        self.collision_count = 0
        self.collision_history = []
    
    def update_safety_margin(self, collision_count: int, increment: float = 5):
        """Adaptively increase safety margin based on collision history"""
        self.collision_count = collision_count
        self.current_safety_margin = self.base_safety_margin + (collision_count * increment)
        print(f"🔧 Safety margin updated to: {self.current_safety_margin}px after {collision_count} collisions")
    
    def will_collide_with_obstacle(self, robot_pos: Point, obstacle: Obstacle) -> bool:
        """Check if robot at given position will collide with obstacle"""
        distance = euclidean_distance(robot_pos, obstacle.center)
        collision_distance = self.robot_radius + obstacle.radius + self.current_safety_margin
        
        return distance < collision_distance
    
    def will_collide_with_any_obstacle(self, robot_pos: Point, obstacles: List[Obstacle]) -> bool:
        """Check if robot will collide with any obstacle"""
        for obstacle in obstacles:
            if self.will_collide_with_obstacle(robot_pos, obstacle):
                return True
        return False
    
    def get_collision_distance(self, robot_pos: Point, obstacle: Obstacle) -> float:
        """Get actual distance to collision boundary"""
        distance_to_center = euclidean_distance(robot_pos, obstacle.center)
        collision_boundary = obstacle.radius + self.robot_radius + self.current_safety_margin
        return distance_to_center - collision_boundary
    
    def get_nearest_obstacle_distance(self, robot_pos: Point, obstacles: List[Obstacle]) -> float:
        """Get distance to nearest obstacle collision boundary"""
        if not obstacles:
            return float('inf')
        
        min_distance = float('inf')
        for obstacle in obstacles:
            distance = self.get_collision_distance(robot_pos, obstacle)
            min_distance = min(min_distance, distance)
        
        return min_distance
    
    def get_nearest_obstacle(self, robot_pos: Point, obstacles: List[Obstacle]) -> tuple:
        """Get nearest obstacle and distance to it"""
        if not obstacles:
            return None, float('inf')
        
        nearest_obstacle = None
        min_distance = float('inf')
        
        for obstacle in obstacles:
            distance = self.get_collision_distance(robot_pos, obstacle)
            if distance < min_distance:
                min_distance = distance
                nearest_obstacle = obstacle
        
        return nearest_obstacle, min_distance
    
    def is_path_clear(self, start: Point, end: Point, obstacles: List[Obstacle], resolution: int = 5) -> bool:
        """Check if straight line path between two points is collision-free"""
        distance = euclidean_distance(start, end)
        if distance == 0:
            return True
        
        # Check intermediate points along the path
        num_checks = max(resolution, int(distance / resolution))
        
        for i in range(num_checks + 1):
            t = i / num_checks if num_checks > 0 else 0
            check_point = Point(
                start.x + t * (end.x - start.x),
                start.y + t * (end.y - start.y)
            )
            
            if self.will_collide_with_any_obstacle(check_point, obstacles):
                return False
        
        return True
    
    def predict_collision_along_path(self, path: List[Point], obstacles: List[Obstacle]) -> int:
        """Predict which waypoint index will cause collision, returns -1 if safe"""
        for i, waypoint in enumerate(path):
            if self.will_collide_with_any_obstacle(waypoint, obstacles):
                return i
        
        # Also check path segments
        for i in range(len(path) - 1):
            if not self.is_path_clear(path[i], path[i + 1], obstacles):
                return i + 1  # Return the target waypoint that's problematic
        
        return -1  # Path is clear
    
    def get_safe_position_near_point(self, target: Point, obstacles: List[Obstacle], 
                                   canvas_width: int, canvas_height: int, max_attempts: int = 16) -> Point:
        """Find a safe position near the target point"""
        if not self.will_collide_with_any_obstacle(target, obstacles):
            return target
        
        # Try positions in expanding circles around target
        search_radius = self.current_safety_margin * 2
        
        for attempt in range(max_attempts):
            for angle in range(0, 360, 22):  # Check 16 directions
                angle_rad = math.radians(angle)
                test_x = target.x + search_radius * math.cos(angle_rad)
                test_y = target.y + search_radius * math.sin(angle_rad)
                
                # Ensure within bounds
                if 0 <= test_x < canvas_width and 0 <= test_y < canvas_height:
                    test_point = Point(test_x, test_y)
                    if not self.will_collide_with_any_obstacle(test_point, obstacles):
                        return test_point
            
            search_radius += self.current_safety_margin  # Expand search
        
        # If no safe position found, return clamped original point
        return Point(
            max(self.robot_radius, min(canvas_width - self.robot_radius, target.x)),
            max(self.robot_radius, min(canvas_height - self.robot_radius, target.y))
        )
    
    def create_expanded_obstacles(self, obstacles: List[Obstacle]) -> List[Obstacle]:
        """Create expanded obstacles for pathfinding (includes robot radius + safety margin)"""
        expanded = []
        expansion = self.robot_radius + self.current_safety_margin
        
        for obs in obstacles:
            expanded_obstacle = Obstacle(
                obs.center.x, 
                obs.center.y, 
                obs.size + 2 * expansion  # Expand in all directions
            )
            expanded.append(expanded_obstacle)
        
        return expanded
    
    def get_avoidance_vector(self, robot_pos: Point, obstacles: List[Obstacle], max_distance: float = 100.0) -> Point:
        """Calculate avoidance vector using artificial potential field"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        for obstacle in obstacles:
            distance = euclidean_distance(robot_pos, obstacle.center)
            if distance > max_distance:
                continue
                
            # Calculate repulsive force (inverse square law)
            if distance > 0:
                force_magnitude = min(100.0, 1000.0 / (distance * distance))
                direction_x = (robot_pos.x - obstacle.center.x) / distance
                direction_y = (robot_pos.y - obstacle.center.y) / distance
                
                avoidance_x += force_magnitude * direction_x
                avoidance_y += force_magnitude * direction_y
        
        return Point(avoidance_x, avoidance_y)
    
    def record_collision(self, robot_pos: Point, obstacle_pos: Point = None):
        """Record collision for analysis and learning"""
        collision_info = {
            'position': robot_pos.copy(),
            'obstacle': obstacle_pos.copy() if obstacle_pos else None,
            'safety_margin': self.current_safety_margin
        }
        self.collision_history.append(collision_info)
        print(f"📝 Collision recorded at {robot_pos}")
    
    def analyze_collision_patterns(self) -> Dict[str, Any]:
        """Analyze collision patterns for learning"""
        if not self.collision_history:
            return {"total_collisions": 0}
        
        # Basic collision analysis
        total_collisions = len(self.collision_history)
        
        # Find collision hotspots
        hotspots = {}
        for collision in self.collision_history:
            pos = collision['position']
            # Group by approximate regions (50x50 pixel squares)
            region_x = int(pos.x // 50)
            region_y = int(pos.y // 50)
            region_key = f"{region_x},{region_y}"
            
            hotspots[region_key] = hotspots.get(region_key, 0) + 1
        
        return {
            "total_collisions": total_collisions,
            "collision_hotspots": hotspots,
            "current_safety_margin": self.current_safety_margin,
            "recommended_margin": min(50, self.base_safety_margin + total_collisions * 3)
        }
    
    @staticmethod
    def parse_obstacles_from_api(obstacles_data: List[Dict[str, Any]]) -> List[Obstacle]:
        """Convert obstacle data from API response to Obstacle objects"""
        obstacles = []
        for obs_data in obstacles_data:
            try:
                obstacle = Obstacle(
                    x=float(obs_data.get('x', 0)),
                    y=float(obs_data.get('y', 0)),
                    size=float(obs_data.get('size', 25))
                )
                obstacles.append(obstacle)
            except (ValueError, TypeError) as e:
                print(f"⚠️ Invalid obstacle data: {obs_data}, error: {e}")
                continue
        return obstacles

# Utility functions for collision testing
def test_collision_detection():
    """Test collision detection functionality"""
    print("🧪 Testing Collision Detection...")
    
    detector = CollisionDetector(robot_radius=18, safety_margin=10)
    
    # Create test obstacles
    obstacles = [
        Obstacle(100, 100, 25),
        Obstacle(200, 200, 25),
        Obstacle(300, 150, 25)
    ]
    
    # Test robot positions
    safe_position = Point(50, 50)
    collision_position = Point(105, 105)  # Too close to first obstacle
    
    safe_collision = detector.will_collide_with_any_obstacle(safe_position, obstacles)
    unsafe_collision = detector.will_collide_with_any_obstacle(collision_position, obstacles)
    
    print(f"Safe position collision: {safe_collision} (should be False)")
    print(f"Collision position collision: {unsafe_collision} (should be True)")
    
    # Test path clearance
    path_clear = detector.is_path_clear(Point(0, 0), Point(50, 50), obstacles)
    path_blocked = detector.is_path_clear(Point(90, 90), Point(110, 110), obstacles)
    
    print(f"Clear path: {path_clear} (should be True)")
    print(f"Blocked path: {path_blocked} (should be False)")
    
    # Test safe position finding
    safe_pos = detector.get_safe_position_near_point(collision_position, obstacles, 650, 600)
    print(f"Safe position near collision: {safe_pos}")
    
    return not safe_collision and unsafe_collision and path_clear and not path_blocked

if __name__ == "__main__":
    test_collision_detection()
