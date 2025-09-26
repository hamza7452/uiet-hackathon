"""
A* Pathfinding Algorithm Implementation for Robot Navigation
"""
import heapq
import math
from typing import List, Dict, Set, Optional, Tuple
from .utils import Point, Node, euclidean_distance, is_point_in_bounds, format_point_list
from .collision_detector import Obstacle, CollisionDetector


class AStarPathfinder:
    """A* pathfinding implementation with obstacle avoidance"""
    
    def __init__(self, canvas_width: int, canvas_height: int, grid_size: int = 10):
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.grid_size = grid_size
        self.collision_detector = None
        
        # Grid dimensions
        self.grid_width = canvas_width // grid_size
        self.grid_height = canvas_height // grid_size
        
        # Statistics
        self.last_search_iterations = 0
        self.last_search_time = 0
        
        print(f"🗺️ A* Grid initialized: {self.grid_width}x{self.grid_height} (grid_size={grid_size})")
    
    def set_collision_detector(self, collision_detector: CollisionDetector):
        """Set the collision detector for obstacle checking"""
        self.collision_detector = collision_detector
    
    def grid_to_world(self, grid_point: Point) -> Point:
        """Convert grid coordinates to world coordinates"""
        return Point(grid_point.x * self.grid_size, grid_point.y * self.grid_size)
    
    def world_to_grid(self, world_point: Point) -> Point:
        """Convert world coordinates to grid coordinates"""
        return Point(
            int(world_point.x // self.grid_size),
            int(world_point.y // self.grid_size)
        )
    
    def snap_to_grid(self, point: Point) -> Point:
        """Snap a world point to the nearest grid point"""
        grid_point = self.world_to_grid(point)
        return self.grid_to_world(grid_point)
    
    def heuristic(self, current: Point, goal: Point, heuristic_type: str = "euclidean") -> float:
        """Heuristic function for A*"""
        if heuristic_type == "manhattan":
            return abs(current.x - goal.x) + abs(current.y - goal.y)
        elif heuristic_type == "diagonal":
            dx = abs(current.x - goal.x)
            dy = abs(current.y - goal.y)
            return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
        else:  # euclidean (default)
            return euclidean_distance(current, goal)
    
    def get_movement_cost(self, from_point: Point, to_point: Point) -> float:
        """Calculate movement cost between two points"""
        dx = abs(from_point.x - to_point.x)
        dy = abs(from_point.y - to_point.y)
        
        # Diagonal movement
        if dx == self.grid_size and dy == self.grid_size:
            return math.sqrt(2) * self.grid_size
        # Straight movement
        else:
            return self.grid_size
    
    def is_valid_grid_point(self, grid_point: Point) -> bool:
        """Check if grid point is within bounds"""
        return (0 <= grid_point.x < self.grid_width and 
                0 <= grid_point.y < self.grid_height)
    
    def is_world_point_safe(self, world_point: Point, obstacles: List[Obstacle]) -> bool:
        """Check if a world point is safe from obstacles"""
        if not self.collision_detector:
            return True
        
        return not self.collision_detector.will_collide_with_any_obstacle(world_point, obstacles)
    
    def get_valid_neighbors(self, current_grid: Point, obstacles: List[Obstacle]) -> List[Point]:
        """Get valid neighboring grid points that are safe from obstacles"""
        neighbors = []
        
        # 8-directional movement
        directions = [
            (-1, -1), (-1, 0), (-1, 1),  # Top row
            (0, -1),           (0, 1),   # Middle row (skip center)
            (1, -1),  (1, 0),  (1, 1)    # Bottom row
        ]
        
        for dx, dy in directions:
            new_grid_x = current_grid.x + dx
            new_grid_y = current_grid.y + dy
            new_grid_point = Point(new_grid_x, new_grid_y)
            
            # Check bounds
            if not self.is_valid_grid_point(new_grid_point):
                continue
            
            # Convert to world coordinates for collision checking
            world_point = self.grid_to_world(new_grid_point)
            
            # Check collision with obstacles
            if self.is_world_point_safe(world_point, obstacles):
                neighbors.append(new_grid_point)
        
        return neighbors
    
    def reconstruct_path(self, came_from: Dict[Point, Point], current: Point) -> List[Point]:
        """Reconstruct path from came_from dictionary"""
        path = []
        
        while current is not None:
            # Convert grid point back to world coordinates
            world_point = self.grid_to_world(current)
            path.append(world_point)
            current = came_from.get(current)
        
        path.reverse()
        return path
    
    def find_path(self, start: Point, goal: Point, obstacles: List[Obstacle], 
                  heuristic_type: str = "euclidean") -> Optional[List[Point]]:
        """
        Find optimal path from start to goal using A* algorithm
        Returns list of world coordinate points, or None if no path found
        """
        import time
        start_time = time.time()
        
        print(f"🔍 A* Pathfinding: {start} → {goal}")
        
        # Convert world coordinates to grid coordinates
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)
        
        # Ensure start and goal are within bounds
        if not (self.is_valid_grid_point(start_grid) and self.is_valid_grid_point(goal_grid)):
            print("❌ Start or goal point is out of bounds")
            return None
        
        # Check if start and goal positions are safe
        if not (self.is_world_point_safe(start, obstacles) and self.is_world_point_safe(goal, obstacles)):
            print("❌ Start or goal position is in collision with obstacles")
            return None
        
        # A* algorithm implementation with counter for tie-breaking
        counter = 0
        open_set = []  # Priority queue: (f_score, counter, grid_point)
        closed_set: Set[Point] = set()  # Explored nodes
        came_from: Dict[Point, Point] = {}  # Path reconstruction
        
        # Cost tracking
        g_score: Dict[Point, float] = {start_grid: 0}
        f_score: Dict[Point, float] = {start_grid: self.heuristic(start, goal, heuristic_type)}
        
        counter += 1
        heapq.heappush(open_set, (f_score[start_grid], counter, start_grid))
        
        iterations = 0
        max_iterations = 10000
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            # Get node with lowest f_score (now includes counter for tie-breaking)
            current_f, _, current_grid = heapq.heappop(open_set)
            
            # Check if we reached the goal
            if current_grid.x == goal_grid.x and current_grid.y == goal_grid.y:
                end_time = time.time()
                self.last_search_time = end_time - start_time
                self.last_search_iterations = iterations
                print(f"✅ Path found in {iterations} iterations ({self.last_search_time:.3f}s)")
                path = self.reconstruct_path(came_from, current_grid)
                return path
            
            # Add current to closed set
            closed_set.add(current_grid)
            
            # Explore neighbors
            for neighbor_grid in self.get_valid_neighbors(current_grid, obstacles):
                if neighbor_grid in closed_set:
                    continue
                
                # Calculate tentative g_score
                current_world = self.grid_to_world(current_grid)
                neighbor_world = self.grid_to_world(neighbor_grid)
                tentative_g = g_score[current_grid] + self.get_movement_cost(current_world, neighbor_world)
                
                # Check if this path to neighbor is better than previous
                if tentative_g < g_score.get(neighbor_grid, float('inf')):
                    # Update path
                    came_from[neighbor_grid] = current_grid
                    g_score[neighbor_grid] = tentative_g
                    f_score[neighbor_grid] = tentative_g + self.heuristic(neighbor_world, goal, heuristic_type)
                    
                    # Add to open set if not already there (updated to check third element)
                    if not any(neighbor_grid.x == item[2].x and neighbor_grid.y == item[2].y for item in open_set):
                        counter += 1
                        heapq.heappush(open_set, (f_score[neighbor_grid], counter, neighbor_grid))
        
        end_time = time.time()
        self.last_search_time = end_time - start_time
        self.last_search_iterations = iterations
        print(f"❌ No path found after {iterations} iterations ({self.last_search_time:.3f}s)")
        return None
    
    def smooth_path(self, path: List[Point], obstacles: List[Obstacle]) -> List[Point]:
        """Smooth the path by removing unnecessary waypoints"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]  # Always keep start
        i = 0
        
        while i < len(path) - 1:
            # Look ahead to find the furthest directly reachable point
            furthest_reachable = i + 1
            
            for j in range(i + 2, len(path)):
                if self.collision_detector and self.collision_detector.is_path_clear(path[i], path[j], obstacles):
                    furthest_reachable = j
                else:
                    break
            
            smoothed.append(path[furthest_reachable])
            i = furthest_reachable
        
        print(f"🔧 Path smoothed: {len(path)} → {len(smoothed)} waypoints")
        return smoothed
    
    def get_search_statistics(self) -> Dict[str, float]:
        """Get statistics from the last search"""
        return {
            "iterations": self.last_search_iterations,
            "search_time": self.last_search_time,
            "grid_size": self.grid_size,
            "grid_dimensions": f"{self.grid_width}x{self.grid_height}"
        }


class RefinedPathfinder(AStarPathfinder):
    """Enhanced pathfinder with additional optimization techniques"""
    
    def __init__(self, canvas_width: int, canvas_height: int, grid_size: int = 10):
        super().__init__(canvas_width, canvas_height, grid_size)
        self.failed_attempts = 0
        self.fallback_strategies_used = []
    
    def find_path_with_fallbacks(self, start: Point, goal: Point, obstacles: List[Obstacle]) -> Optional[List[Point]]:
        """Find path with multiple fallback strategies"""
        self.fallback_strategies_used = []
        
        # Strategy 1: Direct A* with current grid size
        print("🎯 Strategy 1: Standard A* pathfinding")
        path = self.find_path(start, goal, obstacles)
        if path:
            smoothed_path = self.smooth_path(path, obstacles)
            self.fallback_strategies_used.append("standard_astar")
            return smoothed_path
        
        print("🔄 A* failed, trying fallback strategies...")
        self.failed_attempts += 1
        
        # Strategy 2: Try with different heuristics
        for heuristic_type in ["manhattan", "diagonal"]:
            print(f"🔧 Strategy 2a: Trying {heuristic_type} heuristic")
            path = self.find_path(start, goal, obstacles, heuristic_type)
            if path:
                smoothed_path = self.smooth_path(path, obstacles)
                self.fallback_strategies_used.append(f"heuristic_{heuristic_type}")
                return smoothed_path
        
        # Strategy 3: Try with finer grid resolution
        if self.grid_size > 5:
            print("🔧 Strategy 3: Trying finer grid resolution")
            original_grid_size = self.grid_size
            self.grid_size = max(5, self.grid_size // 2)
            self._recalculate_grid_dimensions()
            
            path = self.find_path(start, goal, obstacles)
            
            # Restore original grid size
            self.grid_size = original_grid_size
            self._recalculate_grid_dimensions()
            
            if path:
                smoothed_path = self.smooth_path(path, obstacles)
                self.fallback_strategies_used.append("fine_grid")
                return smoothed_path
        
        # Strategy 4: Try intermediate waypoint
        print("🔧 Strategy 4: Trying intermediate waypoint strategy")
        intermediate_path = self._find_path_via_intermediate_point(start, goal, obstacles)
        if intermediate_path:
            self.fallback_strategies_used.append("intermediate_waypoint")
            return intermediate_path
        
        # Strategy 5: Try safe goal position
        print("🔧 Strategy 5: Trying safe goal position")
        if self.collision_detector:
            safe_goal = self.collision_detector.get_safe_position_near_point(
                goal, obstacles, self.canvas_width, self.canvas_height
            )
            if safe_goal != goal:
                path = self.find_path(start, safe_goal, obstacles)
                if path:
                    smoothed_path = self.smooth_path(path, obstacles)
                    self.fallback_strategies_used.append("safe_goal")
                    return smoothed_path
        
        print("❌ All pathfinding strategies failed")
        return None
    
    def _recalculate_grid_dimensions(self):
        """Recalculate grid dimensions after grid size change"""
        self.grid_width = self.canvas_width // self.grid_size
        self.grid_height = self.canvas_height // self.grid_size
    
    def _find_path_via_intermediate_point(self, start: Point, goal: Point, obstacles: List[Obstacle]) -> Optional[List[Point]]:
        """Try to find path via an intermediate safe point"""
        # Find multiple intermediate points to try
        intermediate_candidates = self._generate_intermediate_candidates(start, goal, obstacles)
        
        for intermediate in intermediate_candidates:
            # Check if intermediate point is safe
            if not self.is_world_point_safe(intermediate, obstacles):
                continue
            
            # Try path: start → intermediate → goal
            path1 = self.find_path(start, intermediate, obstacles)
            if path1:
                path2 = self.find_path(intermediate, goal, obstacles)
                if path2:
                    # Combine paths (remove duplicate intermediate point)
                    combined_path = path1 + path2[1:]
                    smoothed_path = self.smooth_path(combined_path, obstacles)
                    print(f"✅ Path found via intermediate point: {intermediate}")
                    return smoothed_path
        
        return None
    
    def _generate_intermediate_candidates(self, start: Point, goal: Point, obstacles: List[Obstacle]) -> List[Point]:
        """Generate candidate intermediate points"""
        candidates = []
        
        # Midpoint variations
        mid_x = (start.x + goal.x) / 2
        mid_y = (start.y + goal.y) / 2
        
        # Try points around the midpoint
        for radius in [30, 60, 100]:
            for angle in range(0, 360, 45):
                angle_rad = math.radians(angle)
                candidate = Point(
                    mid_x + radius * math.cos(angle_rad),
                    mid_y + radius * math.sin(angle_rad)
                )
                
                # Ensure within bounds
                if is_point_in_bounds(candidate, self.canvas_width, self.canvas_height, 20):
                    candidates.append(candidate)
        
        # Try corners as intermediate points
        corner_margin = 50
        corners = [
            Point(corner_margin, corner_margin),  # Top-left
            Point(self.canvas_width - corner_margin, corner_margin),  # Top-right
            Point(corner_margin, self.canvas_height - corner_margin),  # Bottom-left
            Point(self.canvas_width - corner_margin, self.canvas_height - corner_margin)  # Bottom-right
        ]
        candidates.extend(corners)
        
        return candidates
    
    def get_pathfinding_report(self) -> Dict[str, any]:
        """Get detailed pathfinding report"""
        stats = self.get_search_statistics()
        return {
            **stats,
            "failed_attempts": self.failed_attempts,
            "fallback_strategies_used": self.fallback_strategies_used,
            "success_rate": f"{max(0, 100 - self.failed_attempts * 20):.1f}%"
        }


# Utility functions for testing pathfinding
def test_pathfinding():
    """Test pathfinding functionality"""
    print("🧪 Testing A* Pathfinding...")
    
    # Create pathfinder
    pathfinder = RefinedPathfinder(650, 600, grid_size=10)
    
    # Create collision detector
    collision_detector = CollisionDetector(18, 10)
    pathfinder.set_collision_detector(collision_detector)
    
    # Create test obstacles
    obstacles = [
        Obstacle(200, 200, 50),
        Obstacle(400, 300, 50),
        Obstacle(300, 450, 50)
    ]
    
    # Test path calculation
    start = Point(50, 50)
    goal = Point(550, 550)
    
    path = pathfinder.find_path_with_fallbacks(start, goal, obstacles)
    
    if path:
        print(f"✅ Path found with {len(path)} waypoints")
        print(f"📊 Path details: {format_point_list(path, 3)}")
        
        # Test path length calculation
        from .utils import calculate_path_length
        path_length = calculate_path_length(path)
        print(f"📏 Path length: {path_length:.1f} pixels")
        
        # Show statistics
        report = pathfinder.get_pathfinding_report()
        print(f"📈 Pathfinding report: {report}")
        
        return True
    else:
        print("❌ No path found")
        return False


if __name__ == "__main__":
    test_pathfinding()
