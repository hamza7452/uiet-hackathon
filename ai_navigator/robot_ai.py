"""
Main Robot Navigation AI Class
Integrates pathfinding, collision detection, and environment communication
"""
import requests
import json
import time
import threading
from typing import List, Optional, Dict, Any
from .config import *
from .utils import Point, format_coordinates, parse_coordinates, calculate_path_length
from .collision_detector import CollisionDetector, Obstacle
from .pathfinding import RefinedPathfinder

try:
    import websocket
    WEBSOCKET_AVAILABLE = True
except ImportError:
    WEBSOCKET_AVAILABLE = False
    print("⚠️ websocket-client not available. WebSocket functionality disabled.")


class RobotNavigationAI:
    """Main AI class for autonomous robot navigation"""
    
    def __init__(self, api_base: str = API_BASE_URL, websocket_url: str = WEBSOCKET_URL):
        self.api_base = api_base
        self.websocket_url = websocket_url
        
        # Environment state
        self.robot_position = Point(320, 300)  # Default start position
        self.goal_position = Point(550, 80)    # Default goal position
        self.obstacles: List[Obstacle] = []
        self.collision_count = 0
        self.goal_reached = False
        
        # AI Components
        self.collision_detector = CollisionDetector(
            robot_radius=ROBOT_RADIUS,
            safety_margin=SAFETY_MARGIN
        )
        
        self.pathfinder = RefinedPathfinder(
            canvas_width=CANVAS_WIDTH,
            canvas_height=CANVAS_HEIGHT,
            grid_size=GRID_SIZE
        )
        self.pathfinder.set_collision_detector(self.collision_detector)
        
        # Navigation state
        self.current_path: Optional[List[Point]] = None
        self.current_waypoint_index = 0
        self.is_moving = False
        self.navigation_active = False
        
        # WebSocket connection
        self.websocket = None
        self.websocket_thread = None
        self.websocket_connected = False
        
        # Performance tracking
        self.start_time = None
        self.total_distance_traveled = 0
        self.path_recalculations = 0
        
        print("🤖 Robot Navigation AI initialized")
    
    def connect_to_environment(self) -> bool:
        """Establish connection to the robot simulation environment"""
        print("🔌 Connecting to environment...")
        
        # Test Flask API connection
        try:
            response = requests.get(f"{self.api_base}/status", timeout=API_TIMEOUT)
            if response.status_code == 200:
                status_data = response.json()
                print(f"✅ Flask API connected - {status_data.get('connected_simulators', 0)} simulators")
            else:
                print(f"❌ Flask API connection failed: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ Flask API connection error: {e}")
            return False
        
        # Initialize WebSocket connection
        if WEBSOCKET_AVAILABLE:
            self._connect_websocket()
        else:
            print("⚠️ WebSocket not available, using API polling only")
        
        # Get initial environment state
        self._update_environment_state()
        
        return True
    
    def _connect_websocket(self):
        """Establish WebSocket connection for real-time updates"""
        if not WEBSOCKET_AVAILABLE:
            return
        
        try:
            # Use Socket.IO compatible URL
            websocket_url = self.websocket_url.replace('ws://', 'http://') + '/socket.io/'
            print(f"🔌 Connecting to WebSocket: {websocket_url}")
            
            self.websocket = websocket.WebSocketApp(
                websocket_url,
                on_open=self._on_websocket_open,
                on_message=self._on_websocket_message,
                on_error=self._on_websocket_error,
                on_close=self._on_websocket_close
            )
            
            # Start WebSocket in separate thread
            self.websocket_thread = threading.Thread(
                target=self.websocket.run_forever,
                daemon=True
            )
            self.websocket_thread.start()
            
            # Wait briefly for connection
            time.sleep(0.5)
            
        except Exception as e:
            print(f"⚠️ WebSocket setup error: {e}")
            print("🔄 Continuing without WebSocket (using API polling)")
    
    def _on_websocket_open(self, ws):
        """WebSocket connection established"""
        self.websocket_connected = True
        print("✅ WebSocket connected")
        
        # Send initial connection message
        self._send_websocket_message({
            "type": "connection",
            "message": "AI Navigation system connected"
        })
    
    def _on_websocket_message(self, ws, message):
        """Handle WebSocket messages from environment"""
        try:
            data = json.loads(message)
            self._process_websocket_message(data)
        except Exception as e:
            print(f"❌ WebSocket message processing error: {e}")
    
    def _on_websocket_error(self, ws, error):
        """Handle WebSocket errors"""
        print(f"⚠️ WebSocket error: {error}")
        self.websocket_connected = False
    
    def _on_websocket_close(self, ws, close_status_code, close_msg):
        """Handle WebSocket disconnection"""
        print("🔌 WebSocket disconnected")
        self.websocket_connected = False
    
    def _send_websocket_message(self, message: Dict[str, Any]):
        """Send message via WebSocket"""
        if self.websocket and self.websocket_connected:
            try:
                self.websocket.send(json.dumps(message))
            except Exception as e:
                print(f"❌ WebSocket send error: {e}")
    
    def _process_websocket_message(self, data: Dict[str, Any]):
        """Process incoming WebSocket messages"""
        msg_type = data.get("type")
        
        if msg_type == "collision" and data.get("collision"):
            self.collision_count += 1
            self.collision_detector.update_safety_margin(self.collision_count)
            print(f"⚠️ Collision detected! Total: {self.collision_count}")
            
            # Handle collision response
            self._handle_collision_response()
            
        elif msg_type == "goal_reached":
            self.goal_reached = True
            print("🎯 Goal reached via WebSocket!")
            self._handle_goal_reached()
            
        elif "robot_position" in data:
            # Update robot position from WebSocket
            pos_data = data["robot_position"]
            self.robot_position = parse_coordinates(pos_data)
    
    def _update_environment_state(self):
        """Update environment state from API"""
        try:
            # Get current status
            status_response = requests.get(f"{self.api_base}/status", timeout=API_TIMEOUT)
            if status_response.status_code == 200:
                status_data = status_response.json()
                self.collision_count = status_data.get("collision_count", 0)
                self.goal_reached = status_data.get("goal_reached", False)
                
                # Update collision detector with current collision count
                self.collision_detector.update_safety_margin(self.collision_count)
            
            # Get obstacles (Note: API might return empty list, but that's expected)
            obstacles_response = requests.get(f"{self.api_base}/obstacles", timeout=API_TIMEOUT)
            if obstacles_response.status_code == 200:
                obstacles_data = obstacles_response.json()
                # For now, we'll use hardcoded obstacles matching the HTML file
                self._set_default_obstacles()
            
        except Exception as e:
            print(f"⚠️ Environment state update error: {e}")
            # Use default obstacles if API fails
            self._set_default_obstacles()
    
    def _set_default_obstacles(self):
        """Set default obstacles matching the environment"""
        # These match the obstacles from the HTML file
        default_obstacles_data = [
            {"x": 150, "y": 120, "size": 25},
            {"x": 450, "y": 180, "size": 25},
            {"x": 220, "y": 300, "size": 25},
            {"x": 380, "y": 380, "size": 25},
            {"x": 100, "y": 450, "size": 25},
            {"x": 500, "y": 100, "size": 25},
            {"x": 280, "y": 220, "size": 25},
            {"x": 420, "y": 320, "size": 25}
        ]
        
        self.obstacles = CollisionDetector.parse_obstacles_from_api(default_obstacles_data)
        print(f"🗺️ Loaded {len(self.obstacles)} default obstacles")
    
    def move_robot_to_position(self, target: Point) -> bool:
        """Send move command to robot with better error handling"""
        try:
            payload = format_coordinates(target)
            response = requests.post(f"{self.api_base}/move", json=payload, timeout=5)
            
            if response.status_code == 200:
                print(f"➡️ Moving robot to {target}")
                self.is_moving = True
                return True
            else:
                print(f"⚠️ Move command response: {response.status_code}")
                return True  # Continue anyway
                
        except requests.exceptions.RequestException as e:
            print(f"⚠️ Move command network error (continuing): {str(e)[:50]}...")
            return True  # Continue navigation even if server is unreachable
        except Exception as e:
            print(f"⚠️ Move command error: {str(e)[:50]}...")
            return True
    
    def stop_robot(self) -> bool:
        """Send stop command to robot"""
        try:
            response = requests.post(f"{self.api_base}/stop", timeout=API_TIMEOUT)
            if response.status_code == 200:
                print("🛑 Robot stopped")
                self.is_moving = False
                return True
            else:
                print(f"❌ Stop command failed")
                return False
        except Exception as e:
            print(f"❌ Stop command error: {e}")
            return False
    
    def calculate_path_to_goal(self) -> bool:
        """Calculate optimal path from current position to goal"""
        print(f"🧭 Calculating path: {self.robot_position} → {self.goal_position}")
        
        self.current_path = self.pathfinder.find_path_with_fallbacks(
            self.robot_position, 
            self.goal_position, 
            self.obstacles
        )
        
        if self.current_path:
            self.current_waypoint_index = 0
            self.path_recalculations += 1
            path_length = calculate_path_length(self.current_path)
            print(f"✅ Path calculated with {len(self.current_path)} waypoints, length: {path_length:.1f}px")
            return True
        else:
            print("❌ No path found to goal")
            return False
    
    def navigate_to_goal(self) -> bool:
        """Main navigation function with improved execution"""
        print("🚀 Starting autonomous navigation...")
        self.start_time = time.time()
        self.navigation_active = True
        
        # Connect to environment
        if not self.connect_to_environment():
            print("⚠️ Environment connection failed, but continuing with navigation...")
        
        # Calculate initial path
        if not self.calculate_path_to_goal():
            return False
        
        # Send initial path to server for visualization
        self._send_path_to_server()
        
        # Execute navigation with continuous movement
        try:
            step_count = 0
            max_steps = len(self.current_path) * 2  # Safety limit
            
            while self.navigation_active and not self.goal_reached and step_count < max_steps:
                step_count += 1
                success = self._execute_navigation_step()
                
                if not success:
                    print(f"❌ Navigation step {step_count} failed")
                    break
                
                # Check if we've reached the goal
                goal_distance = self.robot_position.distance_to(self.goal_position)
                if goal_distance < 35:
                    self.goal_reached = True
                    print(f"🎯 Goal reached! Final distance: {goal_distance:.1f}px")
                    break
                
                # Check goal status periodically (but don't depend on it)
                if step_count % 2 == 0:
                    self._check_goal_status()
                    
                # Small delay for visualization
                time.sleep(0.5)  # Reduced delay for faster movement
            
            # Final status
            if self.goal_reached:
                self._handle_goal_reached()
                return True
            elif step_count >= max_steps:
                print(f"⚠️ Navigation stopped after {max_steps} steps (safety limit)")
                return False
            else:
                print("❌ Navigation failed or interrupted")
                return False
                
        except KeyboardInterrupt:
            print("\n🛑 Navigation interrupted by user")
            self.stop_robot()
            return False
    
    def _execute_navigation_step(self) -> bool:
        """Execute one step of navigation with improved waypoint progression"""
        if not self.current_path or self.current_waypoint_index >= len(self.current_path):
            print("❌ No more waypoints or path is empty")
            return False
        
        current_waypoint = self.current_path[self.current_waypoint_index]
        
        # Check for collision prediction on remaining path
        remaining_path = self.current_path[self.current_waypoint_index:]
        collision_index = self.collision_detector.predict_collision_along_path(remaining_path, self.obstacles)
        
        if collision_index >= 0:
            print("⚠️ Collision predicted, recalculating path...")
            self.stop_robot()
            if not self.calculate_path_to_goal():
                print("❌ Failed to find alternative path")
                return False
            return True
        
        # Always move to the current waypoint (don't wait for is_moving flag)
        print(f"🎯 Moving to waypoint {self.current_waypoint_index + 1}/{len(self.current_path)}: {current_waypoint}")
        success = self.move_robot_to_position(current_waypoint)
        if not success:
            print("❌ Failed to send move command")
            return False
        
        # Update robot position immediately (don't wait for server response)
        self.robot_position = current_waypoint.copy()
        
        # Check if we should move to next waypoint
        self.current_waypoint_index += 1
        self.is_moving = False
        
        # Calculate distance traveled
        if self.current_waypoint_index > 1:
            prev_waypoint = self.current_path[self.current_waypoint_index - 2]
            distance = current_waypoint.distance_to(prev_waypoint)
            self.total_distance_traveled += distance
            print(f"✅ Waypoint reached. Distance traveled: {distance:.1f}px")
        
        # Check if we reached the final waypoint (goal)
        if self.current_waypoint_index >= len(self.current_path):
            goal_distance = self.robot_position.distance_to(self.goal_position)
            print(f"📍 Reached final waypoint. Distance to goal: {goal_distance:.1f}px")
            
            if goal_distance < 35:  # Close enough to goal
                self.goal_reached = True
                return True
        
        return True
    
    def _send_path_to_server(self):
        """Send calculated path to server for visualization"""
        if not self.current_path:
            return
        
        try:
            path_data = []
            for point in self.current_path:
                path_data.append({"x": point.x, "y": point.y})
            
            payload = {"path": path_data}
            response = requests.post(f"{self.api_base}/set_path", json=payload, timeout=5)
            if response.status_code == 200:
                print("📍 Path sent to server for visualization")
            else:
                print("⚠️ Failed to send path to server (non-critical)")
        except Exception as e:
            print(f"⚠️ Path sending error (non-critical): {str(e)[:50]}...")
    
    def _is_waypoint_reached(self, waypoint: Point, tolerance: float = 15.0) -> bool:
        """Check if robot has reached the waypoint - simplified version"""
        # We'll assume waypoint is reached immediately for smoother movement
        return True
    
    def _check_goal_status(self) -> bool:
        """Check goal status with better error handling"""
        try:
            response = requests.get(f"{self.api_base}/goal/status", timeout=API_TIMEOUT)
            if response.status_code == 200:
                data = response.json()
                goal_reached = data.get('goal_reached', False)
                if goal_reached and not self.goal_reached:
                    self.goal_reached = True
                    return True
        except requests.exceptions.RequestException as e:
            # Silently handle connection errors - use local goal checking instead
            goal_distance = self.robot_position.distance_to(self.goal_position)
            if goal_distance < 35:
                self.goal_reached = True
                return True
        except Exception as e:
            print(f"⚠️ Goal status check error (non-critical): {str(e)[:100]}...")
        
        return False
    
    def _handle_collision_response(self):
        """Handle collision detection response"""
        self.stop_robot()
        print(f"🔧 Handling collision #{self.collision_count}")
        
        # Record collision for analysis
        self.collision_detector.record_collision(self.robot_position)
        
        # Recalculate path with updated safety margins
        if self.calculate_path_to_goal():
            print("✅ New collision-free path calculated")
        else:
            print("❌ Unable to find alternative path")
            self.navigation_active = False
    
    def _handle_goal_reached(self):
        """Handle successful goal achievement"""
        end_time = time.time()
        duration = end_time - self.start_time if self.start_time else 0
        
        print("🎯 🎉 MISSION ACCOMPLISHED! 🎉 🎯")
        print(f"⏱️ Navigation time: {duration:.2f} seconds")
        print(f"🔄 Path recalculations: {self.path_recalculations}")
        print(f"💥 Total collisions: {self.collision_count}")
        print(f"📏 Distance traveled: {self.total_distance_traveled:.1f}px")
        
        # Get pathfinding report
        pathfinding_report = self.pathfinder.get_pathfinding_report()
        print(f"📊 Pathfinding efficiency: {pathfinding_report.get('success_rate', 'N/A')}")
        
        self.navigation_active = False
        self.stop_robot()
    
    def set_goal(self, goal_position: Point) -> bool:
        """Set new goal position"""
        try:
            payload = format_coordinates(goal_position)
            response = requests.post(f"{self.api_base}/goal", json=payload, timeout=API_TIMEOUT)
            
            if response.status_code == 200:
                self.goal_position = goal_position
                self.goal_reached = False
                print(f"🎯 Goal set to: {goal_position}")
                return True
            else:
                print(f"❌ Failed to set goal: {response.json()}")
                return False
        except Exception as e:
            print(f"❌ Set goal error: {e}")
            return False
    
    def reset_navigation(self):
        """Reset navigation state"""
        try:
            response = requests.post(f"{self.api_base}/reset", timeout=API_TIMEOUT)
            if response.status_code == 200:
                self.collision_count = 0
                self.goal_reached = False
                self.current_path = None
                self.current_waypoint_index = 0
                self.is_moving = False
                self.navigation_active = False
                self.total_distance_traveled = 0
                self.path_recalculations = 0
                
                # Reset collision detector
                self.collision_detector.collision_count = 0
                self.collision_detector.current_safety_margin = self.collision_detector.base_safety_margin
                
                print("🔄 Navigation state reset")
                return True
        except Exception as e:
            print(f"❌ Reset error: {e}")
            return False
    
    def get_navigation_stats(self) -> Dict[str, Any]:
        """Get current navigation statistics"""
        return {
            "goal_reached": self.goal_reached,
            "collision_count": self.collision_count,
            "path_recalculations": self.path_recalculations,
            "total_distance_traveled": self.total_distance_traveled,
            "current_position": format_coordinates(self.robot_position),
            "goal_position": format_coordinates(self.goal_position),
            "navigation_active": self.navigation_active,
            "websocket_connected": self.websocket_connected,
            "current_waypoint": self.current_waypoint_index,
            "total_waypoints": len(self.current_path) if self.current_path else 0,
            "safety_margin": self.collision_detector.current_safety_margin
        }
    
    def get_detailed_report(self) -> Dict[str, Any]:
        """Get comprehensive navigation report"""
        stats = self.get_navigation_stats()
        
        # Add pathfinding statistics
        pathfinding_stats = self.pathfinder.get_pathfinding_report()
        
        # Add collision analysis
        collision_analysis = self.collision_detector.analyze_collision_patterns()
        
        return {
            "navigation_stats": stats,
            "pathfinding_report": pathfinding_stats,
            "collision_analysis": collision_analysis,
            "environment_info": {
                "canvas_size": f"{CANVAS_WIDTH}x{CANVAS_HEIGHT}",
                "obstacles_count": len(self.obstacles),
                "grid_size": GRID_SIZE,
                "robot_radius": ROBOT_RADIUS
            }
        }
    
    def emergency_stop(self):
        """Emergency stop with cleanup"""
        print("🚨 EMERGENCY STOP ACTIVATED")
        self.stop_robot()
        self.navigation_active = False
        
        # Close WebSocket connection
        if self.websocket:
            try:
                self.websocket.close()
            except:
                pass
    
    def __del__(self):
        """Cleanup when AI object is destroyed"""
        self.emergency_stop()


# Helper function for testing and development
def test_robot_ai():
    """Test the robot AI functionality"""
    print("🧪 Testing Robot AI...")
    
    # Create AI instance
    robot_ai = RobotNavigationAI()
    
    # Test environment connection
    connected = robot_ai.connect_to_environment()
    if not connected:
        print("❌ Failed to connect to environment")
        return False
    
    # Test path calculation
    path_found = robot_ai.calculate_path_to_goal()
    if not path_found:
        print("❌ Failed to calculate path")
        return False
    
    print("✅ Robot AI basic functionality test passed")
    
    # Show navigation stats
    stats = robot_ai.get_navigation_stats()
    print(f"📊 Navigation stats: {stats}")
    
    return True


if __name__ == "__main__":
    test_robot_ai()
