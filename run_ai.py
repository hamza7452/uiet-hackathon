"""
Direct AI Runner - No import issues
"""
import sys
import os

# Get the absolute path to the project directory
project_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_dir)

print(f"Project directory: {project_dir}")
print("Python path updated")

# Test the imports step by step
try:
    print("Testing imports...")
    
    from ai_navigator.utils import Point
    print("✅ utils imported")
    
    from ai_navigator.config import API_BASE_URL, WEBSOCKET_URL
    print("✅ config imported")
    
    from ai_navigator.collision_detector import CollisionDetector, Obstacle
    print("✅ collision_detector imported")
    
    from ai_navigator.pathfinding import RefinedPathfinder
    print("✅ pathfinding imported")
    
    from ai_navigator.robot_ai import RobotNavigationAI
    print("✅ robot_ai imported")
    
    print("\n🚀 All imports successful!")
    
except ImportError as e:
    print(f"❌ Import failed: {e}")
    print("\nChecking file structure:")
    print(f"Project dir: {project_dir}")
    
    ai_nav_dir = os.path.join(project_dir, "ai_navigator")
    if os.path.exists(ai_nav_dir):
        print(f"✅ ai_navigator directory exists")
        files = os.listdir(ai_nav_dir)
        print(f"Files in ai_navigator: {files}")
    else:
        print(f"❌ ai_navigator directory NOT found at: {ai_nav_dir}")
    
    sys.exit(1)

def main():
    print("\n🤖 AI Robot Navigation System")
    print("=" * 40)
    
    # Initialize AI
    robot_ai = RobotNavigationAI()
    
    try:
        # Start autonomous navigation
        success = robot_ai.navigate_to_goal()
        
        # Print final statistics
        stats = robot_ai.get_navigation_stats()
        print("\n📊 Final Statistics:")
        for key, value in stats.items():
            print(f"  {key}: {value}")
        
        if success:
            print("\n🎯 Navigation completed successfully!")
        else:
            print("\n❌ Navigation failed")
            
    except KeyboardInterrupt:
        print("\n🛑 Navigation interrupted")
        robot_ai.stop_robot()

if __name__ == "__main__":
    main()
