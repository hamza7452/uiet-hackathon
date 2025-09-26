#!/usr/bin/env python3
"""
Main execution script for Robot Navigation AI
"""
import sys
import os

# Add project root to Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from ai_navigator.robot_ai import RobotNavigationAI

def main():
    print("🤖 AI Robot Navigation System")
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
            return 0
        else:
            print("\n❌ Navigation failed")
            return 1
            
    except KeyboardInterrupt:
        print("\n🛑 Navigation interrupted")
        robot_ai.stop_robot()
        return 1

if __name__ == "__main__":
    exit(main())
