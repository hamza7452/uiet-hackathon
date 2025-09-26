#!/usr/bin/env python3
"""
Run navigation with custom sim-11 configuration
"""
import sys
import os
import argparse

# Add project root to Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from ai_navigator.robot_ai import RobotNavigationAI

def main():
    parser = argparse.ArgumentParser(description="Run AI with sim-11")
    parser.add_argument("--api-port", default=5001, type=int, help="sim-11 API port")
    parser.add_argument("--ws-port", default=8080, type=int, help="sim-11 WebSocket port")
    parser.add_argument("--host", default="localhost", help="sim-11 host")
    
    args = parser.parse_args()
    
    print("🤖 AI Robot Navigation System (sim-11 mode)")
    print("=" * 45)
    print(f"Connecting to sim-11 at {args.host}:{args.api_port}")
    
    # Create custom API URLs
    api_base = f"http://{args.host}:{args.api_port}"
    websocket_url = f"ws://{args.host}:{args.ws_port}"
    
    # Initialize AI with custom URLs
    robot_ai = RobotNavigationAI(api_base=api_base, websocket_url=websocket_url)
    
    try:
        success = robot_ai.navigate_to_goal()
        
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
