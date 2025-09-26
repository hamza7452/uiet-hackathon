#!/usr/bin/env python3
"""
Debug mode navigation with detailed logging
"""
import sys
sys.path.append('.')

from ai_navigator.robot_ai import RobotNavigationAI
import time

def debug_navigation():
    """Run navigation in debug mode with detailed output"""
    print("🐛 DEBUG MODE: AI Robot Navigation")
    print("=" * 50)
    
    # Create AI with debug settings
    robot_ai = RobotNavigationAI()
    
    print("🔍 Environment Connection Test:")
    success = robot_ai.connect_to_environment()
    if not success:
        print("❌ Failed to connect to environment")
        return False
    
    print("\n🗺️ Environment State:")
    print(f"  Robot Position: {robot_ai.robot_position}")
    print(f"  Goal Position: {robot_ai.goal_position}")
    print(f"  Obstacles Count: {len(robot_ai.obstacles)}")
    print(f"  Initial Collisions: {robot_ai.collision_count}")
    
    print("\n📊 AI Configuration:")
    print(f"  Robot Radius: {robot_ai.collision_detector.robot_radius}px")
    print(f"  Safety Margin: {robot_ai.collision_detector.current_safety_margin}px")
    print(f"  Grid Size: {robot_ai.pathfinder.grid_size}px")
    
    # Show obstacles in detail
    print("\n🚧 Obstacle Details:")
    for i, obstacle in enumerate(robot_ai.obstacles):
        print(f"  Obstacle {i+1}: Center({obstacle.center.x}, {obstacle.center.y}), Size: {obstacle.size}px")
    
    print("\n🧭 Path Calculation Test:")
    path_found = robot_ai.calculate_path_to_goal()
    if path_found and robot_ai.current_path:
        print(f"✅ Path found with {len(robot_ai.current_path)} waypoints:")
        for i, waypoint in enumerate(robot_ai.current_path[:5]):  # Show first 5
            print(f"  Waypoint {i+1}: ({waypoint.x:.1f}, {waypoint.y:.1f})")
        if len(robot_ai.current_path) > 5:
            print(f"  ... and {len(robot_ai.current_path) - 5} more waypoints")
    else:
        print("❌ No path found")
        return False
    
    print("\n🚀 Starting Step-by-Step Navigation:")
    user_input = input("Press Enter to start navigation (or 'q' to quit): ")
    if user_input.lower() == 'q':
        return False
    
    # Step-by-step navigation
    step = 0
    while robot_ai.navigation_active and not robot_ai.goal_reached and step < 100:
        step += 1
        print(f"\n--- Step {step} ---")
        
        success = robot_ai._execute_navigation_step()
        if not success:
            print("❌ Navigation step failed")
            break
        
        # Show current status
        print(f"Current Position: {robot_ai.robot_position}")
        if robot_ai.current_path:
            print(f"Current Waypoint: {robot_ai.current_waypoint_index}/{len(robot_ai.current_path)}")
            if robot_ai.current_waypoint_index < len(robot_ai.current_path):
                current_target = robot_ai.current_path[robot_ai.current_waypoint_index]
                distance_to_target = robot_ai.robot_position.distance_to(current_target)
                print(f"Target: {current_target}, Distance: {distance_to_target:.1f}px")
        print(f"Is Moving: {robot_ai.is_moving}")
        print(f"Collisions: {robot_ai.collision_count}")
        print(f"Safety Margin: {robot_ai.collision_detector.current_safety_margin}px")
        
        # Check goal
        if robot_ai._check_goal_status():
            print("🎯 Goal reached!")
            break
        
        # Small delay for observation
        time.sleep(0.5)
        
        # Optional: pause for manual stepping
        if step % 5 == 0:
            response = input("Continue? (Enter/q to quit/s for stats): ")
            if response.lower() == 'q':
                break
            elif response.lower() == 's':
                stats = robot_ai.get_navigation_stats()
                print("\n📊 Current Navigation Statistics:")
                for key, value in stats.items():
                    print(f"  {key}: {value}")
    
    # Final statistics
    stats = robot_ai.get_navigation_stats()
    print("\n📊 Final Debug Statistics:")
    for key, value in stats.items():
        print(f"  {key}: {value}")
    
    # Get detailed report
    detailed_report = robot_ai.get_detailed_report()
    print("\n📈 Detailed Performance Report:")
    print(f"  Pathfinding Report: {detailed_report.get('pathfinding_report', {})}")
    print(f"  Collision Analysis: {detailed_report.get('collision_analysis', {})}")
    
    return robot_ai.goal_reached

def interactive_debugging():
    """Interactive debugging session"""
    print("🎮 Interactive Debug Mode")
    print("Commands: start, stats, report, reset, quit")
    
    robot_ai = RobotNavigationAI()
    connected = False
    
    while True:
        command = input("\nDebug> ").lower().strip()
        
        if command == "quit" or command == "q":
            break
        elif command == "start":
            if not connected:
                connected = robot_ai.connect_to_environment()
                if not connected:
                    print("❌ Failed to connect to environment")
                    continue
            success = debug_navigation()
            print(f"Navigation result: {'✅ Success' if success else '❌ Failed'}")
        elif command == "stats":
            stats = robot_ai.get_navigation_stats()
            print("📊 Current Statistics:")
            for key, value in stats.items():
                print(f"  {key}: {value}")
        elif command == "report":
            report = robot_ai.get_detailed_report()
            print("📈 Detailed Report:")
            for section, data in report.items():
                print(f"  {section}: {data}")
        elif command == "reset":
            robot_ai.reset_navigation()
            print("🔄 Navigation state reset")
        elif command == "connect":
            connected = robot_ai.connect_to_environment()
            print(f"Connection: {'✅ Success' if connected else '❌ Failed'}")
        else:
            print("Available commands: start, stats, report, reset, connect, quit")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Debug AI Robot Navigation")
    parser.add_argument("--interactive", "-i", action="store_true", 
                       help="Run in interactive mode")
    
    args = parser.parse_args()
    
    if args.interactive:
        interactive_debugging()
    else:
        success = debug_navigation()
        if success:
            print("\n🎉 Debug navigation completed successfully!")
        else:
            print("\n❌ Debug navigation failed or incomplete")
        
        exit(0 if success else 1)
