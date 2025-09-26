#!/usr/bin/env python3
"""
Performance benchmarking for AI navigation
"""
import sys
import time
sys.path.append('.')

from ai_navigator.robot_ai import RobotNavigationAI
from ai_navigator.pathfinding import RefinedPathfinder
from ai_navigator.collision_detector import CollisionDetector, Obstacle
from ai_navigator.utils import Point, calculate_path_length

def benchmark_pathfinding():
    print("⚡ AI Navigation Performance Benchmark")
    print("=" * 45)
    
    # Test scenarios
    scenarios = [
        {
            "name": "Simple Path",
            "start": Point(50, 50),
            "goal": Point(550, 500),
            "obstacles": [Obstacle(300, 250, 30)]
        },
        {
            "name": "Complex Maze", 
            "start": Point(50, 50),
            "goal": Point(550, 500),
            "obstacles": [
                Obstacle(200, 150, 40),
                Obstacle(350, 200, 35),
                Obstacle(300, 350, 45),
                Obstacle(450, 300, 30),
                Obstacle(150, 400, 35)
            ]
        },
        {
            "name": "Tight Passage",
            "start": Point(50, 300),
            "goal": Point(600, 300), 
            "obstacles": [
                Obstacle(250, 250, 40),
                Obstacle(250, 350, 40),
                Obstacle(400, 250, 40),
                Obstacle(400, 350, 40)
            ]
        }
    ]
    
    # Initialize pathfinder
    pathfinder = RefinedPathfinder(650, 600, grid_size=10)
    collision_detector = CollisionDetector(18, 10)
    pathfinder.set_collision_detector(collision_detector)
    
    results = []
    
    for scenario in scenarios:
        print(f"\n🎯 Testing: {scenario['name']}")
        
        start_time = time.time()
        path = pathfinder.find_path_with_fallbacks(
            scenario["start"], 
            scenario["goal"], 
            scenario["obstacles"]
        )
        end_time = time.time()
        
        if path:
            path_length = calculate_path_length(path)
            stats = pathfinder.get_pathfinding_report()
            
            result = {
                "scenario": scenario["name"],
                "success": True,
                "time": end_time - start_time,
                "waypoints": len(path),
                "path_length": path_length,
                "strategies_used": stats.get("fallback_strategies_used", [])
            }
            
            print(f"  ✅ Success: {len(path)} waypoints, {path_length:.1f}px, {result['time']:.3f}s")
            
        else:
            result = {
                "scenario": scenario["name"], 
                "success": False,
                "time": end_time - start_time,
                "waypoints": 0,
                "path_length": 0,
                "strategies_used": []
            }
            print(f"  ❌ Failed in {result['time']:.3f}s")
        
        results.append(result)
    
    # Summary
    print(f"\n📊 Benchmark Summary:")
    successful = sum(1 for r in results if r["success"])
    avg_time = sum(r["time"] for r in results) / len(results)
    
    print(f"  Success Rate: {successful}/{len(results)} ({successful/len(results)*100:.1f}%)")
    print(f"  Average Time: {avg_time:.3f}s")
    print(f"  Total Scenarios: {len(results)}")
    
    return results

if __name__ == "__main__":
    benchmark_pathfinding()
