#!/usr/bin/env python3
"""
Test script for pathfinding algorithms
"""
import sys
sys.path.append('.')

from ai_navigator.pathfinding import AStarPathfinder, RefinedPathfinder
from ai_navigator.collision_detector import CollisionDetector, Obstacle
from ai_navigator.utils import Point, calculate_path_length

def test_basic_pathfinding():
    """Test basic A* pathfinding"""
    print("🧪 Testing Basic A* Pathfinding...")
    
    # Create pathfinder
    pathfinder = AStarPathfinder(650, 600, grid_size=10)
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
    
    print(f"  Start: {start}")
    print(f"  Goal: {goal}")
    print(f"  Obstacles: {len(obstacles)}")
    
    path = pathfinder.find_path(start, goal, obstacles)
    
    if path:
        path_length = calculate_path_length(path)
        print(f"  ✅ Path found with {len(path)} waypoints")
        print(f"  📏 Path length: {path_length:.1f}px")
        print(f"  🕒 Search time: {pathfinder.last_search_time:.3f}s")
        print(f"  🔄 Iterations: {pathfinder.last_search_iterations}")
        
        # Show first few waypoints
        for i, waypoint in enumerate(path[:3]):
            print(f"    Waypoint {i+1}: ({waypoint.x:.1f}, {waypoint.y:.1f})")
        if len(path) > 3:
            print(f"    ... and {len(path) - 3} more waypoints")
    else:
        print("  ❌ No path found")
    
    return path is not None

def test_collision_avoidance():
    """Test collision avoidance"""
    print("\n🧪 Testing Collision Avoidance...")
    
    detector = CollisionDetector(18, 10)
    
    # Test obstacle
    obstacle = Obstacle(100, 100, 25)
    
    # Test positions
    test_cases = [
        (Point(50, 50), False, "Safe position (far from obstacle)"),
        (Point(100, 100), True, "Direct collision (center of obstacle)"),
        (Point(115, 115), True, "Edge collision (within safety margin)"), 
        (Point(140, 140), False, "Safe position (outside safety margin)")
    ]
    
    all_passed = True
    for pos, expected_collision, description in test_cases:
        result = detector.will_collide_with_obstacle(pos, obstacle)
        status = "✅" if result == expected_collision else "❌"
        print(f"  {status} {description}: {result} (expected {expected_collision})")
        if result != expected_collision:
            all_passed = False
    
    return all_passed

def test_refined_pathfinding():
    """Test refined pathfinding with fallback strategies"""
    print("\n🧪 Testing Refined Pathfinding with Fallbacks...")
    
    # Create pathfinder
    pathfinder = RefinedPathfinder(650, 600, grid_size=10)
    collision_detector = CollisionDetector(18, 10)
    pathfinder.set_collision_detector(collision_detector)
    
    # Create challenging obstacle scenario
    obstacles = [
        Obstacle(150, 150, 60),  # Large obstacle
        Obstacle(350, 200, 40),
        Obstacle(200, 400, 45),
        Obstacle(450, 350, 35),
        Obstacle(500, 150, 30)
    ]
    
    # Test difficult path
    start = Point(50, 50)
    goal = Point(580, 550)
    
    print(f"  Start: {start}")
    print(f"  Goal: {goal}")
    print(f"  Obstacles: {len(obstacles)} (challenging scenario)")
    
    path = pathfinder.find_path_with_fallbacks(start, goal, obstacles)
    
    if path:
        path_length = calculate_path_length(path)
        report = pathfinder.get_pathfinding_report()
        
        print(f"  ✅ Path found with {len(path)} waypoints")
        print(f"  📏 Path length: {path_length:.1f}px")
        print(f"  🔧 Strategies used: {report.get('fallback_strategies_used', [])}")
        print(f"  📊 Success rate: {report.get('success_rate', 'N/A')}")
        print(f"  🕒 Search time: {report.get('search_time', 0):.3f}s")
        
        return True
    else:
        print("  ❌ No path found even with fallback strategies")
        return False

def test_path_smoothing():
    """Test path smoothing functionality"""
    print("\n🧪 Testing Path Smoothing...")
    
    pathfinder = AStarPathfinder(650, 600, grid_size=10)
    collision_detector = CollisionDetector(18, 10)
    pathfinder.set_collision_detector(collision_detector)
    
    # Create simple obstacle for testing
    obstacles = [Obstacle(300, 300, 40)]
    
    start = Point(100, 100)
    goal = Point(500, 500)
    
    # Get original path
    original_path = pathfinder.find_path(start, goal, obstacles)
    
    if original_path:
        # Test smoothing
        smoothed_path = pathfinder.smooth_path(original_path, obstacles)
        
        original_length = calculate_path_length(original_path)
        smoothed_length = calculate_path_length(smoothed_path)
        
        print(f"  Original path: {len(original_path)} waypoints, {original_length:.1f}px")
        print(f"  Smoothed path: {len(smoothed_path)} waypoints, {smoothed_length:.1f}px")
        print(f"  Reduction: {len(original_path) - len(smoothed_path)} waypoints")
        print(f"  Length change: {smoothed_length - original_length:+.1f}px")
        
        # Smoothed path should have fewer waypoints
        return len(smoothed_path) <= len(original_path)
    else:
        print("  ❌ Could not generate path for smoothing test")
        return False

def test_grid_scaling():
    """Test pathfinding with different grid sizes"""
    print("\n🧪 Testing Grid Size Scaling...")
    
    obstacles = [
        Obstacle(200, 200, 40),
        Obstacle(400, 350, 35)
    ]
    
    start = Point(50, 50)
    goal = Point(550, 500)
    
    grid_sizes = [5, 10, 20]
    results = []
    
    for grid_size in grid_sizes:
        pathfinder = AStarPathfinder(650, 600, grid_size=grid_size)
        collision_detector = CollisionDetector(18, 10)
        pathfinder.set_collision_detector(collision_detector)
        
        path = pathfinder.find_path(start, goal, obstacles)
        
        if path:
            path_length = calculate_path_length(path)
            results.append({
                'grid_size': grid_size,
                'waypoints': len(path),
                'length': path_length,
                'time': pathfinder.last_search_time,
                'iterations': pathfinder.last_search_iterations
            })
            
            print(f"  Grid {grid_size}px: {len(path)} waypoints, {path_length:.1f}px, {pathfinder.last_search_time:.3f}s")
        else:
            print(f"  Grid {grid_size}px: ❌ No path found")
    
    # All grid sizes should find a path
    return len(results) == len(grid_sizes)

def run_all_tests():
    """Run comprehensive pathfinding test suite"""
    print("🤖 AI Navigation Pathfinding Test Suite")
    print("=" * 50)
    
    tests = [
        ("Basic Pathfinding", test_basic_pathfinding),
        ("Collision Avoidance", test_collision_avoidance), 
        ("Refined Pathfinding", test_refined_pathfinding),
        ("Path Smoothing", test_path_smoothing),
        ("Grid Scaling", test_grid_scaling)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        try:
            success = test_func()
            results.append((test_name, success))
        except Exception as e:
            print(f"❌ Test failed with exception: {e}")
            results.append((test_name, False))
    
    # Summary
    print(f"\n{'='*20} TEST SUMMARY {'='*20}")
    passed = sum(1 for _, success in results if success)
    total = len(results)
    
    for test_name, success in results:
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"{status} {test_name}")
    
    print(f"\nOverall Result: {passed}/{total} tests passed ({passed/total*100:.1f}%)")
    
    if passed == total:
        print("🎉 All tests passed! Pathfinding system is working correctly.")
        return True
    else:
        print("⚠️ Some tests failed. Check the implementation.")
        return False

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Test AI Navigation Pathfinding")
    parser.add_argument("--test", "-t", choices=['basic', 'collision', 'refined', 'smoothing', 'grid', 'all'],
                       default='all', help="Which test to run")
    
    args = parser.parse_args()
    
    if args.test == 'all':
        success = run_all_tests()
    elif args.test == 'basic':
        success = test_basic_pathfinding()
    elif args.test == 'collision':
        success = test_collision_avoidance()
    elif args.test == 'refined':
        success = test_refined_pathfinding()
    elif args.test == 'smoothing':
        success = test_path_smoothing()
    elif args.test == 'grid':
        success = test_grid_scaling()
    
    exit(0 if success else 1)
