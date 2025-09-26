#!/usr/bin/env python3
"""
Test collision detection system
"""
import sys
sys.path.append('.')

from ai_navigator.collision_detector import CollisionDetector, Obstacle
from ai_navigator.utils import Point

def test_collision_system():
    print("🧪 Testing Collision Detection System")
    print("=" * 45)
    
    # Initialize collision detector
    detector = CollisionDetector(robot_radius=18, safety_margin=10)
    
    # Create test obstacles
    obstacles = [
        Obstacle(150, 120, 25),
        Obstacle(300, 200, 25),
        Obstacle(450, 350, 25)
    ]
    
    print(f"📍 Created {len(obstacles)} test obstacles")
    
    # Test collision detection
    test_positions = [
        (Point(50, 50), False, "Safe position"),
        (Point(150, 120), True, "Direct collision"),
        (Point(170, 140), True, "Near collision with safety margin"),
        (Point(200, 200), False, "Safe but close position")
    ]
    
    print("\n🔍 Collision Detection Tests:")
    all_passed = True
    
    for pos, expected_collision, description in test_positions:
        collision_detected = detector.will_collide_with_any_obstacle(pos, obstacles)
        status = "✅ PASS" if collision_detected == expected_collision else "❌ FAIL"
        print(f"  {description}: {status}")
        if collision_detected != expected_collision:
            all_passed = False
    
    # Test path clearance
    print("\n🛤️ Path Clearance Tests:")
    clear_path = detector.is_path_clear(Point(0, 0), Point(100, 50), obstacles)
    blocked_path = detector.is_path_clear(Point(140, 110), Point(160, 130), obstacles)
    
    print(f"  Clear path test: {'✅ PASS' if clear_path else '❌ FAIL'}")
    print(f"  Blocked path test: {'✅ PASS' if not blocked_path else '❌ FAIL'}")
    
    # Test adaptive safety margins
    print(f"\n🛡️ Safety Margin Tests:")
    print(f"  Initial margin: {detector.current_safety_margin}px")
    detector.update_safety_margin(3)  # Simulate 3 collisions
    print(f"  After 3 collisions: {detector.current_safety_margin}px")
    
    return all_passed and clear_path and not blocked_path

if __name__ == "__main__":
    success = test_collision_system()
    print(f"\n🎯 Overall result: {'✅ ALL TESTS PASSED' if success else '❌ SOME TESTS FAILED'}")
    exit(0 if success else 1)
