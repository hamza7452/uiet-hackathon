from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import json
import time

app = Flask(__name__)
app.config['SECRET_KEY'] = 'robot_navigation_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global robot state
robot_state = {
    "position": {"x": 320, "y": 300},
    "goal": {"x": 550, "y": 80}, 
    "collision_count": 0,
    "goal_reached": False,
    "is_moving": False,
    "path": []
}

# Obstacles matching your SIM images
obstacles = [
    {"x": 150, "y": 120, "size": 25},
    {"x": 450, "y": 180, "size": 25},
    {"x": 220, "y": 300, "size": 25},
    {"x": 380, "y": 380, "size": 25},
    {"x": 100, "y": 450, "size": 25},
    {"x": 500, "y": 100, "size": 25},
    {"x": 280, "y": 220, "size": 25},
    {"x": 420, "y": 320, "size": 25}
]

@app.route('/')
def index():
    return render_template('simulation.html')

@app.route('/status')
def get_status():
    return jsonify({
        "connected_simulators": 1,
        "collision_count": robot_state["collision_count"],
        "goal_reached": robot_state["goal_reached"]
    })

@app.route('/obstacles')
def get_obstacles():
    return jsonify(obstacles)

@app.route('/move', methods=['POST'])
def move_robot():
    data = request.json
    robot_state["position"] = {"x": data["x"], "y": data["y"]}
    robot_state["is_moving"] = True
    
    # Check if goal is reached
    goal_x, goal_y = robot_state["goal"]["x"], robot_state["goal"]["y"]
    robot_x, robot_y = data["x"], data["y"]
    distance = ((robot_x - goal_x)**2 + (robot_y - goal_y)**2)**0.5
    
    if distance < 30:
        robot_state["goal_reached"] = True
        print(f"🎯 GOAL REACHED! Robot at ({robot_x:.1f}, {robot_y:.1f})")
        socketio.emit('goal_reached', {"message": "🎯 Goal Reached!", "position": robot_state["position"]})
    
    # Emit real-time position update to all connected clients
    socketio.emit('robot_update', {
        'robot_position': robot_state["position"],
        'is_moving': True,
        'goal_reached': robot_state["goal_reached"]
    })
    
    print(f"🤖 Robot moved to: ({data['x']:.1f}, {data['y']:.1f}) - Distance to goal: {distance:.1f}px")
    
    return jsonify({
        "success": True, 
        "goal_reached": robot_state["goal_reached"]
    })

@app.route('/stop', methods=['POST'])
def stop_robot():
    robot_state["is_moving"] = False
    print("🛑 Robot stopped")
    return jsonify({"success": True})

@app.route('/goal/status')
def goal_status():
    return jsonify({"goal_reached": robot_state["goal_reached"]})

@app.route('/reset', methods=['POST'])
def reset_simulation():
    robot_state.update({
        "position": {"x": 320, "y": 300},
        "collision_count": 0,
        "goal_reached": False,
        "is_moving": False,
        "path": []
    })
    socketio.emit('simulation_reset', {"message": "🔄 Simulation reset"})
    print("🔄 Simulation state reset")
    return jsonify({"success": True})

@app.route('/set_path', methods=['POST'])
def set_path():
    """Receive calculated path from AI for visualization"""
    data = request.json
    robot_state["path"] = data.get("path", [])
    socketio.emit('path_update', {"path": robot_state["path"]})
    print(f"📍 Path received with {len(robot_state['path'])} waypoints")
    return jsonify({"success": True})

# WebSocket events
@socketio.on('connect')
def handle_connect():
    print('🔌 Client connected to WebSocket')
    emit('connected', {'data': 'Connected to robot simulation'})

@socketio.on('disconnect')
def handle_disconnect():
    print('🔌 Client disconnected from WebSocket')

if __name__ == '__main__':
    print("🚀 Starting Robot Simulation Server...")
    print("📍 Web Interface: http://localhost:5000")
    print("🔌 WebSocket: http://localhost:5000 (SocketIO)")
    print("🎯 Ready for AI navigation!")
    
    socketio.run(app, host='localhost', port=5000, debug=True, allow_unsafe_werkzeug=True)
