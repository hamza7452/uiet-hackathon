from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import json
import time

app = Flask(__name__)
app.config['SECRET_KEY'] = 'robot_navigation_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global state
robot_state = {
    "position": {"x": 320, "y": 300},
    "goal": {"x": 550, "y": 80},
    "collision_count": 0,
    "goal_reached": False,
    "is_moving": False
}

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
    
    # Emit position update via WebSocket
    socketio.emit('robot_update', {
        'robot_position': robot_state["position"],
        'is_moving': True
    })
    
    return jsonify({"success": True})

@app.route('/stop', methods=['POST'])
def stop_robot():
    robot_state["is_moving"] = False
    return jsonify({"success": True})

@app.route('/goal/status')
def goal_status():
    return jsonify({"goal_reached": robot_state["goal_reached"]})

@app.route('/reset', methods=['POST'])
def reset_simulation():
    robot_state.update({
        "collision_count": 0,
        "goal_reached": False,
        "is_moving": False
    })
    return jsonify({"success": True})

if __name__ == '__main__':
    print("🚀 Starting Robot Simulation Server...")
    print("📍 Flask API: http://localhost:5000")
    print("🔌 WebSocket: ws://localhost:8080")
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
