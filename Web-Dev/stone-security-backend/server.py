from flask import Flask, request, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

@app.route("/")
def home():
    return "Stone Security System is Online"

@app.route("/api/override", methods=["POST"])
def override():
    data = request.json
    command = data.get("command")

    if command in ["LOCK", "UNLOCK", "ALERT"]:
        print(f"Received Override Command: {command}")
        socketio.emit("security-update", {"status": command})  # Send update to frontend
        return jsonify({"message": "Command received", "status": command}), 200

    return jsonify({"error": "Invalid command"}), 400

@socketio.on('client-message')
def handle_client_message(msg):
    print("Client sent: " + msg['data'])

if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000)  # No 'run_with_ngrok'
