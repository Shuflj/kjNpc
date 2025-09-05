import socketio
import time
import random

# Create a Socket.IO client
sio = socketio.Client()

bot_id = None
bot_position = {"x": 100, "y": 100}
bot_speed = 5

@sio.event
def connect():
    print("Connected to the server")
    global bot_id
    bot_id = sio.get_sid()

@sio.on('currentPlayers')
def handle_current_players(data):
    print("Current players:", data)

@sio.on('newPlayer')
def handle_new_player(data):
    print("New player joined:", data)

# Handle player disconnected event
def handle_player_disconnected(player_id):
    print(f"Player {player_id} disconnected")

def move_bot():
    global bot_position
    directions = ["up", "down", "left", "right"]
    direction = random.choice(directions)

    if direction == "up":
        bot_position["y"] -= bot_speed
    elif direction == "down":
        bot_position["y"] += bot_speed
    elif direction == "left":
        bot_position["x"] -= bot_speed
    elif direction == "right":
        bot_position["x"] += bot_speed

    sio.emit("move", bot_position)

@sio.on('hostGame')
def handle_host_game(data):
    print("Hosting game:", data)

@sio.on('startGame')
def handle_start_game(data):
    print("Game started:", data)
    while True:
        move_bot()
        time.sleep(1)

def main():
    sio.connect("http://localhost:8080")
    sio.emit('hostGame', {'username':'bot'})
    time.sleep(10)
    sio.emit('updateReadyStatus',True)
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()