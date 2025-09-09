import random
import sys

import socketio
import time
import math

# Create a Socket.IO client
sio = socketio.Client()
players_data = {}  # key: player id, value: dict with x, y, isCaught, etc.
host, key, diff = None, None, None

bot_id = None
bot_position = {"x": 100, "y": 100, "facingAngle": 0, "isMoving": False}
bot_speed = 175
ready = False
walls = [
    {"x": 0, "y": 0, "width": 2000, "height": 84},
    {"x": 0, "y": 0, "width": 32, "height": 2000},
    {"x": 0, "y": 1960, "width": 2000, "height": 40},
    {"x": 1960, "y": 0, "width": 40, "height": 2000},

    {"x": 151, "y": 199, "width": 220, "height": 73},
    {"x": 151, "y": 273, "width": 32, "height": 301},
    {"x": 480, "y": 200, "width": 31, "height": 166},
    {"x": 290, "y": 366, "width": 221, "height": 72},
    {"x": 151, "y": 1187, "width": 262, "height": 77},
    {"x": 151, "y": 703, "width": 35, "height": 280},
    {"x": 151, "y": 983, "width": 263, "height": 77},
    {"x": 151, "y": 1358, "width": 34, "height": 268},
    {"x": 151, "y": 1627, "width": 630, "height": 74},
    {"x": 754, "y": 1702, "width": 30, "height": 121},
    {"x": 754, "y": 1822, "width": 500, "height": 75},
    {"x": 1225, "y": 1701, "width": 27, "height": 125},
    {"x": 1225, "y": 1626, "width": 628, "height": 74},
    {"x": 1817, "y": 1360, "width": 33, "height": 268},

    {"x": 793, "y": 380, "width": 33, "height": 162},
    {"x": 478, "y": 540, "width": 346, "height": 74},
    {"x": 478, "y": 611, "width": 34, "height": 173},
    {"x": 673, "y": 710, "width": 224, "height": 74},
    {"x": 520, "y": 879, "width": 250, "height": 75},
    {"x": 597, "y": 1066, "width": 32, "height": 295},
    {"x": 372, "y": 1360, "width": 258, "height": 78},
    {"x": 650, "y": 199, "width": 459, "height": 77},

    {"x": 919, "y": 379, "width": 336, "height": 74},
    {"x": 1224, "y": 275, "width": 26, "height": 104},
    {"x": 1222, "y": 199, "width": 194, "height": 71},

    {"x": 1012, "y": 572, "width": 34, "height": 225},
    {"x": 1171, "y": 572, "width": 34, "height": 114},
    {"x": 1171, "y": 686, "width": 287, "height": 73},
    {"x": 1011, "y": 878, "width": 33, "height": 311},
    {"x": 848, "y": 1188, "width": 197, "height": 75},
    {"x": 1152, "y": 880, "width": 93, "height": 76},
    {"x": 1152, "y": 955, "width": 33, "height": 234},
    {"x": 1153, "y": 1189, "width": 180, "height": 78},
    {"x": 1012, "y": 1358, "width": 173, "height": 78},
    {"x": 1547, "y": 200, "width": 301, "height": 77},
    {"x": 1819, "y": 274, "width": 31, "height": 313},
    {"x": 1507, "y": 381, "width": 128, "height": 73},
    {"x": 1604, "y": 453, "width": 34, "height": 326},
    {"x": 1408, "y": 879, "width": 269, "height": 75},
    {"x": 1544, "y": 1065, "width": 34, "height": 122},
    {"x": 1544, "y": 1187, "width": 308, "height": 75},
]
def nearest_walkable(grid, cell):
    from collections import deque
    visited = set([cell])
    q = deque([cell])
    while q:
        x, y = q.popleft()
        if 0 <= x < len(grid[0]) and 0 <= y < len(grid) and grid[int(y)][int(x)]:
            return (x, y)
        for nx, ny in [(1,0),(-1,0),(0,1),(0,-1)]:
            if (nx+x, ny+y) not in visited:
                visited.add((nx+x, ny+y))
                q.append((nx+x, ny+y))
    return cell

waypoints = [
    {"x": 100, "y": 100},
    {"x": 1800, "y": 200},
    {"x": 1800, "y": 1800},
    {"x": 200, "y": 1800},
]
current_waypoint_index = 0

CELL_SIZE = 50
GRID_WIDTH = 2000 // CELL_SIZE
GRID_HEIGHT = 2000 // CELL_SIZE

bot_role = "hider"
import time

last_chase_time = 0
CHASE_INTERVAL = 15 if diff == 1 else 10 if diff == 2 else 5

import time
from math import hypot



def get_target_player(state_players, bot_id, bot_position):
    """
    Returns the closest player that is not the bot itself or caught.
    state_players: dict of player data
    bot_id: id of the bot
    bot_position: dict with keys 'x', 'y'
    """
    try:

        others = [p for p in state_players.values() if p['id'] != bot_id]
    except:
        others = []
    if not others:
        return None

    # find the closest player
    closest = others[0]
    min_dist = hypot(closest['x'] - bot_position['x'], closest['y'] - bot_position['y'])
    for p in others[1:]:
        dist = hypot(p['x'] - bot_position['x'], p['y'] - bot_position['y'])
        if dist < min_dist:
            closest = p
            min_dist = dist
    return closest
def get_flee_target(bot_position, seeker, map_width, map_height, flee_distance=600):
    """
    Compute a target point away from the seeker, clamped within map bounds.
    """
    dx = bot_position['x'] - seeker['x']
    dy = bot_position['y'] - seeker['y']
    distance = hypot(dx, dy)
    if distance == 0:
        distance = 1  # avoid divide by zero

    # target point far away in opposite direction
    target_x = bot_position['x'] + (dx / distance) * flee_distance
    target_y = bot_position['y'] + (dy / distance) * flee_distance

    # clamp within map
    target_x = max(0, min(map_width, target_x))
    target_y = max(0, min(map_height, target_y))

    return {'x': target_x, 'y': target_y}


def update_bot_target(bot_position, bot_id, state_players, walls, compute_path):
    global last_chase_time, bot_path

    now = time.time()
    if now - last_chase_time < CHASE_INTERVAL:
        return  # not time yet

    last_chase_time = now

    target = get_target_player(players_data, bot_id, bot_position)
    if not target:
        return  # no target available
    if bot_role == 'seeker':
        print("Chasing")

        target_pos = {'x': target['x'], 'y': target['y']}
    else:  # hider → move away
        print("hiding")

        target_pos = get_flee_target(bot_position, target, map_width=2000, map_height=2000)

    new_path = compute_path(bot_position, target_pos)

    if new_path:
        bot_path = new_path


BOT_WIDTH = 40
BOT_HEIGHT = 50
BOT_FLASHON = True
def update_player(pid, data):
    global BOT_FLASHON

    """Update or create a player entry safely"""
    global players_data
    if pid not in players_data:
        players_data[pid] = {}

    # Merge only known keys to avoid overwriting missing info
    for key in ['x', 'y', 'facingAngle', 'isMoving', 'id', 'flashOn', 'username', 'isCaught', 'role']:
        if key in data:
            players_data[pid][key] = data[key]
            if data[key] == bot_id:
                if data['isCaught']:
                    players_data[pid]['isMoving'] = False
                    BOT_FLASHON = False

def collides_with_walls(x, y, walls):
    """Check if the bot's bounding box overlaps any wall rectangle"""
    bot_rect = {
        "x": x,
        "y": y,
        "width": BOT_WIDTH,
        "height": BOT_HEIGHT,
    }

    for wall in walls:
        if (bot_rect["x"] < wall["x"] + wall["width" ]+ 10 and
            bot_rect["x"] + bot_rect["width"] > wall["x"] and
            bot_rect["y"] < wall["y"] + wall["height"] + 10 and
            bot_rect["y"] + bot_rect["height"] > wall["y"]):
            return True
    return False

def build_grid(walls):
    grid = [[1 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]  # 1 = walkable, 0 = blocked

    for y in range(GRID_HEIGHT):
        for x in range(GRID_WIDTH):
            world_x = x * CELL_SIZE
            world_y = y * CELL_SIZE
            if collides_with_walls(world_x, world_y, walls):
                grid[y][x] = 0
    return grid

grid = build_grid(walls)


import heapq

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    if start == goal:
        return [start]  # already at goal

    neighbors = [(1,0), (-1,0), (0,1), (0,-1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []

    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        _, current = heapq.heappop(oheap)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
        close_set.add(current)
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            tentative_g_score = gscore[current] + 1
            if 0 <= neighbor[0] < GRID_WIDTH:
                if 0 <= neighbor[1] < GRID_HEIGHT:
                    if grid[int(neighbor[1])][int(neighbor[0])] == 0:
                        continue
                else:
                    continue
            else:
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return []




def world_to_grid(x, y):
    gx = min(max(x // CELL_SIZE, 0), GRID_WIDTH - 1)
    gy = min(max(y // CELL_SIZE, 0), GRID_HEIGHT - 1)
    return (gx, gy)


def grid_to_world(cell):
    return (cell[0] * CELL_SIZE + CELL_SIZE // 2,
            cell[1] * CELL_SIZE + CELL_SIZE // 2)

def compute_path(start_pos, target_pos):
    start = nearest_walkable(grid, world_to_grid(start_pos["x"], start_pos["y"]))
    goal = nearest_walkable(grid, world_to_grid(target_pos["x"], target_pos["y"]))
    path_cells = astar(grid, start, goal)
    if not path_cells:
        print(f"⚠️ No path found {start} → {goal}")
        return []

    return [grid_to_world(c) for c in path_cells]



@sio.on('joinGame')
def handle_host_game(data):
    print("Game hosted:", data)

@sio.event
def connect():
    print("Connected to the server")
    global bot_id
    bot_id = sio.get_sid()
    sio.emit('newPlayer', {"username": "bot", "x": bot_position["x"], "y": bot_position["y"]})

@sio.on('playerMoved')
def handle_player_moved(data):
    pid = data['id']
    update_player(pid, data)


@sio.on('currentPlayers')
def handle_current_players(data):
    print("Current players:", data)



@sio.on('playerDisconnected')
def handle_player_disconnected(player_id):
    print(f"Player {player_id} disconnected")

@sio.on('startGame')
def handle_start_game(data):
    global bot_role, bot_position, ready
    print("Game started:", data)

    bot_role = data[bot_id]['role']
    bot_position = {"x": data[bot_id]['x'], "y": data[bot_id]['y'], "facingAngle": 0, "isMoving": False}
    ready = True

    print("Game started:", data)

@sio.on('playerCaught')
def caught(data):
    global ready, BOT_FLASHON
    if data == bot_id:
        ready = False







bot_path = []

def move_bot(delta_time):
    global bot_position, bot_path, current_waypoint_index

    # If no active path, compute one
    if not bot_path:
        target = waypoints[current_waypoint_index]
        bot_path = compute_path(bot_position, target)

        if not bot_path:
            print("⚠️ No path, skipping waypoint")
            current_waypoint_index = (current_waypoint_index + 1) % len(waypoints)
            return

    # Move towards the next node
    next_x, next_y = bot_path[0]
    dx = next_x - bot_position["x"]
    dy = next_y - bot_position["y"]
    dist = math.hypot(dx, dy)

    if dist < 5:  # close enough → reached this node
        bot_path.pop(0)
        if not bot_path:
            current_waypoint_index = (current_waypoint_index + 1) % len(waypoints)
        return

    # Normalize and step
    nx = dx / dist
    ny = dy / dist
    step = bot_speed * delta_time

    bot_position["x"] += nx * step
    bot_position["y"] += ny * step
    bot_position["facingAngle"] = math.atan2(dy, dx)
    bot_position["isMoving"] = True
    bot_position['flashOn'] = BOT_FLASHON

    sio.emit("move", bot_position)
@sio.on('game:ended')
def end(data):
    global ready
    ready = False

@sio.on('error')
def on_error(data):
    print("Error:", data)

def main():
    global  host, key, diff
    host = sys.argv[1]
    key = sys.argv[2]
    diff = sys.argv[3]


    sio.connect("http://localhost:8080", auth={"key": key})
    sio.emit('joinGame', {'gameId': host, 'username':f'bot_{random.randint(0,20)}' })
    sio.emit('updateReadyStatus',True)


    last_time = time.time()
    while True:
        current_time = time.time()
        delta_time = current_time - last_time
        last_time = current_time

        if ready:
            update_bot_target(bot_position, bot_id, players_data, walls, compute_path)
            move_bot(delta_time)
        else:
            bot_position["isMoving"] = False
            bot_position['flashOn'] = False
            sio.emit("move", bot_position)

        time.sleep(0.016)

if __name__ == "__main__":
    main()