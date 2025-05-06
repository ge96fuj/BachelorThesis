import socket
import json
import time
import threading

SERVER_IP = '192.168.0.103'
SERVER_PORT = 12345
LIGHT_ID = "tl_2"
LOC_X = 0x1234
LOC_Y = 0x0200

# Light states
RED = 0
YELLOW = 1
GREEN = 2

current_state = RED
blink = False
sock = None

def connect_to_server():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        try:
            sock.connect((SERVER_IP, SERVER_PORT))
            print("✅ Connected to server!")
            break
        except Exception as e:
            print(f"❌ Connection failed: {e}. Retrying in 2s...")
            time.sleep(2)

def send_status():
    payload = {
        "command": 60,
        "lightID": LIGHT_ID,
        "loc_x": LOC_X,
        "loc_y": LOC_Y,
        "lightStatus": current_state
    }
    json_string = json.dumps(payload)
    sock.sendall((json_string + "\n").encode())

def send_confirmation():
    payload = {
        "command": 90,
        "lightID": LIGHT_ID
    }
    sock.sendall((json.dumps(payload) + "\n").encode())

def handle_request(cmd):
    global current_state, blink
    if cmd == 0x20:
        print("🔁 Server requested status.")
        send_status()
    elif cmd == 0x21:
        print("🔴 Switching to RED")
        current_state = RED
        blink = False
    elif cmd == 0x22:
        print("🟢 Switching to GREEN")
        current_state = GREEN
        blink = False
    elif cmd == 0x23:
        print("🟡 Switching to YELLOW")
        current_state = YELLOW
        blink = False
    elif cmd == 0x25:
        print("✨ Entering BLINK mode")
        blink = True
    else:
        print(f"⚠️ Unknown command received: {cmd}")

def listen():
    buffer = b''
    while True:
        try:
            data = sock.recv(1)
            if not data:
                raise Exception("Disconnected")
            cmd = data[0]
            handle_request(cmd)
        except Exception as e:
            print(f"❌ Lost connection: {e}")
            reconnect()

def blink_loop():
    while True:
        if blink:
            print("💡 Blinking YELLOW...")
            time.sleep(0.5)
        else:
            time.sleep(1)

def reconnect():
    global sock
    try:
        sock.close()
    except:
        pass
    print("🔌 Attempting reconnection...")
    connect_to_server()
    send_status()
    listen()

def main():
    connect_to_server()
    send_status()

    threading.Thread(target=listen, daemon=True).start()
    threading.Thread(target=blink_loop, daemon=True).start()

    while True:
        time.sleep(5)

if __name__ == '__main__':
    main()
