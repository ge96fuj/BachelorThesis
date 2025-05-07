import socket
import json
import time
import threading
import hmac
import hashlib

# --------------------- Configuration ---------------------
SERVER_IP = '192.168.0.105'
SERVER_PORT = 12345
LIGHT_ID = "tl_1"
LOC_X = 0x1234
LOC_Y = 0x0200

SECRET_KEY_HEX = "f2b7d0c6a3e1c9d56fa43ec0e75bd98b192de4f3914bc7ecb487a3eb5f68a219"
SECRET_KEY = bytes.fromhex(SECRET_KEY_HEX)

USE_HMAC = False
USE_TIMESTAMP = False

# --------------------- States ---------------------
RED = 0
YELLOW = 1
GREEN = 2
current_state = RED
blink = False
sock = None

# --------------------- Networking ---------------------
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

def reconnect():
    global sock
    try:
        sock.close()
    except:
        pass
    print("🔌 Attempting reconnection...")
    connect_to_server()
    send_status()

# --------------------- Secure Message Construction ---------------------
def build_payload(command, extra=None):
    payload = {
        "command": command
    }

    if extra:
        payload.update(extra)

    if USE_TIMESTAMP:
        payload["timestamp"] = int(time.time())

    if USE_HMAC:
        data_to_hash = dict(payload)
        msg_to_hash = json.dumps(data_to_hash, separators=(',', ':'), sort_keys=True)
        h = hmac.new(SECRET_KEY, msg_to_hash.encode(), hashlib.sha256).hexdigest()
        payload["hmac"] = h

    return payload

def send_json(payload):
    msg = json.dumps(payload, separators=(',', ':'))
    sock.sendall(msg.encode() + b'\n')

# --------------------- Outgoing Messages ---------------------
def send_status():
    extra = {
        "lightID": LIGHT_ID
    }
    payload = build_payload(96, extra)
    send_json(payload)

def send_confirmation():
    payload = build_payload(90, {"lightID": LIGHT_ID})
    send_json(payload)

# --------------------- Command Handling ---------------------
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

# --------------------- Server Listener ---------------------
def listen_loop():
    buffer = ""
    while True:
        try:
            chunk = sock.recv(1024).decode()
            if not chunk:
                raise Exception("Disconnected")
            buffer += chunk

            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                try:
                    message = json.loads(line)
                    cmd = message.get("command")
                    handle_request(cmd)
                except json.JSONDecodeError:
                    print(f"⚠️ Invalid JSON: {line}")
        except Exception as e:
            print(f"❌ Lost connection: {e}")
            reconnect()

# --------------------- Blinking ---------------------
def blink_loop():
    while True:
        if blink:
            print("💡 Blinking YELLOW...")
            time.sleep(0.5)
        else:
            time.sleep(1)

# --------------------- Main Entry ---------------------
def main():
    connect_to_server()
    send_status()

    threading.Thread(target=listen_loop, daemon=True).start()
    threading.Thread(target=blink_loop, daemon=True).start()

    while True:
        time.sleep(5)

if __name__ == '__main__':
    main()
