import socket
import json
import time
import threading

server_address = '10.181.241.34'
port_num = 12345
traffic_light_id = "tl_2"


light_red = 0
light_yellow = 1
light_green = 2

light_status = light_red
is_blinking = False
client_socket = None

def connect():
    global client_socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        try:
            client_socket.connect((server_address, port_num))
            print("ok conn")
            break
        except Exception as connection_error:
            print("no conn, retry...")
            time.sleep(2)

def sendStatus():
    msg = {
        "command": 60,
        "lightID": traffic_light_id,
    }
    send_data = json.dumps(msg)
    client_socket.sendall((send_data + "\n").encode())

def confirm_message():
    message = {
        "command": 90,
        "lightID": traffic_light_id
    }
    client_socket.sendall((json.dumps(message) + "\n").encode())

def handleRequests(command_code):
    global light_status, is_blinking
    if command_code == 0x20:
        print("rec 0x20")
        sendStatus()
    elif command_code == 0x21:
        print("-> red")
        light_status = light_red
        is_blinking = False
    elif command_code == 0x22:
        print("-> green")
        light_status = light_green
        is_blinking = False
    elif command_code == 0x23:
        print("-> yellow")
        light_status = light_yellow
        is_blinking = False
    elif command_code == 0x25:
        print("blink on")
        is_blinking = True
    else:
        print(f" invalid req {command_code}")

def listen_for_commands():
    while True:
        try:
            incoming_byte = client_socket.recv(1)
            if not incoming_byte:
                raise Exception("gone")
            cmd = incoming_byte[0]
            handleRequests(cmd)
        except Exception as listen_error:
            print("lost. retry...")
            reconnect_to_server()

def blinking_indicator():
    while True:
        if is_blinking:
            print("blink...")
            time.sleep(0.5)
        else:
            time.sleep(1)

def reconnect_to_server():
    global client_socket
    try:
        client_socket.close()
    except:
        pass
    print("reconnecting")
    connect()
    sendStatus()
    listen_for_commands()

def run_traffic_light():
    connect()
    sendStatus()

    threading.Thread(target=listen_for_commands, daemon=True).start()
    threading.Thread(target=blinking_indicator, daemon=True).start()

    while True:
        time.sleep(5)

if __name__ == '__main__':
    run_traffic_light()
