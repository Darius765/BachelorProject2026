import asyncio
import websockets
import json
import socket
import threading
import queue

message_queue = queue.Queue(maxsize=10)
force_queue = queue.Queue(maxsize=1)
last_force = {"x": 0, "y": 0, "z": 0}

def tcp_receiver(conn):
    """Receive force commands from C++"""
    buffer = ""
    while True:
        try:
            data = conn.recv(4096).decode()
            if not data:
                break
            buffer += data
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                try:
                    force_msg = json.loads(line)
                    try:
                        force_queue.get_nowait()
                    except queue.Empty:
                        pass
                    force_queue.put_nowait(force_msg)
                except json.JSONDecodeError:
                    pass
        except Exception as e:
            print(f"TCP receive error: {e}")
            break

def tcp_sender(conn):
    """Send Haply data to C++"""
    while True:
        try:
            msg = message_queue.get(timeout=1.0)
            conn.sendall((msg + '\n').encode())
        except queue.Empty:
            continue
        except Exception as e:
            print(f"TCP send error: {e}")
            break

async def haply_receiver():
    async with websockets.connect('ws://localhost:10001') as ws:
        print("Connected to Haply service")
        while True:
            try:

                new_force = force_queue.get_nowait()
                last_force.update(new_force)
            except queue.Empty:
                pass

            activate_msg = json.dumps({
                "inverse3": [{
                    "device_id": "05DA",
                    "commands": {
                        "set_cursor_force": {"vector": last_force}
                    }
                }]
            })

            await ws.send(activate_msg)

            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=0.005)
                try:
                    message_queue.put_nowait(msg)
                except queue.Full:
                    pass
            except asyncio.TimeoutError:
                pass

def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('localhost', 10002))
    server.listen(1)
    print("Waiting for C++ connection on port 10002...")
    conn, addr = server.accept()
    print("C++ connected!")

    sender_thread = threading.Thread(target=tcp_sender, args=(conn,), daemon=True)
    sender_thread.start()

    receiver_thread = threading.Thread(target=tcp_receiver, args=(conn,), daemon=True)
    receiver_thread.start()

    asyncio.run(haply_receiver())

main()
