import asyncio
import websockets
import json
import socket
import threading
import queue

message_queue = queue.Queue(maxsize=10)

def tcp_sender(conn):
    while True:
        try:
            msg = message_queue.get(timeout=1.0)
            conn.sendall((msg + '\n').encode())
        except queue.Empty:
            continue
        except Exception as e:
            print(f"TCP error: {e}")
            break

async def haply_receiver():
    async with websockets.connect('ws://localhost:10001') as ws:
        print("Connected to Haply service")
        activate_msg = json.dumps({
            "inverse3": [{
                "device_id": "05DA",
                "command": {
                    "set_cursor_force": {"vector": {"x": 0, "y": 0, "z": 0}}
                }
            }]
        })
        count = 0
        while True:
            await ws.send(activate_msg)
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=0.1)
                data = json.loads(msg)
                inv3 = data.get("inverse3", [])
                if inv3:
                    pos = inv3[0]["state"]["cursor_position"]
                    print(f"Cursor: {pos['x']:.4f} {pos['y']:.4f} {pos['z']:.4f} mode: {inv3[0]['state']['mode']}")
                count += 1
                try:
                    message_queue.put_nowait(msg)
                except queue.Full:
                    pass
            except asyncio.TimeoutError:
                pass

async def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('localhost', 10002))
    server.listen(1)
    print("Waiting for C++ connection on port 10002...")
    conn, addr = server.accept()
    print("C++ connected!")

    sender_thread = threading.Thread(target=tcp_sender, args=(conn,), daemon=True)
    sender_thread.start()

    await haply_receiver()

asyncio.run(main())