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
       
        # Send activation message to wake up the Inverse3
        activate_msg = json.dumps({
            "inverse3": [{
                "device_id": "05DA",
                "command": {
                    "control_mode": "position"
                }
            }]
        })
       
        count = 0
        while True:

            await ws.send(activate_msg)
            try:
                msg = await ws.recv()
                count += 1
                print(f"Received message #{count}")
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

    asyncio.run(haply_receiver())

main()