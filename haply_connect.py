import asyncio, json, websockets

async def main():
    # Connect to the Haply Inverse service WebSocket
    async with websockets.connect("ws://localhost:10001") as ws:
        # Read the first state frame to discover the device id
        first_state = json.loads(await ws.recv())
        device_id = first_state["inverse3"][0]["device_id"]

        # Build a zero-force keepalive command targeting that device
        keepalive = {"inverse3": [{
            "device_id": device_id,
            "commands": {"set_cursor_force": {"vector": {"x": 0, "y": 0, "z": 0}}}
        }]}

        # Realtime loop: one send per tick, then read the resulting state
        while True:
            await ws.send(json.dumps(keepalive))
            state = json.loads(await ws.recv())
            pos = state["inverse3"][0]["state"]["cursor_position"]
            print(f"pos: {pos}")

asyncio.run(main())