import asyncio
import websockets
import json

async def test_haply():
    uri = "ws://localhost:10001"
    async with websockets.connect(uri) as ws:
        response = await ws.recv()
        print(response)
        data = json.loads(response)
        print("Inverse3:", data.get("inverse3", []))
        print("VerseGrip:", data.get("wireless_verse_grip", []))

asyncio.run(test_haply())



# haply-device-manager // This starts up the Haply Hub
# sudo systemctl status haply-inverse-service // Show the status of the haply service when needed
# sudo systemctl start haply-inverse-service // Starts up the service. Start this up possibly when the stylus light is green already
# If the stylus doesn't start blinking green, just try a few more times, possibly with a different usb port