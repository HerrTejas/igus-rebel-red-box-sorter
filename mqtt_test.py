"""
Standalone MQTT test — no robot, no camera. Proves the messaging works.

It connects to the broker in config.json, subscribes to the START topic, and
prints every message it receives. Publish from the HiveMQ web client (or this
script auto-publishes a test message after 3 s) and you should see it appear.

Steps:
    1. python mqtt_test.py
    2. wait for "connected" and "subscribed"
    3. in the HiveMQ web client, publish {"X":150,"Y":150} to the sort topic,
       OR just wait 3 s and this script publishes one to itself.
    4. you should see "RECEIVED ..." printed here.
If you see the message here, MQTT works and the problem is elsewhere. If not,
it is a broker/port/topic issue.
"""
import json
import time

import paho.mqtt.client as mqtt

with open("config.json") as f:
    m = json.load(f)["mqtt"]

# Python uses plain TCP 1883; 8884 etc. are browser WebSocket ports.
port = m["port"]
if port in (8884, 8000, 8083, 443):
    print(f"NOTE: port {port} is a browser/WebSocket port; using 1883 for Python")
    port = 1883

sort_topic = m["topic_sort"]
status_topic = m["topic_status"]


def on_connect(c, u, flags, rc):
    print(f"connected rc={rc} to {m['host']}:{port}" if rc == 0 else f"connect refused rc={rc}")
    c.subscribe(sort_topic)
    c.subscribe(status_topic)
    print(f"subscribed to:\n  {sort_topic}\n  {status_topic}")


def on_message(c, u, msg):
    print(f"RECEIVED on '{msg.topic}': {msg.payload.decode('utf-8')}")


client = mqtt.Client(client_id=f"mqtt-test-{int(time.time())}", clean_session=True)
client.on_connect = on_connect
client.on_message = on_message
client.connect(m["host"], port)
client.loop_start()

time.sleep(5)   # give the connection + subscription time to complete first
test_msg = json.dumps({"X": 150, "Y": 150})
print(f"\npublishing a self-test to '{sort_topic}': {test_msg}")
client.publish(sort_topic, test_msg)   # you should see this echoed back as RECEIVED

print("\nlistening for 30 s — publish from HiveMQ now. Ctrl+C to stop.\n")
try:
    time.sleep(30)
except KeyboardInterrupt:
    pass
client.loop_stop()
client.disconnect()
