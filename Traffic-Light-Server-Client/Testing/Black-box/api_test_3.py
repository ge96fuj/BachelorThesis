import unittest
import paho.mqtt.client as mqtt
import threading
import time
import json
import os
import requests

class apiTester(unittest.TestCase):

    def setUp(self):
        self.MQTT_SERVER_PATH = '/Users/skanderjneyeh/Desktop/Bachelor_Thesis_Rep_6Mai/Traffic-Light-Server-Client/Server/services/mqtt.server.js'
        self.SERVER_PATH = '/Users/skanderjneyeh/Desktop/Bachelor_Thesis_Rep_6Mai/Traffic-Light-Server-Client/Server/server.js'
        self.V1 = '/Users/skanderjneyeh/Desktop/Bachelor_Thesis_Rep_6Mai/Traffic-Light-Server-Client/Server/simulator/virtual_light.py'
        self.V2 = '/Users/skanderjneyeh/Desktop/Bachelor_Thesis_Rep_6Mai/Traffic-Light-Server-Client/Server/simulator/virtual_light2.py'
        self.BROKER_IP = '192.168.0.105'

        self.tl1_topic = "detection/traffic_lights/tl_1"
        self.tl2_topic = "detection/traffic_lights/tl_2"

        print("🚀 Starting MQTT broker...")
        os.system(f"node {self.MQTT_SERVER_PATH} &")
        time.sleep(10)

        print("🚦 Starting traffic light server...")
        os.system(f"node {self.SERVER_PATH} &")
        time.sleep(3)

        print("💡 Starting tl_1...")
        os.system(f"python3 {self.V1} &")
        time.sleep(2)

        print("💡 Starting tl_2...")
        os.system(f"python3 {self.V2} &")
        time.sleep(5)

        self.received_state_tl1 = None
        self.received_state_tl2 = None

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.connected = threading.Event()
        self.message_received = threading.Event()

        self.client.connect(self.BROKER_IP, 1883, 60)
        self.client.loop_start()

        if not self.connected.wait(timeout=5):
            self.fail("❌ MQTT connection failed")

    def tearDown(self):
        self.client.loop_stop()
        self.client.disconnect()

        os.system("pkill -f mqtt.server.js")
        os.system("pkill -f server.js")
        os.system("pkill -f virtual_light.py")
        os.system("pkill -f virtual_light2.py")

    def on_connect(self, client, userdata, flags, rc):
        print("✅ MQTT Connected")
        self.connected.set()
        self.client.subscribe(self.tl1_topic)
        self.client.subscribe(self.tl2_topic)
        print(f"🔔 Subscribed to: {self.tl1_topic}, {self.tl2_topic}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()
        if topic == self.tl1_topic:
            self.received_state_tl1 = payload
        elif topic == self.tl2_topic:
            self.received_state_tl2 = payload
        self.message_received.set()

    def wait(self, light_id, expected, duration=10, hold=False):
        topic = self.tl1_topic if light_id == "tl_1" else self.tl2_topic
        received_state = lambda: self.received_state_tl1 if light_id == "tl_1" else self.received_state_tl2

        print(f"🕒 Waiting for {light_id} to become {expected.upper()} for up to {duration} seconds...")
        end_time = time.time() + duration
        held_since = None

        while time.time() < end_time:
            self.message_received.clear()

            if self.message_received.wait(timeout=2):
                raw_msg = received_state()
                if not raw_msg:
                    print(f"⚠️ No message content received for {light_id}")
                    continue

                print(f"📩 Raw message for {light_id}: {raw_msg}")
                try:
                    payload = json.loads(raw_msg)
                    print(f"✅ Parsed payload: {payload}")

                    status = (
                        payload.get("status") or
                        payload.get("state") or
                        payload.get("lightStatus")
                    )
                    if status is None:
                        print(f"❌ No valid status/state/lightStatus key found in payload.")
                        continue

                    status = str(status).upper()
                    print(f"🔎 {light_id} current status: {status}")

                    if status == expected.upper():
                        if hold:
                            if held_since is None:
                                held_since = time.time()
                                print(f"⏳ Hold started at {held_since}")
                            elif time.time() - held_since >= duration:
                                print(f"✅ {light_id} held {expected} for {duration} seconds")
                                return True
                        else:
                            print(f"✅ {light_id} reached expected state {expected}")
                            return True
                    elif hold:
                        print(f"❌ {light_id} broke hold (status changed)")
                        held_since = None

                except Exception as e:
                    print(f"❌ JSON Parse Error for {light_id}: {e}")
                    print(f"⛔ Received: {raw_msg}")
            else:
                print(f"⏳ Timeout waiting for message on topic {topic}")

            time.sleep(0.5)

        print(f"❌ Timeout: {light_id} did not become {expected.upper()} within {duration} seconds")
        return False

    def test_interrupt_tl1_while_tl2_is_green(self):
        print("🔁 Test 1: interrupt tl_1 while tl_2 is GREEN")
        if not self.wait("tl_2", "GREEN", duration=30):
            self.fail("tl_2 is not green during 30 secs")

        try:
            requests.get("http://localhost:3000/changeGreen/tl_1")
        except Exception as e:
            self.fail(f"API error: {e}")

        if not self.wait("tl_1", "GREEN", duration=8):
            self.fail("tl_1 did not become GREEN after interrupt")

        try:
            requests.get("http://localhost:3000/reset/tl_1")
        except Exception as e:
            self.fail(f"Reset API error: {e}")

        if not self.wait("tl_1", "RED", duration=10):
            self.fail("tl_1 did not return to RED after reset")

    def test_interrupt_tl2_while_tl1_is_green(self):
        print("🔁 Test 2: interrupt tl_2 while tl_1 is GREEN")
        if not self.wait("tl_1", "GREEN", duration=30):
            self.fail("tl_1 is not GREEN during 30 secs")

        try:
            requests.get("http://localhost:3000/changeGreen/tl_2")
        except Exception as e:
            self.fail(f"API error: {e}")

        if not self.wait("tl_2", "GREEN", duration=8):
            self.fail("tl_2 did not become GREEN after interrupt")

        try:
            requests.get("http://localhost:3000/reset/tl_2")
        except Exception as e:
            self.fail(f"Reset API error: {e}")

        if not self.wait("tl_2", "RED", duration=10):
            self.fail("tl_2 did not return to RED after reset")

    def test_interrupt_tl1_during_yellow_of_tl2(self):
        print("🔁 Test 3: interrupt tl_1 while tl_2 is YELLOW")
        if not self.wait("tl_2", "YELLOW", duration=30):
            self.fail("tl_2 is not YELLOW during 30 secs")

        try:
            requests.get("http://localhost:3000/changeGreen/tl_1")
        except Exception as e:
            self.fail(f"API error: {e}")

        if not self.wait("tl_1", "GREEN", duration=5):
            self.fail("tl_1 is not GREEN after interrupt")

        try:
            requests.get("http://localhost:3000/reset/tl_1")
        except Exception as e:
            self.fail(f"Reset API error: {e}")

        if not self.wait("tl_1", "RED", duration=10):
            self.fail("tl_1 did not return to RED after reset")

    def test_interrupt_tl2_while_already_green(self):
        print("🔁 Test 4: interrupt tl_2 while it's already GREEN (should stay GREEN)")
        if not self.wait("tl_2", "GREEN", duration=30):
            self.fail("tl_2 is not GREEN during 30 secs")

        try:
            requests.get("http://localhost:3000/changeGreen/tl_2")
        except Exception as e:
            self.fail(f"API error: {e}")

        if not self.wait("tl_2", "GREEN", duration=60):
            self.fail("tl_2 did not stay GREEN after interrupt")

        try:
            requests.get("http://localhost:3000/reset/tl_2")
        except Exception as e:
            self.fail(f"Reset API error: {e}")

        if not self.wait("tl_2", "RED", duration=10):
            self.fail("tl_2 did not return to RED after reset")

if __name__ == "__main__":
    unittest.main()
