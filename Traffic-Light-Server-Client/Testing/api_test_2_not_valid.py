import unittest
import paho.mqtt.client as mqtt
import threading
import unittest
import paho.mqtt.client as mqtt
import threading
import time
import json
import os
import requests

class apiTester(unittest.TestCase):

    def setUp(self):
        self.MQTT_SERVER_PATH = '/Users/skanderjneyeh/Documents/git_rep_BA/TrafficLightThesis/Server/services/mqtt.server.js'
        self.SERVER_PATH = '/Users/skanderjneyeh/Documents/git_rep_BA/TrafficLightThesis/Server/server.js'
        self.V1 = '/Users/skanderjneyeh/Documents/git_rep_BA/TrafficLightThesis/Server/simulator/virtual_light.py'
        self.V2 = '/Users/skanderjneyeh/Documents/git_rep_BA/TrafficLightThesis/Server/simulator/virtual_light2.py'
        self.BROKER_IP = '10.181.241.34'

        self.tl1_topic = "detection/traffic_lights/tl_1"
        self.tl2_topic = "detection/traffic_lights/tl_2"

        print("Start MQTT ..")
        os.system(f"node {self.MQTT_SERVER_PATH} &")
        time.sleep(10)

        print("Start sv ...")
        os.system(f"node {self.SERVER_PATH} &")
        time.sleep(3)

        print("Start tl_1...")
        os.system(f"python3 {self.V1} &")
        time.sleep(2)

        print("Start tl_2...")
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
            self.fail("Mqtt fail")

    def tearDown(self):
        self.client.loop_stop()
        self.client.disconnect()

        os.system("pkill -f mqtt.server.js")
        os.system("pkill -f server.js")
        os.system("pkill -f virtual_light.py")
        os.system("pkill -f virtual_light2.py")

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected")
        self.connected.set()
        self.client.subscribe(self.tl1_topic)
        self.client.subscribe(self.tl2_topic)
        print(f"Subscribed to: {self.tl1_topic}, {self.tl2_topic}")

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
        end = time.time() + duration
        held_since = None

        while time.time() < end:
            self.message_received.clear()
            if self.message_received.wait(timeout=2):
                try:
                    payload = json.loads(received_state())
                    status = payload.get("status", "").upper()
                    print(f"{light_id} status: {status}")
                    if status == expected.upper():
                        if hold:
                            if held_since is None:
                                held_since = time.time()
                            elif time.time() - held_since >= duration:
                                return True
                        else:
                            return True
                    elif hold:
                        held_since = None
                except Exception as e:
                    print(f"Parse error for {light_id}: {e}")
            time.sleep(0.5)

        return False

    def test_interrupt_tl1_then_reset(self):
        print("Waiting for tl_2 to be GREEN...")

        timeout = time.time() + 60
        while time.time() < timeout:
            self.message_received.clear()

            if self.message_received.wait(timeout=2):
                try:
                    status = json.loads(self.received_state_tl2).get("status", "").upper()
                    print(f"tl_2 status: {status}")
                    if status == "GREEN":
                        print("tl_2 is GREEN — proceeding with interrupt")
                        break
                except Exception as e:
                    print(f"Could not parse tl_2 payload: {e}")
            time.sleep(0.5)
        else:
            self.fail("tl_2 never turned GREEN in time")

        try:
            res = requests.get("http://localhost:3000/changeGreen/tl_1")
            print(f"Triggered interrupt: changeGreen/tl_1 → {res.status_code}")
            self.assertEqual(res.status_code, 200)
        except Exception as e:
            self.fail(f"Failed to send interrupt: {e}")

        print("Waiting for tl_1 to turn GREEN after interrupt...")
        green_timeout = time.time() + 10
        while time.time() < green_timeout:
            self.message_received.clear()
            if self.message_received.wait(timeout=2):
                try:
                    status = json.loads(self.received_state_tl1).get("status", "").upper()
                    print(f"tl_1 status: {status}")
                    if status == "GREEN":
                        print("tl_1 is GREEN — interrupt accepted.")
                        break
                except Exception as e:
                    print(f"Could not parse tl_1 payload: {e}")
            time.sleep(0.5)
        else:
            self.fail("tl_1 did not turn GREEN after interrupt")

        try:
            res = requests.get("http://localhost:3000/reset/tl_1")
            print(f"Sent reset to tl_1 → {res.status_code}")
            self.assertEqual(res.status_code, 200)
        except Exception as e:
            self.fail(f"Failed to send reset: {e}")

        print("Waiting for tl_1 to turn RED after reset...")
        red_timeout = time.time() + 10
        while time.time() < red_timeout:
            self.message_received.clear()
            if self.message_received.wait(timeout=2):
                try:
                    status = json.loads(self.received_state_tl1).get("status", "").upper()
                    print(f"tl_1 status: {status}")
                    if status == "RED":
                        print("tl_1 returned to RED after reset.")
                        self.assertTrue(True)
                        return
                except Exception as e:
                    print(f"Could not parse tl_1 payload: {e}")
            time.sleep(0.5)

        self.fail("tl_1 did not turn RED after reset")

    def test_tl2_green_triggers_http_then_tl1_should_be_green(self):
        print("Phase 1: Waiting for tl_2 to be GREEN...")

        timeout = time.time() + 60
        while time.time() < timeout:
            self.message_received.clear()

            if self.message_received.wait(timeout=2):
                try:
                    status_obj = json.loads(self.received_state_tl2)
                    tl2_status = status_obj.get("status", "").upper()
                    print(f"tl_2 status: {tl2_status}")

                    if tl2_status == "GREEN":
                        print("tl_2 is GREEN — triggering HTTP action...")
                        break
                except Exception as e:
                    print(f"Could not parse tl_2 payload: {e}")
            time.sleep(0.5)
        else:
            self.fail("tl_2 never turned GREEN within timeout")

        try:
            response = requests.get("http://localhost:3000/changeGreen/tl_1")
            print(f"HTTP request triggered, response code: {response.status_code}")
        except Exception as e:
            self.fail(f"Failed to send HTTP request: {e}")

        print("Phase 3: Waiting for tl_1 to be GREEN...")
        tl1_timeout = time.time() + 10
        while time.time() < tl1_timeout:
            self.message_received.clear()

            if self.message_received.wait(timeout=2):
                try:
                    status_obj = json.loads(self.received_state_tl1)
                    tl1_status = status_obj.get("status", "").upper()
                    print(f"tl_1 status: {tl1_status}")

                    if tl1_status == "GREEN":
                        print("tl_1 is GREEN. Test successful!")
                        self.assertTrue(True)
                        return
                except Exception as e:
                    print(f"Could not parse tl_1 payload: {e}")
            time.sleep(0.5)

        self.fail("tl_1 did not turn GREEN within 10 seconds after HTTP trigger")

if __name__ == "__main__":
    unittest.main()
