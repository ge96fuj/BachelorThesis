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

        os.system(f"node {self.MQTT_SERVER_PATH} &")
        time.sleep(10)

        os.system(f"node {self.SERVER_PATH} &")
        time.sleep(3)

        os.system(f"python3 {self.V1} &")
        time.sleep(2)

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
            self.fail("MQTT connection failed")

    def tearDown(self):
        self.client.loop_stop()
        self.client.disconnect()
        os.system("pkill -f mqtt.server.js")
        os.system("pkill -f server.js")
        os.system("pkill -f virtual_light.py")
        os.system("pkill -f virtual_light2.py")

    def on_connect(self, client, userdata, flags, rc):
        self.connected.set()
        self.client.subscribe(self.tl1_topic)
        self.client.subscribe(self.tl2_topic)

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
                except:
                    pass
            time.sleep(0.5)

        return False

    def test_invalid_light(self):
        res = requests.get("http://localhost:3000/changeGreen/invalid_id")
        self.assertEqual(res.status_code, 404)

    def test_duplicate_interrupt(self):
        res1 = requests.get("http://localhost:3000/changeGreen/tl_1")
        self.assertEqual(res1.status_code, 200)
        res2 = requests.get("http://localhost:3000/changeGreen/tl_1")
        self.assertEqual(res2.status_code, 400)

    def test_group_reset(self):
        requests.get("http://localhost:3000/changeGreen/tl_1")
        time.sleep(1)
        res = requests.get("http://localhost:3000/reset/tl_1")
        self.assertEqual(res.status_code, 200)

    def test_two_interrupts(self):
        try:
            requests.get("http://localhost:3000/changeGreen/tl_2")
            time.sleep(1)
            res = requests.get("http://localhost:3000/changeGreen/tl_1")
            self.assertEqual(res.status_code, 400)
        except Exception as e:
            self.fail(f"API error: {e}")
        if not self.wait("tl_2", "GREEN", duration=5):
            self.fail("tl_2 is not GREEN")

    def test_simu_request(self):
        try:
            res1 = requests.get("http://localhost:3000/changeGreen/tl_1")
            res2 = requests.get("http://localhost:3000/changeGreen/tl_2")

            valid = (res1.status_code == 200 and res2.status_code == 400) or \
                    (res1.status_code == 400 and res2.status_code == 200)
            self.assertTrue(valid, "Expected one request to succeed and one to fail")
        except Exception as e:
            self.fail(f"API error: {e}")

        green_lights = []
        if self.wait("tl_1", "GREEN", duration=10):
            green_lights.append("tl_1")
        if self.wait("tl_2", "GREEN", duration=10):
            green_lights.append("tl_2")

        if len(green_lights) > 1:
            self.fail(f"Both lights GREEN simultaneously: {green_lights}")
        elif len(green_lights) == 0:
            self.fail("No light turned GREEN after simultaneous requests")

    def test_reset_without_interrupt(self):
        res = requests.get("http://localhost:3000/reset/tl_1")
        self.assertEqual(res.status_code, 400)
        try:
            res = requests.get("http://localhost:3000/changeGreen/tl_1")
            self.assertEqual(res.status_code, 200)
        except Exception as e:
            self.fail(f"API error: {e}")
        time.sleep(5)
        if not self.wait("tl_1", "GREEN", duration=10, hold=True):
            self.fail("tl_1 should stay GREEN for 10 seconds")

if __name__ == "__main__":
    unittest.main()
