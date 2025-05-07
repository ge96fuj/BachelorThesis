import unittest
import paho.mqtt.client as mqtt
import threading
import time
import json

#---------Synchronising test between Client-Server-------------
#How is really the publishing of the server true ?

class MQTT_SYNC_TEST(unittest.TestCase):
    def setUp(self):
        self.broker = "192.168.0.104"
        self.state_topic = "lights/tl_1/current_state"
        self.detection_topic = "detection/traffic_lights/tl_1"
        self.arduino_state = None
        self.sv_pub = None
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.connected = threading.Event()
        self.message_received = threading.Event()
        self.client.connect(self.broker, 1883, 60)
        self.client.loop_start()
        if not self.connected.wait(timeout=5):
            self.fail("Failed to connect to MQTT sv")

    def tearDown(self):
        self.client.loop_stop()
        self.client.disconnect()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected.set()
            client.subscribe(self.state_topic)
            client.subscribe(self.detection_topic)
        else:
            self.fail(f"Failed to connect to sv, {rc}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()

        if topic == self.state_topic:
            self.arduino_state = payload
        elif topic == self.detection_topic:
                self.sv_pub = json.loads(payload).get("status")
                self.sv_pub = None

        if self.arduino_state is not None and self.sv_pub is not None:
            self.message_received.set()

    def mqtt_sync_test(self):
        Cnt = 200000
        failures = []

        for i in range(Cnt):
            match_found = False

            for attempt in range(3):
                self.arduino_state = None
                self.sv_pub = None
                self.message_received.clear()

                print(f"\nTest {i+1}/{Cnt}, Attempt {attempt+1}/3: ")

                if not self.message_received.wait(timeout=1):
                    print(" Timeout")
                    time.sleep(0.05)
                    continue

                print(f"  Client_st :     {self.arduino_state}")
                print(f"  Sv_st: {self.sv_pub}")

                status_map = {
                    "RED": 0,
                    "YELLOW": 1,
                    "GREEN": 2,
                    "BLINK": 4,
                    "UNKNOWN": 5
                }
                converted_detection = status_map.get(self.sv_pub, 40)

                if str(self.arduino_state).strip() == str(converted_detection):
                    print("  Match")
                    match_found = True
                    break
                else:
                    print(f"  no matching arduino , {self.arduino_state}  sv ={converted_detection}")
                    time.sleep(0.05)

            if not match_found:
                failures.append(
                    f"Test {i+1}: all tries failed  Arduin ={self.arduino_state}, Sv={self.sv_pub}"
                )

            time.sleep(0.01)  

        if failures:
            print(f"\n{len(failures)} of {Cnt} tests failed:")
            for fail in failures:
                print(fail)
            self.fail(f"{len(failures)} test(s) failed")
        else:
            print(f"\nAll {Cnt} tests passed")

if __name__ == "__main__":
    unittest.main()
