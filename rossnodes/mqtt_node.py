#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
from mixed_reality.msg import Obstacles , Obstacle
#from sensor_msgs.msg import Image as SensorImage
#from cv_bridge import CvBridge
from std_msgs.msg import String


TRAFFIC_LIGHTS = {}


OBSTACLE_TOPIC = "obstacles"
OBSTACLE_MSG_TYPE = Obstacles
BROKER_IP = "192.168.0.103"
Mqtt_client = None
Mqtt_status = {}
Mqtt_Connected = False 
mqtt_status_publishers = {}
last_command = {}


def update_traffic_lights(msg):
    global TRAFFIC_LIGHTS, last_command, Mqtt_status, Mqtt_client
    print(msg)
    for obstacle in msg.data:
        if "traffic" in obstacle.name:
            TRAFFIC_LIGHTS[obstacle.name] = (obstacle.x, obstacle.y)
            if obstacle.name not in last_command and Mqtt_client is not None:
                last_command[obstacle.name] = True
                Mqtt_status[obstacle.name] = "unknown"
                topic = f"detection/status/{obstacle.name}"
                Mqtt_client.subscribe(topic)
                print(f"Subscribed to MQTT topic: {topic}")

def on_mqtt_message(client, userdata, message):
    topic = message.topic
    tl_id = topic.split("/")[-1]
    Mqtt_status[tl_id] = message.payload.decode()
    print("MQTT:", tl_id, "status is:", Mqtt_status[tl_id])
    status_str = Mqtt_status[tl_id]
    topic_name = f"/traffic_light/{tl_id}"
    print(topic_name)

    if tl_id not in mqtt_status_publishers:
        mqtt_status_publishers[tl_id] = rospy.Publisher(topic_name, String, queue_size=10)

    mqtt_status_publishers[tl_id].publish(status_str)
    print("publishing " , status_str , "to", topic_name)


def mqtt_connect():
    global Mqtt_client, Mqtt_Connected 
    while not Mqtt_Connected:
        try:
            print("Trying to connect to MQTT")
            Mqtt_client.connect(BROKER_IP, 1883, 60)
            Mqtt_client.loop_start()
            
            
        except Exception as e:
            print(f"MQTT connection failed: {e}")
        rospy.sleep(2)      

        

def on_connect(client, userdata, flags, rc):
    global Mqtt_Connected
    if rc == 0:
        print("Connected.")
        Mqtt_Connected = True
    else:
        print("Failed connecting to MQTT", rc)
        Mqtt_Connected = False

           
   





def traffic_light_node():
    global TRAFFIC_LIGHTS, Mqtt_client, Mqtt_status, Mqtt_Connected
    global mqtt_status_pub
    rospy.init_node('traffic_light_node')
    mqtt_status_pub = rospy.Publisher("mqtt_status", String, queue_size=10)
    rospy.Subscriber(OBSTACLE_TOPIC, OBSTACLE_MSG_TYPE, update_traffic_lights)
    
    TRAFFIC_LIGHTS["traffic1"]=(50,23)
    Mqtt_client = mqtt.Client()
    Mqtt_client.on_message = on_mqtt_message
    Mqtt_client.on_connect = on_connect
    mqtt_connect()
    
    
    rospy.spin()


    

if __name__ == '__main__':
    try:
        traffic_light_node()
    except rospy.ROSInterruptException:
        pass
