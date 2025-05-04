#FIX IMPORTS 
import rospy
import torch
import paho.mqtt.client as mqtt
from mixed_reality.msg import SimPose, WaypointList, Waypoint, Obstacles
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
from collections import deque, defaultdict
import json

# Configuration
#Traffic lights informations should be done manually or over the system ... ? 
TRAFFIC_LIGHTS = {}

POSE_TOPIC = "sim/euler"
POSE_MSG_TYPE = SimPose

IMAGE_TOPIC = "camera"
IMAGE_MSG_TYPE = SensorImage

OBSTACLE_TOPIC = "obstacles"
OBSTACLE_MSG_TYPE = Obstacles

YOLO_MODEL_PATH = '/Users/skanderjneyeh/Downloads/yolov5/runs/train/traffic_light_red_green3/weights/best.pt'
BROKER_IP = "localhost"

DETECTION_RADIUS = 5.0


last_command = {}
mqtt_status = {}
mqtt_history = defaultdict(lambda: deque(maxlen=3)) #STORE LAST 3 Msg for each Traffic light
current_pose = [0.0,0.0,0.0]
model = torch.hub.load('ultralytics/yolov5', 'custom', path=YOLO_MODEL_PATH)
mqtt_client = None
bridge = CvBridge()


def is_near_traffic_light(tl_id, tl_position):
    #TODO : IMPLEMENT 
    return True


def process_image(msg):
    global current_pose, model, bridge, TRAFFIC_LIGHTS, last_command, mqtt_status
    if current_pose is None:
        return

    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    results = model(frame)
    labels = results.pandas().xyxy[0]['name'].tolist()

    for tl_id, tl_position in TRAFFIC_LIGHTS.items():
        if not is_near_traffic_light(tl_id, tl_position):
            continue

        camera_status = "unknown"
        if "traffic-light_red" in labels:
            camera_status = "red"
        elif "traffic-light_green" in labels:
            camera_status = "green"
        elif "traffic-light_yellow" in labels:
            camera_status = "yellow"

        mqtt_val = mqtt_status.get(tl_id, "unknown")
        final_status = camera_status if camera_status != "unknown" else mqtt_val

        rospy.loginfo(f"[{tl_id}] Camera: {camera_status}, MQTT: {mqtt_val}, Final: {final_status}")

        if final_status == "red" and last_command[tl_id] != "s":
            #publish Stop 's'
            print('STOOOOOOOOOOOOP')
            last_command[tl_id] = "s"
            rospy.loginfo(f"[{tl_id}] Red detected — sent 's'")
        elif final_status == "green" and last_command[tl_id] != "g":
            #publish go 's'
            print('GOOOOOOOOOOOOOOO')
            last_command[tl_id] = "g"
            rospy.loginfo(f"[{tl_id}] Green detected — sent 'g'")


def on_mqtt_message(client, userdata, message):
    #to reduce the wrong results percentage , 3 last msg of mqtt should match
    global mqtt_status, mqtt_history

    topic = message.topic
    tl_id = topic.split("/")[-1]

    try:
        data = json.loads(message.payload.decode())
        status = data.get("status")
    except Exception as e:
        print(f"Error decoding MQTT payload: {e}")
        return

    
    mqtt_history[tl_id].append(status)

    # check if 3 status are matching
    if len(mqtt_history[tl_id]) == 3 and len(set(mqtt_history[tl_id])) == 1:
        mqtt_status[tl_id] = status
        print(f"[MQTT] {tl_id}  Confirmed: {status}")
    else:
        mqtt_status ="UNKNOWN"
        print(f"[MQTT] {tl_id}  NOT Confirmed: {status}")
        
     
     

def update_pose(msg):
    global current_pose
    current_pose = [msg.x,msg.z,msg.yaw]    

    
def update_traffic_lights(msg):
    global TRAFFIC_LIGHTS
    global last_command 
    global mqtt_status 
    for obstacle in msg.data:
        if "traffic_light" in obstacle.name:
            TRAFFIC_LIGHTS[obstacle.name] = (obstacle.x, obstacle.y)
            if obstacle.name not in last_command:
                last_command[obstacle.name] = None
                mqtt_status[obstacle.name] = "unknown"
                topic = f"detection/status/{obstacle.name}"
                mqtt_client.subscribe(topic)
                rospy.loginfo(f"Subscribed to MQTT topic: {topic}")


def traffic_light_node():
        global model
        global current_pose
        global mqtt_client
        global last_command
        global mqtt_status
        
        rospy.init_node("traffic_light_node")
        
        rospy.Subscriber(POSE_TOPIC, POSE_MSG_TYPE, update_pose)
        rospy.Subscriber(IMAGE_TOPIC , IMAGE_MSG_TYPE, process_image)
        rospy.Subscriber(OBSTACLE_TOPIC ,OBSTACLE_MSG_TYPE , update_traffic_lights)

    
        # Load Traffic Light detection model
       # model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')  
        model.conf = 0.8
        

        for tl_id in TRAFFIC_LIGHTS:
            last_command[tl_id] = None
            mqtt_status[tl_id] = "unknown"

        # MQTT setup
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = on_mqtt_message
        try:
            mqtt_client.connect(BROKER_IP, 1883, 60)
            for tl_id in TRAFFIC_LIGHTS:
             topic = f"detection/status/{tl_id}"
             mqtt_client.subscribe(topic)
             print("Subscribed to MQTT TOPIC ", topic)
             
        except Exception as e :
            rospy.logwarn(f"MQTT connection failed: {e}")
            print("MQTT connection failed:" , e)
            mqtt_client = None      
        mqtt_client.loop_start()
          
        


        

if __name__ == '__main__':
    try:
        traffic_light_node()
    except rospy.ROSInterruptException:
            pass
