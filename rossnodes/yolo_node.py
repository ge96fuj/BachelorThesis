#!/usr/bin/env python3
import rospy
import torch
import paho.mqtt.client as mqtt
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String


camera_status_pub = {}
IMAGE_TOPIC = "camera"
IMAGE_MSG_TYPE = SensorImage
YOLO_MODEL_PATH = '/home/ast/catkin_ws/src/mixed_reality/models/Model/yolov5/runs/train/traffic_light_red_green3/weights/best.pt'
model = None
bridge = CvBridge()

def process_image(msg):
    global  model, bridge , camera_status_pub
    print("image here")
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    #cv2.imshow("Camera View", frame)
    #cv2.waitKey(1)  
    results = model(frame)
    print("results ",results)
    labels = results.pandas().xyxy[0].value_counts('name')
    print("labels ",labels)

    
    status = "unknown"
    df = results.pandas().xyxy[0]
    if df.empty:
        rospy.logwarn("No traffic lightin the img")
        camera_status_pub.publish(status)
        return
    label = df['name'].value_counts().idxmax()
    
   

    if "red" in label:
        status = "RED"
    elif "yellow" in label:
        status = "YELLOW"
    elif "green" in label:
        status = "GREEN"
    else:
        status = "unknown"

    camera_status_pub.publish(status)
    print("publishig :" , status)
 




def model_node():
        global model , camera_status_pub 

        model = torch.hub.load('ultralytics/yolov5', 'custom', path=YOLO_MODEL_PATH)
        model.conf = 0.8
        rospy.init_node("model_node")
        rospy.Subscriber(IMAGE_TOPIC , IMAGE_MSG_TYPE, process_image)
    
        # Load Traffic Light detection model
        # model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')  
        camera_status_pub = rospy.Publisher("camera/traffic_light", String, queue_size=10)
        rospy.spin()
        


          
        


        

if __name__ == '__main__':
    try:
        model_node()
    except rospy.ROSInterruptException:
            pass
