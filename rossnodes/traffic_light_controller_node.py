#!/usr/bin/env python3
#FIX IMPORTS LATER
import rospy
from mixed_reality.msg import SimPose, Obstacles
from std_msgs.msg import String
import functools
import math
# Configuration

TRAFFIC_LIGHTS = {}
CHOICE_METHOD = 2 #0 both should be the same , mqtt and camera #1 mqtt or camera ,
POSE_TOPIC = "sim/euler"
POSE_MSG_TYPE = SimPose

OBSTACLE_TOPIC = "obstacles"
OBSTACLE_MSG_TYPE = Obstacles

THRESHOLD = 5.0

last_command = {}
mqtt_tl_status = {}
camera_status = {}
Traffic_Light_subscribed = {}
current_pose = [0.0,0.0,0.0]
traffic_light_controller_publisher = {}







def check_car_proximity():
    global traffic_light_controller_publisher
    for tl_id, tl_position in TRAFFIC_LIGHTS.items():
        if is_near_traffic_light(tl_id, tl_position):
            status = mqtt_tl_status.get(tl_id, "unknown")
            camera = camera_status
            action = 'nothing'
            if (status == 'RED' and camera == 'RED' ) or (status == 'YELLOW' and camera == 'YELLOW' ):
                action = 's'
            if (status == 'GREEN' and camera == 'GREEN') or (status == 'BLINK' and camera == 'BLINK' ):
                action = 'g'
            print('ACTION ISSSSSS : ' , action)    
            print('CAMERA STATUS IS : ' , camera)    
            print('MQTT STATUS IS : ' , status)  

            traffic_light_controller_publisher = rospy.Publisher("/traffic_light/action", String, queue_size=10)
            traffic_light_controller_publisher.publish(action)

            

            




def is_near_traffic_light(tl_id, tl_position):
    ego_x, ego_y = current_pose[0], current_pose[1]
    tl_x, tl_y = tl_position
    distance = math.sqrt((tl_x - ego_x) ** 2 + (tl_y - ego_y) ** 2)

    if distance < THRESHOLD:
        print("CAR IS NEAR TRAFFIC LIGHT")
        return True
    print("CAR IS NOOOOOT NEAR TRAFFIC LIGHT")
    return False




def update_pose(msg):
    global current_pose
    current_pose = [msg.x,msg.y,msg.yaw]    



def update_mqtt_traffic_light_status(msg,obstacle_name):
    mqtt_tl_status[obstacle_name]=msg.data




    
def update_traffic_lights(msg):
    global TRAFFIC_LIGHTS
    global last_command 
    global mqtt_tl_status 
    for obstacle in msg.data:
        if "traffic" in obstacle.name:
            TRAFFIC_LIGHTS[obstacle.name] = (obstacle.x, obstacle.y)
            if obstacle.name not in last_command:
                last_command[obstacle.name] = None
                mqtt_tl_status[obstacle.name] = "unknown"
                #if obstacle.name not in Traffic_Light_subscribed :
                print("subscribing to topic " , obstacle.name)
                #Traffic_Light_subscribed[obstacle.name] = True
                topic = f"/traffic_light/{obstacle.name}"
                #rospy.Subscriber(topic , String , update_mqtt_traffic_light_status)
                rospy.Subscriber(topic, String,functools.partial(update_mqtt_traffic_light_status, obstacle_name=obstacle.name))

    
def update_camera_status(msg):
    global camera_status
    camera_status = msg.data



def traffic_light_controller_node():

        global current_pose
        global last_command
        global mqtt_tl_status
        
        rospy.init_node("traffic_light_controller_node")
        rospy.Subscriber(POSE_TOPIC, POSE_MSG_TYPE, update_pose)
        rospy.Subscriber(OBSTACLE_TOPIC ,OBSTACLE_MSG_TYPE , update_traffic_lights)
        rospy.Subscriber("camera/traffic_light" , String , update_camera_status)
        
        
        for tl_id in TRAFFIC_LIGHTS:
            last_command[tl_id] = None
            mqtt_tl_status[tl_id] = "unknown"


        rospy.Timer(rospy.Duration(1.0), lambda event: check_car_proximity())

        rospy.spin()     


          
        


        

if __name__ == '__main__':
    try:
        traffic_light_controller_node()
    except rospy.ROSInterruptException:
            pass
