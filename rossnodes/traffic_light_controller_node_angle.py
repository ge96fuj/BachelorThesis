#!/usr/bin/env python3
#FIX IMPORTS LATER
import rospy
from mixed_reality.msg import SimPose, Obstacles
from std_msgs.msg import String , Bool
from mixed_reality.utils.control_utils import Waypoint_control_utils
import functools
import math
# Configuration

TRAFFIC_LIGHTS = {}
CHOICE_METHOD = 2 #0 both should be the same , mqtt and camera #1 mqtt or camera , TO IMPLEMENT LATER 
POSE_TOPIC = "sim/euler"
POSE_MSG_TYPE = SimPose

OBSTACLE_TOPIC = "obstacles"
OBSTACLE_MSG_TYPE = Obstacles

THRESHOLD = 8.0

last_command = {}
mqtt_tl_status = {}
camera_status = {}
Traffic_Light_subscribed = {}
current_pose = [0.0,0.0,0.0]
traffic_light_controller_publisher = {}
# pub_going = {}






def check_car_proximity():
    global traffic_light_controller_publisher 
    for tl_id, tl_position in TRAFFIC_LIGHTS.items():
        if is_near_traffic_light(tl_id, tl_position):
            mqtt_status = mqtt_tl_status.get(tl_id, "unknown")
            camera = camera_status
            source = "MQTT" if mqtt_status != "unknown" else "Camera"
            final_status = mqtt_status if mqtt_status != "unknown" else camera

            action = 'nothing'
            print(f"[TL {tl_id}] USING {source} STATUS: {final_status}")

            if final_status in ['RED', 'YELLOW']:
                action = 's'
                print("STOOOOOOOOOOOOP(", source, ")")
                pub_going = rospy.Publisher("/going", Bool, queue_size=10)
                pub_going.publish(False)

            elif final_status in ['GREEN', 'BLINK']:
                action = 'g'
                print("GOOOOOOOOOO (", source, ")")
                pub_going = rospy.Publisher("/going", Bool, queue_size=10)
                pub_going.publish(True)

            print(f"ACTION: {action} | CAMERA: {camera} | MQTT: {mqtt_status}")


            

            
def is_near_traffic_light(tl_id, tl_position):
    ego_x, ego_y, ego_yaw = current_pose
    tl_x, tl_y = tl_position

 
    distance = math.sqrt((tl_x - ego_x) ** 2 + (tl_y - ego_y) ** 2)
    print(f"Current TL: x {tl_x} y {tl_y} and CAR: x {ego_x} Y {ego_y} distance: {distance}")


    if distance >= THRESHOLD:
        print("CAR IS NOOOOOT NEAR TRAFFIC LIGHT")
        return False


    car_pose = [ego_x, ego_y, 0]
    car_orientation = [0, ego_yaw, 0]


    _, _, _, angle_diff = waypoint_checker.calculate_control(tl_x, tl_y, car_pose, car_orientation)

    if abs(angle_diff) < 90:
        print("CAR IS NEAR AND FACING TRAFFIC LIGHT")
        return True

    print("CAR IS NEAR BUT NOT FACING TRAFFIC LIGHT")
    return False




# def is_near_traffic_light(tl_id, tl_position):
#     ego_x, ego_y, ego_yaw = current_pose
#     tl_x, tl_y = tl_position

#     dx = tl_x - ego_x
#     dy = tl_y - ego_y
#     distance = math.sqrt(dx**2 + dy**2)


#     angle_to_tl = math.atan2(dy, dx)
    

#     def normalize_angle(angle):
#         return (angle + math.pi) % (2 * math.pi) - math.pi
    
#     angle_diff = normalize_angle(angle_to_tl - ego_yaw)
#     angle_diff_deg = math.degrees(abs(angle_diff))

#     print(f"Distance: {distance}, Angle Diff (deg): {angle_diff_deg}")

#     if distance < THRESHOLD and angle_diff_deg < 90:
#         print("CAR IS NEAR AND FACING TL")
#         return True

#     print("CAR IS NOT NEAR OR FACING AWAY FROM TL")
#     return False




def update_pose(msg):
    global current_pose
    current_pose = [msg.x,msg.z,msg.yaw]    



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
                print("we are here")
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
      
        print("STARTING NODE")
        
        rospy.init_node("traffic_light_controller_node")
        rospy.Subscriber(POSE_TOPIC, POSE_MSG_TYPE, update_pose)
        rospy.Subscriber(OBSTACLE_TOPIC ,OBSTACLE_MSG_TYPE , update_traffic_lights)
        rospy.Subscriber("camera/traffic_light" , String , update_camera_status)
        # pub_going=rospy.Publisher("/going",Bool,queue_size=10)
        
        
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
