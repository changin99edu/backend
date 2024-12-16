import rospy
from nav_msgs.msg import Odometry
import websocket
import json
import time

def odom_callback(data):
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation
    robot_data = {
        'x': position.x,
        'y': position.y,
        'orientation': {
            'x': orientation.x,
            'y': orientation.y,
            'z': orientation.z,
            'w': orientation.w
        }
    }
    try:
        ws.send(json.dumps(robot_data))
    except (websocket.WebSocketConnectionClosedException, BrokenPipeError):
        rospy.logwarn("WebSocket connection closed, trying to reconnect...")
        connect_websocket()

def connect_websocket():
    global ws
    while True:
        try:
            ws = websocket.WebSocket()
            ws.connect("ws://172.30.1.33:5050")  # WebSocket 서버의 새로운 포트로 변경
            rospy.loginfo("WebSocket connected")
            break
        except Exception as e:
            rospy.logwarn("Failed to connect WebSocket: %s", e)
            time.sleep(5)  # 5초 후에 재연결 시도

if __name__ == '__main__':
    rospy.init_node('robot_position_publisher')
    connect_websocket()
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()
