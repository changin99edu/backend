import rospy
from nav_msgs.msg import Odometry
import websocket
import json

# WebSocket 서버와 연결 설정 (WebSocket 서버 주소로 변경)
ws = websocket.WebSocket()
ws.connect("ws://localhost:5050")  # WebSocket 서버 주소를 설정하세요

# /odom 토픽에서 수신한 데이터를 WebSocket을 통해 전송하는 콜백 함수
def odom_callback(data):
    # 로봇의 위치와 방향 정보
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

    # 좌표 데이터를 JSON 형식으로 변환
    robot_data = {
        "x": position.x,
        "y": position.y,
        "z": position.z,
        "orientation": {
            "x": orientation.x,
            "y": orientation.y,
            "z": orientation.z,
            "w": orientation.w
        }
    }

    # JSON 형식으로 WebSocket을 통해 좌표 데이터 전송
    #ws.send(json.dumps(robot_data))
    #rospy.loginfo(f"Sent data: {robot_data}")  # 전송된 데이터 로그

# ROS 노드를 초기화하고 /odom 토픽을 구독하여 데이터를 처리하는 함수
def listener():
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()  # 노드를 계속 실행 상태로 유지

if __name__ == '__main__':
    listener()
